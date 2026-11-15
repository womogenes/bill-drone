'use client';

import { useEffect, useMemo, useRef } from 'react';
import { useSize } from '@/hooks/use-size';
import type { TrajectoryChannel } from '@/lib/trajectory';
import type { Palette } from './palette';

// Small multiples, one strip per channel group, sharing the replay's time: each line labeled at its value under the cursor instead of a legend, the zero line where it falls, and nothing else (BRIEF.md). Dragging across a strip seeks.
export function Strips({
  channels,
  rate,
  time,
  onSeek,
  palette,
}: {
  channels: TrajectoryChannel[];
  rate: number;
  time: number;
  onSeek: (time: number) => void;
  palette: Palette;
}) {
  const groups = useMemo(() => {
    const order: string[] = [];
    for (const channel of channels) if (!order.includes(channel.group)) order.push(channel.group);
    return order.map((group) => channels.filter((channel) => channel.group === group));
  }, [channels]);
  return (
    <div className="flex h-full flex-col divide-y divide-border">
      {groups.map((group) => (
        <Strip key={group[0].group} channels={group} rate={rate} time={time} onSeek={onSeek} palette={palette} />
      ))}
    </div>
  );
}

const LABEL_WIDTH = 92;
const PAD = 6;

function Strip({ channels, rate, time, onSeek, palette }: { channels: TrajectoryChannel[]; rate: number; time: number; onSeek: (time: number) => void; palette: Palette }) {
  const box = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const { width, height } = useSize(box);
  const frames = channels[0].values.length;
  const [low, high] = useMemo(() => {
    let low = Infinity;
    let high = -Infinity;
    for (const channel of channels)
      for (const value of channel.values)
        if (value !== null) {
          low = Math.min(low, value);
          high = Math.max(high, value);
        }
    if (!Number.isFinite(low)) return [0, 1];
    const margin = (high - low) * 0.08 || 1;
    return [low - margin, high + margin];
  }, [channels]);
  const plot = width - LABEL_WIDTH;
  const x = (frame: number) => (frame / Math.max(frames - 1, 1)) * plot;
  const y = (value: number) => PAD + (1 - (value - low) / (high - low)) * (height - 2 * PAD);

  useEffect(() => {
    const element = canvas.current;
    if (!element || width < LABEL_WIDTH + 10) return;
    const scale = window.devicePixelRatio || 1;
    element.width = width * scale;
    element.height = height * scale;
    const context = element.getContext('2d')!;
    context.scale(scale, scale);
    context.clearRect(0, 0, width, height);
    if (low < 0 && high > 0) {
      context.strokeStyle = palette.hairline;
      context.lineWidth = 1;
      context.beginPath();
      context.moveTo(0, Math.round(y(0)) + 0.5);
      context.lineTo(plot, Math.round(y(0)) + 0.5);
      context.stroke();
    }
    // Every channel decimated to one point per pixel column at most.
    const step = Math.max(1, Math.floor(frames / plot));
    channels.forEach((channel, index) => {
      context.strokeStyle = channel.dashed ? palette.gray : palette.shades[Math.min(index, palette.shades.length - 1)];
      context.setLineDash(channel.dashed ? [3, 3] : []);
      context.lineWidth = 1.25;
      context.beginPath();
      let pen = false;
      for (let frame = 0; frame < frames; frame += step) {
        const value = channel.values[frame];
        if (value === null) {
          pen = false;
          continue;
        }
        if (pen) context.lineTo(x(frame), y(value));
        else context.moveTo(x(frame), y(value));
        pen = true;
      }
      context.stroke();
    });
    context.setLineDash([]);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [channels, width, height, low, high, frames, palette]);

  const frame = Math.min(Math.round(time * rate), frames - 1);
  const seek = (event: React.PointerEvent) => {
    if (event.buttons !== 1) return;
    const left = event.currentTarget.getBoundingClientRect().left;
    onSeek((Math.min(Math.max(event.clientX - left, 0), plot) / plot) * ((frames - 1) / rate));
  };
  const solid = channels.filter((channel) => !channel.dashed);
  return (
    <div ref={box} className="relative min-h-0 flex-1 touch-none select-none" onPointerDown={seek} onPointerMove={seek}>
      <canvas ref={canvas} className="absolute inset-0 h-full w-full" />
      <div className="pointer-events-none absolute top-0 h-full w-px" style={{ left: x(frame), background: palette.signal }} />
      <div className="pointer-events-none absolute inset-y-0 right-0 flex flex-col justify-center gap-0.5 pr-3 text-right" style={{ width: LABEL_WIDTH }}>
        <span className="text-[11px] text-muted-foreground">
          {channels[0].group}
          {channels[0].unit && ` ${channels[0].unit}`}
        </span>
        {solid.map((channel, index) => {
          const value = channel.values[frame];
          return (
            <span key={channel.name} className="text-xs tabular-nums" style={{ color: palette.shades[Math.min(index, palette.shades.length - 1)] }}>
              {channel.name} {value === null ? '–' : value.toFixed(Math.abs(value) >= 100 ? 0 : 1)}
            </span>
          );
        })}
      </div>
    </div>
  );
}
