'use client';

import { Pause, Play } from 'lucide-react';
import { Button } from '@/components/ui/button';
import type { TrajectoryEvent } from '@/lib/trajectory';
import { SPEEDS } from './use-playback';

function clock(seconds: number) {
  const minutes = Math.floor(seconds / 60);
  return `${minutes}:${(seconds - minutes * 60).toFixed(1).padStart(4, '0')}`;
}

// Play, the time, the track with its events, and the speed. The track seeks on press and drag; an event's label shows on hover.
export function Timeline({
  time,
  length,
  events,
  playing,
  speed,
  onToggle,
  onSeek,
  onSpeed,
}: {
  time: number;
  length: number;
  events: TrajectoryEvent[];
  playing: boolean;
  speed: number;
  onToggle: () => void;
  onSeek: (time: number) => void;
  onSpeed: (speed: number) => void;
}) {
  const share = (value: number) => `${(Math.min(Math.max(value / (length || 1), 0), 1) * 100).toFixed(3)}%`;
  const seek = (event: React.PointerEvent<HTMLDivElement>) => {
    if (event.buttons !== 1) return;
    const { left, width } = event.currentTarget.getBoundingClientRect();
    onSeek(((event.clientX - left) / width) * length);
  };
  return (
    <div className="flex h-12 items-center gap-4 px-4">
      <Button variant="ghost" size="icon" onClick={onToggle} aria-label={playing ? 'Pause' : 'Play'}>
        {playing ? <Pause /> : <Play />}
      </Button>
      <span className="w-32 whitespace-nowrap text-sm tabular-nums text-foreground">
        {clock(time)} <span className="text-muted-foreground">/ {clock(length)}</span>
      </span>
      <div className="relative h-8 flex-1 cursor-pointer touch-none" onPointerDown={seek} onPointerMove={seek}>
        <div className="absolute inset-x-0 top-1/2 h-px bg-border" />
        <div className="absolute left-0 top-1/2 h-px bg-foreground" style={{ width: share(time) }} />
        {events.map((event, index) => (
          <div
            key={index}
            title={`${clock(event.time)} ${event.label}`}
            className="absolute top-1/2 h-3 w-px -translate-y-1/2"
            style={{ left: share(event.time), background: event.signal ? 'var(--replay-signal)' : 'var(--muted-foreground)' }}
          />
        ))}
        <div className="absolute top-1/2 size-3 -translate-x-1/2 -translate-y-1/2 rounded-full" style={{ left: share(time), background: 'var(--replay-signal)' }} />
      </div>
      <div className="flex gap-1">
        {SPEEDS.map((value) => (
          <Button key={value} size="xs" variant={value === speed ? 'secondary' : 'ghost'} onClick={() => onSpeed(value)} className="tabular-nums">
            {value}×
          </Button>
        ))}
      </div>
    </div>
  );
}
