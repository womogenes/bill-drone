'use client';

import { Pause, Play } from 'lucide-react';
import type { TrajectoryEvent } from '@/lib/trajectory';
import { SIGNAL } from './palette';
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
      <button
        type="button"
        onClick={onToggle}
        aria-label={playing ? 'Pause' : 'Play'}
        className="grid size-8 place-items-center rounded-full text-[var(--replay-ink)] transition-colors duration-150 hover:bg-[var(--replay-hairline)]"
      >
        {playing ? <Pause className="size-4" /> : <Play className="size-4" />}
      </button>
      <span className="w-32 whitespace-nowrap text-sm tabular-nums text-[var(--replay-ink)]">
        {clock(time)} <span className="text-[var(--replay-gray)]">/ {clock(length)}</span>
      </span>
      <div className="relative h-8 flex-1 cursor-pointer touch-none" onPointerDown={seek} onPointerMove={seek}>
        <div className="absolute inset-x-0 top-1/2 h-px bg-[var(--replay-hairline)]" />
        <div className="absolute left-0 top-1/2 h-px bg-[var(--replay-ink)]" style={{ width: share(time) }} />
        {events.map((event, index) => (
          <div
            key={index}
            title={`${clock(event.time)} ${event.label}`}
            className="absolute top-1/2 h-3 w-px -translate-y-1/2"
            style={{ left: share(event.time), background: event.signal ? SIGNAL : 'var(--replay-gray)' }}
          />
        ))}
        <div className="absolute top-1/2 size-3 -translate-x-1/2 -translate-y-1/2 rounded-full" style={{ left: share(time), background: SIGNAL }} />
      </div>
      <div className="flex gap-1">
        {SPEEDS.map((value) => (
          <button
            key={value}
            type="button"
            onClick={() => onSpeed(value)}
            className={`rounded px-1.5 py-0.5 text-xs tabular-nums transition-colors duration-150 ${value === speed ? 'bg-[var(--replay-ink)] text-[var(--replay-paper)]' : 'text-[var(--replay-gray)] hover:text-[var(--replay-ink)]'}`}
          >
            {value}×
          </button>
        ))}
      </div>
    </div>
  );
}
