'use client';

import dynamic from 'next/dynamic';
import { useEffect } from 'react';
import { duration, frameCount, poseAt, type Trajectory } from '@/lib/trajectory';
import { paletteStyle } from './palette';
import { Strips } from './strips';
import { Timeline } from './timeline';
import { SPEEDS, usePlayback } from './use-playback';

// three.js loads after the page, on its own: the strips and timeline paint first.
const Scene = dynamic(() => import('./scene').then((module) => module.Scene), { ssr: false });

// A trajectory replayed: the drone in 3D (the hero), the strips beside it, the timeline under both. Space plays and pauses, the arrows step a tenth of a second (a second with shift), [ and ] change the speed.
export function Replay({ trajectory }: { trajectory: Trajectory }) {
  const length = duration(trajectory);
  const { time, seek, playing, toggle, speed, setSpeed } = usePlayback(length);
  const frame = Math.min(Math.round(time * trajectory.rate), frameCount(trajectory) - 1);

  useEffect(() => {
    const key = (event: KeyboardEvent) => {
      if (event.target instanceof HTMLInputElement || event.metaKey || event.ctrlKey) return;
      const step = event.shiftKey ? 1 : 0.1;
      const at = SPEEDS.indexOf(speed);
      if (event.key === ' ') toggle();
      else if (event.key === 'ArrowRight') seek(time + step);
      else if (event.key === 'ArrowLeft') seek(time - step);
      else if (event.key === ']') setSpeed(SPEEDS[Math.min(at + 1, SPEEDS.length - 1)]);
      else if (event.key === '[') setSpeed(SPEEDS[Math.max(at - 1, 0)]);
      else return;
      event.preventDefault();
    };
    window.addEventListener('keydown', key);
    return () => window.removeEventListener('keydown', key);
  }, [time, speed, toggle, seek, setSpeed]);

  return (
    <div
      className="grid h-full min-h-0 grid-rows-[1fr_auto] bg-[var(--replay-paper)] text-[var(--replay-ink)]"
      style={paletteStyle}
    >
      <div className="grid min-h-0 grid-rows-[minmax(320px,1fr)_auto] lg:grid-cols-[1fr_minmax(320px,36%)] lg:grid-rows-1">
        <div className="min-h-0">
          <Scene pose={poseAt(trajectory, time)} position={trajectory.position} frame={frame} />
        </div>
        <div className="h-[420px] min-h-0 border-t border-[var(--replay-hairline)] lg:h-auto lg:border-t-0 lg:border-l">
          <Strips channels={trajectory.channels} rate={trajectory.rate} time={time} onSeek={seek} />
        </div>
      </div>
      <div className="border-t border-[var(--replay-hairline)]">
        <Timeline
          time={time}
          length={length}
          events={trajectory.events}
          playing={playing}
          speed={speed}
          onToggle={toggle}
          onSeek={seek}
          onSpeed={setSpeed}
        />
      </div>
    </div>
  );
}
