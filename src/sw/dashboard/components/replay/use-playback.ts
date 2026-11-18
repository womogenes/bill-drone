'use client';

import { useCallback, useEffect, useRef, useState } from 'react';

export const SPEEDS = [0.25, 0.5, 1, 2];

// The replay's clock: the time shown (s, within 0 to length), whether it runs, and how fast; it stops at the end, and playing from the end starts over. A new trajectory gets a new clock by remounting the viewer (the page keys it by name).
export function usePlayback(length: number) {
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const clock = useRef({ time: 0, speed: 1 });
  useEffect(() => {
    clock.current.speed = speed;
  }, [speed]);

  const seek = useCallback(
    (value: number) => {
      const next = Math.min(Math.max(value, 0), length);
      clock.current.time = next;
      setTime(next);
    },
    [length]
  );

  useEffect(() => {
    if (!playing) return;
    let frame = 0;
    let before = performance.now();
    const tick = (now: number) => {
      const next = clock.current.time + ((now - before) / 1000) * clock.current.speed;
      before = now;
      if (next >= length) {
        seek(length);
        setPlaying(false);
        return;
      }
      seek(next);
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, length, seek]);

  const toggle = useCallback(() => {
    if (!playing && clock.current.time >= length) seek(0);
    setPlaying(!playing);
  }, [playing, length, seek]);

  return { time, seek, playing, toggle, speed, setSpeed };
}
