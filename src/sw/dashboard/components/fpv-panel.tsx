'use client';

import { useEffect, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';

const HOST_KEY = 'camera-host';

// Rendered in the browser only (the page loads it without server rendering), so the saved address is there from the start. The camera's own JPEG video over its Wi-Fi, through /api/camera. While it plays the camera stops tracking the tag, so camera hover has no sightings; it resumes tracking 3 s after the video closes.
export function FpvPanel() {
  // Empty: the address the camera reports through the drone.
  const [host, setHost] = useState(() => {
    try {
      return localStorage.getItem(HOST_KEY) ?? '';
    } catch {
      return '';
    }
  });
  const where = host ? `&host=${encodeURIComponent(host)}` : '';
  const [on, setOn] = useState(false);
  const [attempt, setAttempt] = useState(0);
  const [status, setStatus] = useState<{ mode: string; fps: number } | null>(
    null
  );

  useEffect(() => {
    if (!on) return;
    const controller = new AbortController();
    let previous: { count: number; time: number } | undefined;
    const timer = setInterval(async () => {
      try {
        const response = await fetch(`/api/camera?mode=status${where}`, {
          signal: controller.signal,
        });
        const value = await response.json();
        if (!response.ok) throw new Error(value.error);
        const time = performance.now();
        const fps = previous
          ? ((value.preview_count - previous.count) * 1000) /
            (time - previous.time)
          : 0;
        previous = { count: value.preview_count, time };
        setStatus({ mode: value.camera, fps: Math.max(0, fps) });
      } catch {
        if (!controller.signal.aborted) setStatus(null);
      }
    }, 1000);
    return () => {
      controller.abort();
      clearInterval(timer);
      setStatus(null);
    };
  }, [on, where]);

  return (
    <section
      aria-labelledby="fpv-heading"
      className="w-full max-w-md space-y-2 px-4 py-3"
    >
      <div className="flex items-center gap-2">
        <h2 id="fpv-heading" className="font-semibold">
          FPV
        </h2>
        <span className="text-sm text-muted-foreground tabular-nums">
          {on &&
            (status
              ? `${status.mode} · ${Math.round(status.fps)} fps`
              : 'connecting')}
        </span>
        <Input
          aria-label="Camera address"
          className="ml-auto h-8 w-36"
          placeholder="auto"
          value={host}
          disabled={on}
          onChange={(event) => {
            setHost(event.target.value);
            try {
              localStorage.setItem(HOST_KEY, event.target.value);
            } catch {}
          }}
        />
        <Button
          size="sm"
          variant={on ? 'default' : 'outline'}
          aria-pressed={on}
          onClick={() => setOn(!on)}
        >
          {on ? 'On' : 'Off'}
        </Button>
      </div>
      {on && (
        // eslint-disable-next-line @next/next/no-img-element
        <img
          key={attempt}
          src={`/api/camera?fpv=1${where}&attempt=${attempt}`}
          alt="Live camera video"
          className="aspect-[3/2] w-full bg-black object-contain"
          onError={() => setTimeout(() => setAttempt((n) => n + 1), 1000)}
        />
      )}
      {on && (
        <p className="text-sm text-muted-foreground">
          Tag tracking is off while the video plays.
        </p>
      )}
    </section>
  );
}
