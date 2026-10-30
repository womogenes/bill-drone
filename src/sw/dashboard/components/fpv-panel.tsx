'use client';

import { useEffect, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';

const HOST_KEY = 'camera-host';
const MODE_KEY = 'camera-mode';
type Mode = 'tag' | 'fpv';

function saved(key: string) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}

function save(key: string, value: string) {
  try {
    localStorage.setItem(key, value);
  } catch {}
}

// Rendered in the browser only (the page loads it without server rendering), so the saved choices are there from the start. Always streams: the tracking preview (the flow's 80x60 reduction, tracking undisturbed, over the camera's network) or FPV, the camera's own 480x320 video over the radio to the FPV receiver or over its network, during which it does not track the tag, so camera hover has no sightings; it resumes tracking 3 s after FPV ends.
export function FpvPanel() {
  // Empty: the address the camera reports through the drone.
  const [host, setHost] = useState(() => saved(HOST_KEY) ?? '');
  const [mode, setMode] = useState<Mode>(() =>
    saved(MODE_KEY) === 'fpv' ? 'fpv' : 'tag'
  );
  const [attempt, setAttempt] = useState(0);
  const [status, setStatus] = useState<{
    state: string;
    fps?: number;
  } | null>(null);
  const where = host ? `&host=${encodeURIComponent(host)}` : '';

  useEffect(() => {
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
        // FPV frames the dashboard received; the camera's own state otherwise.
        const fps =
          previous && value.received >= previous.count
            ? ((value.received - previous.count) * 1000) /
              (time - previous.time)
            : undefined;
        previous =
          value.received === undefined
            ? undefined
            : { count: value.received, time };
        setStatus({ state: value.camera, fps });
      } catch {
        if (!controller.signal.aborted) setStatus(null);
      }
    }, 1000);
    return () => {
      controller.abort();
      clearInterval(timer);
    };
  }, [where]);

  return (
    <section
      aria-labelledby="camera-heading"
      className="w-full max-w-md space-y-2 px-4 py-3"
    >
      <div className="flex items-center gap-2">
        <h2 id="camera-heading" className="font-semibold">
          Camera
        </h2>
        <span
          className={`text-sm tabular-nums ${status ? 'text-muted-foreground' : 'text-red-700'}`}
        >
          {status
            ? status.fps === undefined
              ? status.state
              : `${status.state} · ${Math.round(status.fps)} fps`
            : 'Offline'}
        </span>
        <Input
          aria-label="Camera address"
          className="ml-auto h-8 w-28"
          placeholder="auto"
          value={host}
          onChange={(event) => {
            setHost(event.target.value);
            save(HOST_KEY, event.target.value);
          }}
        />
        <div className="flex gap-1" role="group" aria-label="Camera mode">
          {(['tag', 'fpv'] as const).map((choice) => (
            <Button
              key={choice}
              size="sm"
              variant={mode === choice ? 'default' : 'outline'}
              aria-pressed={mode === choice}
              onClick={() => {
                setMode(choice);
                save(MODE_KEY, choice);
              }}
            >
              {choice === 'tag' ? 'Tag' : 'FPV'}
            </Button>
          ))}
        </div>
      </div>
      {/* eslint-disable-next-line @next/next/no-img-element */}
      <img
        key={`${mode}-${where}-${attempt}`}
        src={`/api/camera?${mode === 'fpv' ? 'fpv=1' : 'preview=1'}${where}&attempt=${attempt}`}
        alt={mode === 'fpv' ? 'Live camera video' : 'Tracking preview'}
        className="aspect-[3/2] w-full bg-black object-contain"
        onError={() => setTimeout(() => setAttempt((n) => n + 1), 1000)}
      />
      {mode === 'fpv' && (
        <p className="text-sm text-muted-foreground">
          Tag tracking off: camera hover has no sightings.
        </p>
      )}
    </section>
  );
}
