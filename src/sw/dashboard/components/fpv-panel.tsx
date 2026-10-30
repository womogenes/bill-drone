'use client';

import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { readParts } from '@/lib/multipart';

const HOST_KEY = 'camera-host';
const MODE_KEY = 'camera-mode';
// FPV frames go on screen this long after the earliest any frame could have, by their capture time, so the link's jitter up to it does not show.
const PLAYOUT_MS = 80;
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
  const canvas = useRef<HTMLCanvasElement>(null);
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

  // FPV frames carry their capture time (X-Time, microseconds on the camera's clock): each is shown at that time, mapped to this clock by the smallest delay any frame has had (rising slowly, for clock drift), plus PLAYOUT_MS. Preview frames have none and are shown at once. The stream reconnects whenever it ends or fails.
  useEffect(() => {
    const closed = new AbortController();
    const queue: { at: number; bitmap: ImageBitmap }[] = [];
    let last: number | undefined;
    let capture = 0;
    let base = Infinity;
    let frame = 0;
    const render = () => {
      let shown: ImageBitmap | undefined;
      while (queue.length && queue[0].at <= performance.now()) {
        shown?.close();
        shown = queue.shift()!.bitmap;
      }
      const target = canvas.current;
      if (shown && target) {
        target.width = shown.width;
        target.height = shown.height;
        target.getContext('2d')!.drawImage(shown, 0, 0);
      }
      shown?.close();
      frame = requestAnimationFrame(render);
    };
    frame = requestAnimationFrame(render);
    const show = async (jpeg: Uint8Array, header: string) => {
      const arrived = performance.now();
      const stamp = /X-Time: *(\d+)/i.exec(header);
      let at = arrived;
      if (stamp) {
        const us = Number(stamp[1]);
        const step = last === undefined ? 0 : ((us - last) >>> 0) / 1000;
        last = us;
        // A restarted camera starts its clock again.
        if (step > 10_000) base = Infinity;
        capture += step > 10_000 ? 0 : step;
        base = Math.min(arrived - capture, base + 0.05);
        at = capture + base + PLAYOUT_MS;
      }
      const bitmap = await createImageBitmap(
        new Blob([jpeg as BlobPart], { type: 'image/jpeg' })
      ).catch(() => undefined);
      if (!bitmap || closed.signal.aborted) return bitmap?.close();
      const index = queue.findIndex((queued) => queued.at > at);
      queue.splice(index < 0 ? queue.length : index, 0, { at, bitmap });
      while (queue.length > 8) queue.shift()!.bitmap.close();
    };
    void (async () => {
      while (!closed.signal.aborted) {
        await readParts(
          `/api/camera?${mode === 'fpv' ? 'fpv=1' : 'preview=1'}${where}`,
          closed.signal,
          (jpeg, header) => void show(jpeg, header)
        ).catch(() => {});
        await new Promise((done) => setTimeout(done, 1000));
      }
    })();
    return () => {
      closed.abort();
      cancelAnimationFrame(frame);
      for (const queued of queue) queued.bitmap.close();
    };
  }, [mode, where]);

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
      <canvas
        ref={canvas}
        aria-label={mode === 'fpv' ? 'Live camera video' : 'Tracking preview'}
        className="aspect-[3/2] w-full bg-black object-contain"
      />
      {mode === 'fpv' && (
        <p className="text-sm text-muted-foreground">
          Tag tracking off: camera hover has no sightings.
        </p>
      )}
    </section>
  );
}
