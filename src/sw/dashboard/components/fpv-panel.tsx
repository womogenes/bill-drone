'use client';

import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { typing } from '@/components/control-panel';
import { Input } from '@/components/ui/input';
import { readParts } from '@/lib/multipart';

const HOST_KEY = 'camera-host';
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

// Rendered in the browser only (the page loads it without server rendering), so the saved address is there from the start. Always streams, by the drone's hover_camera setting: the tracking preview (the flow's 80x60 reduction, tracking undisturbed, over the camera's network) or FPV, the camera's own 480x320 video over the radio to the FPV receiver or over its network, during which it does not track the tag, so camera hover has no sightings; it resumes tracking 3 s after FPV ends.
// A preview frame's tag (X-Tag): the corners in the camera's 320x240 pixels and the tag centre in the camera frame (m; right, down, forward).
export type PreviewTag = {
  valid: boolean;
  corners: [number, number][];
  position: [number, number, number];
};

function parseTag(header: string): PreviewTag | null {
  const match = /X-Tag: *(valid|tracked) ([^\r\n]+)/i.exec(header);
  if (!match) return null;
  const parts = match[2].trim().split(/\s+/);
  const corners = parts
    .slice(0, 4)
    .map((pair) => pair.split(',').map(Number) as [number, number]);
  const position = (parts[4] ?? '').split(',').map(Number);
  if (corners.some((c) => c.some(Number.isNaN)) || position.length !== 3)
    return null;
  return {
    valid: match[1].toLowerCase() === 'valid',
    corners,
    position: position as [number, number, number],
  };
}

export function FpvPanel({
  free,
  onFree,
  onTag,
  roll = 0,
}: {
  // Each shown tag-mode frame's tag, or null when it has none.
  onTag?: (tag: PreviewTag | null) => void;
  // The camera's roll on its mount (the drone's hover_cam_roll, degrees): the picture is turned back by it, so it shows upright.
  roll?: number;
  // The drone's hover_camera setting turned around (undefined without a drone): every dashboard shows what the drone flies.
  free?: boolean;
  onFree: (free: boolean) => void;
}) {
  // Empty: the address the camera reports through the drone.
  const [host, setHost] = useState(() => saved(HOST_KEY) ?? '');
  const mode: Mode = free ? 'fpv' : 'tag';
  const canvas = useRef<HTMLCanvasElement>(null);
  const onTagRef = useRef(onTag);
  useEffect(() => {
    onTagRef.current = onTag;
  }, [onTag]);
  const rollRef = useRef(roll);
  useEffect(() => {
    rollRef.current = roll;
  }, [roll]);
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
    const queue: {
      at: number;
      bitmap: ImageBitmap;
      tag: PreviewTag | null;
    }[] = [];
    let last: number | undefined;
    let capture = 0;
    let base = Infinity;
    let frame = 0;
    const render = () => {
      let shown: ImageBitmap | undefined;
      let tag: PreviewTag | null = null;
      while (queue.length && queue[0].at <= performance.now()) {
        shown?.close();
        ({ bitmap: shown, tag } = queue.shift()!);
      }
      const target = canvas.current;
      if (shown && target) {
        // The picture turned back by the mount's roll, drawn into a canvas sized to the turned picture, so a quarter-turned camera's portrait picture fits the panel (object-contain) instead of overflowing it.
        const turn = (-rollRef.current * Math.PI) / 180;
        const cos = Math.abs(Math.cos(turn)),
          sin = Math.abs(Math.sin(turn));
        const width = shown.width * cos + shown.height * sin;
        const height = shown.width * sin + shown.height * cos;
        // The element takes the turned picture's shape across the panel's width, so it fills it without bars (up to max-h).
        target.style.aspectRatio = `${Math.round(width)} / ${Math.round(height)}`;
        // Drawn at the element's own pixel size with smoothing, so the small preview (80x60) is scaled smoothly here rather than stretched blocky by the browser.
        const zoom = Math.max(
          1,
          (target.clientWidth * devicePixelRatio) / width
        );
        target.width = Math.round(width * zoom);
        target.height = Math.round(height * zoom);
        const context = target.getContext('2d')!;
        context.imageSmoothingEnabled = true;
        context.imageSmoothingQuality = 'high';
        // A blur of about one source pixel hides the small JPEG's block edges and ringing, which smoothing alone only enlarges.
        context.filter =
          zoom > 2 ? `blur(${(zoom * 0.6).toFixed(1)}px)` : 'none';
        context.translate(target.width / 2, target.height / 2);
        context.scale(zoom, zoom);
        context.rotate(turn);
        context.translate(-shown.width / 2, -shown.height / 2);
        context.drawImage(shown, 0, 0);
        // The tag's outline, its corners scaled from the camera's 320x240 to this picture.
        if (tag) {
          context.filter = 'none';
          const scale = shown.width / 320;
          context.strokeStyle = '#22c55e';
          context.lineWidth = Math.max(1, shown.width / 160);
          context.beginPath();
          tag.corners.forEach(([x, y], i) =>
            i
              ? context.lineTo(x * scale, y * scale)
              : context.moveTo(x * scale, y * scale)
          );
          context.closePath();
          context.stroke();
        }
        onTagRef.current?.(tag);
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
      queue.splice(index < 0 ? queue.length : index, 0, {
        at,
        bitmap,
        tag: parseTag(header),
      });
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

  // F switches between tag and free flight, like the buttons.
  useEffect(() => {
    if (free === undefined) return;
    function keydown(event: KeyboardEvent) {
      if (event.code !== 'KeyF' || event.repeat || typing(event)) return;
      event.preventDefault();
      onFree(!free);
    }
    window.addEventListener('keydown', keydown);
    return () => window.removeEventListener('keydown', keydown);
  }, [free, onFree]);

  return (
    <section aria-labelledby="camera-heading" className="space-y-2 px-4 py-3">
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
        <div className="flex gap-1" role="group" aria-label="Camera mode (F)">
          {(['tag', 'fpv'] as const).map((choice) => (
            <Button
              key={choice}
              size="sm"
              variant={
                free !== undefined && mode === choice ? 'default' : 'outline'
              }
              aria-pressed={free !== undefined && mode === choice}
              disabled={free === undefined}
              title="F"
              onClick={() => onFree(choice === 'fpv')}
            >
              {choice === 'tag' ? 'Tag' : 'Free'}
            </Button>
          ))}
        </div>
      </div>
      <canvas
        ref={canvas}
        aria-label={mode === 'fpv' ? 'Live camera video' : 'Tracking preview'}
        className="aspect-[3/2] max-h-[calc(100dvh-10rem)] w-full bg-black object-contain"
      />
    </section>
  );
}
