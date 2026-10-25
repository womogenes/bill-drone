'use client';

import Link from 'next/link';
import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';

type Corners = number[][];
type Result = { rms: number; views: number };
type Status = {
  calibrated: boolean;
  valid: boolean;
  capture_count: number;
  pose_count: number;
  preview_count: number;
  position_m: number[];
};
// The tag in one preview frame, from its X-Tag header: corners in 320x240 pixels.
type Tag = { valid: boolean; corners: number[][]; position: number[] } | null;

function parseTag(text: string): Tag {
  const [state, ...fields] = text.trim().split(' ');
  if (state === 'none' || fields.length < 5) return null;
  return {
    valid: state === 'valid',
    corners: fields.slice(0, 4).map((pair) => pair.split(',').map(Number)),
    position: fields[4].split(',').map(Number),
  };
}

function indexOf(bytes: Uint8Array, pattern: number[], from = 0) {
  outer: for (let i = from; i <= bytes.length - pattern.length; i++) {
    for (let j = 0; j < pattern.length; j++)
      if (bytes[i + j] !== pattern[j]) continue outer;
    return i;
  }
  return -1;
}

// Reads the camera's multipart stream: each part is a JPEG with its tag in an X-Tag header, so the outline drawn over a frame is that frame's.
async function readPreview(
  url: string,
  signal: AbortSignal,
  onFrame: (jpeg: Uint8Array, tag: string) => void
) {
  const response = await fetch(url, { signal });
  if (!response.ok || !response.body)
    throw new Error((await response.json()).error ?? 'Preview unavailable.');
  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = new Uint8Array(0);
  for (;;) {
    const { value, done } = await reader.read();
    if (done) throw new Error('Preview ended.');
    const joined = new Uint8Array(buffer.length + value.length);
    joined.set(buffer);
    joined.set(value, buffer.length);
    buffer = joined;
    for (;;) {
      const headerEnd = indexOf(buffer, [13, 10, 13, 10]);
      if (headerEnd < 0) break;
      const header = decoder.decode(buffer.subarray(0, headerEnd));
      const length = Number(/Content-Length: *(\d+)/i.exec(header)?.[1] ?? 0);
      const start = headerEnd + 4;
      if (buffer.length < start + length + 2) break;
      if (length)
        onFrame(
          buffer.slice(start, start + length),
          /X-Tag: *(.*)/i.exec(header)?.[1] ?? 'none'
        );
      buffer = buffer.slice(start + length + 2);
    }
  }
}

async function calculate(body: object) {
  const response = await fetch('/api/camera', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  return result;
}

export default function CameraPage() {
  const [host, setHost] = useState('192.168.86.28');
  const [connectedHost, setConnectedHost] = useState('');
  const [connection, setConnection] = useState(0);
  const [tagId, setTagId] = useState('0');
  const [sizeMm, setSizeMm] = useState('');
  const [views, setViews] = useState<Corners[]>([]);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [streamError, setStreamError] = useState('');
  const [stalled, setStalled] = useState(false);
  const [result, setResult] = useState<Result | null>(null);
  const [status, setStatus] = useState<Status | null>(null);
  const [rates, setRates] = useState({ preview: 0, pose: 0 });
  const [full, setFull] = useState(false);
  const [tag, setTag] = useState<Tag>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const captureButton = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    function captureOnSpace(event: KeyboardEvent) {
      if (
        event.code !== 'Space' ||
        (event.target instanceof HTMLElement &&
          event.target.closest('input, textarea, select, [contenteditable]'))
      )
        return;
      event.preventDefault();
      if (!event.repeat) captureButton.current?.click();
    }
    window.addEventListener('keydown', captureOnSpace);
    return () => window.removeEventListener('keydown', captureOnSpace);
  }, []);

  useEffect(() => {
    if (!connectedHost) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    let previous: { value: Status; time: number } | undefined;
    async function refresh() {
      try {
        const response = await fetch(
          `/api/camera?mode=status&host=${encodeURIComponent(connectedHost)}`,
          { signal: controller.signal }
        );
        const value = await response.json();
        if (!response.ok) throw new Error(value.error);
        const time = performance.now();
        if (previous) {
          const seconds = (time - previous.time) / 1000;
          setRates({
            preview: Math.max(
              0,
              (value.preview_count - previous.value.preview_count) / seconds
            ),
            pose: Math.max(
              0,
              (value.pose_count - previous.value.pose_count) / seconds
            ),
          });
        }
        previous = { value, time };
        setStatus(value);
      } catch (error) {
        if (controller.signal.aborted) return;
        setStatus(null);
        setStreamError(error instanceof Error ? error.message : String(error));
      }
      timer = setTimeout(refresh, 1000);
    }
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [connectedHost, connection]);

  // The preview reconnects, backing off, whenever its stream ends, fails or brings no frame for 6 s.
  useEffect(() => {
    if (!connectedHost) return;
    const closed = new AbortController();
    const url = `/api/camera?host=${encodeURIComponent(connectedHost)}${full ? '&full=1' : ''}`;
    let decoding = false;
    let lastFrame = performance.now();
    const stallCheck = setInterval(
      () => setStalled(performance.now() - lastFrame > 2000),
      500
    );
    function draw(jpeg: Uint8Array, header: string) {
      lastFrame = performance.now();
      // Decoding behind the stream drops frames rather than queueing them.
      if (decoding) return;
      decoding = true;
      createImageBitmap(new Blob([jpeg as BlobPart], { type: 'image/jpeg' }))
        .then((bitmap) => {
          const canvas = canvasRef.current;
          if (!canvas || closed.signal.aborted) return;
          canvas.width = bitmap.width;
          canvas.height = bitmap.height;
          canvas.getContext('2d')!.drawImage(bitmap, 0, 0);
          bitmap.close();
          setTag(parseTag(header));
          setStreamError('');
        })
        .catch(() => {})
        .finally(() => (decoding = false));
    }
    async function run() {
      let backoff = 250;
      while (!closed.signal.aborted) {
        const connection = new AbortController();
        const started = performance.now();
        const watchdog = setInterval(() => {
          if (performance.now() - Math.max(lastFrame, started) > 6000)
            connection.abort();
        }, 1000);
        try {
          await readPreview(
            url,
            AbortSignal.any([closed.signal, connection.signal]),
            (jpeg, header) => {
              backoff = 250;
              draw(jpeg, header);
            }
          );
        } catch (error) {
          if (!closed.signal.aborted && !connection.signal.aborted)
            setStreamError(
              error instanceof Error ? error.message : String(error)
            );
        }
        clearInterval(watchdog);
        await new Promise((done) => setTimeout(done, backoff));
        backoff = Math.min(backoff * 2, 4000);
      }
    }
    void run();
    return () => {
      closed.abort();
      clearInterval(stallCheck);
      setStalled(false);
      setTag(null);
    };
  }, [connectedHost, connection, full]);

  function reset() {
    setViews([]);
    setResult(null);
    setError('');
  }
  function settings() {
    return {
      tag_id: Number(tagId),
      tag_size_mm: Number(sizeMm),
      host: connectedHost,
    };
  }

  async function capture() {
    setBusy(true);
    setError('');
    setResult(null);
    try {
      const canvas = canvasRef.current!;
      if (canvas.width !== 320 || canvas.height !== 240)
        throw new Error('Turn on the full-resolution preview to capture.');
      const { corners } = await calculate({
        ...settings(),
        image: canvas.toDataURL('image/png').split(',')[1],
      });
      if (
        views.some(
          (view) =>
            view.reduce(
              (sum, point, i) =>
                sum +
                Math.hypot(point[0] - corners[i][0], point[1] - corners[i][1]),
              0
            ) /
              4 <
            5
        )
      )
        throw new Error('Move or tilt the tag before the next capture.');
      setViews((previous) => [...previous, corners]);
    } catch (error) {
      setError(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  async function save() {
    setBusy(true);
    setError('');
    try {
      setResult(await calculate({ ...settings(), views }));
    } catch (error) {
      setError(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  return (
    <main className="mx-auto max-w-3xl space-y-6 p-6">
      <header className="flex items-center justify-between border-b pb-4">
        <h1 className="text-lg font-semibold">Camera calibration</h1>
        <Link href="/" className="text-sm underline">
          Dashboard
        </Link>
      </header>
      <div className="flex gap-2">
        <Input
          aria-label="Camera address"
          value={host}
          onChange={(event) => setHost(event.target.value)}
        />
        <Button
          disabled={busy || !host.trim()}
          onClick={() => {
            reset();
            setStatus(null);
            setStreamError('');
            setConnectedHost(host.trim());
            setConnection((value) => value + 1);
          }}
        >
          Connect
        </Button>
      </div>
      <div className="relative flex aspect-[4/3] items-center justify-center overflow-hidden rounded bg-muted">
        {connectedHost ? (
          <canvas
            ref={canvasRef}
            aria-label="Live camera preview"
            className="h-full w-full object-contain"
          />
        ) : (
          <p className="text-sm text-muted-foreground">Camera disconnected</p>
        )}
        {connectedHost && tag && (
          <svg
            viewBox="0 0 320 240"
            className="pointer-events-none absolute inset-0 h-full w-full"
          >
            <polygon
              points={tag.corners.map((p) => p.join(',')).join(' ')}
              fill="none"
              stroke={tag.valid ? '#22c55e' : '#f59e0b'}
              strokeWidth="1.5"
            />
            <circle
              cx={tag.corners.reduce((sum, p) => sum + p[0], 0) / 4}
              cy={tag.corners.reduce((sum, p) => sum + p[1], 0) / 4}
              r="2"
              fill={tag.valid ? '#22c55e' : '#f59e0b'}
            />
          </svg>
        )}
      </div>
      {stalled && (
        <p role="alert" className="text-sm text-red-700">
          {streamError || 'Reconnecting to camera…'}
        </p>
      )}
      {status && (
        <p role="status" className="text-sm text-muted-foreground tabular-nums">
          Preview {rates.preview.toFixed(1)} fps · Pose {rates.pose.toFixed(1)}{' '}
          Hz ·{' '}
          {status.calibrated
            ? status.valid
              ? 'Tag tracked'
              : 'Tag not tracked'
            : 'Lens calibration needed'}
          {tag && ` · ${tag.position.map((v) => v.toFixed(3)).join(', ')} m`}
        </p>
      )}
      <label className="flex items-center gap-2 text-sm">
        <input
          type="checkbox"
          checked={full}
          onChange={(event) => setFull(event.target.checked)}
        />
        Full-resolution preview (needed to capture views; adds latency to some
        pose frames)
      </label>
      <div className="flex gap-6">
        <label className="flex flex-col gap-3 text-sm">
          Tag16h5 ID
          <Input
            type="number"
            min="0"
            max="29"
            value={tagId}
            disabled={busy}
            onChange={(event) => {
              setTagId(event.target.value);
              reset();
            }}
          />
        </label>
        <label className="flex flex-col gap-3 text-sm">
          Black-square width (mm)
          <Input
            type="number"
            min="0.1"
            step="0.1"
            value={sizeMm}
            disabled={busy}
            onChange={(event) => {
              setSizeMm(event.target.value);
              reset();
            }}
          />
        </label>
      </div>
      <p className="text-sm text-muted-foreground">
        Capture at least 25 views of the same flat AprilTag, with different
        tilts, distances, and positions across the image. Measure the black
        square, excluding the white margin.
      </p>
      <div className="flex flex-wrap items-center gap-3">
        <Button
          disabled={
            busy ||
            !connectedHost ||
            !status ||
            !full ||
            !(Number(sizeMm) > 0) ||
            views.length >= 60
          }
          ref={captureButton}
          onClick={capture}
        >
          Capture view <kbd className="font-sans text-xs">Space</kbd>
        </Button>
        <span className="text-sm tabular-nums" role="status">
          {views.length} {views.length === 1 ? 'view' : 'views'}
        </span>
        <Button
          variant="outline"
          disabled={busy || views.length < 25}
          onClick={save}
        >
          Calibrate and save
        </Button>
        <Button
          variant="ghost"
          disabled={busy || !views.length}
          onClick={reset}
        >
          Reset
        </Button>
      </div>
      {busy && (
        <p role="status" className="text-sm text-muted-foreground">
          Processing…
        </p>
      )}
      {error && (
        <p role="alert" className="text-sm text-red-700">
          {error}
        </p>
      )}
      {result && (
        <p role="status" className="text-sm text-green-700 tabular-nums">
          Calibration applied to camera and saved from {result.views} views.
          Error: {result.rms.toFixed(3)} px.
        </p>
      )}
    </main>
  );
}
