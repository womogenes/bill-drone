'use client';

import Link from 'next/link';
import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { typing } from '@/lib/keys';
import { parseTag, readParts, type PreviewTag } from '@/lib/multipart';

type Corners = number[][];
type Result = { rms: number; views: number };
// The camera's own state through the drone (app/api/camera GET mode=status): running, fpv, stalled or stopped.
type Status = { camera?: string };

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
  const [full, setFull] = useState(false);
  const [tag, setTag] = useState<PreviewTag | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const captureButton = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    function captureOnSpace(event: KeyboardEvent) {
      if (event.code !== 'Space' || typing(event)) return;
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
    async function refresh() {
      try {
        const response = await fetch(
          `/api/camera?mode=status&host=${encodeURIComponent(connectedHost)}`,
          { signal: controller.signal }
        );
        const value = await response.json();
        if (!response.ok) throw new Error(value.error);
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
          await readParts(
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
        <h1 className="font-semibold">Camera calibration</h1>
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
              className={`fill-none ${tag.valid ? 'stroke-green-500' : 'stroke-amber-500'}`}
              strokeWidth="1.5"
            />
            <circle
              cx={tag.corners.reduce((sum, p) => sum + p[0], 0) / 4}
              cy={tag.corners.reduce((sum, p) => sum + p[1], 0) / 4}
              r="2"
              className={tag.valid ? 'fill-green-500' : 'fill-amber-500'}
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
          {status.camera ? `Camera ${status.camera} · ` : ''}
          {tag?.valid
            ? 'Tag tracked'
            : tag
              ? 'Tag seen without a pose'
              : 'Tag not tracked'}
          {tag?.valid &&
            ` · ${tag.position.map((v) => v.toFixed(3)).join(', ')} m`}
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
