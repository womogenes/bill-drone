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
  const [result, setResult] = useState<Result | null>(null);
  const [status, setStatus] = useState<Status | null>(null);
  const [rates, setRates] = useState({ preview: 0, pose: 0 });
  const frameRef = useRef<HTMLImageElement>(null);
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
        setStreamError(String(error));
      }
      timer = setTimeout(refresh, 1000);
    }
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [connectedHost, connection]);

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
      const canvas = document.createElement('canvas');
      canvas.width = 320;
      canvas.height = 240;
      const image = frameRef.current!;
      if (image.naturalWidth !== 320 || image.naturalHeight !== 240)
        throw new Error('Waiting for a 320×240 frame.');
      canvas.getContext('2d')!.drawImage(image, 0, 0);
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
      <div className="flex aspect-[4/3] items-center justify-center overflow-hidden rounded bg-muted">
        {connectedHost ? (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            ref={frameRef}
            src={`/api/camera?host=${encodeURIComponent(connectedHost)}&connection=${connection}`}
            alt="Live camera preview"
            onLoad={() => setStreamError('')}
            onError={() =>
              setStreamError('Preview unavailable. Check the camera address.')
            }
            className="h-full w-full object-contain"
          />
        ) : (
          <p className="text-sm text-muted-foreground">Camera disconnected</p>
        )}
      </div>
      {streamError && (
        <p role="alert" className="text-sm text-red-700">
          {streamError}
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
          {status.valid &&
            ` · ${status.position_m.map((v) => v.toFixed(3)).join(', ')} m`}
        </p>
      )}
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
