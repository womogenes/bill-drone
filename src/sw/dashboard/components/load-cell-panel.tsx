'use client';

import { useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import type { ImuReading } from '@/components/imu-panel';
import { Activity, CircleHelp, Download, Unplug, Trash2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import {
  ChartContainer,
  ChartTooltip,
  ChartTooltipContent,
} from '@/components/ui/chart';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { CartesianGrid, Line, LineChart, XAxis, YAxis } from 'recharts';

export type Sample = { time: number; force: number; session: number };
export type Reading = {
  imu?: ImuReading;
  shunt_raw?: number;
  servo_angle?: number;
  time: number;
  force: number | null;
  force_age_ms?: number | null;
  force_sequence?: number;
};

const format = new Intl.NumberFormat(undefined, {
  maximumFractionDigits: 0,
}).format;
const clock = new Intl.DateTimeFormat(undefined, {
  hour: '2-digit',
  minute: '2-digit',
  second: '2-digit',
  hour12: false,
}).format;

// Keep endpoints and extrema in each bucket, including brief spikes.
function plotSamples(samples: Sample[], bucketMs: number) {
  const points: { time: number; force: number | null }[] = [];
  let bucket: Sample[] = [];
  function flush() {
    if (!bucket.length) return;
    const low = bucket.reduce((a, b) => (a.force < b.force ? a : b));
    const high = bucket.reduce((a, b) => (a.force > b.force ? a : b));
    points.push(
      ...[...new Set([bucket[0], low, high, bucket[bucket.length - 1]])].sort(
        (a, b) => a.time - b.time
      )
    );
    bucket = [];
  }
  for (const point of samples) {
    const previous = bucket[bucket.length - 1];
    const gap =
      previous &&
      (point.time - previous.time > 1000 || point.session !== previous.session);
    if (
      previous &&
      (gap ||
        Math.floor(point.time / bucketMs) !==
          Math.floor(previous.time / bucketMs))
    ) {
      flush();
      if (gap) points.push({ time: previous.time + 1, force: null });
    }
    bucket.push(point);
  }
  flush();
  return points;
}

export function LoadCellPanel({
  samples,
  reading,
  connected,
  onClear,
}: {
  samples: Sample[];
  reading: Reading | null;
  connected: boolean;
  onClear: () => void;
}) {
  const [now, setNow] = useState<number | null>(null);
  const searchParams = useSearchParams();
  const range = Number(searchParams.get('range'));
  const windowSeconds = [30, 60, 300].includes(range) ? range : 60;

  function setRange(value: string | null) {
    if (!value) return;
    const url = new URL(window.location.href);
    url.searchParams.set('range', value);
    window.history.replaceState(null, '', url);
  }

  useEffect(() => {
    if (connected) return;
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, [connected]);

  const end = Math.max(now ?? 0, reading?.time ?? 0);
  const start = end - windowSeconds * 1000;
  const visible = samples.filter((sample) => sample.time >= start);
  const chartData = plotSamples(visible, (windowSeconds * 1000) / 75);
  const age =
    reading?.force_age_ms == null
      ? null
      : reading.force_age_ms + Math.max(0, end - reading.time);
  const state = !connected
    ? {
        label: 'Offline',
        Icon: Unplug,
        color: 'text-red-700',
      }
    : !reading || reading.force === null || reading.force_age_ms === null
      ? {
          label: 'Waiting for sample',
          Icon: Activity,
          color: 'text-yellow-600/70 dark:text-yellow-400/70',
        }
      : age === null
        ? {
            label: 'Freshness unavailable',
            Icon: CircleHelp,
            color: 'text-yellow-600/70 dark:text-yellow-400/70',
          }
        : age > 1000
          ? {
              label: 'Stale · check sensor',
              Icon: Unplug,
              color: 'text-red-700',
            }
          : {
              label: 'Sampling',
              Icon: Activity,
              color: 'text-green-700',
            };

  function download() {
    const csv =
      'received_at,force_raw,connection_session\n' +
      samples
        .map(
          (point) =>
            `${new Date(point.time).toISOString()},${point.force},${point.session}`
        )
        .join('\n');
    const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    const link = document.createElement('a');
    link.href = url;
    link.download = `load-cell-${new Date().toISOString().replaceAll(':', '-')}.csv`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  return (
    <section
      aria-labelledby="load-cell-heading"
      className="w-full max-w-md space-y-6 px-6 py-8"
    >
      <div className="space-y-4">
        <div className="flex flex-wrap items-center gap-4">
          <h2 id="load-cell-heading" className="font-semibold">
            Load cell
          </h2>
          <span
            role="status"
            className={`flex items-center gap-2 text-sm ${state.color}`}
          >
            <state.Icon aria-hidden="true" className="size-4" />
            {state.label}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <Select
            value={String(windowSeconds)}
            onValueChange={setRange}
            items={[
              { value: '30', label: 'Last 30 seconds' },
              { value: '60', label: 'Last minute' },
              { value: '300', label: 'Last 5 minutes' },
            ]}
          >
            <SelectTrigger aria-label="History range" className="w-48">
              <SelectValue />
            </SelectTrigger>
            <SelectContent align="start" alignItemWithTrigger={false}>
              <SelectGroup>
                <SelectLabel>History</SelectLabel>
                <SelectItem value="30">Last 30 seconds</SelectItem>
                <SelectItem value="60">Last minute</SelectItem>
                <SelectItem value="300">Last 5 minutes</SelectItem>
              </SelectGroup>
            </SelectContent>
          </Select>
          <Button
            variant="ghost"
            size="icon"
            className="ml-auto"
            title="Download retained history as CSV"
            aria-label="Download history as CSV"
            disabled={!samples.length}
            onClick={download}
          >
            <Download />
          </Button>
          <Button
            variant="ghost"
            size="icon"
            title="Clear history"
            aria-label="Clear history"
            disabled={!samples.length}
            onClick={onClear}
          >
            <Trash2 />
          </Button>
        </div>
      </div>
      <dl className="tabular-nums">
        <div>
          <dt className="text-sm text-muted-foreground">
            {connected && age !== null && age <= 1000
              ? 'Latest reading'
              : 'Last reported reading'}
          </dt>
          <dd className="mt-1 text-3xl font-medium">
            {reading?.force != null ? format(reading.force) : '—'}
          </dd>
        </div>
      </dl>
      <div className="relative">
        <ChartContainer
          config={{
            force: { label: 'Reading', color: 'var(--muted-foreground)' },
          }}
          className="h-64 w-full"
        >
          <LineChart
            accessibilityLayer
            data={chartData}
            margin={{ top: 12, right: 12, bottom: 0, left: 8 }}
          >
            <CartesianGrid vertical={false} />
            <XAxis
              dataKey="time"
              type="number"
              domain={[start, end]}
              tickFormatter={(time: number) => {
                const seconds = Math.max(0, Math.round((end - time) / 1000));
                const minutes = Math.floor(seconds / 60);
                return `T-${String(minutes).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
              }}
              axisLine={false}
              tickLine={false}
              minTickGap={48}
              tickMargin={12}
            />
            <YAxis
              domain={['auto', 'auto']}
              tickFormatter={format}
              axisLine={false}
              tickLine={false}
              width={80}
              tickMargin={8}
            />
            <ChartTooltip
              content={
                <ChartTooltipContent
                  labelFormatter={(_, payload) =>
                    clock(payload[0]?.payload.time)
                  }
                />
              }
            />
            <Line
              dataKey="force"
              type="linear"
              stroke="var(--color-force)"
              strokeWidth={1.5}
              dot={false}
              isAnimationActive={false}
              connectNulls={false}
            />
          </LineChart>
        </ChartContainer>
        {!visible.length && (
          <div className="pointer-events-none absolute inset-0 flex items-center justify-center text-sm text-muted-foreground">
            {connected ? 'Waiting for measurements' : 'No measurements'}
          </div>
        )}
      </div>
    </section>
  );
}
