'use client';

import { useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import type { RollSample, Reading } from '@/lib/telemetry';
import { Download, Trash2 } from 'lucide-react';
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

const format = new Intl.NumberFormat(undefined, {
  maximumFractionDigits: 1,
}).format;
const clock = new Intl.DateTimeFormat(undefined, {
  hour: '2-digit',
  minute: '2-digit',
  second: '2-digit',
  hour12: false,
}).format;

// Keep endpoints and extrema in each bucket, including brief spikes.
function plotSamples(samples: RollSample[], bucketMs: number) {
  const points: { time: number; roll: number | null }[] = [];
  let bucket: RollSample[] = [];
  function flush() {
    if (!bucket.length) return;
    const low = bucket.reduce((a, b) => (a.roll < b.roll ? a : b));
    const high = bucket.reduce((a, b) => (a.roll > b.roll ? a : b));
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
      (point.time - previous.time > 1000 ||
        point.session !== previous.session ||
        Math.abs(point.roll - previous.roll) > 180);
    if (
      previous &&
      (gap ||
        Math.floor(point.time / bucketMs) !==
          Math.floor(previous.time / bucketMs))
    ) {
      flush();
      if (gap) points.push({ time: previous.time + 1, roll: null });
    }
    bucket.push(point);
  }
  flush();
  return points;
}

export function RollHistoryPanel({
  samples,
  reading,
  connected,
  onClear,
}: {
  samples: RollSample[];
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
  function download() {
    const csv =
      'received_at,roll_deg,connection_session\n' +
      samples
        .map(
          (point) =>
            `${new Date(point.time).toISOString()},${point.roll},${point.session}`
        )
        .join('\n');
    const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
    const link = document.createElement('a');
    link.href = url;
    link.download = `roll-history-${new Date().toISOString().replaceAll(':', '-')}.csv`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  return (
    <section
      aria-labelledby="roll-history-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <div className="space-y-4">
        <h2 id="roll-history-heading" className="font-semibold">
          Roll history
        </h2>
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
      <div className="relative">
        <ChartContainer
          config={{
            roll: { label: 'Roll (°)', color: 'var(--muted-foreground)' },
          }}
          className="h-44 w-full"
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
              tickFormatter={(value: number) => `${format(value)}°`}
              axisLine={false}
              tickLine={false}
              width={60}
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
              dataKey="roll"
              type="linear"
              stroke="var(--color-roll)"
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
