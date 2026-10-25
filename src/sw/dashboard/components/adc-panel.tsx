'use client';

import { memo, useEffect, useState } from 'react';
import { Button } from '@/components/ui/button';
import { relativeTime } from '@/lib/format';
import { FastChart, type ChartSeries } from '@/components/fast-chart';
import type { VoltageSample } from '@/lib/telemetry';

// The minute view shows every reading; the hour view one a second, to watch discharge over a session.
const SPANS = {
  minute: { label: '1 min', range: 60_000, maxGap: 1000 },
  hour: { label: '1 hour', range: 3_600_000, maxGap: 5000 },
} as const;
type Span = keyof typeof SPANS;
// Nominal 4S LiPo span, 3.0 to 4.2 V per cell.
const VOLTAGE_RANGE = [12, 16.8] as const;
const series: ChartSeries<VoltageSample>[] = [
  {
    label: 'Raw',
    color:
      'color-mix(in oklab, var(--color-muted-foreground) 45%, transparent)',
    width: 1,
    value: (point) => point.voltage,
  },
  {
    label: 'EMA',
    color: 'var(--color-foreground)',
    width: 2,
    value: (point) => point.ema,
  },
];
const time = (point: VoltageSample) => point.time;
const formatValue = (value: number) => value.toFixed(2);

export const ADCPanel = memo(function ADCPanel({
  raw,
  voltage,
  ema,
  history,
  hourHistory,
  time: latest,
  connected,
}: {
  raw: number | undefined;
  voltage: number | undefined;
  ema: number | undefined;
  history: (start: number, end: number) => VoltageSample[];
  hourHistory: (start: number, end: number) => VoltageSample[];
  time: number;
  connected: boolean;
}) {
  const [now, setNow] = useState(0);
  useEffect(() => {
    if (connected) return;
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, [connected]);
  const end = Math.max(now, latest);
  const [span, setSpan] = useState<Span>('minute');
  const { range, maxGap } = SPANS[span];
  const gap = (previous: VoltageSample, next: VoltageSample) =>
    previous.session !== next.session || next.time - previous.time > maxGap;

  return (
    <section
      aria-labelledby="adc-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <h2 id="adc-heading" className="font-semibold">
        ADC
      </h2>
      <div>
        <p className="text-sm text-muted-foreground">Voltage</p>
        <p className="mt-1 text-3xl font-medium tabular-nums">
          {connected && ema !== undefined ? `${ema.toFixed(2)} V` : '—'}
        </p>
      </div>
      <div className="grid grid-cols-2 gap-4 tabular-nums">
        <div>
          <p className="text-sm text-muted-foreground">Raw voltage</p>
          <p className="mt-1 text-xl font-medium">
            {connected && voltage !== undefined
              ? `${voltage.toFixed(2)} V`
              : '—'}
          </p>
        </div>
        <div>
          <p className="text-sm text-muted-foreground">Raw ADC</p>
          <p className="mt-1 text-xl font-medium">
            {connected && raw !== undefined ? raw.toLocaleString() : '—'}
          </p>
        </div>
      </div>
      <div className="flex gap-2" role="group" aria-label="Chart span">
        {(Object.keys(SPANS) as Span[]).map((key) => (
          <Button
            key={key}
            size="sm"
            variant={span === key ? 'default' : 'outline'}
            aria-pressed={span === key}
            onClick={() => setSpan(key)}
          >
            {SPANS[key].label}
          </Button>
        ))}
      </div>
      <FastChart
        history={span === 'minute' ? history : hourHistory}
        series={series}
        time={time}
        start={end - range}
        end={end}
        gap={gap}
        range={VOLTAGE_RANGE}
        formatTime={relativeTime}
        formatValue={formatValue}
        label={`Battery voltage over the last ${span}, raw and smoothed, in volts`}
      />
    </section>
  );
});
