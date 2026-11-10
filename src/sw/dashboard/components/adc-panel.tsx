'use client';

import { memo, useState } from 'react';
import { Button } from '@/components/ui/button';
import { relativeTime } from '@/lib/format';
import {
  FastChart,
  sampleTime,
  sessionGap,
  useChartEnd,
  type ChartSeries,
} from '@/components/fast-chart';
import { Stat } from '@/components/stat';
import type { VoltageSample } from '@/lib/telemetry';

// The minute view shows every reading; the hour view one a second, to watch discharge over a session.
const SPANS = {
  minute: { label: '1 min', range: 60_000, gap: sessionGap(1000) },
  hour: { label: '1 hour', range: 3_600_000, gap: sessionGap(5000) },
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
  const end = useChartEnd(connected, latest);
  const [span, setSpan] = useState<Span>('minute');
  const { range, gap } = SPANS[span];

  return (
    <section aria-labelledby="adc-heading" className="space-y-2 px-4 py-3">
      <div className="flex items-center gap-2">
        <h2 id="adc-heading" className="font-semibold">
          ADC
        </h2>
        <div
          className="ml-auto flex gap-2"
          role="group"
          aria-label="Chart span"
        >
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
      </div>
      <dl className="grid grid-cols-3 gap-4 tabular-nums">
        <Stat label="Voltage" valueClassName="font-medium">
          {connected && ema !== undefined ? `${ema.toFixed(2)} V` : '—'}
        </Stat>
        <Stat label="Raw voltage">
          {connected && voltage !== undefined ? `${voltage.toFixed(2)} V` : '—'}
        </Stat>
        <Stat label="Raw ADC">
          {connected && raw !== undefined ? raw.toLocaleString() : '—'}
        </Stat>
      </dl>
      <FastChart
        history={span === 'minute' ? history : hourHistory}
        series={series}
        time={sampleTime}
        start={end - range}
        end={end}
        gap={gap}
        range={VOLTAGE_RANGE}
        formatTime={relativeTime}
        formatValue={formatValue}
        label={`Battery voltage over the last ${span}, raw and smoothed, in volts`}
        className="h-24"
      />
    </section>
  );
});
