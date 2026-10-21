'use client';

import { memo, useEffect, useState } from 'react';
import { relativeTime } from '@/lib/format';
import { FastChart, type ChartSeries } from '@/components/fast-chart';
import type { VoltageSample } from '@/lib/telemetry';

const RANGE = 60_000;
const series: ChartSeries<VoltageSample>[] = [
  {
    label: 'Raw',
    color: 'var(--color-muted-foreground)',
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
const gap = (previous: VoltageSample, next: VoltageSample) =>
  previous.session !== next.session || next.time - previous.time > 1000;
const formatValue = (value: number) => value.toFixed(2);

export const ADCPanel = memo(function ADCPanel({
  raw,
  voltage,
  ema,
  history,
  time: latest,
  connected,
}: {
  raw: number | undefined;
  voltage: number | undefined;
  ema: number | undefined;
  history: (start: number, end: number) => VoltageSample[];
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
      <FastChart
        history={history}
        series={series}
        time={time}
        start={end - RANGE}
        end={end}
        gap={gap}
        formatTime={relativeTime}
        formatValue={formatValue}
        label="Battery voltage over the last minute, raw and smoothed, in volts"
      />
    </section>
  );
});
