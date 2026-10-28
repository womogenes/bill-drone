'use client';

import { memo } from 'react';
import { relativeTime, signed } from '@/lib/format';
import { FastChart, type ChartSeries } from '@/components/fast-chart';
import type { ImuSample } from '@/lib/telemetry';

// A fixed short window, independent of the IMU range, so the attitude stays readable in flight.
const SPAN_MS = 15_000;
const DASH = [4, 3];
const series: ChartSeries<ImuSample>[] = [
  {
    label: 'Roll',
    color: 'var(--color-red-700)',
    value: (point) => point.roll,
  },
  {
    label: 'Pitch',
    color: 'var(--color-green-700)',
    value: (point) => point.pitch,
  },
  {
    label: 'Roll target',
    color: 'var(--color-red-700)',
    width: 1,
    dash: DASH,
    value: (point) => point.target?.[0] ?? null,
  },
  {
    label: 'Pitch target',
    color: 'var(--color-green-700)',
    width: 1,
    dash: DASH,
    value: (point) => point.target?.[1] ?? null,
  },
];
const time = (point: ImuSample) => point.time;
const gap = (previous: ImuSample, next: ImuSample) =>
  previous.session !== next.session || next.time - previous.time > 1000;
const formatValue = (value: number) => signed(value, 1);

export const AttitudeChart = memo(function AttitudeChart({
  history,
  end,
}: {
  history: (start: number, end: number) => ImuSample[];
  end: number;
}) {
  return (
    <FastChart
      history={history}
      series={series}
      time={time}
      start={end - SPAN_MS}
      end={end}
      gap={gap}
      formatTime={relativeTime}
      formatValue={formatValue}
      label="Roll and pitch over the last 15 seconds in degrees, with the targets dashed while armed"
      className="h-32"
    />
  );
});
