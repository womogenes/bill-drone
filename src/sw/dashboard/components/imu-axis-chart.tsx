'use client';

import { memo } from 'react';
import { relativeTime, signed } from '@/lib/format';
import {
  FastChart,
  sampleTime,
  sessionGap,
  type ChartSeries,
} from '@/components/fast-chart';
import type { ImuSample } from '@/lib/telemetry';

const axes = [
  ['X', 'var(--color-red-700)'],
  ['Y', 'var(--color-green-700)'],
  ['Z', 'var(--color-blue-700)'],
] as const;
const lines = (field: 'accel' | 'gyro'): ChartSeries<ImuSample>[] =>
  axes.map(([label, color], index) => ({
    label,
    color,
    value: (point) => point[field][index],
  }));
const series = { accel: lines('accel'), gyro: lines('gyro') };
const gap = sessionGap();
const formats = [0, 1, 2, 3].map(
  (digits) => (value: number) => signed(value, digits)
);

export const ImuAxisChart = memo(function ImuAxisChart({
  history,
  field,
  start,
  end,
  digits,
  className,
}: {
  history: (start: number, end: number) => ImuSample[];
  field: 'accel' | 'gyro';
  start: number;
  end: number;
  digits: number;
  className?: string;
}) {
  return (
    <FastChart
      history={history}
      series={series[field]}
      time={sampleTime}
      start={start}
      end={end}
      gap={gap}
      formatTime={relativeTime}
      formatValue={formats[digits]}
      // A still drone's noise stays flat: 0.2 g and 4 degrees/s at least.
      minSpan={field === 'accel' ? 0.2 : 4}
      className={className}
      label={
        field === 'accel'
          ? 'Acceleration over time, X, Y and Z in g'
          : 'Angular velocity over time, X, Y and Z in degrees per second'
      }
    />
  );
});
