'use client';

import { memo } from 'react';
import { signed } from '@/lib/format';
import { FastChart, type ChartSeries } from '@/components/fast-chart';
import type { ImuSample } from '@/lib/telemetry';

const axes = [
  ['X', 'var(--color-red-700)'],
  ['Y', 'var(--color-green-700)'],
  ['Z', 'var(--color-blue-700)'],
] as const;
const series: Record<'accel' | 'gyro', ChartSeries<ImuSample>[]> = {
  accel: axes.map(([label, color], index) => ({
    label,
    color,
    value: (point) => point.accel[index],
  })),
  gyro: axes.map(([label, color], index) => ({
    label,
    color,
    value: (point) => point.gyro[index],
  })),
};
const time = (point: ImuSample) => point.time;
const gap = (previous: ImuSample, next: ImuSample) =>
  previous.session !== next.session || next.time - previous.time > 1000;
const formats = [0, 1, 2, 3].map(
  (digits) => (value: number) => signed(value, digits)
);

function relativeTime(time: number, end: number) {
  const seconds = Math.max(0, Math.round((end - time) / 1000));
  return `T-${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

export const ImuAxisChart = memo(function ImuAxisChart({
  history,
  field,
  start,
  end,
  digits,
}: {
  history: (start: number, end: number) => ImuSample[];
  field: 'accel' | 'gyro';
  start: number;
  end: number;
  digits: number;
}) {
  return (
    <FastChart
      history={history}
      series={series[field]}
      time={time}
      start={start}
      end={end}
      gap={gap}
      formatTime={relativeTime}
      formatValue={formats[digits]}
      label={
        field === 'accel'
          ? 'Acceleration over time, X, Y and Z in g'
          : 'Angular velocity over time, X, Y and Z in degrees per second'
      }
    />
  );
});
