'use client';

import { useMemo } from 'react';
import dynamic from 'next/dynamic';
import { FastChart, type ChartSeries } from '@/components/fast-chart';
import type { FlightReport, FlightTrackPoint } from '@/lib/flight-report';
import type { GpsSample } from '@/lib/telemetry';

// Leaflet needs the browser's window as it loads.
const GpsMap = dynamic(
  () => import('@/components/gps-map').then((module) => module.GpsMap),
  { ssr: false }
);

type Point = {
  time: number;
  height: number | null;
  height_target: number | null;
  baro: number | null;
  roll: number;
  pitch: number;
  roll_target: number;
  pitch_target: number;
  shake: number | null;
};

const DASH = [4, 3];
const heightSeries: ChartSeries<Point>[] = [
  {
    label: 'Estimate',
    color: 'var(--color-blue-700)',
    value: (point) => point.height,
  },
  {
    label: 'Barometer',
    color: 'var(--muted-foreground)',
    width: 1,
    value: (point) => point.baro,
  },
  {
    label: 'Target',
    color: 'var(--foreground)',
    width: 1,
    dash: DASH,
    value: (point) => point.height_target,
  },
];
const attitudeSeries: ChartSeries<Point>[] = [
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
    value: (point) => point.roll_target,
  },
  {
    label: 'Pitch target',
    color: 'var(--color-green-700)',
    width: 1,
    dash: DASH,
    value: (point) => point.pitch_target,
  },
];
const shakeSeries: ChartSeries<Point>[] = [
  {
    label: 'Shake',
    color: 'var(--color-blue-700)',
    value: (point) => point.shake,
  },
];
const speedSeries: ChartSeries<FlightTrackPoint>[] = [
  {
    label: 'Ground speed',
    color: 'var(--color-blue-700)',
    value: (point) => point.speed,
  },
];
const time = (point: { time: number }) => point.time;
// GPS fixes come at 10 Hz: a longer silence is a lost link or fix, not a line to draw across.
const fixGap = (previous: FlightTrackPoint, next: FlightTrackPoint) =>
  next.time - previous.time > 1;

export function clock(seconds: number) {
  const whole = Math.max(0, Math.round(seconds));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, '0')}`;
}

const formatTime = (value: number) => clock(value);
const metres = (value: number) => value.toFixed(1);
const degrees = (value: number) => value.toFixed(0);
const rate = (value: number) => value.toFixed(0);
const speed = (value: number) => value.toFixed(1);

function fixed(value: number | null | undefined, digits: number, unit: string) {
  return value === null || value === undefined
    ? '—'
    : `${value.toFixed(digits)} ${unit}`;
}

// The rough fore-aft wind (lib/flight-report.ts windEstimate): positive blows from behind.
function windText(value: number | null) {
  return value === null
    ? '—'
    : `${Math.abs(value).toFixed(1)} m/s ${value < 0 ? 'head' : 'tail'}`;
}

// The start in this laptop's local time, which is how flights are named.
export function startTime(iso: string) {
  return new Date(iso).toLocaleString(undefined, {
    month: 'short',
    day: 'numeric',
    hour: 'numeric',
    minute: '2-digit',
    second: '2-digit',
  });
}

function Chart<T extends { time: number }>({
  title,
  points,
  series,
  end,
  format,
  minSpan,
  gap,
}: {
  title: string;
  points: T[];
  series: ChartSeries<T>[];
  end: number;
  format: (value: number) => string;
  minSpan?: number;
  gap?: (previous: T, next: T) => boolean;
}) {
  const history = useMemo(() => () => points, [points]);
  return (
    <section className="space-y-2 px-4 py-3" aria-label={title}>
      <div className="flex flex-wrap items-center gap-x-4 gap-y-1">
        <h2 className="mr-auto font-semibold">{title}</h2>
        {series.map((line) => (
          <span
            key={line.label}
            className="text-sm"
            style={{ color: line.color }}
          >
            {line.label}
          </span>
        ))}
      </div>
      <FastChart
        history={history}
        series={series}
        time={time}
        start={0}
        end={end}
        gap={gap}
        formatTime={formatTime}
        formatValue={format}
        minSpan={minSpan}
        label={title}
        className="h-48"
      />
    </section>
  );
}

export function FlightReportView({ report }: { report: FlightReport }) {
  const points = useMemo(() => {
    const { series } = report;
    return series.time.map((time, index): Point => ({
      time,
      height: series.height[index],
      height_target: series.height_target[index],
      baro: series.baro[index],
      roll: series.roll[index],
      pitch: series.pitch[index],
      roll_target: series.roll_target[index],
      pitch_target: series.pitch_target[index],
      shake: series.shake[index],
    }));
  }, [report]);
  // The map's samples: its gaps and trail are in milliseconds.
  const fixes = useMemo(
    () =>
      report.track.map((point): GpsSample => ({
        ...point,
        time: point.time * 1000,
        session: 0,
      })),
    [report]
  );
  const mapHistory = useMemo(() => () => fixes, [fixes]);
  const [lift] = report.airborne ?? [0];
  const home = fixes.find((fix) => fix.time >= lift * 1000) ?? fixes[0];
  const furthest = home
    ? Math.max(
        0,
        ...fixes.map((fix) => {
          const radians = Math.PI / 180;
          const east =
            (fix.longitude - home.longitude) *
            radians *
            6_371_000 *
            Math.cos(home.latitude * radians);
          const north = (fix.latitude - home.latitude) * radians * 6_371_000;
          return Math.hypot(east, north);
        })
      )
    : null;
  const end = Math.max(report.duration, report.series.time.at(-1) ?? 0, 1);
  const summary: [string, string][] = [
    ['Started', startTime(report.started)],
    ['Duration', clock(report.duration)],
    [
      'Airborne',
      report.airborne
        ? `${clock(report.airborne[1] - report.airborne[0])} from ${clock(report.airborne[0])}`
        : 'Never lifted off',
    ],
    ['Stop', report.stop_reason ?? 'No stop record'],
    ['Highest', fixed(report.max_height, 2, 'm')],
    ['Top speed', fixed(report.top_speed, 1, 'm/s')],
    ['Shake (2.2-3.5 Hz)', fixed(report.shake_rms, 1, '°/s RMS')],
    ['Wind (rough)', windText(report.wind)],
    ['GPS hold', report.gps_hold ? 'On' : 'Off'],
    ['Drone', report.drone ?? '—'],
  ];

  return (
    <div className="divide-y divide-border">
      <section
        className="space-y-3 px-4 py-3"
        aria-labelledby="summary-heading"
      >
        <h2 id="summary-heading" className="font-semibold">
          Summary
        </h2>
        <dl className="grid grid-cols-2 gap-4 tabular-nums sm:grid-cols-5">
          {summary.map(([label, value]) => (
            <div key={label}>
              <dt className="text-sm text-muted-foreground">{label}</dt>
              <dd className="text-lg">{value}</dd>
            </div>
          ))}
        </dl>
        {report.warnings.length > 0 && (
          <ul className="space-y-1 text-red-700 tabular-nums">
            {report.warnings.map((warning, index) => (
              <li key={index}>
                {warning.time !== null && (
                  <span className="mr-2 text-muted-foreground">
                    {clock(warning.time)}
                  </span>
                )}
                {warning.text}
              </li>
            ))}
          </ul>
        )}
      </section>
      <div className="grid divide-y divide-border lg:grid-cols-2 lg:divide-x lg:divide-y-0">
        <div className="min-w-0 divide-y divide-border">
          <Chart
            title="Height (m)"
            points={points}
            series={heightSeries}
            end={end}
            format={metres}
            minSpan={0.5}
          />
          <Chart
            title="Attitude (°)"
            points={points}
            series={attitudeSeries}
            end={end}
            format={degrees}
            minSpan={4}
          />
          <Chart
            title="Pitch shake (°/s RMS, 2.2-3.5 Hz, 1 s)"
            points={points}
            series={shakeSeries}
            end={end}
            format={rate}
            minSpan={5}
          />
          <Chart
            title="GPS speed (m/s)"
            points={report.track}
            series={speedSeries}
            end={end}
            format={speed}
            minSpan={1}
            gap={fixGap}
          />
        </div>
        <div className="min-w-0 divide-y divide-border">
          {fixes.length > 0 && (
            <GpsMap
              key={report.name}
              gps={null}
              latest={fixes.at(-1)}
              home={home}
              history={mapHistory}
              hover={null}
              title="Track"
              summary={[
                ['Furthest from takeoff', fixed(furthest, 1, 'm')],
                ['Top speed', fixed(report.top_speed, 1, 'm/s')],
              ]}
            />
          )}
          <section
            className="space-y-2 px-4 py-3"
            aria-labelledby="spans-heading"
          >
            <h2 id="spans-heading" className="font-semibold">
              Hands-off hover
            </h2>
            {report.spans.length ? (
              <div className="overflow-x-auto">
                <table className="w-full text-sm tabular-nums">
                  <thead className="text-left text-muted-foreground">
                    <tr className="border-b">
                      <th className="py-1 pr-3 font-normal">Span</th>
                      <th className="py-1 pr-3 font-normal">Pitch sd</th>
                      <th className="py-1 pr-3 font-normal">Roll sd</th>
                      <th className="py-1 pr-3 font-normal">Height sd</th>
                      <th className="py-1 pr-3 font-normal">Baro range</th>
                      <th className="py-1 pr-3 font-normal">Drift</th>
                      <th className="py-1 pr-3 font-normal">Furthest</th>
                      <th className="py-1 font-normal">Wind</th>
                    </tr>
                  </thead>
                  <tbody>
                    {report.spans.map((span) => (
                      <tr key={span.start} className="border-b last:border-0">
                        <td className="py-1 pr-3">
                          {clock(span.start)}–{clock(span.end)} (
                          {(span.end - span.start).toFixed(0)} s)
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.pitch_sd, 1, '°')}
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.roll_sd, 1, '°')}
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.height_sd, 2, 'm')}
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.baro_range, 2, 'm')}
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.drift_m, 1, 'm')}
                        </td>
                        <td className="py-1 pr-3">
                          {fixed(span.wander_m, 1, 'm')}
                        </td>
                        <td className="py-1">{windText(span.wind)}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            ) : (
              <p className="text-muted-foreground">None</p>
            )}
          </section>
        </div>
      </div>
    </div>
  );
}
