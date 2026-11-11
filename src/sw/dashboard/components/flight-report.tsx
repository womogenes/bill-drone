'use client';

import { useMemo } from 'react';
import dynamic from 'next/dynamic';
import { TriangleAlert } from 'lucide-react';
import {
  DASH,
  FastChart,
  sampleTime,
  type ChartSeries,
} from '@/components/fast-chart';
import { Stat } from '@/components/stat';
import { Alert, AlertDescription } from '@/components/ui/alert';
import { Badge } from '@/components/ui/badge';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table';
import type {
  FlightReport,
  FlightTrackPoint,
  HandsOffSpan,
} from '@/lib/flight-report';
import { clock } from '@/lib/format';
import { distance } from '@/lib/gps';
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

// One hue per quantity, its target dashed in the same hue: blue is the measured value, orange the second axis (pitch), and the barometer recedes in gray.
const BLUE = 'var(--color-blue-700)';
const ORANGE = 'var(--color-orange-600)';
const heightSeries: ChartSeries<Point>[] = [
  {
    label: 'Barometer',
    color: 'var(--color-zinc-400)',
    width: 1,
    value: (point) => point.baro,
  },
  { label: 'Estimate', color: BLUE, value: (point) => point.height },
  {
    label: 'Target',
    color: BLUE,
    width: 1,
    dash: DASH,
    value: (point) => point.height_target,
  },
];
const attitudeSeries: ChartSeries<Point>[] = [
  {
    label: 'Roll target',
    color: BLUE,
    width: 1,
    dash: DASH,
    value: (point) => point.roll_target,
  },
  {
    label: 'Pitch target',
    color: ORANGE,
    width: 1,
    dash: DASH,
    value: (point) => point.pitch_target,
  },
  { label: 'Roll', color: BLUE, value: (point) => point.roll },
  { label: 'Pitch', color: ORANGE, value: (point) => point.pitch },
];
const shakeSeries: ChartSeries<Point>[] = [
  {
    label: 'Pitch rate, 2.2-3.5 Hz',
    color: BLUE,
    value: (point) => point.shake,
  },
];
const speedSeries: ChartSeries<FlightTrackPoint>[] = [
  { label: 'Ground speed', color: BLUE, value: (point) => point.speed },
];
// GPS fixes come at 10 Hz: a longer silence is a lost link or fix, not a line to draw across.
const fixGap = (previous: FlightTrackPoint, next: FlightTrackPoint) =>
  next.time - previous.time > 1;

const tenths = (value: number) => value.toFixed(1);
const whole = (value: number) => value.toFixed(0);

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

// The hands-off table: each column's heading and its text for a span.
const SPAN_COLUMNS: [string, (span: HandsOffSpan) => string][] = [
  [
    'Span',
    (span) =>
      `${clock(span.start)}–${clock(span.end)} (${(span.end - span.start).toFixed(0)} s)`,
  ],
  ['Pitch sd', (span) => fixed(span.pitch_sd, 1, '°')],
  ['Roll sd', (span) => fixed(span.roll_sd, 1, '°')],
  ['Height sd', (span) => fixed(span.height_sd, 2, 'm')],
  ['Baro range', (span) => fixed(span.baro_range, 2, 'm')],
  ['Drift', (span) => fixed(span.drift_m, 1, 'm')],
  ['Furthest', (span) => fixed(span.wander_m, 1, 'm')],
  ['Wind', (span) => windText(span.wind)],
];

function Swatch({ color, dash }: { color: string; dash?: number[] }) {
  return (
    <svg aria-hidden="true" width="14" height="6" className="shrink-0">
      <line
        x1="0"
        y1="3"
        x2="14"
        y2="3"
        stroke={color}
        strokeWidth={dash ? 1.5 : 2}
        strokeDasharray={dash?.join(' ')}
      />
    </svg>
  );
}

function Chart<T extends { time: number }>({
  title,
  unit,
  points,
  series,
  start,
  end,
  format,
  minSpan,
  gap,
}: {
  title: string;
  unit: string;
  points: T[];
  series: ChartSeries<T>[];
  start: number;
  end: number;
  format: (value: number) => string;
  minSpan?: number;
  gap?: (previous: T, next: T) => boolean;
}) {
  const history = useMemo(() => () => points, [points]);
  return (
    <section
      className="flex min-h-0 flex-col gap-2 px-4 py-3 lg:flex-1"
      aria-label={title}
    >
      <div className="flex flex-wrap items-baseline gap-x-4 gap-y-1">
        <h2 className="mr-auto font-semibold">
          {title}{' '}
          <span className="text-sm font-normal text-muted-foreground">
            {unit}
          </span>
        </h2>
        {series.map((line) => (
          <span
            key={line.label}
            className="flex items-center gap-1.5 text-sm text-muted-foreground"
          >
            <Swatch color={line.color} dash={line.dash} />
            {line.label}
          </span>
        ))}
      </div>
      <FastChart
        history={history}
        series={series}
        time={sampleTime}
        start={start}
        end={end}
        gap={gap}
        formatTime={clock}
        formatValue={format}
        minSpan={minSpan}
        label={title}
        className="h-40 lg:h-auto lg:min-h-16 lg:flex-1"
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
    ? Math.max(0, ...fixes.map((fix) => distance(home, fix)))
    : null;
  const total = Math.max(report.duration, report.series.time.at(-1) ?? 0, 1);
  // The charts show the flight itself: a second before liftoff to just after the motors stop, so the tumble after the cut doesn't set the scales.
  const [chartStart, chartEnd] = report.airborne
    ? [
        Math.max(0, report.airborne[0] - 1),
        Math.min(total, report.airborne[1] + 0.25),
      ]
    : [0, total];
  const stats: [string, string][] = [
    [
      'In the air',
      report.airborne ? clock(report.airborne[1] - report.airborne[0]) : '—',
    ],
    ['Highest', fixed(report.max_height, 2, 'm')],
    ['Top speed', fixed(report.top_speed, 1, 'm/s')],
    ['Furthest', fixed(furthest, 1, 'm')],
    ['Shake', fixed(report.shake_rms, 1, '°/s')],
    ['Wind', windText(report.wind)],
  ];
  const stopped = report.stop_reason ?? 'No stop record';

  return (
    <div className="flex flex-col divide-y divide-border lg:min-h-0 lg:flex-1">
      <section
        className="flex shrink-0 flex-wrap items-center gap-x-10 gap-y-3 px-4 py-3"
        aria-labelledby="summary-heading"
      >
        <h2 id="summary-heading" className="sr-only">
          Summary
        </h2>
        <dl className="flex flex-wrap gap-x-8 gap-y-2 tabular-nums">
          {stats.map(([label, value]) => (
            <Stat key={label} label={label}>
              {value}
            </Stat>
          ))}
        </dl>
        <div className="ml-auto flex flex-wrap items-center gap-2">
          <Badge
            variant={
              stopped === 'Landed' || stopped === 'Disarmed'
                ? 'outline'
                : 'destructive'
            }
          >
            {stopped}
          </Badge>
          <Badge variant="outline">
            GPS hold {report.gps_hold ? 'on' : 'off'}
          </Badge>
          {report.drone && (
            <span className="text-sm text-muted-foreground tabular-nums">
              {report.drone}
            </span>
          )}
        </div>
        {report.warnings.length > 0 && (
          <Alert variant="destructive" className="w-full py-2">
            <TriangleAlert />
            <AlertDescription>
              <ul className="max-h-14 space-y-0.5 overflow-y-auto tabular-nums">
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
            </AlertDescription>
          </Alert>
        )}
      </section>
      <div className="grid divide-y divide-border lg:min-h-0 lg:flex-1 lg:grid-cols-2 lg:divide-x lg:divide-y-0">
        <div className="flex min-w-0 flex-col divide-y divide-border lg:min-h-0">
          <Chart
            title="Height"
            unit="m"
            points={points}
            series={heightSeries}
            start={chartStart}
            end={chartEnd}
            format={tenths}
            minSpan={0.5}
          />
          <Chart
            title="Attitude"
            unit="°"
            points={points}
            series={attitudeSeries}
            start={chartStart}
            end={chartEnd}
            format={whole}
            minSpan={4}
          />
          <Chart
            title="Shake"
            unit="°/s RMS over 1 s"
            points={points}
            series={shakeSeries}
            start={chartStart}
            end={chartEnd}
            format={whole}
            minSpan={5}
          />
          <Chart
            title="Speed"
            unit="m/s"
            points={report.track}
            series={speedSeries}
            start={chartStart}
            end={chartEnd}
            format={tenths}
            minSpan={1}
            gap={fixGap}
          />
        </div>
        <div className="flex min-w-0 flex-col divide-y divide-border lg:min-h-0">
          {fixes.length > 0 && (
            <GpsMap
              key={report.name}
              gps={null}
              latest={fixes.at(-1)}
              home={home}
              history={mapHistory}
              hover={null}
              title="Track"
              fill
              heading={false}
              minSpan={6}
            />
          )}
          <section
            className="max-h-48 shrink-0 space-y-2 overflow-y-auto px-4 py-3"
            aria-labelledby="spans-heading"
          >
            <h2 id="spans-heading" className="font-semibold">
              Hands-off hover
            </h2>
            {report.spans.length ? (
              <Table className="tabular-nums">
                <TableHeader>
                  <TableRow>
                    {SPAN_COLUMNS.map(([title]) => (
                      <TableHead key={title} className="h-8">
                        {title}
                      </TableHead>
                    ))}
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {report.spans.map((span) => (
                    <TableRow key={span.start}>
                      {SPAN_COLUMNS.map(([title, text]) => (
                        <TableCell key={title} className="py-1">
                          {text(span)}
                        </TableCell>
                      ))}
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            ) : (
              <p className="text-sm text-muted-foreground">None</p>
            )}
          </section>
        </div>
      </div>
    </div>
  );
}
