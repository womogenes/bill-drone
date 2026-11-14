'use client';

import { useEffect, useState } from 'react';
import { CircleAlert, CircleCheck } from 'lucide-react';
import {
  FastChart,
  useChartEnd,
  type ChartSeries,
} from '@/components/fast-chart';
import { Panel } from '@/components/panel';
import { Stat, Stats } from '@/components/stat';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table';
import { relativeTime, startTime } from '@/lib/format';
import type { StandSample, StandState } from '@/lib/stand';
import type { StandRun } from '@/app/api/stand/runs/route';

// The drone's stop pulse (us); the stand sends it while no throttle is commanded.
const PULSE_STOP = 950;
// A 4S pack: full, and the least at rest worth flying a table on.
const FULL_V = 16.8;
const LOW_CELL_V = 3.6;
const CELLS = 4;
// An HX711 whose bridge is disconnected reads near zero with little noise (2026-11-14: about 1,590 ± 10); a healthy one here rests near 256,000.
const NO_SIGNAL_COUNTS = 20_000;
const SATURATED = 8_388_000;
const SPAN_MS = 60_000;

type Point = { time: number; value: number };
const gap = (previous: Point, next: Point) => next.time - previous.time > 1500;
const pointTime = (point: Point) => point.time;
const within = (points: Point[], start: number, end: number) =>
  points.filter((point) => point.time >= start && point.time <= end);
const mean = (values: number[]) =>
  values.reduce((sum, value) => sum + value, 0) / values.length;
const spread = (values: number[]) => {
  const middle = mean(values);
  return Math.sqrt(mean(values.map((value) => (value - middle) ** 2)));
};

type Check = { label: string; ok: boolean; detail: string };

function CheckList({ checks }: { checks: Check[] }) {
  return (
    <ul className="space-y-1">
      {checks.map((check) => (
        <li key={check.label} className="flex gap-2">
          {check.ok ? (
            <CircleCheck className="mt-0.5 size-4 shrink-0 text-green-700" />
          ) : (
            <CircleAlert className="mt-0.5 size-4 shrink-0 text-red-700" />
          )}
          <span>
            <span className="font-medium">{check.label}</span>{' '}
            <span
              className={check.ok ? 'text-muted-foreground' : 'text-red-700'}
            >
              {check.detail}
            </span>
          </span>
        </li>
      ))}
    </ul>
  );
}

type Readings = {
  status: StandState | null;
  // Laptop time (ms) of the poll that brought these.
  now: number;
  raw: Point[];
  volts: Point[];
  pulse: Point[];
  hz: number;
};

// The stand's readings, polled five times a second; each poll makes new arrays, so a render only reads state.
function useStand() {
  const [readings, setReadings] = useState<Readings>({
    status: null,
    now: 0,
    raw: [],
    volts: [],
    pulse: [],
    hz: 0,
  });
  useEffect(() => {
    let stopped = false;
    let seq = 0;
    const poll = async () => {
      const next = await fetch(`/api/stand?since=${seq}`)
        .then((response) => response.json())
        .catch(() => null);
      if (stopped || !next) return;
      seq = next.seq;
      const raw: Point[] = [];
      const volts: Point[] = [];
      const pulse: Point[] = [];
      let hz = 0;
      for (const sample of next.samples as StandSample[]) {
        if (sample.kind === 'cell' && sample.cell === 0)
          raw.push({ time: sample.time, value: sample.raw });
        else if (sample.kind === 'supply')
          volts.push({ time: sample.time, value: sample.volts });
        else if (sample.kind === 'esc') {
          pulse.push({ time: sample.time, value: sample.pulse });
          hz = sample.hz;
        }
      }
      const now = Date.now();
      const keep = (old: Point[], added: Point[]) =>
        old.concat(added).filter((point) => point.time >= now - 10 * 60_000);
      setReadings((old) => ({
        status: next.status,
        now,
        raw: keep(old.raw, raw),
        volts: keep(old.volts, volts),
        // The pulse is sent only when it changes: keep the last one however old.
        pulse: old.pulse.concat(pulse).slice(-500),
        hz: hz || old.hz,
      }));
    };
    void poll();
    const timer = setInterval(poll, 200);
    return () => {
      stopped = true;
      clearInterval(timer);
    };
  }, []);
  return readings;
}

// The cell's resting count: its mean over the latest stretch with the motor stopped, from 2 s after the stop to the end of the stretch (or now), at most the last 3 s of it.
function restingCount(raw: Point[], pulse: Point[], now: number) {
  let end = now;
  let index = pulse.length - 1;
  // Back over the current run, if any, to the stop before it.
  while (index >= 0 && pulse[index].value > PULSE_STOP)
    end = pulse[index--].time;
  // Back to where that stop began.
  while (index > 0 && pulse[index - 1].value <= PULSE_STOP) index--;
  const start = index >= 0 ? pulse[index].time + 2000 : 0;
  const window = within(raw, Math.max(start, end - 3000), end);
  return window.length >= 5 ? mean(window.map((point) => point.value)) : null;
}

function Live() {
  const {
    status,
    now,
    raw,
    volts: voltPoints,
    pulse: pulsePoints,
    hz,
  } = useStand();
  const connected = status?.state === 'connected';
  const latest = Math.max(raw.at(-1)?.time ?? 0, voltPoints.at(-1)?.time ?? 0);
  const end = useChartEnd(connected, latest);
  const [gutter] = useState(() => ({ current: 48 }));

  const pulse = pulsePoints.at(-1)?.value ?? PULSE_STOP;
  const running = pulse > PULSE_STOP;
  const runStart = running
    ? (pulsePoints.findLast(
        (point, index) =>
          point.value > PULSE_STOP &&
          (index === 0 || pulsePoints[index - 1].value <= PULSE_STOP)
      )?.time ?? now)
    : now;
  const tare = restingCount(raw, pulsePoints, now);
  const perCount = Math.abs(status?.scale['0'] ?? 0);
  // The prop's lift raises the count on this stand (2026-11-14), the calibration mass lowered it: lift is the rise, scaled.
  const thrust = raw.map((point) => ({
    time: point.time,
    value: tare === null ? 0 : (point.value - tare) * (perCount || 1),
  }));
  const throttle = pulsePoints.map((point) => ({
    time: point.time,
    value: point.value <= PULSE_STOP ? 0 : point.value - 1000,
  }));
  // The throttle holds until the next change: carry its last value to the chart's right edge.
  if (throttle.length)
    throttle.push({ time: end, value: throttle.at(-1)!.value });

  const recentRaw = within(raw, now - 2000, now).map((point) => point.value);
  const recentVolts = within(voltPoints, now - 1000, now).map(
    (point) => point.value
  );
  const volts = recentVolts.length ? mean(recentVolts) : null;
  const restVolts = within(voltPoints, runStart - 1500, runStart - 200).map(
    (point) => point.value
  );
  const thrustNow = within(thrust, now - 500, now).map((point) => point.value);
  const peak = Math.max(
    0,
    ...within(thrust, end - SPAN_MS, end).map((point) => point.value)
  );

  const checks: Check[] = [];
  const silent = status?.heard ? (now - status.heard) / 1000 : null;
  checks.push(
    !status || status.state === 'offline' || status.state === 'searching'
      ? {
          label: 'Wi-Fi',
          ok: false,
          detail: 'Stand not on the network: check its power',
        }
      : silent !== null && silent > 1
        ? {
            label: 'Wi-Fi',
            ok: false,
            detail: `Stand silent for ${silent.toFixed(0)} s`,
          }
        : { label: 'Wi-Fi', ok: true, detail: `${status.host}` }
  );
  if (status?.restarts)
    checks.push({
      label: 'Restarts',
      ok: false,
      detail: `${status.restarts} since the dashboard connected: power, brownout or an update`,
    });
  if (connected) {
    const rate = recentRaw.length / 2;
    checks.push(
      !recentRaw.length
        ? {
            label: 'Load cell',
            ok: false,
            detail: 'No readings: check the HX711’s DT, SCK and power',
          }
        : Math.abs(mean(recentRaw)) < NO_SIGNAL_COUNTS &&
            spread(recentRaw) < 200
          ? {
              label: 'Load cell',
              ok: false,
              detail: `No signal (reads ${Math.round(mean(recentRaw)).toLocaleString()}): check the cell’s four wires to the HX711`,
            }
          : recentRaw.some((raw) => Math.abs(raw) >= SATURATED)
            ? {
                label: 'Load cell',
                ok: false,
                detail: 'Saturated: overloaded or a wire off',
              }
            : rate < 8
              ? {
                  label: 'Load cell',
                  ok: false,
                  detail: `Slow: ${rate.toFixed(1)} readings/s`,
                }
              : {
                  label: 'Load cell',
                  ok: true,
                  detail: `${rate.toFixed(0)} readings/s, ${perCount ? 'scaled' : 'not scaled: run sw.stand scale'}`,
                }
    );
    checks.push(
      volts === null
        ? {
            label: 'Supply',
            ok: false,
            detail: 'No voltage readings: flash the stand',
          }
        : volts < 5
          ? {
              label: 'Supply',
              ok: false,
              detail: `Reads ${volts.toFixed(1)} V: no pack on the GPIO35 divider`,
            }
          : volts > FULL_V + 0.4
            ? {
                label: 'Supply',
                ok: false,
                detail: `${volts.toFixed(2)} V is above a full 4S: check the divider`,
              }
            : !running && volts / CELLS < LOW_CELL_V
              ? {
                  label: 'Supply',
                  ok: false,
                  detail: `Pack low: ${(volts / CELLS).toFixed(2)} V a cell at rest`,
                }
              : {
                  label: 'Supply',
                  ok: true,
                  detail: `${(volts / CELLS).toFixed(2)} V a cell`,
                }
    );
  }

  const thrustSeries: ChartSeries<Point>[] = [
    {
      label: 'Thrust',
      color: 'var(--color-chart-1)',
      value: (point) => point.value,
    },
  ];
  const voltSeries: ChartSeries<Point>[] = [
    {
      label: 'Supply',
      color: 'var(--color-chart-2)',
      value: (point) => point.value,
    },
  ];
  const throttleSeries: ChartSeries<Point>[] = [
    {
      label: 'Throttle',
      color: 'var(--color-chart-3)',
      value: (point) => point.value,
    },
  ];

  return (
    <div className="grid lg:grid-cols-[22rem_1fr] lg:divide-x">
      <div className="divide-y">
        <Panel title="Checks">
          <CheckList checks={checks} />
        </Panel>
        <Panel title="Now">
          <Stats>
            <Stat label="Thrust" note={`peak ${peak.toFixed(0)} g`}>
              {thrustNow.length && tare !== null
                ? `${mean(thrustNow).toFixed(0)} g`
                : '–'}
            </Stat>
            <Stat
              label="Supply"
              note={
                running && restVolts.length && volts !== null
                  ? `sag ${(mean(restVolts) - volts).toFixed(2)} V`
                  : volts !== null
                    ? `${(volts / CELLS).toFixed(2)} V a cell`
                    : undefined
              }
            >
              {volts !== null ? `${volts.toFixed(2)} V` : '–'}
            </Stat>
            <Stat label="Throttle" note={`${pulse} µs at ${hz || '–'} Hz`}>
              {!pulsePoints.length ? '–' : running ? pulse - 1000 : 'Stopped'}
            </Stat>
          </Stats>
        </Panel>
      </div>
      <div className="divide-y">
        <Panel title="Thrust (g)">
          <FastChart
            history={(start, stop) => within(thrust, start, stop)}
            series={thrustSeries}
            time={pointTime}
            start={end - SPAN_MS}
            end={end}
            gap={gap}
            minSpan={50}
            gutter={gutter}
            formatTime={relativeTime}
            formatValue={(value) => value.toFixed(0)}
            label="Thrust over the last minute, in grams"
          />
        </Panel>
        <Panel title="Supply (V)">
          <FastChart
            history={(start, stop) => within(voltPoints, start, stop)}
            series={voltSeries}
            time={pointTime}
            start={end - SPAN_MS}
            end={end}
            gap={gap}
            minSpan={1}
            gutter={gutter}
            formatTime={relativeTime}
            formatValue={(value) => value.toFixed(1)}
            label="Supply voltage over the last minute, in volts"
            className="h-28"
          />
        </Panel>
        <Panel title="Throttle">
          <FastChart
            history={(start, stop) => within(throttle, start, stop)}
            series={throttleSeries}
            time={pointTime}
            start={end - SPAN_MS}
            end={end}
            range={[0, 1200]}
            gutter={gutter}
            formatTime={relativeTime}
            formatValue={(value) => value.toFixed(0)}
            label="Throttle over the last minute, 0 to 1200"
            className="h-24"
          />
        </Panel>
      </div>
    </div>
  );
}

// The analysis's per-step lines (sw/stand.py analyze): thrust's 63% time and the held supply, by throttle.
function parseAnalysis(text: string) {
  const steps = new Map<number, { ms?: number; volts?: number }>();
  const order: number[] = [];
  const notes: string[] = [];
  const checks: Check[] = [];
  for (const line of text.split('\n')) {
    const timing = line.match(/to throttle\s+(-?\d+) at\s+[\d.]+ s\s+(-?\d+)/);
    const supply = line.match(/^\s+throttle\s+(-?\d+)\s+([\d.]+)$/);
    const check = line.match(/^check (ok|FAIL) ([^:]+): (.*)$/);
    if (check)
      checks.push({ ok: check[1] === 'ok', label: check[2], detail: check[3] });
    else if (timing) {
      const throttle = Number(timing[1]);
      order.push(throttle);
      steps.set(order.length - 1, {
        ...steps.get(order.length - 1),
        ms: Number(timing[2]),
      });
    } else if (supply) {
      const index = [...steps.keys()].find(
        (key) =>
          steps.get(key)!.volts === undefined &&
          order[key] === Number(supply[1])
      );
      if (index !== undefined)
        steps.set(index, { ...steps.get(index), volts: Number(supply[2]) });
      else {
        order.push(Number(supply[1]));
        steps.set(order.length - 1, { volts: Number(supply[2]) });
      }
    } else if (
      line.trim() &&
      !/^(step|supply at)/.test(line.trim()) &&
      !line.startsWith('saved')
    )
      notes.push(line.trim());
  }
  return {
    checks,
    rows: order.map((throttle, index) => ({ throttle, ...steps.get(index) })),
    notes,
  };
}

function Runs() {
  const [runs, setRuns] = useState<StandRun[] | null>(null);
  const [name, setName] = useState<string | null>(null);
  const [analysis, setAnalysis] = useState<{
    name: string;
    text?: string;
    error?: string;
  } | null>(null);
  useEffect(() => {
    const load = () =>
      fetch('/api/stand/runs')
        .then((response) => response.json())
        .then((value) => Array.isArray(value) && setRuns(value))
        .catch(() => {});
    load();
    const timer = setInterval(load, 10_000);
    return () => clearInterval(timer);
  }, []);
  const chosen = name ?? runs?.[0]?.name ?? null;
  useEffect(() => {
    if (!chosen) return;
    let current = true;
    fetch(`/api/stand/runs?name=${chosen}`)
      .then((response) => response.json())
      .then(
        (value) =>
          current &&
          setAnalysis({
            name: chosen,
            text: value.analysis,
            error: value.error,
          })
      )
      .catch(
        (failure: Error) =>
          current && setAnalysis({ name: chosen, error: failure.message })
      );
    return () => {
      current = false;
    };
  }, [chosen]);
  const items = (runs ?? []).map((run) => ({
    value: run.name,
    label: `${startTime(run.started)} · ${run.profile}`,
  }));
  const ready = analysis?.name === chosen ? analysis : null;
  const parsed = ready?.text ? parseAnalysis(ready.text) : null;
  return (
    <Panel
      title="Runs"
      header={
        <Select
          value={chosen}
          items={items}
          onValueChange={(value) => setName(String(value))}
        >
          <SelectTrigger aria-label="Run" className="w-64">
            <SelectValue />
          </SelectTrigger>
          <SelectContent align="start" alignItemWithTrigger={false}>
            <SelectGroup>
              {items.map((item) => (
                <SelectItem key={item.value} value={item.value}>
                  {item.label}
                </SelectItem>
              ))}
            </SelectGroup>
          </SelectContent>
        </Select>
      }
    >
      {!chosen ? null : !ready ? (
        <p className="text-sm text-muted-foreground">Analyzing…</p>
      ) : ready.error ? (
        <p className="text-sm text-red-700">{ready.error}</p>
      ) : (
        <div className="grid gap-4 xl:grid-cols-[20rem_1fr]">
          <div className="space-y-2">
            {parsed && <CheckList checks={parsed.checks} />}
            {parsed?.notes.map((note) => (
              <p key={note} className="text-sm text-muted-foreground">
                {note}
              </p>
            ))}
            <Table className="tabular-nums">
              <TableHeader>
                <TableRow>
                  <TableHead>Throttle</TableHead>
                  <TableHead className="text-right">63% time</TableHead>
                  <TableHead className="text-right">Supply</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {parsed?.rows.map((row, index) => (
                  <TableRow key={index}>
                    <TableCell>
                      {row.throttle < 0 ? 'Stop' : row.throttle}
                    </TableCell>
                    <TableCell className="text-right">
                      {row.ms !== undefined ? `${row.ms} ms` : '–'}
                    </TableCell>
                    <TableCell className="text-right">
                      {row.volts !== undefined
                        ? `${row.volts.toFixed(2)} V`
                        : '–'}
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </div>
          {/* eslint-disable-next-line @next/next/no-img-element */}
          <img
            src={`/api/stand/runs?name=${chosen}&plot`}
            alt={`Stand run ${chosen}: throttle, thrust, supply, rpm and sound`}
            className="w-full max-w-4xl"
          />
        </div>
      )}
    </Panel>
  );
}

export default function StandPage() {
  return (
    <div className="flex min-h-dvh flex-col">
      <header className="flex h-14 shrink-0 items-center gap-6 border-b px-4">
        <h1 className="font-semibold">Thrust stand</h1>
      </header>
      <main className="divide-y">
        <Live />
        <Runs />
      </main>
    </div>
  );
}
