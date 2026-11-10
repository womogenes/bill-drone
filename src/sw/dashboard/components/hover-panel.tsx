'use client';

import { memo, useEffect, useRef, useState } from 'react';
import type { HoverSample, HoverState } from '@/lib/telemetry';

const TRAIL_MS = 10_000;
// An older sighting no longer reflects where the drone is.
const STALE_MS = 150;
// Trail samples further apart than this are separate stretches, not a path.
const TRAIL_GAP_MS = 500;
// The plot is drawn at its real pixel size, so its text stays text-xs however wide the panel is (a viewBox scaled the labels up to 24 px on a wide screen).
const LEFT = 40;
const RIGHT = 8;
const TOP = 8;
const BOTTOM = 22;
const MAX_HEIGHT = 360;
const STEPS = [0.1, 0.2, 0.25, 0.5, 1, 2];

// The drone in a tag-aligned frame, rotated so the hold line points straight out from the tag: [0] toward the tag (the hold sits at -range_target), [1] to the left of that line.
function aligned(sample: HoverState): [number, number] | null {
  if (!sample.tag) return null;
  const [x, y] = sample.tag;
  const around =
    sample.arc_deg !== null
      ? (sample.arc_deg * Math.PI) / 180
      : Math.atan2(y, x);
  const cos = Math.cos(around);
  const sin = Math.sin(around);
  return [-(x * cos + y * sin), x * sin - y * cos];
}

// Top-down screen coordinates in meters, tag at the origin and toward the tag up.
function screen([toward, left]: [number, number]): [number, number] {
  return [-left, -toward];
}

export const HoverPanel = memo(function HoverPanel({
  state,
  history,
}: {
  state: HoverSample;
  history: (start: number, end: number) => HoverSample[];
}) {
  const trail = history(state.time - TRAIL_MS, state.time);
  const lastSeen = state.tag ? state : trail.findLast((sample) => sample.tag);
  const stale =
    !state.tag || state.tag_age_ms === null || state.tag_age_ms > STALE_MS;
  const drone = lastSeen ? aligned(lastSeen) : null;
  const range = state.range_target;
  // Horizontal distance to the tag, as the hold measures it.
  const distance = state.tag ? Math.hypot(state.tag[0], state.tag[1]) : null;
  const distanceError =
    distance !== null && range !== null ? distance - range : null;
  const sidewaysError = state.tag && drone ? drone[1] : null;
  const box = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(400);
  useEffect(() => {
    const element = box.current;
    if (!element) return;
    const resize = new ResizeObserver(() =>
      setWidth(Math.max(200, element.clientWidth))
    );
    resize.observe(element);
    return () => resize.disconnect();
  }, []);
  const height = Math.min(MAX_HEIGHT, Math.round(width * 0.75));

  const points = trail.map((sample) => {
    const position = aligned(sample);
    return position && screen(position);
  });
  const extent: [number, number][] = [[0, 0]];
  if (range !== null) extent.push([0, range]);
  if (drone) extent.push(screen(drone));
  for (const point of points) if (point) extent.push(point);
  let minX = Math.min(...extent.map(([x]) => x)) - 0.15;
  let maxX = Math.max(...extent.map(([x]) => x)) + 0.15;
  let minY = Math.min(...extent.map(([, y]) => y)) - 0.15;
  let maxY = Math.max(...extent.map(([, y]) => y)) + 0.15;
  const plotWidth = width - LEFT - RIGHT;
  const plotHeight = height - TOP - BOTTOM;
  // Equal scale on both axes: widen the tighter one about its center.
  const scale = Math.min(
    plotWidth / Math.max(maxX - minX, 0.6),
    plotHeight / Math.max(maxY - minY, 0.6)
  );
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  minX = centerX - plotWidth / scale / 2;
  maxX = centerX + plotWidth / scale / 2;
  minY = centerY - plotHeight / scale / 2;
  maxY = centerY + plotHeight / scale / 2;
  const px = (x: number) => LEFT + (x - minX) * scale;
  const py = (y: number) => TOP + (y - minY) * scale;
  const step =
    STEPS.find((value) => Math.max(maxX - minX, maxY - minY) / value <= 6) ??
    STEPS.at(-1)!;
  const ticks = (low: number, high: number) => {
    const values = [];
    for (let value = Math.ceil(low / step) * step; value <= high; value += step)
      values.push(Math.round(value * 100) / 100);
    return values;
  };
  const tickText = (value: number) =>
    (Math.abs(value) < 1e-9 ? 0 : value).toFixed(step < 0.5 ? 2 : 1);

  return (
    <section className="space-y-3 px-4 py-4" aria-labelledby="hover-heading">
      <h2 id="hover-heading" className="font-semibold">
        Tag position
      </h2>
      <dl className="grid grid-cols-2 gap-x-4 gap-y-1 tabular-nums">
        <dt className="text-muted-foreground">Distance</dt>
        <dd>
          {distance === null
            ? '—'
            : `${distance.toFixed(2)} m${range !== null ? ` (hold ${range.toFixed(2)} m)` : ''}`}
        </dd>
        <dt className="text-muted-foreground">Distance error</dt>
        <dd>
          {distanceError === null
            ? '—'
            : `${Math.abs(distanceError).toFixed(2)} m ${distanceError > 0 ? 'too far' : 'too close'}`}
        </dd>
        <dt className="text-muted-foreground">Sideways error</dt>
        <dd>
          {sidewaysError === null
            ? '—'
            : `${Math.abs(sidewaysError).toFixed(2)} m ${sidewaysError > 0 ? 'left' : 'right'}`}
        </dd>
        <dt className="text-muted-foreground">Height</dt>
        <dd>
          {state.height.toFixed(2)} / {state.height_target.toFixed(2)} m
        </dd>
        <dt className="text-muted-foreground">Tag</dt>
        <dd className={stale ? 'text-red-700' : undefined}>
          {state.tag_age_ms === null || !state.tag
            ? 'Lost'
            : `${stale ? 'Stale · ' : ''}${state.tag_age_ms} ms`}
        </dd>
      </dl>
      <div ref={box} className="w-full">
        <svg
          width={width}
          height={height}
          className="block"
          role="img"
          aria-label="Drone position relative to the tag, top down, with the hold target and the last 10 seconds"
        >
          {ticks(minX, maxX).map((value) => (
            <g key={`x${value}`}>
              <line
                x1={px(value)}
                x2={px(value)}
                y1={TOP}
                y2={height - BOTTOM}
                className="stroke-border"
              />
              <text
                x={px(value)}
                y={height - BOTTOM + 14}
                textAnchor="middle"
                className="fill-muted-foreground text-xs"
              >
                {tickText(value)}
              </text>
            </g>
          ))}
          {ticks(minY, maxY).map((value) => (
            <g key={`y${value}`}>
              <line
                x1={LEFT}
                x2={width - RIGHT}
                y1={py(value)}
                y2={py(value)}
                className="stroke-border"
              />
              <text
                x={LEFT - 6}
                y={py(value)}
                textAnchor="end"
                dominantBaseline="middle"
                className="fill-muted-foreground text-xs"
              >
                {tickText(value)}
              </text>
            </g>
          ))}
          <line
            x1={px(0)}
            x2={px(0)}
            y1={py(0)}
            y2={height - BOTTOM}
            strokeDasharray="4 4"
            className="stroke-muted-foreground"
          />
          <rect
            x={px(0) - 5}
            y={py(0) - 5}
            width={10}
            height={10}
            className="fill-foreground"
          />
          {range !== null && (
            <circle
              cx={px(0)}
              cy={py(range)}
              r={7}
              fill="none"
              strokeWidth={1.5}
              className="stroke-foreground"
            />
          )}
          {points.map((point, index) => {
            const previous = points[index - 1];
            if (
              !point ||
              !previous ||
              trail[index].time - trail[index - 1].time > TRAIL_GAP_MS
            )
              return null;
            return (
              <line
                key={index}
                x1={px(previous[0])}
                y1={py(previous[1])}
                x2={px(point[0])}
                y2={py(point[1])}
                strokeWidth={1.5}
                strokeOpacity={
                  1 - (0.9 * (state.time - trail[index].time)) / TRAIL_MS
                }
                className="stroke-blue-700 dark:stroke-blue-400"
              />
            );
          })}
          {drone && (
            <circle
              cx={px(screen(drone)[0])}
              cy={py(screen(drone)[1])}
              r={5}
              className={
                stale
                  ? 'fill-muted-foreground'
                  : 'fill-blue-700 dark:fill-blue-400'
              }
            />
          )}
        </svg>
      </div>
      <p className="text-xs text-muted-foreground">
        Top down, meters from the tag (■), up toward it. Circle: hold point.
      </p>
    </section>
  );
});
