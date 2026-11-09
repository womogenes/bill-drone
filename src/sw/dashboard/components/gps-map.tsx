'use client';

import 'leaflet/dist/leaflet.css';
import * as L from 'leaflet';
import { memo, useEffect, useMemo, useRef, useState } from 'react';
import { gpsSolved } from '@/components/gps-panel';
import type {
  GpsFix,
  GpsReading,
  GpsSample,
  HoverSample,
} from '@/lib/telemetry';

const TILES =
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}';
const ATTRIBUTION =
  'Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community';
// Imagery is sharp to 19 in most places; one level of upscaling past it still helps at hover scale.
const NATIVE_ZOOM = 19;
const MAX_ZOOM = 20;
// The view always spans at least this much around the takeoff point (m).
const MIN_SPAN = 20;
// Fixes further apart than this are separate stretches, not a path.
const TRAIL_GAP_MS = 2000;
// Drawn points per trail: the history holds minutes at the GPS's 10 Hz.
const TRAIL_POINTS = 600;
const EARTH_RADIUS = 6_371_000;
const COMPASS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
const WIDTH = 400;
const HEIGHT = 300;
const LEFT = 48;
const RIGHT = 8;
const TOP = 8;
const BOTTOM = 36;
const STEPS = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];

// Metres east and north of origin: flat over a flight's few hundred metres.
function local(origin: GpsFix, point: GpsFix): [number, number] {
  const radians = Math.PI / 180;
  return [
    (point.longitude - origin.longitude) *
      radians *
      EARTH_RADIUS *
      Math.cos(origin.latitude * radians),
    (point.latitude - origin.latitude) * radians * EARTH_RADIUS,
  ];
}

// The trail thinned to at most TRAIL_POINTS, split where fixes stopped coming.
function stretches(samples: GpsSample[]): GpsSample[][] {
  const stride = Math.ceil(samples.length / TRAIL_POINTS);
  const kept = samples.filter(
    (_, index) => index % stride === 0 || index === samples.length - 1
  );
  const result: GpsSample[][] = [];
  kept.forEach((sample, index) => {
    if (index === 0 || sample.time - kept[index - 1].time > TRAIL_GAP_MS)
      result.push([]);
    result.at(-1)!.push(sample);
  });
  return result;
}

type Layers = {
  view: L.Map;
  trail: L.Polyline;
  accuracy: L.Circle;
  drone: L.CircleMarker;
  home: L.Marker;
  placed: boolean;
};

export const GpsMap = memo(function GpsMap({
  gps,
  latest,
  home,
  history,
  hover,
}: {
  gps: GpsReading | null | undefined;
  latest: GpsSample | undefined;
  home: GpsFix | undefined;
  history: (start: number, end: number) => GpsSample[];
  hover: HoverSample | null;
}) {
  // Rebuilt per fix (10 Hz), not per status (20 Hz).
  const trail = useMemo(
    () => (latest ? stretches(history(0, latest.time)) : []),
    [history, latest]
  );
  const fix = gpsSolved(gps) ? latest : undefined;
  // The last connection's track, if this one has no fix yet.
  const origin = home ?? trail.at(-1)?.at(-1);
  const offset = home && fix ? local(home, fix) : null;
  const distance = offset && Math.hypot(offset[0], offset[1]);
  const bearing = offset && Math.atan2(offset[0], offset[1]);

  const container = useRef<HTMLDivElement>(null);
  const layers = useRef<Layers | null>(null);
  // Offline in the field: the local plot until a tile arrives.
  const [tiles, setTiles] = useState(false);

  useEffect(() => {
    const element = container.current!;
    // A display, not a map to explore: it follows the drone, and keys stay with the flight controls.
    const view = L.map(element, {
      zoomControl: false,
      dragging: false,
      touchZoom: false,
      scrollWheelZoom: false,
      doubleClickZoom: false,
      boxZoom: false,
      keyboard: false,
      maxZoom: MAX_ZOOM,
      attributionControl: false,
    });
    L.control.attribution({ prefix: false }).addTo(view);
    L.control.scale({ imperial: false }).addTo(view);
    const layer = L.tileLayer(TILES, {
      attribution: ATTRIBUTION,
      maxNativeZoom: NATIVE_ZOOM,
      maxZoom: MAX_ZOOM,
    }).addTo(view);
    let loaded = 0;
    layer.on('loading', () => (loaded = 0));
    layer.on('tileload', () => {
      loaded++;
      setTiles(true);
    });
    // Fires once every tile of the view has loaded or failed.
    layer.on('load', () => setTiles(loaded > 0));
    const retry = () => layer.redraw();
    window.addEventListener('online', retry);
    const resize = new ResizeObserver(() => view.invalidateSize(false));
    resize.observe(element);
    layers.current = {
      view,
      trail: L.polyline([], {
        interactive: false,
        weight: 2,
        className: 'stroke-blue-700 dark:stroke-blue-400',
      }).addTo(view),
      accuracy: L.circle([0, 0], {
        interactive: false,
        radius: 0,
        weight: 1,
        fillOpacity: 0.15,
        className:
          'fill-blue-700 stroke-blue-700 dark:fill-blue-400 dark:stroke-blue-400',
      }),
      drone: L.circleMarker([0, 0], {
        interactive: false,
        radius: 5,
        weight: 2,
        fillOpacity: 1,
        className: 'fill-blue-700 stroke-white dark:fill-blue-400',
      }),
      home: L.marker([0, 0], {
        interactive: false,
        keyboard: false,
        icon: L.divIcon({
          className: 'border border-background bg-foreground',
          iconSize: [10, 10],
        }),
      }),
      placed: false,
    };
    return () => {
      window.removeEventListener('online', retry);
      resize.disconnect();
      view.remove();
      layers.current = null;
    };
  }, []);

  useEffect(() => {
    const map = layers.current;
    if (!map) return;
    const { view } = map;
    map.trail.setLatLngs(
      trail.map((stretch) =>
        stretch.map((sample): L.LatLngTuple => [
          sample.latitude,
          sample.longitude,
        ])
      )
    );
    if (fix) {
      map.drone.setLatLng([fix.latitude, fix.longitude]).addTo(view);
      map.accuracy
        .setLatLng([fix.latitude, fix.longitude])
        .setRadius(fix.horizontal_accuracy)
        .addTo(view);
    } else {
      map.drone.remove();
      map.accuracy.remove();
    }
    if (home) map.home.setLatLng([home.latitude, home.longitude]).addTo(view);
    else map.home.remove();
    if (!origin || !container.current?.clientWidth) return;
    const bounds = L.latLng(origin.latitude, origin.longitude).toBounds(
      MIN_SPAN
    );
    if (fix)
      bounds.extend(
        L.latLng(fix.latitude, fix.longitude).toBounds(
          2 * fix.horizontal_accuracy
        )
      );
    for (const stretch of trail)
      for (const sample of stretch)
        bounds.extend([sample.latitude, sample.longitude]);
    // Refit only when the flight leaves the view or would fit two levels closer, so tiles are not refetched every fix.
    const fits = Math.min(
      view.getBoundsZoom(bounds, false, L.point(48, 48)),
      MAX_ZOOM
    );
    if (
      !map.placed ||
      !view.getBounds().contains(bounds) ||
      fits >= view.getZoom() + 2
    ) {
      view.fitBounds(bounds, {
        padding: [24, 24],
        maxZoom: MAX_ZOOM,
        animate: false,
      });
      map.placed = true;
    }
  }, [trail, fix, home, origin]);

  // The local plot: metres from the takeoff point, north up, equal scale.
  const points = origin
    ? trail.map((stretch) => stretch.map((sample) => local(origin, sample)))
    : [];
  const extent: [number, number][] = [
    [-MIN_SPAN / 2, -MIN_SPAN / 2],
    [MIN_SPAN / 2, MIN_SPAN / 2],
  ];
  for (const stretch of points) extent.push(...stretch);
  if (offset && fix) {
    extent.push([
      offset[0] - fix.horizontal_accuracy,
      offset[1] - fix.horizontal_accuracy,
    ]);
    extent.push([
      offset[0] + fix.horizontal_accuracy,
      offset[1] + fix.horizontal_accuracy,
    ]);
  }
  let minX = Math.min(...extent.map(([x]) => x));
  let maxX = Math.max(...extent.map(([x]) => x));
  let minY = Math.min(...extent.map(([, y]) => y));
  let maxY = Math.max(...extent.map(([, y]) => y));
  const plotWidth = WIDTH - LEFT - RIGHT;
  const plotHeight = HEIGHT - TOP - BOTTOM;
  // Equal scale on both axes, with a margin: widen the tighter one about its center.
  const scale =
    0.9 * Math.min(plotWidth / (maxX - minX), plotHeight / (maxY - minY));
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  minX = centerX - plotWidth / scale / 2;
  maxX = centerX + plotWidth / scale / 2;
  minY = centerY - plotHeight / scale / 2;
  maxY = centerY + plotHeight / scale / 2;
  const px = (east: number) => LEFT + (east - minX) * scale;
  const py = (north: number) => TOP + (maxY - north) * scale;
  const step =
    STEPS.find((value) => Math.max(maxX - minX, maxY - minY) / value <= 6) ??
    STEPS.at(-1)!;
  const ticks = (low: number, high: number) => {
    const values = [];
    for (let value = Math.ceil(low / step) * step; value <= high; value += step)
      values.push(value);
    return values;
  };

  return (
    <section className="space-y-3 px-4 py-4" aria-labelledby="gps-map-heading">
      <h2 id="gps-map-heading" className="font-semibold">
        Position
      </h2>
      <dl className="grid grid-cols-2 gap-x-4 gap-y-1 tabular-nums">
        <dt className="text-muted-foreground">From takeoff</dt>
        <dd>
          {distance === null || bearing === null
            ? '—'
            : `${distance.toFixed(1)} m ${COMPASS[Math.round((bearing * 4) / Math.PI + 8) % 8]}`}
        </dd>
        <dt className="text-muted-foreground">Height</dt>
        <dd>
          {hover
            ? `${hover.height.toFixed(2)} / ${hover.height_target.toFixed(2)} m`
            : '—'}
        </dd>
      </dl>
      <div className="relative isolate aspect-4/3">
        <div ref={container} className="absolute inset-0 z-0 bg-muted!" />
        {!tiles && (
          <svg
            viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
            className="absolute inset-0 size-full bg-background"
            role="img"
            aria-label="Drone position from the takeoff point, north up, with its accuracy and recent track"
          >
            {ticks(minX, maxX).map((value) => (
              <g key={`x${value}`}>
                <line
                  x1={px(value)}
                  x2={px(value)}
                  y1={TOP}
                  y2={HEIGHT - BOTTOM}
                  className="stroke-border"
                />
                <text
                  x={px(value)}
                  y={HEIGHT - BOTTOM + 14}
                  textAnchor="middle"
                  className="fill-muted-foreground text-xs"
                >
                  {value}
                </text>
              </g>
            ))}
            {ticks(minY, maxY).map((value) => (
              <g key={`y${value}`}>
                <line
                  x1={LEFT}
                  x2={WIDTH - RIGHT}
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
                  {value}
                </text>
              </g>
            ))}
            <text
              x={LEFT + plotWidth / 2}
              y={HEIGHT - 4}
              textAnchor="middle"
              className="fill-muted-foreground text-xs"
            >
              east (m)
            </text>
            <text
              transform={`translate(10 ${TOP + plotHeight / 2}) rotate(-90)`}
              textAnchor="middle"
              dominantBaseline="middle"
              className="fill-muted-foreground text-xs"
            >
              north (m)
            </text>
            {home && (
              <rect
                x={px(0) - 5}
                y={py(0) - 5}
                width={10}
                height={10}
                className="fill-foreground"
              />
            )}
            {points.map((stretch, index) => (
              <polyline
                key={index}
                points={stretch
                  .map(([east, north]) => `${px(east)},${py(north)}`)
                  .join(' ')}
                fill="none"
                strokeWidth={1.5}
                className="stroke-blue-700 dark:stroke-blue-400"
              />
            ))}
            {offset && fix && (
              <>
                <circle
                  cx={px(offset[0])}
                  cy={py(offset[1])}
                  r={fix.horizontal_accuracy * scale}
                  fillOpacity={0.15}
                  strokeWidth={1}
                  className="fill-blue-700 stroke-blue-700 dark:fill-blue-400 dark:stroke-blue-400"
                />
                <circle
                  cx={px(offset[0])}
                  cy={py(offset[1])}
                  r={5}
                  className="fill-blue-700 dark:fill-blue-400"
                />
              </>
            )}
          </svg>
        )}
      </div>
    </section>
  );
});
