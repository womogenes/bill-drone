'use client';

import { memo } from 'react';
import type { BaroReading, GpsReading } from '@/lib/telemetry';

const FIXES: Record<number, string> = {
  0: 'No fix',
  1: 'Dead reckoning',
  2: '2D',
  3: '3D',
  4: '3D + DR',
  5: 'Time only',
};

export const GpsPanel = memo(function GpsPanel({
  gps,
  baro,
  connected,
}: {
  gps: GpsReading | null | undefined;
  baro?: BaroReading | null;
  connected: boolean;
}) {
  const live = connected && gps && gps.age_ms !== null && gps.age_ms < 1000;
  const state =
    !connected || !gps
      ? '—'
      : !gps.configured
        ? 'Searching'
        : !live
          ? 'Stale'
          : `${FIXES[gps.fix] ?? gps.fix}${gps.ok ? '' : ' (weak)'}`;
  // Without a fix the module still sends its filter's placeholders (accuracy 4294967 m, a position anywhere): shown only once it has a 2D or 3D fix it trusts.
  const solved = live && gps.ok && gps.fix >= 2 && gps.fix <= 4;
  const speed = solved ? Math.hypot(gps.velocity[0], gps.velocity[1]) : undefined;
  return (
    <section
      aria-labelledby="gps-heading"
      className="space-y-2 px-4 py-3"
    >
      <h2 id="gps-heading" className="font-semibold">
        GPS
      </h2>
      <dl className="grid grid-cols-3 gap-4 tabular-nums">
        <div>
          <dt className="text-sm text-muted-foreground">Fix</dt>
          <dd
            className={`text-lg ${live && gps.ok ? 'text-green-700' : connected && gps ? 'text-red-700' : ''}`}
          >
            {state}
          </dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Satellites</dt>
          <dd className="text-lg">{live ? gps.satellites : '—'}</dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Accuracy</dt>
          <dd className="text-lg">
            {solved
              ? `${gps.horizontal_accuracy.toFixed(1)} / ${gps.vertical_accuracy.toFixed(1)} m`
              : '—'}
          </dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Ground speed</dt>
          <dd className="text-lg">
            {solved && speed !== undefined
              ? `${speed.toFixed(2)} ± ${gps.speed_accuracy.toFixed(2)} m/s`
              : '—'}
          </dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Climb</dt>
          <dd className="text-lg">
            {solved ? `${(-gps.velocity[2]).toFixed(2)} m/s` : '—'}
          </dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Altitude</dt>
          <dd className="text-lg">
            {solved ? `${gps.height.toFixed(1)} m` : '—'}
          </dd>
        </div>
        <div className="col-span-3">
          <dt className="text-sm text-muted-foreground">Position</dt>
          <dd className="text-lg">
            {solved
              ? `${gps.latitude.toFixed(7)}, ${gps.longitude.toFixed(7)}`
              : '—'}
          </dd>
        </div>
        <div className="col-span-3">
          <dt className="text-sm text-muted-foreground">Barometer</dt>
          <dd className="text-lg">
            {!connected || !baro
              ? '—'
              : !baro.found
                ? 'Not found'
                : baro.altitude === null
                  ? 'Stale'
                  : `${baro.altitude.toFixed(1)} m pressure altitude`}
          </dd>
        </div>
      </dl>
    </section>
  );
});
