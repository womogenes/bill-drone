'use client';

import { memo } from 'react';
import type { GpsReading, GpsReport } from '@/lib/telemetry';

const FIXES: Record<number, string> = {
  0: 'No fix',
  1: 'Dead reckoning',
  2: '2D',
  3: '3D',
  4: '3D + DR',
  5: 'Time only',
};

// Short names for the satellites used per system.
const SYSTEMS: Record<string, string> = {
  GPS: 'GPS',
  Galileo: 'GAL',
  BeiDou: 'BDS',
  GLONASS: 'GLO',
  QZSS: 'QZS',
  SBAS: 'SBAS',
};

// What SBAS is doing: corrections in the solution, a WAAS satellite heard but not yet applied, searching, or the drone's SBAS configuration not taken.
function sbasState(gps: GpsReading | null | undefined, report: GpsReport) {
  if (report.config.sbas === 'refused' || report.config.signals === 'refused')
    return 'Not configured';
  if (gps?.differential || report.sbas_corrected) return 'Corrections in use';
  if (report.sbas_heard.length)
    return `Heard PRN ${report.sbas_heard.join(', ')}`;
  return report.config.sbas === 'set' ? 'Searching' : '—';
}

export const GpsPanel = memo(function GpsPanel({
  gps,
  report,
  time,
  connected,
}: {
  gps: GpsReading | null | undefined;
  report?: GpsReport | null;
  time?: number;
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
  const speed = solved
    ? Math.hypot(gps.velocity[0], gps.velocity[1])
    : undefined;
  // The report comes once a second; older than 3 s (or from a drone or bridge without it) shows nothing.
  const diagnosed =
    connected &&
    report &&
    time !== undefined &&
    time - report.time < 3000 &&
    report.satellites_age_ms !== null &&
    report.satellites_age_ms < 3000
      ? report
      : null;
  const used = diagnosed
    ? Object.entries(diagnosed.used)
        .map(([system, count]) => `${SYSTEMS[system] ?? system} ${count}`)
        .join(' · ') || 'none'
    : '—';
  const jammed =
    diagnosed?.jamming === 'warning' || diagnosed?.jamming === 'critical';
  const unconfigured = diagnosed
    ? Object.entries(diagnosed.config)
        .filter(([, state]) => state === 'refused' || state === 'unanswered')
        .map(([group, state]) => `${group} ${state}`)
    : [];
  return (
    <section aria-labelledby="gps-heading" className="space-y-2 px-4 py-3">
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
        <div>
          <dt className="text-sm text-muted-foreground">Signal (top 4)</dt>
          <dd className="text-lg">
            {diagnosed?.top4_cno != null
              ? `${diagnosed.top4_cno.toFixed(1)} dB-Hz`
              : '—'}
          </dd>
          {diagnosed && (
            <dd className="text-sm text-muted-foreground">
              {diagnosed.heard} heard of {diagnosed.listed}
            </dd>
          )}
        </div>
        <div className="col-span-2">
          <dt className="text-sm text-muted-foreground">Used</dt>
          <dd className="text-lg">{used}</dd>
        </div>
        <div>
          <dt className="text-sm text-muted-foreground">Jamming</dt>
          <dd className={`text-lg ${jammed ? 'text-red-700' : ''}`}>
            {diagnosed?.rf_age_ms != null && diagnosed.rf_age_ms < 3000
              ? `${diagnosed.jamming} · ${diagnosed.jam_indicator}/255`
              : '—'}
          </dd>
          {diagnosed?.rf_age_ms != null && diagnosed.rf_age_ms < 3000 && (
            <dd className="text-sm text-muted-foreground">
              noise {diagnosed.noise_per_ms}, AGC{' '}
              {(diagnosed.agc * 100).toFixed(0)}%
            </dd>
          )}
        </div>
        <div className="col-span-2">
          <dt className="text-sm text-muted-foreground">SBAS</dt>
          <dd className="text-lg">
            {diagnosed ? sbasState(gps, diagnosed) : '—'}
          </dd>
        </div>
        {unconfigured.length > 0 && (
          <div className="col-span-3">
            <dt className="text-sm text-muted-foreground">Configuration</dt>
            <dd className="text-lg text-red-700">{unconfigured.join(', ')}</dd>
          </div>
        )}
      </dl>
    </section>
  );
});
