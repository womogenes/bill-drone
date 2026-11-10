'use client';

import { memo } from 'react';
import { Stat } from '@/components/stat';
import { gpsSolved } from '@/lib/gps';
import type { BaroReading, GpsReading, GpsReport } from '@/lib/telemetry';

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
  baro,
  report,
  time,
  connected,
}: {
  gps: GpsReading | null | undefined;
  baro?: BaroReading | null;
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
  const solved = connected && gpsSolved(gps);
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
  // The RF report (jamming, noise, AGC) has its own age.
  const rf =
    diagnosed?.rf_age_ms != null && diagnosed.rf_age_ms < 3000
      ? diagnosed
      : null;
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
        <Stat
          label="Fix"
          valueClassName={
            live && gps.ok
              ? 'text-green-700'
              : connected && gps
                ? 'text-red-700'
                : ''
          }
        >
          {state}
        </Stat>
        <Stat label="Satellites">{live ? gps.satellites : '—'}</Stat>
        <Stat label="Accuracy">
          {solved
            ? `${gps.horizontal_accuracy.toFixed(1)} / ${gps.vertical_accuracy.toFixed(1)} m`
            : '—'}
        </Stat>
        <Stat label="Ground speed">
          {solved && speed !== undefined
            ? `${speed.toFixed(2)} ± ${gps.speed_accuracy.toFixed(2)} m/s`
            : '—'}
        </Stat>
        <Stat label="Climb">
          {solved ? `${(-gps.velocity[2]).toFixed(2)} m/s` : '—'}
        </Stat>
        <Stat label="Altitude">
          {solved ? `${gps.height.toFixed(1)} m` : '—'}
        </Stat>
        <Stat label="Position" className="col-span-3">
          {solved
            ? `${gps.latitude.toFixed(7)}, ${gps.longitude.toFixed(7)}`
            : '—'}
        </Stat>
        <Stat
          label="Signal (top 4)"
          note={diagnosed && `${diagnosed.heard} heard of ${diagnosed.listed}`}
        >
          {diagnosed?.top4_cno != null
            ? `${diagnosed.top4_cno.toFixed(1)} dB-Hz`
            : '—'}
        </Stat>
        <Stat label="Used" className="col-span-2">
          {used}
        </Stat>
        <Stat
          label="Jamming"
          valueClassName={jammed ? 'text-red-700' : ''}
          note={
            rf && `noise ${rf.noise_per_ms}, AGC ${(rf.agc * 100).toFixed(0)}%`
          }
        >
          {rf ? `${rf.jamming} · ${rf.jam_indicator}/255` : '—'}
        </Stat>
        <Stat label="SBAS" className="col-span-2">
          {diagnosed ? sbasState(gps, diagnosed) : '—'}
        </Stat>
        {unconfigured.length > 0 && (
          <Stat
            label="Configuration"
            className="col-span-3"
            valueClassName="text-red-700"
          >
            {unconfigured.join(', ')}
          </Stat>
        )}
        <Stat label="Barometer" className="col-span-3">
          {!connected || !baro
            ? '—'
            : !baro.found
              ? 'Not found'
              : baro.altitude === null
                ? 'Stale'
                : `${baro.altitude.toFixed(1)} m pressure altitude`}
        </Stat>
      </dl>
    </section>
  );
});
