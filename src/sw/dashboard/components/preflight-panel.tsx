'use client';

import { memo, useEffect, useState } from 'react';
import { CircleCheck, CircleX, TriangleAlert } from 'lucide-react';
import { readPad } from '@/components/gamepad';
import type { CameraStatus } from '@/components/fpv-panel';
import type { Reading } from '@/lib/telemetry';

// The drone refuses to arm under ARM_MIN_V (fw/drone/main/flight_rt.c); under LOW_V (3.7 V a cell on 4S) a flight is short. NO_PACK_V and below is USB power alone.
const ARM_MIN_V = 14;
const LOW_V = 14.8;
const NO_PACK_V = 5;
// The bridge drops the dashboard after 0.5 s without a status (src/fw/controller/bridge.py monitor).
const LINK_STALE_MS = 500;
// The status arrives at 20 Hz; well under it the radio is losing packets.
const LINK_MIN_HZ = 15;
// The GPS hold takes fixes to 8 m (fw/drone/main/flight_rt.c GPS_MAX_ACCURACY); a good outdoor fix is well inside GOOD_ACCURACY with GOOD_SATELLITES or more.
const HOLD_ACCURACY = 8;
const GOOD_ACCURACY = 2;
const GOOD_SATELLITES = 8;

type Level = 'go' | 'caution' | 'stop';
type Check = { label: string; level: Level; reason: string };

const LEVELS = {
  go: { icon: CircleCheck, color: 'text-green-700' },
  caution: { icon: TriangleAlert, color: 'text-amber-600' },
  stop: { icon: CircleX, color: 'text-red-700' },
};

function checks(
  reading: Reading | null,
  connected: boolean,
  fps: number | null,
  camera: CameraStatus,
  pad: string | null,
  now: number
): Check[] {
  const age = reading ? now - reading.time : Infinity;
  const link: Check = !connected
    ? { label: 'Drone link', level: 'stop', reason: 'Not connected' }
    : age > LINK_STALE_MS
      ? {
          label: 'Drone link',
          level: 'stop',
          reason: `No status for ${(age / 1000).toFixed(1)} s`,
        }
      : fps !== null && fps < LINK_MIN_HZ
        ? {
            label: 'Drone link',
            level: 'caution',
            reason: `Status at ${Math.round(fps)} Hz of 20: packets lost`,
          }
        : { label: 'Drone link', level: 'go', reason: 'Fresh' };
  if (!connected || !reading) return [link];

  const imu = reading.imu;
  const control = reading.control;
  const gps = reading.gps;
  const baro = reading.baro;
  const volts = reading.voltage_ema;
  // Off (0) the hover never reads the barometer, so a missing one only cautions.
  const baroUsed = control?.hover_baro_tau;
  const baroMiss: Level = baroUsed ? 'stop' : 'caution';
  // Free flight (hover_camera 0) holds on the GPS; on the tag it needs the camera instead.
  const free = control?.hover_camera === 0;
  const live = !!gps && gps.age_ms !== null && gps.age_ms < 1000;
  const hold = control?.hover_gps_tau;
  const recorder = reading.recorder;

  return [
    link,
    !imu
      ? { label: 'IMU', level: 'stop', reason: 'Offline' }
      : imu.calibrating
        ? { label: 'IMU', level: 'caution', reason: 'Calibrating' }
        : imu.calibrated
          ? { label: 'IMU', level: 'go', reason: 'Live and calibrated' }
          : { label: 'IMU', level: 'stop', reason: 'Not calibrated' },
    !baro
      ? {
          label: 'Barometer',
          level: baroMiss,
          reason: 'No data from this firmware',
        }
      : !baro.found
        ? { label: 'Barometer', level: baroMiss, reason: 'Not found' }
        : !baro.fresh || baro.altitude === null
          ? {
              label: 'Barometer',
              level: baroMiss,
              reason: 'Found, readings stale',
            }
          : {
              label: 'Barometer',
              level: baroUsed ? 'go' : 'caution',
              reason: baroUsed
                ? `Fresh, ${baro.altitude.toFixed(1)} m`
                : 'Fresh, but the hover does not use it (hover_baro_tau 0)',
            },
    !gps || !gps.configured
      ? { label: 'GPS', level: 'stop', reason: 'Module not found' }
      : !live
        ? { label: 'GPS', level: 'stop', reason: 'No fresh solution' }
        : !gps.ok || gps.fix !== 3
          ? {
              label: 'GPS',
              level: 'stop',
              reason: `No 3D fix (${gps.satellites} satellites)`,
            }
          : gps.horizontal_accuracy > HOLD_ACCURACY
            ? {
                label: 'GPS',
                level: 'stop',
                reason: `3D fix but ${gps.horizontal_accuracy.toFixed(1)} m: the hold ignores it`,
              }
            : {
                label: 'GPS',
                level:
                  gps.horizontal_accuracy <= GOOD_ACCURACY &&
                  gps.satellites >= GOOD_SATELLITES
                    ? 'go'
                    : 'caution',
                reason: `3D fix, ${gps.satellites} satellites, ${gps.horizontal_accuracy.toFixed(1)} m`,
              },
    hold === undefined
      ? { label: 'GPS hold', level: 'caution', reason: 'Not in this firmware' }
      : hold > 0
        ? { label: 'GPS hold', level: 'go', reason: `On (${hold} s)` }
        : {
            label: 'GPS hold',
            level: free ? 'stop' : 'caution',
            reason: 'Off (hover_gps_tau 0)',
          },
    volts === undefined
      ? { label: 'Battery', level: 'stop', reason: 'No reading' }
      : volts <= NO_PACK_V
        ? { label: 'Battery', level: 'stop', reason: 'No pack: USB power' }
        : volts < ARM_MIN_V
          ? {
              label: 'Battery',
              level: 'stop',
              reason: `${volts.toFixed(1)} V, under the ${ARM_MIN_V} V arm minimum`,
            }
          : {
              label: 'Battery',
              level: volts < LOW_V ? 'caution' : 'go',
              reason:
                volts < LOW_V
                  ? `${volts.toFixed(1)} V, close to the ${ARM_MIN_V} V arm minimum`
                  : `${volts.toFixed(1)} V`,
            },
    recorder?.error
      ? { label: 'Recorder', level: 'stop', reason: recorder.error }
      : reading.logging?.active
        ? { label: 'Recorder', level: 'go', reason: 'Recording' }
        : reading.logging?.ready
          ? { label: 'Recorder', level: 'go', reason: 'Ready' }
          : { label: 'Recorder', level: 'stop', reason: 'Not ready' },
    pad
      ? { label: 'Gamepad', level: 'go', reason: pad }
      : {
          label: 'Gamepad',
          level: 'caution',
          reason: 'Not connected: keyboard only',
        },
    // The camera's Wi-Fi sits beside the GPS antenna: streaming may jam the GPS.
    free
      ? !camera
        ? { label: 'Camera', level: 'go', reason: 'Not answering (off)' }
        : {
            label: 'Camera',
            level: 'caution',
            reason: `${camera.state === 'fpv' ? 'FPV streaming' : `On (${camera.state})`}: may jam the GPS`,
          }
      : !camera
        ? {
            label: 'Camera',
            level: 'stop',
            reason: 'Not answering: the tag hover needs it',
          }
        : camera.state === 'running'
          ? { label: 'Camera', level: 'go', reason: 'Tracking' }
          : {
              label: 'Camera',
              level: camera.state === 'fpv' ? 'caution' : 'stop',
              reason:
                camera.state === 'fpv'
                  ? 'FPV streaming: no tag tracking, may jam the GPS'
                  : `Camera ${camera.state}`,
            },
  ];
}

// Go/no-go before arming: each check green, amber or red with its reason, and the overall call (red anywhere is no-go).
export const PreflightPanel = memo(function PreflightPanel({
  reading,
  connected,
  fps,
  camera,
}: {
  reading: Reading | null;
  connected: boolean;
  fps: number | null;
  camera: CameraStatus;
}) {
  // A silent link sends no readings to re-render on, and the gamepad has none: look again twice a second.
  const [now, setNow] = useState(() => Date.now());
  const [pad, setPad] = useState<string | null>(null);
  useEffect(() => {
    const timer = setInterval(() => {
      setNow(Date.now());
      setPad(readPad()?.id ?? null);
    }, 500);
    return () => clearInterval(timer);
  }, []);
  const list = checks(reading, connected, fps, camera, pad, now);
  const stops = list.filter((check) => check.level === 'stop').length;
  const cautions = list.filter((check) => check.level === 'caution').length;
  const overall: Level = stops ? 'stop' : cautions ? 'caution' : 'go';
  const { icon: OverallIcon, color } = LEVELS[overall];
  return (
    <section
      className="space-y-3 px-4 py-3"
      aria-labelledby="preflight-heading"
    >
      <div className="flex items-center gap-2">
        <h2 id="preflight-heading" className="mr-auto font-semibold">
          Preflight
        </h2>
        <span
          role="status"
          className={`flex items-center gap-1.5 font-medium ${color}`}
        >
          <OverallIcon aria-hidden="true" className="size-4" />
          {stops ? 'Not ready' : 'Ready'}
        </span>
      </div>
      <ul className="space-y-1 text-sm tabular-nums">
        {list.map(({ label, level, reason }) => {
          const { icon: Icon, color } = LEVELS[level];
          return (
            <li key={label} className="flex items-center gap-2">
              <Icon aria-hidden="true" className={`size-4 shrink-0 ${color}`} />
              <span className="w-24 shrink-0">{label}</span>
              <span className="truncate text-muted-foreground" title={reason}>
                {reason}
              </span>
            </li>
          );
        })}
      </ul>
    </section>
  );
});
