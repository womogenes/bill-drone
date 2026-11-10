'use client';

import { memo, useEffect, useState } from 'react';
import { CircleCheck, CircleX, TriangleAlert } from 'lucide-react';
import { readPad } from '@/components/gamepad';
import type { CameraStatus } from '@/components/fpv-panel';
import {
  ARM_MIN_V,
  LINK_MIN_HZ,
  NO_PACK_V,
  STATUS_HZ,
  imuState,
  type ImuState,
  type Reading,
} from '@/lib/telemetry';

// Under LOW_V (3.7 V a cell on 4S) a flight is short.
const LOW_V = 14.8;
// The bridge drops the dashboard after 0.5 s without a status (src/fw/controller/bridge.py monitor).
const LINK_STALE_MS = 500;
// The GPS hold takes fixes to 8 m (fw/drone/main/flight_rt.c GPS_MAX_ACCURACY); a good outdoor fix is well inside GOOD_ACCURACY with GOOD_SATELLITES or more.
const HOLD_ACCURACY = 8;
const GOOD_ACCURACY = 2;
const GOOD_SATELLITES = 8;
// Flying free on the GPS hold, the settings the 2026-11-09 fly-aways are fixed with (src/sw/sim/models/NOTES.md): each with the value to set and whether a value will do. The thrust direction and the damping start at the indoor values in new firmware; the heading search and the return speed start on.
const OUTDOOR: [
  (
    | 'hover_thrust_direction'
    | 'hover_kd'
    | 'hover_gps_calibrate'
    | 'hover_return_speed'
  ),
  number,
  (value: number) => boolean,
][] = [
  ['hover_thrust_direction', 0.8, (value) => value >= 0.5],
  ['hover_kd', 1.5, (value) => value <= 1.5],
  ['hover_gps_calibrate', 5, (value) => value > 0],
  ['hover_return_speed', 1, (value) => value > 0 && value <= 1.5],
];

type Level = 'go' | 'caution' | 'stop';
// A check's call and its reason.
type Verdict = [Level, string];
type Check = { label: string; level: Level; reason: string };

const LEVELS = {
  go: { icon: CircleCheck, color: 'text-green-700' },
  caution: { icon: TriangleAlert, color: 'text-amber-600' },
  stop: { icon: CircleX, color: 'text-red-700' },
};
const IMU: Record<ImuState, Verdict> = {
  offline: ['stop', 'Offline'],
  calibrating: ['caution', 'Calibrating'],
  calibrated: ['go', 'Live and calibrated'],
  uncalibrated: ['stop', 'Not calibrated'],
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
  const link: Verdict = !connected
    ? ['stop', 'Not connected']
    : age > LINK_STALE_MS
      ? ['stop', `No status for ${(age / 1000).toFixed(1)} s`]
      : fps !== null && fps < LINK_MIN_HZ
        ? [
            'caution',
            `Status at ${Math.round(fps)} Hz of ${STATUS_HZ}: packets lost`,
          ]
        : ['go', 'Fresh'];
  const rows: [string, Verdict][] = [['Drone link', link]];
  if (connected && reading) {
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
    const unset = OUTDOOR.filter(([key, , fine]) => {
      const value = control?.[key];
      return value === undefined || !fine(value);
    });

    rows.push(
      ['IMU', IMU[imuState(reading.imu)]],
      [
        'Barometer',
        !baro
          ? [baroMiss, 'No data from this firmware']
          : !baro.found
            ? [baroMiss, 'Not found']
            : !baro.fresh || baro.altitude === null
              ? [baroMiss, 'Found, readings stale']
              : baroUsed
                ? ['go', `Fresh, ${baro.altitude.toFixed(1)} m`]
                : [
                    'caution',
                    'Fresh, but the hover does not use it (hover_baro_tau 0)',
                  ],
      ],
      [
        'GPS',
        !gps || !gps.configured
          ? ['stop', 'Module not found']
          : !live
            ? ['stop', 'No fresh solution']
            : !gps.ok || gps.fix !== 3
              ? ['stop', `No 3D fix (${gps.satellites} satellites)`]
              : gps.horizontal_accuracy > HOLD_ACCURACY
                ? [
                    'stop',
                    `3D fix but ${gps.horizontal_accuracy.toFixed(1)} m: the hold ignores it`,
                  ]
                : [
                    gps.horizontal_accuracy <= GOOD_ACCURACY &&
                    gps.satellites >= GOOD_SATELLITES
                      ? 'go'
                      : 'caution',
                    `3D fix, ${gps.satellites} satellites, ${gps.horizontal_accuracy.toFixed(1)} m`,
                  ],
      ],
      [
        'GPS hold',
        hold === undefined
          ? ['caution', 'Not in this firmware']
          : hold > 0
            ? ['go', `On (${hold} s)`]
            : [free ? 'stop' : 'caution', 'Off (hover_gps_tau 0)'],
      ]
    );
    if (free && hold)
      rows.push([
        'Outdoor hold',
        !unset.length
          ? ['go', 'Thrust direction, heading search and return speed set']
          : [
              'caution',
              unset.some(([key]) => control?.[key] === undefined)
                ? 'Not in this firmware'
                : `Set ${unset.map(([key, value]) => `${key} ${value}`).join(', ')}`,
            ],
      ]);
    rows.push(
      [
        'Battery',
        volts === undefined
          ? ['stop', 'No reading']
          : volts <= NO_PACK_V
            ? ['stop', 'No pack: USB power']
            : volts < ARM_MIN_V
              ? [
                  'stop',
                  `${volts.toFixed(1)} V, under the ${ARM_MIN_V} V arm minimum`,
                ]
              : volts < LOW_V
                ? [
                    'caution',
                    `${volts.toFixed(1)} V, close to the ${ARM_MIN_V} V arm minimum`,
                  ]
                : ['go', `${volts.toFixed(1)} V`],
      ],
      [
        'Recorder',
        recorder?.error
          ? ['stop', recorder.error]
          : reading.logging?.active
            ? ['go', 'Recording']
            : reading.logging?.ready
              ? ['go', 'Ready']
              : ['stop', 'Not ready'],
      ],
      [
        'Gamepad',
        pad ? ['go', pad] : ['caution', 'Not connected: keyboard only'],
      ],
      [
        'Camera',
        // The camera's Wi-Fi sits beside the GPS antenna: streaming may jam the GPS.
        free
          ? !camera
            ? ['go', 'Not answering (off)']
            : [
                'caution',
                `${camera.state === 'fpv' ? 'FPV streaming' : `On (${camera.state})`}: may jam the GPS`,
              ]
          : !camera
            ? ['stop', 'Not answering: the tag hover needs it']
            : camera.state === 'running'
              ? ['go', 'Tracking']
              : camera.state === 'fpv'
                ? ['caution', 'FPV streaming: no tag tracking, may jam the GPS']
                : ['stop', `Camera ${camera.state}`],
      ]
    );
  }
  return rows.map(([label, [level, reason]]) => ({ label, level, reason }));
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
