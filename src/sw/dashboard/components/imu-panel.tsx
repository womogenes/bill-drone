'use client';

import { Activity, Unplug } from 'lucide-react';
import type { ImuReading } from '@/lib/telemetry';

export function ImuPanel({
  reading,
  connected,
}: {
  reading: ImuReading | null | undefined;
  connected: boolean;
}) {
  const online = connected && reading != null;
  const StatusIcon = online ? Activity : Unplug;

  return (
    <section
      aria-labelledby="imu-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <div className="flex items-center gap-4">
        <h2 id="imu-heading" className="font-semibold">
          IMU
        </h2>
        <span
          role="status"
          className={`flex items-center gap-2 text-sm ${online ? 'text-green-700' : 'text-red-700'}`}
        >
          <StatusIcon className="size-4" aria-hidden="true" />
          {online ? 'Online' : 'Offline'}
        </span>
      </div>
      <div className="grid grid-cols-2 gap-4 tabular-nums">
        <div>
          <p className="text-sm text-muted-foreground">Roll</p>
          <p className="mt-1 text-xl font-medium">
            {online ? `${reading.roll.toFixed(1)}°` : '—'}
          </p>
        </div>
        <div>
          <p className="text-sm text-muted-foreground">Temperature</p>
          <p className="mt-1 text-xl font-medium">
            {online ? `${reading.temperature.toFixed(1)} °C` : '—'}
          </p>
        </div>
      </div>
      {(
        [
          ['Acceleration', 'g', 'accel', 3],
          ['Angular velocity', '°/s', 'gyro', 2],
        ] as const
      ).map(([label, unit, key, digits]) => (
        <div key={key} className="space-y-2">
          <h3 className="text-sm text-muted-foreground">
            {label} ({unit})
          </h3>
          <dl className="grid grid-cols-3 gap-4">
            {['X', 'Y', 'Z'].map((axis, index) => (
              <div key={axis}>
                <dt className="text-sm text-muted-foreground">{axis}</dt>
                <dd className="mt-1 text-xl font-medium tabular-nums">
                  {online ? reading[key][index].toFixed(digits) : '—'}
                </dd>
              </div>
            ))}
          </dl>
        </div>
      ))}
    </section>
  );
}
