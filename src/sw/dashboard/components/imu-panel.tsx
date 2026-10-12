'use client';

import { Activity, Unplug } from 'lucide-react';

export type ImuReading = {
  accel: [number, number, number];
  gyro: [number, number, number];
  temperature: number;
};

export function ImuPanel({
  reading,
  connected,
}: {
  reading: ImuReading | undefined;
  connected: boolean;
}) {
  const online = connected && reading !== undefined;
  const StatusIcon = online ? Activity : Unplug;

  return (
    <section
      aria-labelledby="imu-heading"
      className="w-full max-w-md space-y-6 px-6 py-8"
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
      {(
        [
          ['Acceleration', 'g', 'accel', 3],
          ['Angular velocity', '°/s', 'gyro', 2],
        ] as const
      ).map(([label, unit, key, digits]) => (
        <div key={key} className="space-y-3">
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
      <div>
        <p className="text-sm text-muted-foreground">Temperature</p>
        <p className="mt-1 text-xl font-medium tabular-nums">
          {online ? `${reading.temperature.toFixed(1)} °C` : '—'}
        </p>
      </div>
    </section>
  );
}
