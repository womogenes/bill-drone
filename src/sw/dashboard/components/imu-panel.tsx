'use client';

import { Activity, LoaderCircle, Unplug } from 'lucide-react';
import { memo, useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import { signed } from '@/lib/format';
import { Button } from '@/components/ui/button';
import { AttitudeChart } from '@/components/attitude-chart';
import { ImuAxisChart } from '@/components/imu-axis-chart';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import type { ImuReading, ImuSample } from '@/lib/telemetry';

const ranges = [
  { value: '5', label: 'Last 5 seconds' },
  { value: '15', label: 'Last 15 seconds' },
  { value: '60', label: 'Last minute' },
];

const ImuRange = memo(function ImuRange({ seconds }: { seconds: number }) {
  return (
    <Select
      value={String(seconds)}
      items={ranges}
      onValueChange={(value) => {
        const url = new URL(window.location.href);
        url.searchParams.set('imuRange', String(value));
        window.history.replaceState(null, '', url);
      }}
    >
      <SelectTrigger
        aria-label="IMU history range"
        className="w-44 cursor-pointer"
      >
        <SelectValue />
      </SelectTrigger>
      <SelectContent align="start" alignItemWithTrigger={false}>
        <SelectGroup>
          {ranges.map((range) => (
            <SelectItem key={range.value} value={range.value}>
              {range.label}
            </SelectItem>
          ))}
        </SelectGroup>
      </SelectContent>
    </Select>
  );
});

const ImuHeader = memo(function ImuHeader({
  online,
  calibrating,
  pending,
  onCalibrate,
}: {
  online: boolean;
  calibrating: boolean;
  pending: boolean;
  onCalibrate: () => void;
}) {
  const busy = online && (pending || calibrating);
  const StatusIcon = online ? Activity : Unplug;
  return (
    <div className="flex items-center gap-4">
      <h2 id="imu-heading" className="font-semibold">
        IMU
      </h2>
      <span
        role="status"
        className={`flex items-center gap-2 ${online ? 'text-green-700' : 'text-red-700'}`}
      >
        <StatusIcon className="size-4" aria-hidden="true" />
        {online ? 'Online' : 'Offline'}
      </span>
      <div className="ml-auto flex items-center gap-2">
        {busy && (
          <LoaderCircle
            className="size-4 animate-spin text-muted-foreground"
            aria-label="Calibrating"
          />
        )}
        <Button
          variant="outline"
          className="cursor-pointer"
          disabled={!online || busy}
          title="Keep the IMU still for two seconds"
          onClick={onCalibrate}
        >
          Calibrate
        </Button>
      </div>
    </div>
  );
});

export function ImuPanel({
  reading,
  connected,
  history,
  time,
  onCalibrate,
  calibrationPending,
}: {
  reading: ImuReading | null | undefined;
  connected: boolean;
  history: (start: number, end: number) => ImuSample[];
  time: number;
  onCalibrate: () => void;
  calibrationPending: boolean;
}) {
  const searchParams = useSearchParams();
  const range = Number(searchParams.get('imuRange'));
  const seconds = [5, 15, 60].includes(range) ? range : 15;
  const [now, setNow] = useState(0);
  useEffect(() => {
    if (connected) return;
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, [connected]);
  const end = Math.max(now, time);
  const online = connected && reading != null;

  // Each value keeps the width of its widest reading, so the labels beside it don't move as it changes: in tabular nums every digit is 1ch, and the sign and point together about another.
  const value = (number: number | undefined, digits: number, whole: number) => (
    <span
      className="inline-block text-right"
      style={{ width: `${whole + digits + 1}ch` }}
    >
      {online && number !== undefined ? signed(number, digits) : '—'}
    </span>
  );
  return (
    <section aria-labelledby="imu-heading" className="space-y-3 px-4 py-4">
      <ImuHeader
        online={online}
        calibrating={reading?.calibrating ?? false}
        pending={calibrationPending}
        onCalibrate={onCalibrate}
      />
      <div className="space-y-1">
        <div className="flex items-baseline gap-4 tabular-nums">
          <h3 className="text-sm whitespace-nowrap text-muted-foreground">
            Attitude
          </h3>
          <span className="text-red-700">
            Roll {value(reading?.roll, 1, 3)}°
          </span>
          <span className="text-green-700">
            Pitch {value(reading?.pitch, 1, 2)}°
          </span>
          <span className="ml-auto text-sm text-muted-foreground">
            {online ? `${reading.temperature.toFixed(1)} °C` : ''}
          </span>
        </div>
        <AttitudeChart history={history} end={end} className="h-32" />
      </div>
      {(
        [
          ['Acceleration', 'g', 'accel', 3, 1],
          ['Gyro', '°/s', 'gyro', 2, 3],
        ] as const
      ).map(([label, unit, key, digits, whole]) => (
        <div key={key} className="space-y-1">
          <div className="flex items-baseline gap-4 tabular-nums">
            <h3 className="text-sm whitespace-nowrap text-muted-foreground">
              {label} ({unit})
            </h3>
            {['X', 'Y', 'Z'].map((axis, index) => (
              <span
                key={axis}
                className={
                  ['text-red-700', 'text-green-700', 'text-blue-700'][index]
                }
              >
                {axis} {value(reading?.[key][index], digits, whole)}
              </span>
            ))}
          </div>
          <ImuAxisChart
            history={history}
            field={key}
            start={end - seconds * 1000}
            end={end}
            digits={digits}
            className="h-32"
          />
        </div>
      ))}
      <ImuRange seconds={seconds} />
    </section>
  );
}
