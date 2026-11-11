'use client';

import { Activity, LoaderCircle, Unplug } from 'lucide-react';
import { memo, useRef, type ReactNode } from 'react';
import { useSearchParams } from 'next/navigation';
import { signed } from '@/lib/format';
import { Button } from '@/components/ui/button';
import { AttitudeChart } from '@/components/attitude-chart';
import { useChartEnd } from '@/components/fast-chart';
import { ImuAxisChart } from '@/components/imu-axis-chart';
import { Panel } from '@/components/panel';
import { Status } from '@/components/status';
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
      <SelectTrigger aria-label="IMU history range" className="w-44">
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
  return (
    <>
      <Status
        role="status"
        icon={online ? Activity : Unplug}
        className={online ? 'text-green-700' : 'text-red-700'}
      >
        {online ? 'Online' : 'Offline'}
      </Status>
      <div className="ml-auto flex items-center gap-2">
        {busy && (
          <LoaderCircle
            className="size-4 animate-spin text-muted-foreground"
            aria-label="Calibrating"
          />
        )}
        <Button
          variant="outline"
          disabled={!online || busy}
          title="Keep the IMU still for two seconds"
          onClick={onCalibrate}
        >
          Calibrate
        </Button>
      </div>
    </>
  );
});

// A chart under its name and its series' latest readings, each in its line's color.
function Readings({
  label,
  children,
  chart,
}: {
  label: string;
  children: ReactNode;
  chart: ReactNode;
}) {
  return (
    <div className="space-y-2">
      <div className="flex items-baseline gap-4 text-sm whitespace-nowrap tabular-nums">
        <h3 className="text-muted-foreground">{label}</h3>
        {children}
      </div>
      {chart}
    </div>
  );
}

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
  const seconds = ranges.some(({ value }) => Number(value) === range)
    ? range
    : 15;
  const end = useChartEnd(connected, time);
  const online = connected && reading != null;

  // The three charts' value axes take the width of the widest one's labels.
  const gutter = useRef(48);

  // Each value keeps the width of its widest reading, so the labels beside it don't move as it changes: in tabular nums every digit is 1ch, and the sign and point together about another. With nothing to show, a dash right after its label.
  const value = (number: number | undefined, digits: number, whole: number) =>
    online && number !== undefined ? (
      <span
        className="inline-block text-right"
        style={{ width: `${whole + digits + 1}ch` }}
      >
        {signed(number, digits)}
      </span>
    ) : (
      '—'
    );
  return (
    <Panel
      title="IMU"
      header={
        <ImuHeader
          online={online}
          calibrating={reading?.calibrating ?? false}
          pending={calibrationPending}
          onCalibrate={onCalibrate}
        />
      }
    >
      <Readings
        label="Attitude"
        chart={
          <AttitudeChart
            history={history}
            end={end}
            gutter={gutter}
            className="h-28"
          />
        }
      >
        <span className="text-red-700">
          Roll {value(reading?.roll, 1, 3)}
          {online && '°'}
        </span>
        <span className="text-green-700">
          Pitch {value(reading?.pitch, 1, 2)}
          {online && '°'}
        </span>
        <span className="ml-auto text-muted-foreground">
          {online ? `${reading.temperature.toFixed(1)} °C` : ''}
        </span>
      </Readings>
      {(
        [
          ['Acceleration', 'g', 'accel', 3, 1],
          ['Gyro', '°/s', 'gyro', 2, 3],
        ] as const
      ).map(([label, unit, key, digits, whole]) => (
        <Readings
          key={key}
          label={`${label} (${unit})`}
          chart={
            <ImuAxisChart
              history={history}
              field={key}
              start={end - seconds * 1000}
              end={end}
              digits={digits}
              gutter={gutter}
              className="h-28"
            />
          }
        >
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
        </Readings>
      ))}
      <ImuRange seconds={seconds} />
    </Panel>
  );
}
