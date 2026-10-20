'use client';

import {
  Suspense,
  startTransition,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import { LoaderCircle, RefreshCw, Wifi, WifiOff } from 'lucide-react';
import Link from 'next/link';

import { ControlPanel } from '@/components/control-panel';
import { Button } from '@/components/ui/button';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { ImuPanel } from '@/components/imu-panel';
import { EspConnection, type DroneRequest } from '@/lib/esp-connection';
import { createHistory } from '@/lib/history';
import type {
  ImuSample,
  Reading,
  Side,
  ControlSettings,
  Targets,
} from '@/lib/telemetry';

// The drone service (src/fw/controller/bridge.py) owns the controller USB link on this laptop.
const SERVICE = '127.0.0.1:8765';
const SETPOINTS = ['base_throttle', 'roll', 'pitch', 'yaw_rate'] as const;
const ZERO_TARGETS: Targets = {
  base_throttle: 0,
  roll: 0,
  pitch: 0,
  yaw_rate: 0,
  servo: [0, 0],
  throttle: [0, 0],
};

const statuses = {
  offline: {
    label: 'Offline',
    icon: WifiOff,
    color: 'text-red-700',
  },
  connecting: {
    label: 'Connecting',
    icon: LoaderCircle,
    color: 'text-yellow-600/70 dark:text-yellow-400/70',
  },
  connected: {
    label: 'Connected',
    icon: Wifi,
    color: 'text-green-700',
  },
};

function reuseUnchanged<T extends object>(
  previous: T | undefined,
  next: T | undefined
): T | undefined {
  if (
    previous &&
    next &&
    Object.keys(next).length === Object.keys(previous).length &&
    (Object.keys(next) as (keyof T)[]).every(
      (key) => next[key] === previous[key]
    )
  ) {
    return previous;
  }
  return next;
}

export default function Page() {
  const [status, setStatus] = useState<keyof typeof statuses>('offline');
  const [reading, setReading] = useState<Reading | null>(null);
  const [requestError, setRequestError] = useState('');
  const [calibrationPending, setCalibrationPending] = useState(false);
  const latestReading = useRef<Reading | null>(null);
  const [history] = useState(() => createHistory<ImuSample>());
  const displayFrame = useRef<number | undefined>(undefined);
  const publishReading = useCallback(() => {
    if (displayFrame.current !== undefined) return;
    displayFrame.current = requestAnimationFrame(() => {
      displayFrame.current = undefined;
      const current = latestReading.current;
      startTransition(() => {
        setReading(current);
      });
    });
  }, []);
  const [stopEpoch, setStopEpoch] = useState(0);
  const [fps, setFps] = useState<number | null>(null);
  const packetCount = useRef(0);
  const socket = useRef<EspConnection | null>(null);
  const targets = useRef<Targets>(ZERO_TARGETS);

  const setTargets = useCallback((update: Partial<Targets>) => {
    targets.current = { ...targets.current, ...update };
    socket.current?.targets(targets.current);
  }, []);

  const request = useCallback((message: DroneRequest) => {
    socket.current?.send(message);
  }, []);

  const connect = useCallback(() => {
    if (displayFrame.current !== undefined)
      cancelAnimationFrame(displayFrame.current);
    displayFrame.current = undefined;
    socket.current?.close();
    socket.current = null;
    // A new connection starts from zero targets; arming needs an explicit request.
    targets.current = ZERO_TARGETS;
    setStatus('connecting');
    setReading(null);
    setRequestError('');
    setCalibrationPending(false);
    latestReading.current = null;
    const session = Date.now();
    function offline() {
      if (socket.current !== ws) return;
      socket.current = null;
      setStatus('offline');
      setCalibrationPending(false);
      ws.close();
    }

    const ws = new EspConnection(
      SERVICE,
      targets.current,
      () => {
        if (socket.current === ws) setStatus('connected');
      },
      (message) => {
        if (socket.current !== ws) return;
        if (message.type === 'result') {
          if (message.request === 'calibrate') setCalibrationPending(false);
          if (message.request === 'zero' && !message.error) {
            // The drone has captured the old angle and holds it until this zero target arrives.
            const servo = [...targets.current.servo] as [number, number];
            servo[message.side === 'left' ? 0 : 1] = 0;
            setTargets({ servo });
          }
          setRequestError(
            message.error ? `${message.request}: ${message.error}` : ''
          );
          return;
        }
        const data = message as unknown as Reading;
        packetCount.current++;
        data.control = reuseUnchanged(
          latestReading.current?.control,
          data.control
        );
        data.control_status = reuseUnchanged(
          latestReading.current?.control_status,
          data.control_status && {
            ...data.control_status,
            mapping_confirmed: true,
          }
        );
        const time = Date.now();
        latestReading.current = { ...data, time };
        publishReading();
        if (data.imu == null) return;
        history.append({ ...data.imu, time, session });
      },
      offline
    );
    socket.current = ws;
  }, [history, publishReading, setTargets]);

  useEffect(() => {
    const start = setTimeout(connect, 0);
    return () => {
      clearTimeout(start);
      if (displayFrame.current !== undefined)
        cancelAnimationFrame(displayFrame.current);
      displayFrame.current = undefined;
      socket.current?.close();
      socket.current = null;
    };
  }, [connect]);

  useEffect(() => {
    if (status !== 'connected') return;
    const marks = [{ time: performance.now(), count: packetCount.current }];
    const timer = setInterval(() => {
      const time = performance.now();
      marks.push({ time, count: packetCount.current });
      if (marks.length > 5) marks.shift();
      setFps(
        ((marks.at(-1)!.count - marks[0].count) * 1000) / (time - marks[0].time)
      );
    }, 500);
    return () => {
      clearInterval(timer);
      setFps(null);
    };
  }, [status]);

  useEffect(() => {
    if (status !== 'offline') return;
    const timer = setTimeout(connect, 1000);
    return () => clearTimeout(timer);
  }, [status, connect]);

  const calibrateImu = useCallback(() => {
    setCalibrationPending(true);
    request({ type: 'calibrate' });
  }, [request]);

  const stopMotors = useCallback(() => {
    setTargets(ZERO_TARGETS);
    setStopEpoch((previous) => previous + 1);
    request({ type: 'stop' });
  }, [request, setTargets]);

  // Setpoints become streamed targets, Enable an arm request, and everything else a tuning request.
  const sendControl = useCallback(
    (settings: Partial<ControlSettings>) => {
      const { enabled, ...rest } = settings;
      const setpoints: Partial<Targets> = {};
      const tuning: Record<string, number> = {};
      for (const [key, value] of Object.entries(rest)) {
        if ((SETPOINTS as readonly string[]).includes(key))
          setpoints[key as (typeof SETPOINTS)[number]] = Number(value);
        else tuning[key] = Number(value);
      }
      setTargets(setpoints);
      if (enabled === true) request({ type: 'arm' });
      if (enabled === false) stopMotors();
      if (Object.keys(tuning).length)
        request({ type: 'tune', settings: tuning });
    },
    [request, setTargets, stopMotors]
  );

  const sendActuator = useCallback(
    (command: 'servo' | 'throttle', side: Side, value: number) => {
      const values = [...targets.current[command]] as [number, number];
      values[side === 'left' ? 0 : 1] = value;
      setTargets({ [command]: values });
    },
    [setTargets]
  );

  const zeroServo = useCallback(
    (side: Side) => {
      request({ type: 'zero', side });
    },
    [request]
  );

  useEffect(() => {
    function stop(event: KeyboardEvent) {
      if (event.code !== 'Space') return;
      event.preventDefault();
      event.stopPropagation();
      if (event.repeat) return;
      stopMotors();
    }
    function close() {
      socket.current?.close();
    }
    function restore(event: PageTransitionEvent) {
      if (event.persisted) connect();
    }
    window.addEventListener('keydown', stop, true);
    window.addEventListener('pagehide', close);
    window.addEventListener('pageshow', restore);
    return () => {
      window.removeEventListener('keydown', stop, true);
      window.removeEventListener('pagehide', close);
      window.removeEventListener('pageshow', restore);
    };
  }, [connect, stopMotors]);

  const recorder = reading?.recorder;
  const reason = requestError || reading?.control_status?.reason || '';
  const mappingConfirmed = reading?.control_status?.mapping_confirmed ?? false;
  const controlStatus = useMemo(
    () => ({ reason, mapping_confirmed: mappingConfirmed }),
    [reason, mappingConfirmed]
  );

  const { label, icon: StatusIcon, color } = statuses[status];

  return (
    <>
      <header className="flex h-16 items-center justify-between border-b px-6">
        <h1 className="text-lg font-semibold">Drone dashboard</h1>
        <div className="flex items-center gap-3">
          <Link href="/camera" className="text-sm underline">
            Camera
          </Link>
          <span className="text-sm text-muted-foreground tabular-nums">
            {SERVICE}
          </span>
          {fps !== null && (
            <span className="text-sm text-muted-foreground tabular-nums">
              {Math.round(fps)} FPS
            </span>
          )}
          <span
            role="status"
            className={`flex items-center gap-2 text-sm ${color}`}
          >
            <StatusIcon
              aria-hidden="true"
              className={`size-4 ${status === 'connecting' ? 'animate-spin' : ''}`}
            />
            {label}
          </span>
          <Button
            variant="ghost"
            size="icon"
            aria-label="Refresh connection"
            title="Refresh connection"
            disabled={status === 'connecting'}
            onClick={connect}
          >
            <RefreshCw aria-hidden="true" />
          </Button>
        </div>
      </header>
      <main className="grid min-h-[calc(100dvh-4rem)] divide-y divide-border lg:h-[calc(100dvh-4rem)] lg:grid-cols-[repeat(3,minmax(0,28rem))] lg:divide-x lg:divide-y-0 lg:overflow-hidden">
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          <Suspense>
            <ImuPanel
              reading={reading?.imu}
              history={history.read}
              time={reading?.time ?? 0}
              onCalibrate={calibrateImu}
              calibrationPending={calibrationPending}
              connected={status === 'connected'}
            />
          </Suspense>
          <ADCPanel
            raw={reading?.adc_raw}
            voltage={reading?.adc_voltage}
            connected={status === 'connected'}
          />
        </div>
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          {(['left', 'right'] as const).flatMap((side) =>
            (['servo', 'throttle'] as const).map((kind) => (
              <ActuatorPanel
                key={`${side}-${kind}-${status}-${stopEpoch}-${reading?.control?.enabled}`}
                side={side}
                kind={kind}
                locked={reading?.control?.enabled}
                value={
                  kind === 'servo'
                    ? reading?.actuators &&
                      Math.round(reading.actuators[side].servo_angle * 10) / 10
                    : reading?.actuators?.[side].throttle
                }
                connected={status === 'connected'}
                onChange={sendActuator}
                onZero={zeroServo}
                zeroDisabled={
                  !!(
                    reading?.actuators?.left.throttle ||
                    reading?.actuators?.right.throttle
                  )
                }
              />
            ))
          )}
        </div>
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
          <p
            className={`px-4 pt-4 tabular-nums ${recorder?.error || (reading?.logging?.dropped ?? 0) > 0 ? 'text-red-700' : 'text-muted-foreground'}`}
          >
            {recorder?.error ??
              (reading?.logging?.active
                ? `Recording · ${reading.logging.samples} samples · ${reading.logging.dropped} dropped`
                : reading?.logging?.ready
                  ? recorder?.file
                    ? `Saved ${recorder.file}`
                    : 'Recorder ready'
                  : 'Recorder connecting')}
          </p>
          {reading?.control && (
            <ControlPanel
              key={`${status}-${stopEpoch}-${reading.control.enabled}`}
              settings={reading.control}
              recorderReady={!!reading.logging?.ready}
              recorderError={recorder?.error ?? undefined}
              status={controlStatus}
              connected={status === 'connected'}
              imu={
                !reading.imu
                  ? 'offline'
                  : reading.imu.calibrating
                    ? 'calibrating'
                    : reading.imu.calibrated
                      ? 'calibrated'
                      : 'uncalibrated'
              }
              onChange={sendControl}
              onStop={stopMotors}
            />
          )}
        </div>
      </main>
    </>
  );
}
