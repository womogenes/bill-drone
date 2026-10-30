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
import dynamic from 'next/dynamic';
import Link from 'next/link';

import { ControlPanel } from '@/components/control-panel';
import { HoverPanel } from '@/components/hover-panel';
import { Button } from '@/components/ui/button';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { ImuPanel } from '@/components/imu-panel';
import { EspConnection, type DroneRequest } from '@/lib/esp-connection';
import { createHistory } from '@/lib/history';
import type {
  HoverSample,
  ImuSample,
  Reading,
  Side,
  ControlSettings,
  Targets,
  VoltageSample,
} from '@/lib/telemetry';

const FpvPanel = dynamic(
  () => import('@/components/fpv-panel').then((module) => module.FpvPanel),
  { ssr: false }
);

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
  mode: 0,
  nudge: [0, 0, 0],
};
const NO_NUDGE: [number, number, number] = [0, 0, 0];
// The drone accepts 1.5 m each way; the keys stop short of that.
const NUDGE_MAX = 1;
// A hover message this recent means a hold is being flown and recorded.
const HOVER_FRESH_MS = 3000;
// Battery voltage EMA time constant; time-based so irregular packet timing doesn't change the smoothing.
const VOLTAGE_TAU = 1500;

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
  const [hoverState, setHoverState] = useState<HoverSample | null>(null);
  const latestHover = useRef<HoverSample | null>(null);
  const [hoverHistory] = useState(() =>
    createHistory<HoverSample>(20_000, 1000)
  );
  const [history] = useState(() => createHistory<ImuSample>());
  const [voltageHistory] = useState(() => createHistory<VoltageSample>());
  // One sample a second for the hour view, so discharge over a session fits in a small buffer.
  const [voltageHourHistory] = useState(() =>
    createHistory<VoltageSample>(3_600_000, 3700)
  );
  const displayFrame = useRef<number | undefined>(undefined);
  const publishReading = useCallback(() => {
    if (displayFrame.current !== undefined) return;
    displayFrame.current = requestAnimationFrame(() => {
      displayFrame.current = undefined;
      const current = latestReading.current;
      const hover = latestHover.current;
      startTransition(() => {
        setReading(current);
        setHoverState(hover);
      });
    });
  }, []);
  const [stopEpoch, setStopEpoch] = useState(0);
  const [fps, setFps] = useState<number | null>(null);
  const packetCount = useRef(0);
  const socket = useRef<EspConnection | null>(null);
  const targets = useRef<Targets>(ZERO_TARGETS);
  const [mode, setModeState] = useState(0);
  const [nudge, setNudgeState] = useState(NO_NUDGE);
  // H while disarmed arms first and enters hover only once the arm request succeeds.
  const hoverPending = useRef(false);

  const setTargets = useCallback((update: Partial<Targets>) => {
    // Hover entered from manual holds where the drone is, so a nudge never carries over between hops.
    if (update.mode === 1 && targets.current.mode === 0)
      update = { ...update, nudge: NO_NUDGE };
    targets.current = { ...targets.current, ...update };
    socket.current?.targets(targets.current);
    if (update.mode !== undefined) setModeState(update.mode);
    if (update.nudge !== undefined) setNudgeState(update.nudge);
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
    setModeState(0);
    setNudgeState(NO_NUDGE);
    hoverPending.current = false;
    setStatus('connecting');
    setReading(null);
    setRequestError('');
    setCalibrationPending(false);
    latestReading.current = null;
    latestHover.current = null;
    setHoverState(null);
    const session = Date.now();
    let lastVoltage: VoltageSample | undefined;
    let lastHourVoltage: VoltageSample | undefined;
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
          if (message.request === 'arm' && hoverPending.current) {
            hoverPending.current = false;
            if (!message.error) setTargets({ mode: 1 });
          }
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
        if (message.type === 'hover') {
          latestHover.current = { ...message, time: Date.now(), session };
          hoverHistory.append(latestHover.current);
          publishReading();
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
          data.control_status
        );
        const time = Date.now();
        if (data.adc_voltage !== undefined) {
          const dt = lastVoltage ? time - lastVoltage.time : Infinity;
          const ema =
            dt > 1000
              ? data.adc_voltage
              : lastVoltage!.ema +
                (1 - Math.exp(-dt / VOLTAGE_TAU)) *
                  (data.adc_voltage - lastVoltage!.ema);
          lastVoltage = { time, session, voltage: data.adc_voltage, ema };
          voltageHistory.append(lastVoltage);
          if (!lastHourVoltage || time - lastHourVoltage.time >= 1000) {
            lastHourVoltage = lastVoltage;
            voltageHourHistory.append(lastVoltage);
          }
        }
        latestReading.current = {
          ...data,
          time,
          voltage_ema: lastVoltage?.ema,
        };
        publishReading();
        if (data.imu == null) return;
        history.append({
          ...data.imu,
          time,
          session,
          // The status carries the laptop's setpoints, which hover and landing ignore, so they are plotted in manual only.
          target:
            data.control?.enabled && targets.current.mode === 0
              ? [data.control.roll, data.control.pitch]
              : undefined,
        });
      },
      offline
    );
    socket.current = ws;
  }, [
    history,
    voltageHistory,
    voltageHourHistory,
    hoverHistory,
    publishReading,
    setTargets,
  ]);

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
    hoverPending.current = false;
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

  const setMode = useCallback(
    (mode: number) => setTargets({ mode }),
    [setTargets]
  );

  // Arming needs zero base throttle, as with Enable, so hover starts from the ground with base throttle 0.
  const hover = useCallback(() => {
    if (latestReading.current?.control?.enabled) {
      setTargets({ mode: 1 });
      return;
    }
    hoverPending.current = true;
    sendControl({
      enabled: true,
      base_throttle: 0,
      roll: 0,
      pitch: 0,
      yaw_rate: 0,
    });
  }, [sendControl, setTargets]);

  // A drone that disarms itself drops back to manual, so a later Enable never arms straight into hover.
  const armed = reading?.control?.enabled;
  useEffect(() => {
    if (armed === false && targets.current.mode !== 0) setTargets({ mode: 0 });
  }, [armed, setTargets]);
  // Arming or disarming starts a new hop from the unnudged hold.
  useEffect(() => {
    if (targets.current.nudge.some(Boolean)) setTargets({ nudge: NO_NUDGE });
  }, [armed, setTargets]);

  const nudgeHold = useCallback(
    (forward: number, left: number, up: number) => {
      const step = (value: number, change: number) =>
        Math.round(
          Math.max(-NUDGE_MAX, Math.min(NUDGE_MAX, value + change)) * 1000
        ) / 1000;
      const [x, y, z] = targets.current.nudge;
      setTargets({ nudge: [step(x, forward), step(y, left), step(z, up)] });
    },
    [setTargets]
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
  const controlStatus = useMemo(() => ({ reason }), [reason]);

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
          <FpvPanel />
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
            ema={reading?.voltage_ema}
            history={voltageHistory.read}
            hourHistory={voltageHourHistory.read}
            time={reading?.time ?? 0}
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
              mode={mode}
              nudge={nudge}
              onChange={sendControl}
              onMode={setMode}
              onHover={hover}
              onNudge={nudgeHold}
              onStop={stopMotors}
            />
          )}
          {hoverState &&
            (reading?.time ?? 0) - hoverState.time < HOVER_FRESH_MS && (
              <HoverPanel state={hoverState} history={hoverHistory.read} />
            )}
        </div>
      </main>
    </>
  );
}
