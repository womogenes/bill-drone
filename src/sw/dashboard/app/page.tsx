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
import type { PreviewTag } from '@/components/fpv-panel';
import { Button } from '@/components/ui/button';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { GpsPanel } from '@/components/gps-panel';
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
// As far as the drone takes it (fw/drone/main/flight_rt.c NUDGE_MAX).
const NUDGE_MAX = 1.5;
// A steady rotor tilt beyond this (degrees, averaged over TILT_TAU_S seconds) means the CoM is well off centre or moving: by about 15-20 degrees the drone has no pitch authority left.
const TILT_WARNING = 10;
const TILT_TAU_S = 1;

// Updates the steady rotor tilt from a reading and returns it when it is beyond TILT_WARNING, else null; disarmed or without thrust it starts over.
// The tag relative to the drone (forward, left, up; m) from a preview sighting in the camera frame (right, down, forward), through the camera's mounting (hover_cam_roll, hover_cam_pitch; degrees) as fw/drone/main/hover.c level() turns it, taking the drone as level, as it is on the ground.
function cameraToBody(
  [right0, down0, ahead]: [number, number, number],
  rollDeg: number,
  pitchDeg: number
): [number, number, number] {
  const roll = (rollDeg * Math.PI) / 180;
  const pitch = (pitchDeg * Math.PI) / 180;
  const right = Math.cos(roll) * right0 - Math.sin(roll) * down0;
  const down = Math.sin(roll) * right0 + Math.cos(roll) * down0;
  return [
    down * Math.sin(pitch) + ahead * Math.cos(pitch),
    -right,
    ahead * Math.sin(pitch) - down * Math.cos(pitch),
  ];
}

const noHistory = () => [];
const EMPTY_SAMPLE: HoverSample = {
  elapsed_us: 0,
  mode: 0,
  tracking: false,
  tag: null,
  tag_age_ms: null,
  velocity: [0, 0],
  height: 0,
  height_target: 0,
  range_target: null,
  arc_deg: null,
  time: 0,
  session: 0,
};

// A pack under ARM_MIN_V, or one a disarmed drone is running down faster than DRAIN_V_PER_MIN (from the once-a-second history), else null.
function batteryWarningFor(
  reading: Reading | null,
  hourHistory: (start: number, end: number) => VoltageSample[]
) {
  const volts = reading?.voltage_ema;
  if (volts === undefined || volts <= NO_PACK_V) return null;
  if (volts < ARM_MIN_V)
    return `Battery at ${volts.toFixed(1)} V, under ${ARM_MIN_V} V: it will not arm. Unplug it and charge or swap it.`;
  if (reading?.control?.enabled) return null;
  const now = Date.now();
  const minute = hourHistory(now - 60_000, now);
  const drain =
    minute.length > 30 ? minute[0].ema - minute[minute.length - 1].ema : 0;
  return drain >= DRAIN_V_PER_MIN
    ? `Battery falling ${drain.toFixed(1)} V a minute while disarmed. Unplug it between tests.`
    : null;
}

function tiltWarningFor(
  reading: Reading | null,
  steady: { current: { time: number; tilt: number } | null }
) {
  const control = reading?.control;
  const actuators = reading?.actuators;
  if (!reading || !control?.enabled || !actuators || !actuators.left.throttle) {
    steady.current = null;
    return null;
  }
  // Left tilts forward with positive angles, right with negative (sw/sim/flight.py MODEL servo directions).
  const tilt =
    (actuators.left.servo_angle -
      control.left_trim -
      (actuators.right.servo_angle - control.right_trim)) /
    2;
  const last = steady.current;
  const dt = last ? Math.max(0, (reading.time - last.time) / 1000) : 0;
  const mean = last
    ? last.tilt + (tilt - last.tilt) * (1 - Math.exp(-dt / TILT_TAU_S))
    : tilt;
  steady.current = { time: reading.time, tilt: mean };
  return Math.abs(mean) > TILT_WARNING ? mean : null;
}
// Battery voltage EMA time constant; time-based so irregular packet timing doesn't change the smoothing.
const VOLTAGE_TAU = 1500;
// The drone refuses to arm under ARM_MIN_V (fw/drone/main/flight_rt.c); NO_PACK_V and below is no pack at all. A disarmed pack falling faster than DRAIN_V_PER_MIN is being run down on the bench: on 2026-11-02 a half hour of hang tests took a 4S pack from 14.5 V to 9.9 V.
const ARM_MIN_V = 14;
const NO_PACK_V = 5;
const DRAIN_V_PER_MIN = 0.3;

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
  // The drone's refusal of the last ESC calibration stage, if any.
  const [escError, setEscError] = useState<string | null>(null);
  const latestReading = useRef<Reading | null>(null);
  // The rotors' steady forward tilt (degrees, positive forward) over the last second of an armed flight: holding the nose against a CoM off centre (negative: forward of it) also points the thrust, and a growing one is a battery sliding (2026-10-31).
  const steadyTilt = useRef<{ time: number; tilt: number } | null>(null);
  const [tiltWarning, setTiltWarning] = useState<number | null>(null);
  const [batteryWarning, setBatteryWarning] = useState<string | null>(null);
  // Bench: the servo and ESC panels, for setup and tests; flight leaves them out.
  const [bench, setBench] = useState(false);
  const [hoverState, setHoverState] = useState<HoverSample | null>(null);
  // The camera's latest tag-mode preview frame's tag (null: no tag in it; undefined: no preview yet), for the map while disarmed.
  const [previewTag, setPreviewTag] = useState<PreviewTag | null>();
  const showPreviewTag = useCallback(
    (tag: PreviewTag | null) => setPreviewTag(tag),
    []
  );
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
      const warning = tiltWarningFor(current, steadyTilt);
      const battery = batteryWarningFor(current, voltageHourHistory.read);
      startTransition(() => {
        setReading(current);
        setHoverState(hover);
        setTiltWarning(warning);
        setBatteryWarning(battery);
      });
    });
  }, [voltageHourHistory]);
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
          if (message.request === 'esc') setEscError(message.error);
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
  const control = reading?.control;
  const cameraSample: HoverSample | null =
    previewTag === undefined || !control
      ? null
      : {
          elapsed_us: 0,
          mode: 0,
          tracking: false,
          tag: previewTag
            ? cameraToBody(
                previewTag.position,
                control.hover_cam_roll ?? 0,
                control.hover_cam_pitch ?? 0
              )
            : null,
          tag_age_ms: previewTag ? 0 : null,
          velocity: [0, 0],
          height: 0,
          height_target: 0,
          range_target: control.hover_distance ?? null,
          arc_deg: null,
          time: reading?.time ?? 0,
          session: 0,
        };
  const mapState =
    (armed && hoverState) || cameraSample || hoverState || EMPTY_SAMPLE;
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

  const volts = reading?.voltage_ema;
  const imuState = !reading?.imu
    ? 'IMU offline'
    : reading.imu.calibrating
      ? 'IMU calibrating'
      : reading.imu.calibrated
        ? 'IMU ready'
        : 'IMU calibration';
  const gps = reading?.gps;
  const recorderText =
    recorder?.error ??
    (reading?.logging?.active
      ? `Recording · ${reading.logging.samples} samples · ${reading.logging.dropped} dropped`
      : reading?.logging?.ready
        ? recorder?.file
          ? `Saved ${recorder.file}`
          : 'Recorder ready'
        : 'Recorder connecting');
  const connected = status === 'connected';
  return (
    <div className="flex flex-col lg:h-dvh">
      <header className="flex h-16 shrink-0 items-center justify-between border-b px-6">
        <div className="flex items-center gap-4">
          <h1 className="text-lg font-semibold">Drone dashboard</h1>
          <div className="flex gap-1" role="group" aria-label="View">
            {([false, true] as const).map((choice) => (
              <Button
                key={String(choice)}
                size="sm"
                variant={bench === choice ? 'default' : 'outline'}
                aria-pressed={bench === choice}
                onClick={() => setBench(choice)}
              >
                {choice ? 'Bench' : 'Fly'}
              </Button>
            ))}
          </div>
        </div>
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
      {/* What decides whether the drone can fly, in one line; warnings in red. */}
      <div className="flex shrink-0 flex-wrap items-center gap-x-6 gap-y-1 border-b px-6 py-2 text-sm tabular-nums">
        <span
          className={
            connected &&
            volts !== undefined &&
            volts > NO_PACK_V &&
            volts < ARM_MIN_V
              ? 'text-red-700'
              : 'text-muted-foreground'
          }
        >
          {!connected || volts === undefined
            ? 'Battery —'
            : volts <= NO_PACK_V
              ? 'No pack (USB)'
              : `Battery ${volts.toFixed(1)} V`}
        </span>
        <span
          className={
            connected &&
            imuState !== 'IMU ready' &&
            imuState !== 'IMU calibrating'
              ? 'text-red-700'
              : 'text-muted-foreground'
          }
        >
          {connected ? imuState : 'IMU —'}
        </span>
        <span className="text-muted-foreground">
          {!connected || !gps
            ? 'GPS —'
            : !gps.configured
              ? 'GPS searching'
              : gps.ok && gps.fix >= 2 && gps.fix <= 4
                ? `GPS ${gps.fix === 2 ? '2D' : '3D'} · ${gps.satellites} satellites · ${gps.horizontal_accuracy.toFixed(1)} m`
                : 'GPS no fix'}
        </span>
        <span
          className={
            recorder?.error || (reading?.logging?.dropped ?? 0) > 0
              ? 'text-red-700'
              : 'text-muted-foreground'
          }
        >
          {recorderText}
        </span>
        {batteryWarning && (
          <span className="font-medium text-red-700">{batteryWarning}</span>
        )}
        {tiltWarning !== null && (
          <span className="font-medium text-red-700">
            Rotors held {Math.abs(tiltWarning).toFixed(0)}°{' '}
            {tiltWarning < 0
              ? 'back: CoM forward of centre'
              : 'forward: CoM behind centre'}{' '}
            or moving. Land and check the battery.
          </span>
        )}
      </div>
      <main
        className={`grid divide-y divide-border lg:min-h-0 lg:flex-1 lg:divide-x lg:divide-y-0 lg:overflow-hidden ${bench ? 'lg:grid-cols-4' : 'lg:grid-cols-3'}`}
      >
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
            ema={reading?.voltage_ema}
            history={voltageHistory.read}
            hourHistory={voltageHourHistory.read}
            time={reading?.time ?? 0}
            connected={status === 'connected'}
          />
          <GpsPanel gps={reading?.gps} connected={status === 'connected'} />
        </div>
        {bench && (
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
                        Math.round(reading.actuators[side].servo_angle * 10) /
                          10
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
        )}
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
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
              escError={escError}
              onEscStage={(stage) => {
                setEscError(null);
                request({ type: 'esc', stage });
              }}
            />
          )}
          {/* Always shown, so the layout stays put: armed, the flight's map; else the camera's latest sighting, the last flight's map, or an empty one. */}
          <HoverPanel
            state={mapState}
            history={mapState === hoverState ? hoverHistory.read : noHistory}
          />
        </div>
        <div className="min-h-0 min-w-0 lg:overflow-y-auto lg:overscroll-contain">
          <FpvPanel
            free={
              reading?.control ? reading.control.hover_camera === 0 : undefined
            }
            onFree={(free) => sendControl({ hover_camera: free ? 0 : 1 })}
            roll={reading?.control?.hover_cam_roll}
            onTag={showPreviewTag}
          />
        </div>
      </main>
    </div>
  );
}
