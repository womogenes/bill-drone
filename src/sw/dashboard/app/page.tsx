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
import {
  Activity,
  BatteryMedium,
  Disc,
  Eye,
  LoaderCircle,
  Plug,
  Radio,
  RefreshCw,
  Satellite,
  TriangleAlert,
  Wifi,
  WifiOff,
} from 'lucide-react';
import dynamic from 'next/dynamic';
import Link from 'next/link';

import { GamepadView } from '@/components/gamepad';
import { ControlPanel } from '@/components/control-panel';
import { HoverPanel } from '@/components/hover-panel';
import { Status } from '@/components/status';
import { Button } from '@/components/ui/button';
import { Tabs, TabsList, TabsTrigger } from '@/components/ui/tabs';
import { ActuatorPanel } from '@/components/actuator-panel';
import { ADCPanel } from '@/components/adc-panel';
import { GpsPanel } from '@/components/gps-panel';
import { ImuPanel } from '@/components/imu-panel';
import { EspConnection, type DroneRequest } from '@/lib/esp-connection';
import { gpsSolved } from '@/lib/gps';
import { createHistory } from '@/lib/history';
import type { PreviewTag } from '@/lib/multipart';
import {
  ARM_MIN_V,
  HOVER,
  LINK_MIN_HZ,
  MANUAL,
  NO_PACK_V,
  STATUS_HZ,
  imuState,
  rotorTilt,
  type ControlSettings,
  type GpsFix,
  type GpsSample,
  type HoverSample,
  type ImuSample,
  type ImuState,
  type Reading,
  type Side,
  type Targets,
  type VoltageSample,
} from '@/lib/telemetry';

const FpvPanel = dynamic(
  () => import('@/components/fpv-panel').then((module) => module.FpvPanel),
  { ssr: false }
);
// Leaflet needs the browser's window as it loads.
const GpsMap = dynamic(
  () => import('@/components/gps-map').then((module) => module.GpsMap),
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
  mode: MANUAL,
  nudge: [0, 0, 0],
};
const NO_NUDGE: [number, number, number] = [0, 0, 0];
// As far as the drone takes it (fw/drone/main/flight_rt.c NUDGE_MAX).
const NUDGE_MAX = 1.5;
// A steady rotor tilt beyond this (degrees, averaged over TILT_TAU_S seconds) means the CoM is well off center or moving: by about 15-20 degrees the drone has no pitch authority left.
const TILT_WARNING = 10;
const TILT_TAU_S = 1;
// Battery voltage EMA time constant; time-based so irregular packet timing doesn't change the smoothing.
const VOLTAGE_TAU = 1500;
// A disarmed pack falling faster than DRAIN_V_PER_MIN is being run down on the bench: on 2026-11-02 a half hour of hang tests took a 4S pack from 14.5 V to 9.9 V.
const DRAIN_V_PER_MIN = 0.3;
// A gap this long between statuses came before every link timeout on 2026-11-08.
const LINK_GAP_MS = 300;
const IMU_TEXT: Record<ImuState, string> = {
  offline: 'offline',
  calibrating: 'calibrating',
  calibrated: 'ready',
  uncalibrated: 'calibration',
};

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

// Updates the steady rotor tilt from a reading and returns it when it is beyond TILT_WARNING, else null; disarmed or without thrust it starts over.
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
  const tilt = rotorTilt(
    actuators.left.servo_angle,
    actuators.right.servo_angle,
    control.left_trim,
    control.right_trim
  );
  const last = steady.current;
  const dt = last ? Math.max(0, (reading.time - last.time) / 1000) : 0;
  const mean = last
    ? last.tilt + (tilt - last.tilt) * (1 - Math.exp(-dt / TILT_TAU_S))
    : tilt;
  steady.current = { time: reading.time, tilt: mean };
  return Math.abs(mean) > TILT_WARNING ? mean : null;
}

// An RSSI after a space, or nothing when no packet was heard.
function dbm(rssi: number | null | undefined) {
  return rssi === null || rssi === undefined ? '' : ` ${rssi} dBm`;
}

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
  // Another tab flies; this one watches (the service's /view) and takes over when that one closes.
  viewing: {
    label: 'View only: another tab has control',
    icon: Eye,
    color: 'text-muted-foreground',
  },
};

// What the service says of the chain (app/api/link/route.ts).
type Chain = {
  state: 'bridge_down' | 'controller_missing' | 'drone_silent' | 'online';
  heard_s?: number | null;
  in_use?: boolean;
  errors?: number;
};

// Why the page is offline, in the chain's own words.
function offlineText(chain: Chain | null) {
  if (chain?.state === 'bridge_down') return 'Drone service not running';
  if (chain?.state === 'controller_missing') return 'Controller not plugged in';
  if (chain?.state === 'drone_silent')
    return chain.heard_s == null
      ? 'Drone not answering'
      : `Drone not answering for ${chain.heard_s < 60 ? `${chain.heard_s.toFixed(0)} s` : `${(chain.heard_s / 60).toFixed(0)} min`}`;
  return 'Offline';
}

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
  const [chain, setChain] = useState<Chain | null>(null);
  const latestChain = useRef<Chain | null>(null);
  const viewing = useRef(false);
  const [requestError, setRequestError] = useState('');
  const [calibrationPending, setCalibrationPending] = useState(false);
  // Set as an arm request goes out, before the drone's reply and the reading that confirms it.
  const [arming, setArming] = useState(false);
  // The drone's refusal of the last ESC calibration stage, if any.
  const [escError, setEscError] = useState<string | null>(null);
  const latestReading = useRef<Reading | null>(null);
  // The rotors' steady forward tilt (degrees, positive forward) over the last second of an armed flight: holding the nose against a CoM off center (negative: forward of it) also points the thrust, and a growing one is a battery sliding (2026-10-31).
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
  // The GPS track: minutes of fixes at 10 Hz.
  const [gpsHistory] = useState(() => createHistory<GpsSample>(300_000, 3000));
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
  const [mode, setModeState] = useState(MANUAL);
  const [nudge, setNudgeState] = useState(NO_NUDGE);

  const setTargets = useCallback((update: Partial<Targets>) => {
    // Hover entered from manual holds where the drone is, so a nudge never carries over between hops.
    if (update.mode === HOVER && targets.current.mode === MANUAL)
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
    // A tab that had control asks for it again; any other watches while one has it.
    const view = !!latestChain.current?.in_use && !socket.current?.connected;
    viewing.current = view;
    socket.current?.close();
    socket.current = null;
    // A new connection starts from zero targets; arming needs an explicit request.
    targets.current = ZERO_TARGETS;
    setModeState(MANUAL);
    setNudgeState(NO_NUDGE);
    setStatus('connecting');
    setReading(null);
    setRequestError('');
    setCalibrationPending(false);
    setArming(false);
    latestReading.current = null;
    latestHover.current = null;
    setHoverState(null);
    const session = Date.now();
    let lastVoltage: VoltageSample | undefined;
    let lastHourVoltage: VoltageSample | undefined;
    let lastFix: GpsSample | undefined;
    let home: GpsFix | undefined;
    // Arming moves home to the next fix: the takeoff point.
    let homePending = false;
    let wasArmed = false;
    let lastItow: number | undefined;
    let armTimer: ReturnType<typeof setTimeout> | undefined;
    function offline() {
      if (socket.current !== ws) return;
      socket.current = null;
      setStatus('offline');
      setCalibrationPending(false);
      ws.close();
    }

    const ws = new EspConnection(
      `ws://${SERVICE}/${view ? 'view' : 'ws'}`,
      targets.current,
      () => {
        if (socket.current === ws) setStatus(view ? 'viewing' : 'connected');
      },
      (message) => {
        if (socket.current !== ws) return;
        if (message.type === 'result') {
          if (message.request === 'calibrate') setCalibrationPending(false);
          // A refused arm ends Arming… now; an accepted one when the drone reports itself enabled, or a moment later if it stopped again at once (its reason then shows under the controls).
          if (message.request === 'arm') {
            clearTimeout(armTimer);
            if (message.error) setArming(false);
            else armTimer = setTimeout(() => setArming(false), 3000);
          }
          if (message.request === 'esc') setEscError(message.error);
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
        if (data.control) {
          if (data.control.enabled && !wasArmed) homePending = true;
          wasArmed = data.control.enabled;
        }
        if (gpsSolved(data.gps)) {
          const { latitude, longitude, itow_ms } = data.gps;
          if (!home || homePending) home = { latitude, longitude };
          homePending = false;
          // The status repeats each fix until the next one.
          if (itow_ms !== lastItow) {
            lastItow = itow_ms;
            lastFix = {
              time,
              session,
              latitude,
              longitude,
              horizontal_accuracy: data.gps.horizontal_accuracy,
              speed: Math.hypot(data.gps.velocity[0], data.gps.velocity[1]),
            };
            gpsHistory.append(lastFix);
          }
        }
        latestReading.current = {
          ...data,
          time,
          voltage_ema: lastVoltage?.ema,
          gps_fix: lastFix,
          gps_home: home,
        };
        publishReading();
        if (data.control?.enabled) setArming(false);
        if (data.imu == null) return;
        history.append({
          ...data.imu,
          time,
          session,
          // The status carries the laptop's setpoints, which hover and landing ignore, so they are plotted in manual only.
          target:
            data.control?.enabled && targets.current.mode === MANUAL
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
    gpsHistory,
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
    const timer = setInterval(async () => {
      const next: Chain = await fetch('/api/link')
        .then((response) => response.json())
        .catch(() => ({ state: 'bridge_down' }));
      latestChain.current = next;
      // Whole seconds, so the page redraws for it once a second at most, and not at all while nothing changes.
      if (next.heard_s != null) next.heard_s = Math.round(next.heard_s);
      setChain((previous) => reuseUnchanged(previous ?? undefined, next)!);
      // The tab that had control has gone: this one takes it.
      if (viewing.current && next.in_use === false) connect();
    }, 1000);
    return () => clearInterval(timer);
  }, [connect]);

  useEffect(() => {
    if (status === 'offline' || status === 'connecting') return;
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
    setArming(false);
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
      if (enabled === true) {
        setArming(true);
        request({ type: 'arm' });
      }
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

  // One roll flip, if the drone finds the hold ready for it (fw/drone/main/flip.h); a refusal shows as the request's error.
  const flip = useCallback(() => request({ type: 'flip' }), [request]);
  // Level-and-hold (fw/drone/main/recover.h): from any mode the drone ends a pattern or flip, levels and holds; STOP stays the motor cut.
  const level = useCallback(() => request({ type: 'level' }), [request]);

  const escStage = useCallback(
    (stage: 0 | 1 | 2) => {
      setEscError(null);
      request({ type: 'esc', stage });
    },
    [request]
  );

  // Flying free (the GPS hold) is hover_camera 0; 1 holds on the tag.
  const setFree = useCallback(
    (free: boolean) => sendControl({ hover_camera: free ? 0 : 1 }),
    [sendControl]
  );

  // Only Enable arms: hover takes off from an armed drone and does nothing to a disarmed one.
  const hover = useCallback(() => {
    if (latestReading.current?.control?.enabled) setTargets({ mode: HOVER });
  }, [setTargets]);

  const armed = reading?.control?.enabled;
  const control = reading?.control;
  const cameraSample: HoverSample | null =
    previewTag === undefined || !control
      ? null
      : {
          ...EMPTY_SAMPLE,
          tag: previewTag
            ? cameraToBody(
                previewTag.position,
                control.hover_cam_roll ?? 0,
                control.hover_cam_pitch ?? 0
              )
            : null,
          tag_age_ms: previewTag ? 0 : null,
          range_target: control.hover_distance ?? null,
          time: reading?.time ?? 0,
        };
  const mapState =
    (armed && hoverState) || cameraSample || hoverState || EMPTY_SAMPLE;
  // A drone that disarms itself drops back to manual, so a later Enable never arms straight into hover.
  useEffect(() => {
    if (armed === false && targets.current.mode !== MANUAL)
      setTargets({ mode: MANUAL });
  }, [armed, setTargets]);
  // Arming or disarming starts a new hop from the unnudged hold.
  useEffect(() => {
    if (targets.current.nudge.some(Boolean)) setTargets({ nudge: NO_NUDGE });
  }, [armed, setTargets]);

  // On the tag the nudge is the hold point's offset, so it stops at NUDGE_MAX. Without it (wrap) only its speed counts, so forward and left wrap around the limit and a held stick keeps flying; the drone ignores the jump (fw/drone/main/hover.c NUDGE_JUMP).
  const nudgeHold = useCallback(
    (forward: number, left: number, up: number, wrap = false) => {
      const step = (value: number, change: number, around: boolean) => {
        let next = value + change;
        if (around && Math.abs(next) > NUDGE_MAX)
          next -= Math.sign(next) * 2 * NUDGE_MAX;
        return (
          Math.round(Math.max(-NUDGE_MAX, Math.min(NUDGE_MAX, next)) * 1000) /
          1000
        );
      };
      const [x, y, z] = targets.current.nudge;
      setTargets({
        nudge: [
          step(x, forward, wrap),
          step(y, left, wrap),
          step(z, up, false),
        ],
      });
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
  // The fly-away failsafe outranks the stop reason while it acts; a stopped test pattern shows when nothing else does.
  const guard = reading?.control_status;
  const failsafe = guard?.failsafe
    ? `Failsafe (${guard.failsafe_far ? 'too far' : 'flying off'}): ${guard.failsafe === 'brake' ? 'braking' : guard.failsafe === 'hold' ? 'holding, stick to take over' : 'landing'}`
    : '';
  const reason =
    requestError ||
    failsafe ||
    reading?.control_status?.reason ||
    (guard?.pattern_stop ? `Test pattern stopped: ${guard.pattern_stop}` : '');
  const flipPhase = guard?.flip;
  const recovery = guard?.recovery;
  const recoveryCause = guard?.recovery_cause;
  const controlStatus = useMemo(
    () => ({
      reason,
      flip: flipPhase,
      recovery,
      recovery_cause: recoveryCause,
    }),
    [reason, flipPhase, recovery, recoveryCause]
  );

  // A retry while the service names a broken hop keeps showing that hop: it is retried every second, and "Connecting" says nothing of what is missing.
  const shown =
    status === 'connecting' && chain && chain.state !== 'online'
      ? 'offline'
      : status;
  const { label, icon: StatusIcon, color } = statuses[shown];

  // connected: readings arrive; piloting: this tab's controls reach the drone.
  const piloting = status === 'connected';
  const connected = piloting || status === 'viewing';
  const volts = reading?.voltage_ema;
  // No pack: the drone runs from USB.
  const usb = volts !== undefined && volts <= NO_PACK_V;
  const imu = imuState(reading?.imu);
  const gps = reading?.gps;
  const fix = gpsSolved(gps) ? gps : null;
  const logging = reading?.logging;
  const recorderText =
    recorder?.error ??
    (logging?.active
      ? `Recording · ${logging.samples} samples · ${logging.dropped} dropped`
      : logging?.ready
        ? recorder?.file
          ? `Saved ${recorder.file}`
          : 'Recorder ready'
        : 'Recorder connecting');
  // The radio link over the last second (src/fw/controller/bridge.py decode_link): status messages reaching the laptop of the drone's STATUS_HZ and the controller's RSSI of them, then the controller's packets reaching the drone and the drone's RSSI of them. Weak: a quarter of the status lost or a gap of LINK_GAP_MS.
  const link = reading?.link;
  const linkWeak =
    !!link?.bridge &&
    (link.bridge.statuses < LINK_MIN_HZ ||
      link.bridge.longest_gap_ms >= LINK_GAP_MS);
  const linkText = !link?.bridge
    ? 'Radio link'
    : [
        `Laptop: ${link.bridge.statuses} status/s, longest gap ${link.bridge.longest_gap_ms} ms, ${link.bridge.late} late, ${link.bridge.timeouts} timeouts${chain?.errors ? `, ${chain.errors} packets not decoded since the service started` : ''}`,
        link.controller
          ? `Controller: hears drone ${link.controller.rx}/s${dbm(link.controller.rssi)} (min${dbm(link.controller.rssi_min)}, noise ${link.controller.noise_floor} dBm), ${link.controller.gaps} missed; sends ${link.controller.tx_ok} ok, ${link.controller.tx_fail} failed, ${link.controller.refused} refused, ${link.controller.rejected} dropped; ch ${link.controller.channel}, ${link.controller.rate_name}, ${link.controller.power} dBm`
          : 'Controller: no link report (older firmware)',
        link.drone
          ? `Drone: hears controller ${link.drone.rx}/s${dbm(link.drone.rssi)} (min${dbm(link.drone.rssi_min)}, noise ${link.drone.noise_floor} dBm), ${link.drone.gaps} missed, ${link.drone.rejected} stale, longest silence ${link.drone.silence_ms} ms; sends ${link.drone.tx_ok} ok, ${link.drone.tx_fail} failed, ${link.drone.refused} refused, slowest ${link.drone.latency_ms} ms; ch ${link.drone.channel}${link.drone.hopping ? ' (searching)' : ''}, ${link.drone.rate_name}, ${link.drone.power} dBm`
          : 'Drone: no link report (older firmware)',
      ].join('\n');
  // Drawn whether or not the drone is connected, so the controller can be checked on its own.
  const controller = (
    <section className="px-4 py-3" aria-label="Controller">
      <GamepadView />
    </section>
  );
  return (
    <div className="flex flex-col lg:h-dvh">
      {(arming || armed) && (
        <div className="pointer-events-none fixed inset-0 z-50 border-4 border-destructive" />
      )}
      <header className="flex h-14 shrink-0 items-center gap-6 border-b px-4">
        <h1 className="font-semibold">Drone dashboard</h1>
        <Tabs
          value={bench ? 'bench' : 'fly'}
          onValueChange={(value) => setBench(value === 'bench')}
        >
          <TabsList>
            <TabsTrigger value="fly">Fly</TabsTrigger>
            <TabsTrigger value="bench">Bench</TabsTrigger>
          </TabsList>
        </Tabs>
        {/* What decides whether the drone can fly, as icons; the details are in each one's title. */}
        <div className="flex items-center gap-4 tabular-nums">
          <Status
            title={
              volts === undefined
                ? 'Battery'
                : usb
                  ? 'No pack: powered over USB'
                  : (batteryWarning ?? `Battery ${volts.toFixed(2)} V`)
            }
            className={
              connected && batteryWarning
                ? 'text-red-700'
                : 'text-muted-foreground'
            }
            icon={connected && usb ? Plug : BatteryMedium}
          >
            {!connected || volts === undefined
              ? '—'
              : usb
                ? 'USB'
                : `${volts.toFixed(1)} V${batteryWarning && volts >= ARM_MIN_V ? ', falling' : ''}`}
          </Status>
          <Status
            title={connected ? `IMU ${IMU_TEXT[imu]}` : 'IMU'}
            className={
              connected && imu === 'calibrated'
                ? 'text-green-700'
                : !connected || imu === 'calibrating'
                  ? 'text-muted-foreground'
                  : 'text-red-700'
            }
            icon={Activity}
          >
            {connected ? IMU_TEXT[imu] : '—'}
          </Status>
          <Status
            title={
              fix
                ? `GPS ${fix.fix === 2 ? '2D' : '3D'} fix, ${fix.satellites} satellites, ${fix.horizontal_accuracy.toFixed(1)} m`
                : gps?.configured
                  ? 'GPS: no fix'
                  : 'GPS: searching for the module'
            }
            className={
              connected && fix ? 'text-green-700' : 'text-muted-foreground'
            }
            icon={Satellite}
          >
            {connected && fix
              ? `${fix.satellites} · ${fix.horizontal_accuracy.toFixed(1)} m`
              : '—'}
          </Status>
          <Status
            title={linkText}
            className={
              !connected || !link?.bridge
                ? 'text-muted-foreground'
                : linkWeak
                  ? 'text-red-700'
                  : 'text-green-700'
            }
            icon={Radio}
          >
            {connected && link?.bridge
              ? `${link.bridge.statuses}/${STATUS_HZ}${dbm(link.controller?.rssi)} · ${link.drone ? `${link.drone.rx}/s${dbm(link.drone.rssi)}` : '—'}`
              : '—'}
          </Status>
          <Status
            title={recorderText}
            className={
              // Dropped records warn only while that flight records: the count stays until the next flight starts.
              recorder?.error || (logging?.active && logging.dropped > 0)
                ? 'text-red-700'
                : 'text-muted-foreground'
            }
            icon={Disc}
            iconClassName={logging?.active ? 'fill-red-700 text-red-700' : ''}
          >
            {logging?.active ? logging.samples : logging?.ready ? 'ready' : '—'}
          </Status>
          {/* Warnings show here, last in the row, so nothing on the page moves when one appears. */}
          {tiltWarning !== null && (
            <Status
              title={`Rotors held ${Math.abs(tiltWarning).toFixed(0)}° ${tiltWarning < 0 ? 'back: CoM forward of center' : 'forward: CoM behind center'} or moving. Land and check the battery.`}
              className="font-medium text-red-700"
              icon={TriangleAlert}
            >
              Rotors {Math.abs(tiltWarning).toFixed(0)}°{' '}
              {tiltWarning < 0 ? 'back' : 'forward'}
            </Status>
          )}
        </div>
        <div className="ml-auto flex items-center gap-3">
          <Link href="/camera" className="text-sm underline">
            Camera
          </Link>
          {/* Its own tab: it reads recordings only, and this one keeps the drone link (the bridge serves one dashboard). */}
          <a
            href="/flights"
            target="_blank"
            rel="noopener"
            className="text-sm underline"
          >
            Flights
          </a>
          {fps !== null && (
            <span
              className="min-w-[6ch] text-right text-sm text-muted-foreground tabular-nums"
              title={`Telemetry from ${SERVICE}`}
            >
              {Math.round(fps)} Hz
            </span>
          )}
          <Status
            role="status"
            icon={StatusIcon}
            iconClassName={shown === 'connecting' ? 'animate-spin' : ''}
            className={color}
          >
            {shown === 'offline' ? offlineText(chain) : label}
          </Status>
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
      {/* Fly, three columns of about one height, so nothing scrolls on a laptop: the controls with the drone's attitude under them; the camera with the GPS under it; where the drone is against the tag (or on the map), with the controller under it. Bench puts the servo and ESC panels in the third column's place, and the controller at the foot of the first. */}
      <main
        className={`grid divide-y divide-border lg:min-h-0 lg:flex-1 lg:grid-cols-[26rem_minmax(0,1fr)_minmax(0,1fr)] lg:divide-x lg:divide-y-0 lg:overflow-hidden`}
      >
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          {reading?.control && (
            <ControlPanel
              key={`${status}-${stopEpoch}-${reading.control.enabled}`}
              settings={reading.control}
              gpsHold={
                !!reading.gps?.ok &&
                reading.gps.fix === 3 &&
                (reading.control.hover_gps_tau ?? 0) > 0
              }
              recorderReady={!!logging?.ready}
              recorderError={recorder?.error ?? undefined}
              status={controlStatus}
              arming={arming}
              connected={piloting}
              viewing={status === 'viewing'}
              imu={imu}
              mode={mode}
              nudge={nudge}
              onChange={sendControl}
              onMode={setMode}
              onHover={hover}
              onNudge={nudgeHold}
              onStop={stopMotors}
              escError={escError}
              onEscStage={escStage}
              onFlip={flip}
              onLevel={level}
            />
          )}
          <Suspense>
            <ImuPanel
              reading={reading?.imu}
              history={history.read}
              time={reading?.time ?? 0}
              onCalibrate={calibrateImu}
              calibrationPending={calibrationPending}
              connected={connected}
            />
          </Suspense>
          {bench && controller}
        </div>
        {bench && (
          <div className="order-last min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
            <ADCPanel
              raw={reading?.adc_raw}
              voltage={reading?.adc_voltage}
              ema={reading?.voltage_ema}
              history={voltageHistory.read}
              hourHistory={voltageHourHistory.read}
              time={reading?.time ?? 0}
              connected={connected}
            />
            <div className="grid grid-cols-2 divide-x divide-border [&>*:nth-child(-n+2)]:border-b">
              {(['servo', 'throttle'] as const).flatMap((kind) =>
                (['left', 'right'] as const).map((side) => (
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
                    connected={piloting}
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
          </div>
        )}
        <div className="min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain">
          <FpvPanel
            free={
              reading?.control ? reading.control.hover_camera === 0 : undefined
            }
            onFree={setFree}
            roll={reading?.control?.hover_cam_roll}
            onTag={showPreviewTag}
          />
          <GpsPanel
            gps={reading?.gps}
            baro={reading?.baro}
            report={reading?.gps_report}
            time={reading?.time}
            connected={connected}
          />
        </div>
        {/* On the bench the servo and ESC panels take its place. */}
        <div
          className={`min-h-0 min-w-0 divide-y divide-border lg:overflow-y-auto lg:overscroll-contain ${bench ? 'hidden' : ''}`}
        >
          {/* Always shown, so the layout stays put: without the camera, the GPS map; armed, the flight's map; else the camera's latest sighting, the last flight's map, or an empty one. */}
          {reading?.control?.hover_camera === 0 ? (
            <GpsMap
              gps={connected ? reading.gps : null}
              latest={reading.gps_fix}
              home={reading.gps_home}
              history={gpsHistory.read}
              hover={hoverState}
            />
          ) : (
            <HoverPanel
              state={mapState}
              history={mapState === hoverState ? hoverHistory.read : noHistory}
            />
          )}
          {!bench && controller}
        </div>
      </main>
    </div>
  );
}
