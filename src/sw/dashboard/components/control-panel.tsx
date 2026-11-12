'use client';

import { memo, useCallback, useEffect, useRef, useState } from 'react';
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Space,
  Settings2,
  Square,
  type LucideIcon,
} from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Kbd, KbdGroup } from '@/components/ui/kbd';
import { EscCalibration } from '@/components/esc-calibration';
import { Panel } from '@/components/panel';
import { BUTTONS, readPad, stick } from '@/components/gamepad';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';
import { typing } from '@/lib/keys';
import {
  FLIGHT_MODES,
  HOVER,
  LAND,
  LAND_FAST,
  MANUAL,
  type ControlSettings,
  type ImuState,
  type NumberSetting,
  type Reading,
} from '@/lib/telemetry';

const FLIP_PHASES = {
  climb: 'climbing',
  flip: 'spinning',
  arrest: 'arresting',
};

const RECOVERY_CAUSES = {
  tilt: 'tipped',
  flip: 'flip failed',
  gyro: 'gyro saturated',
  level: 'level',
};

const keyIcons: Record<string, LucideIcon> = {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Space,
};

// L hands the drone its controlled landing (a steady descent, then motors off at touchdown), from hover or manual flight; holding it past LAND_FAST_MS descends faster.
const LAND_FAST_MS = 300;

// Mode 2, like the gamepad: the arrows tilt (positive pitch is nose down, so the up arrow tilts forward), and W and S climb and A and D yaw (yawKeys).
const tiltKeys = {
  ArrowUp: { axis: 'pitch', direction: 1 },
  ArrowDown: { axis: 'pitch', direction: -1 },
  ArrowLeft: { axis: 'roll', direction: -1 },
  ArrowRight: { axis: 'roll', direction: 1 },
} as const;
const yawKeys = ['KeyA', 'KeyD'];
// In hover and landing W and S raise and lower the hold point while held, at the drone's hover_climb_rate. On the tag the arrows move it too (toward the tag and to the drone's left: [forward, left]), at hover_nudge_speed, and the map shows it as the target; without the tag but on the GPS hold they fly that speed, as the gamepad's stick does, and without either they tilt the drone directly (fw/drone/main/hover.c stick).
const NUDGE_TICK_MS = 50;
const GAMEPAD_TICK_MS = 50;
const nudgeKeys = { KeyW: 1, KeyS: -1 } as const;
const moveKeys = {
  ArrowUp: [1, 0],
  ArrowDown: [-1, 0],
  ArrowLeft: [0, 1],
  ArrowRight: [0, -1],
} as const;

function nudgeText([forward, left, up]: [number, number, number]) {
  const parts = [];
  if (forward)
    parts.push(
      `${Math.abs(forward).toFixed(2)} m ${forward > 0 ? 'closer' : 'farther'}`
    );
  if (left)
    parts.push(`${Math.abs(left).toFixed(2)} m ${left > 0 ? 'left' : 'right'}`);
  if (up) parts.push(`${Math.abs(up).toFixed(2)} m ${up > 0 ? 'up' : 'down'}`);
  return `Hold nudge: ${parts.join(', ') || 'none'}`;
}

// The setpoints the keys stream (the drone's targets).
type Setpoint = 'roll' | 'pitch' | 'yaw_rate' | 'base_throttle';
// The gains dialog's sections: each field is a drone setting; label is read within its section's title.
type Field = { key: NumberSetting; label: string; min?: number; max?: number };
const gainSections: {
  title: string;
  needs?: NumberSetting;
  fields: Field[];
}[] = [
  {
    title: 'Roll',
    fields: [
      { key: 'roll_p', label: 'P', min: 0 },
      { key: 'roll_i', label: 'I', min: 0 },
      { key: 'roll_damping', label: 'Damping', min: 0 },
      { key: 'roll_bias', label: 'Bias (rad/s²)', min: -20, max: 20 },
    ],
  },
  {
    title: 'Pitch',
    fields: [
      { key: 'pitch_p', label: 'P', min: 0 },
      { key: 'pitch_i', label: 'I', min: 0 },
      { key: 'pitch_damping', label: 'Damping', min: 0 },
      { key: 'pitch_bias', label: 'Bias (rad/s²)', min: -20, max: 20 },
      { key: 'pitch_trim', label: 'Trim (°)', min: -15, max: 15 },
    ],
  },
  {
    title: 'Yaw',
    fields: [
      { key: 'yaw_damping', label: 'Damping', min: 0 },
      { key: 'yaw_i', label: 'I', min: 0 },
      { key: 'yaw_bias', label: 'Bias (rad/s²)', min: -10, max: 10 },
    ],
  },
  {
    title: 'Attitude loop',
    fields: [
      { key: 'integral_limit', label: 'Integral limit', min: 0 },
      { key: 'integrate_throttle', label: 'Integrate above throttle', min: 0 },
      { key: 'target_rate', label: 'Target ramp (°/s)', min: 1 },
      { key: 'velocity_damping', label: 'Velocity damping (°/(m/s))', min: 0 },
      { key: 'velocity_leak', label: 'Velocity leak (s)', min: 0.1 },
      { key: 'auto_trim', label: 'Self-trim per hop (0-1)', min: 0, max: 1 },
    ],
  },
  {
    title: 'Servos and thrust',
    fields: [
      { key: 'left_trim', label: 'Left servo trim (°)', min: -15, max: 15 },
      { key: 'right_trim', label: 'Right servo trim (°)', min: -15, max: 15 },
      { key: 'servo_compensation', label: 'Servo compensation (°)', min: 0 },
      { key: 'thrust_voltage', label: 'Thrust reference (V, 0 off)', min: 0 },
      {
        key: 'left_thrust_scale',
        label: 'Left thrust scale',
        min: 0.5,
        max: 1.5,
      },
      {
        key: 'right_thrust_scale',
        label: 'Right thrust scale',
        min: 0.5,
        max: 1.5,
      },
    ],
  },
  {
    title: 'Hover on the tag',
    fields: [
      { key: 'hover_center', label: 'Center on tag (1 on, 0 off)', min: 0 },
      {
        key: 'hover_distance',
        label: 'Distance (m, 0 = takeoff range)',
        min: 0,
      },
      { key: 'hover_approach', label: 'Approach speed (m/s)', min: 0.02 },
      { key: 'hover_ceiling', label: 'Ceiling above takeoff (m)', min: 0.2 },
      {
        key: 'hover_square',
        label: 'Move square to tag (1 on, 0 off)',
        min: 0,
      },
      { key: 'hover_kside', label: 'Pull to square-on (m/s² per m)', min: 0 },
      { key: 'hover_cam_pitch', label: 'Camera tilt up (°)', min: -30 },
      {
        key: 'hover_cam_roll',
        label: 'Camera roll (°, 180 upside down)',
        min: -180,
      },
    ],
  },
  {
    title: 'Hover without the tag',
    fields: [
      { key: 'hover_drag', label: 'Rotor drag (1/s, 0 off)', min: 0 },
      {
        key: 'hover_drag_tilt',
        label: 'Drag thrust tilt offset (°)',
        min: -10,
      },
      { key: 'hover_drag_tau', label: 'Drag correction time (s)', min: 0.1 },
      {
        key: 'hover_speed_tilt',
        label: 'Servo speed tilt (°/(m/s), 0 off)',
        min: 0,
      },
      { key: 'hover_speed_pitch', label: 'Servo speed pitch share', min: -2 },
      {
        key: 'hover_speed_takeoff',
        label: 'Servo speed takeoff offset (°)',
        min: -10,
      },
      {
        key: 'hover_speed_tau',
        label: 'Servo speed correction time (s)',
        min: 0.05,
      },
      { key: 'hover_trim_tau', label: 'Trim build-up time (s)', min: 0.1 },
      { key: 'hover_trim_max', label: 'Trim limit (m/s²)', min: 0 },
      {
        key: 'hover_gps_tau',
        label: 'GPS velocity correction time (s, 0 off)',
        min: 0,
      },
      {
        key: 'hover_gps_place_tau',
        label: 'GPS position correction time (s, 0 off)',
        min: 0,
      },
      {
        key: 'hover_gps_height_tau',
        label: 'GPS height correction time (s, 0 off)',
        min: 0,
      },
      {
        key: 'hover_baro_tau',
        label: 'Barometer height correction time (s, 0 off)',
        min: 0,
        max: 10,
      },
      {
        key: 'hover_gps_heading',
        label: 'Heading at takeoff (° from north)',
        min: -180,
        max: 180,
      },
      {
        key: 'hover_gps_calibrate',
        label: 'Heading search tilt (°, 0 off)',
        min: 0,
        max: 10,
      },
      {
        key: 'hover_thrust_direction',
        label: 'Thrust direction share (0 off)',
        min: 0,
        max: 1.5,
      },
      { key: 'hover_kd', label: 'Velocity damping (1/s)', min: 0 },
      {
        key: 'hover_return_speed',
        label: 'Return speed to the hold point (m/s, 0 no limit)',
        min: 0,
        max: 5,
      },
      {
        key: 'hover_ekf',
        label: 'Kalman filter (0 logged, 1 flown, 2 and its height)',
        min: 0,
        max: 2,
      },
    ],
  },
  {
    title: 'Fly-away failsafe',
    fields: [
      { key: 'failsafe_speed', label: 'Speed (m/s, 0 off)', min: 0 },
      { key: 'failsafe_distance', label: 'Distance (m, 0 off)', min: 0 },
      { key: 'failsafe_time', label: 'Time over the speed (s)', min: 0.2 },
      { key: 'failsafe_land', label: 'Brake before landing (s)', min: 1 },
      { key: 'failsafe_tilt', label: 'Brake tilt (°)', min: 0, max: 30 },
    ],
  },
  {
    title: 'Tilt and flip',
    fields: [
      { key: 'tip_limit', label: 'Tip cut (°)', min: 20, max: 80 },
      { key: 'bank_tilt', label: 'Gyro-only past (°, 0 off)', min: 0, max: 80 },
      {
        key: 'flip_enable',
        label: 'Flip allowed (0 off, 1 flip, 2 burst to 60 and back, 3 to 90)',
        min: 0,
        max: 3,
      },
      { key: 'flip_height', label: 'Flip from at least (m)', min: 2.5, max: 5 },
      {
        key: 'recover_enable',
        label: 'Recover, no tip cut (0 or 1)',
        min: 0,
        max: 1,
      },
    ],
  },
  {
    title: 'Model',
    needs: 'arm',
    fields: [
      { key: 'pivot_height', label: 'Pivot height (m)', min: 0.000001 },
      { key: 'com_x', label: 'CoM x (m)', min: -0.02, max: 0.02 },
      { key: 'roll_inertia', label: 'Roll inertia (kg·m²)', min: 0.000001 },
      { key: 'pitch_inertia', label: 'Pitch inertia (kg·m²)', min: 0.000001 },
      { key: 'yaw_inertia', label: 'Yaw inertia (kg·m²)', min: 0.000001 },
      {
        key: 'reaction_torque_ratio',
        label: 'Reaction torque ratio',
        min: -0.1,
        max: 0.1,
      },
      { key: 'arm', label: 'Arm (m)', min: 0.001 },
    ],
  },
];
const gainKeys = gainSections.flatMap(({ fields }) =>
  fields.map(({ key }) => key)
);

export const ControlPanel = memo(function ControlPanel({
  settings,
  gpsHold,
  recorderReady,
  recorderError,
  imu,
  status,
  arming,
  connected,
  viewing,
  mode,
  nudge,
  onChange,
  onMode,
  onHover,
  onNudge,
  onStop,
  escError,
  onEscStage,
  onFlip,
  onLevel,
}: {
  settings: ControlSettings;
  // A 3D GPS fix and the GPS hold on: without the tag the right stick then flies a speed instead of a tilt.
  gpsHold: boolean;
  recorderReady: boolean;
  recorderError?: string;
  imu: ImuState;
  status: Reading['control_status'];
  arming: boolean;
  connected: boolean;
  mode: number;
  nudge: [number, number, number];
  onChange: (settings: Partial<ControlSettings>) => void;
  onMode: (mode: number) => void;
  onHover: () => void;
  onNudge: (forward: number, left: number, up: number, wrap?: boolean) => void;
  onStop: () => void;
  viewing: boolean;
  escError: string | null;
  onEscStage: (stage: 0 | 1 | 2) => void;
  onFlip: () => void;
  onLevel: () => void;
}) {
  const available = connected && settings.pitch_p !== undefined;
  const ready = available && recorderReady && imu === 'calibrated';
  const problem = !connected
    ? viewing
      ? 'View only: another tab has control'
      : 'Disconnected'
    : !available
      ? 'Firmware update required'
      : imu === 'calibrating'
        ? 'Calibrating IMU'
        : status?.reason ||
          (imu === 'offline'
            ? 'IMU offline'
            : imu === 'uncalibrated'
              ? 'Calibrate IMU before enabling'
              : !recorderReady
                ? (recorderError ?? 'Flight recorder not ready')
                : '');
  const [draft, setDraft] = useState<Partial<Record<Setpoint, number>>>({});
  const [gains, setGains] = useState<Record<string, string>>({});
  // A setting by name, undefined when this drone's firmware has none.
  const setting = (key: NumberSetting): number | undefined => settings[key];
  const [open, setOpen] = useState(false);
  // Every key's size is a drone setting, so it can be tuned over the bridge like the rest: the arrows tilt as far as the hold itself may (hover_tilt; a per-browser step had been left at 5 degrees, too little to bring back a drifting drone, 2026-11-02), on the tag they move the hold point at hover_nudge_speed, A and D turn at hover_yaw_rate and W and S climb at hover_climb_rate.
  const keySizes = useRef({ tilt: 15, yaw: 30, climb: 0.3, move: 0.5 });
  keySizes.current = {
    tilt: settings.hover_tilt ?? 15,
    yaw: settings.hover_yaw_rate ?? 30,
    climb: settings.hover_climb_rate ?? 0.3,
    move: settings.hover_nudge_speed ?? 0.5,
  };
  const free = settings.hover_camera === 0;
  const freeRef = useRef(free);
  freeRef.current = free;
  const gpsHoldRef = useRef(gpsHold);
  gpsHoldRef.current = gpsHold;
  const throttle = useRef(settings.base_throttle);
  // Key handlers read the mode through a ref so a mode change doesn't re-register them and forget held keys.
  const modeRef = useRef(mode);
  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);
  const setMode = useCallback(
    (next: number) => {
      modeRef.current = next;
      onMode(next);
    },
    [onMode]
  );

  useEffect(() => {
    throttle.current = draft.base_throttle ?? settings.base_throttle;
  }, [draft.base_throttle, settings.base_throttle]);

  useEffect(() => {
    if (!available || !settings.enabled || open) return;

    const held = new Set<string>();
    // Arrows held for a hover nudge, and the timer moving the hold point while any is.
    const nudging = new Set<string>();
    let nudgeTimer: ReturnType<typeof setInterval> | undefined;
    let landFast: ReturnType<typeof setTimeout> | undefined;

    function land() {
      // Manual with no collective is already on the ground.
      if (modeRef.current === MANUAL && throttle.current <= 0) return;
      setMode(LAND);
      clearTimeout(landFast);
      landFast = setTimeout(() => {
        if (held.has('KeyL') && modeRef.current === LAND) setMode(LAND_FAST);
      }, LAND_FAST_MS);
    }

    function releaseLand() {
      clearTimeout(landFast);
      landFast = undefined;
      if (modeRef.current === LAND_FAST) setMode(LAND);
    }

    function changeYaw() {
      const yaw_rate =
        keySizes.current.yaw *
        (Number(held.has('KeyA')) - Number(held.has('KeyD')));
      setDraft((previous) => ({ ...previous, yaw_rate }));
      onChange({ yaw_rate });
    }

    function changeTilt(code: keyof typeof tiltKeys) {
      const { axis } = tiltKeys[code];
      const direction = Object.entries(tiltKeys).reduce(
        (sum, [key, binding]) =>
          sum +
          (binding.axis === axis && held.has(key) ? binding.direction : 0),
        0
      );
      const value = keySizes.current.tilt * direction;
      setDraft((previous) => ({ ...previous, [axis]: value }));
      onChange({ [axis]: value });
    }

    function nudge() {
      const tick = NUDGE_TICK_MS / 1000;
      let forward = 0;
      let left = 0;
      let up = 0;
      for (const code of nudging) {
        if (code in nudgeKeys)
          up +=
            nudgeKeys[code as keyof typeof nudgeKeys] *
            keySizes.current.climb *
            tick;
        else {
          const [x, y] = moveKeys[code as keyof typeof moveKeys];
          forward += x * keySizes.current.move * tick;
          left += y * keySizes.current.move * tick;
        }
      }
      if (forward || left || up)
        onNudge(forward, left, up, freeRef.current && gpsHoldRef.current);
    }

    function stopNudging() {
      nudging.clear();
      clearInterval(nudgeTimer);
      nudgeTimer = undefined;
    }

    function keydown(event: KeyboardEvent) {
      if (event.code === 'Space') {
        stopNudging();
        held.clear();
        clearTimeout(landFast);
        return;
      }
      if (
        !['KeyW', 'KeyS', 'KeyL', ...yawKeys].includes(event.code) &&
        !(event.code in tiltKeys)
      )
        return;
      if (typing(event)) return;

      event.preventDefault();
      event.stopPropagation();
      if (event.repeat || held.has(event.code)) return;
      // There is no manual flight: outside hover only L and yaw act. In hover W and S, and on the tag or the GPS hold the arrows, move the hold point while held, and are not held flight keys, so their keyup and a blur leave hover alone; with neither the arrows are held tilt.
      const hovering = modeRef.current !== MANUAL;
      if (!hovering && (event.code in nudgeKeys || event.code in tiltKeys))
        return;
      if (
        event.code in nudgeKeys ||
        (event.code in moveKeys && (!freeRef.current || gpsHoldRef.current))
      ) {
        nudging.add(event.code);
        nudge();
        if (nudgeTimer === undefined)
          nudgeTimer = setInterval(nudge, NUDGE_TICK_MS);
        return;
      }
      held.add(event.code);

      if (event.code === 'KeyL') {
        land();
      } else if (event.code in tiltKeys) {
        changeTilt(event.code as keyof typeof tiltKeys);
      } else {
        changeYaw();
      }
    }

    function keyup(event: KeyboardEvent) {
      if (nudging.delete(event.code)) {
        if (!nudging.size) stopNudging();
        return;
      }
      if (!held.delete(event.code)) return;
      event.preventDefault();
      if (event.code === 'KeyL') releaseLand();
      if (yawKeys.includes(event.code)) changeYaw();
      if (event.code in tiltKeys)
        changeTilt(event.code as keyof typeof tiltKeys);
    }

    // A lost keyup can't be trusted: in hover the tilt and yaw held fall back to zero and the hold carries on, otherwise held keys stop the motors; L alone just drops back from land fast, and landing carries on.
    function blur() {
      stopNudging();
      if (held.has('KeyL')) releaseLand();
      held.delete('KeyL');
      const flying = held.size > 0;
      held.clear();
      if (!flying) return;
      if (modeRef.current === MANUAL) {
        onStop();
        return;
      }
      changeYaw();
      changeTilt('ArrowUp');
      changeTilt('ArrowLeft');
    }

    window.addEventListener('keydown', keydown, true);
    window.addEventListener('keyup', keyup);
    window.addEventListener('blur', blur);
    return () => {
      clearInterval(nudgeTimer);
      clearTimeout(landFast);
      window.removeEventListener('keydown', keydown, true);
      window.removeEventListener('keyup', keyup);
      window.removeEventListener('blur', blur);
    };
  }, [available, settings.enabled, open, onChange, setMode, onNudge, onStop]);

  // H (or T) takes off once Enable has armed the drone; disarmed, it does nothing.
  useEffect(() => {
    if (!available || open) return;
    function keydown(event: KeyboardEvent) {
      if ((event.code !== 'KeyH' && event.code !== 'KeyT') || typing(event))
        return;
      event.preventDefault();
      event.stopPropagation();
      if (event.repeat) return;
      if (!settings.enabled) return;
      onHover();
    }
    window.addEventListener('keydown', keydown, true);
    return () => window.removeEventListener('keydown', keydown, true);
  }, [available, settings.enabled, open, onHover]);

  // The gamepad flies like the keys, polled every GAMEPAD_TICK_MS: in Mode 2 the right stick tilts in proportion up to hover_tilt (without the tag), flies a speed up to hover_nudge_speed (without the tag, on the GPS hold) or moves the hold point at hover_nudge_speed (on it), the left stick climbs at hover_climb_rate and yaws at hover_yaw_rate, A hovers and B lands (held: fast), all only once the drone is enabled on the dashboard; both bumpers stop the motors at any time. A lost gamepad zeroes its tilt and yaw and the hold carries on.
  const enabledRef = useRef(settings.enabled);
  enabledRef.current = settings.enabled;
  useEffect(() => {
    if (!available || open) return;
    const zero = { roll: 0, pitch: 0, yaw_rate: 0 };
    let sent = { ...zero };
    let was = { a: false, b: false, stop: false };
    let landSince = 0;

    function send(next: typeof zero) {
      const changed = Object.fromEntries(
        Object.entries(next).filter(
          ([key, value]) => value !== sent[key as keyof typeof zero]
        )
      );
      if (!Object.keys(changed).length) return;
      sent = next;
      setDraft((previous) => ({ ...previous, ...changed }));
      onChange(changed);
    }

    function tick() {
      const pad = readPad();
      if (!pad) {
        if (enabledRef.current) send(zero);
        was = { a: false, b: false, stop: false };
        landSince = 0;
        return;
      }
      const pressed = (index: number) => (pad.buttons[index] ?? 0) > 0.5;
      const now = {
        a: pressed(BUTTONS.a),
        b: pressed(BUTTONS.b),
        stop: pressed(BUTTONS.leftBumper) && pressed(BUTTONS.rightBumper),
      };
      const before = was;
      was = now;
      if (now.stop) {
        if (!before.stop) onStop();
        // Stopping zeroes the dashboard's targets.
        sent = { ...zero };
        landSince = 0;
        return;
      }
      if (!enabledRef.current) return;

      if (now.a && !before.a) onHover();
      if (
        now.b &&
        !before.b &&
        !(modeRef.current === MANUAL && throttle.current <= 0)
      ) {
        setMode(LAND);
        landSince = Date.now();
      }
      if (
        now.b &&
        landSince &&
        Date.now() - landSince > LAND_FAST_MS &&
        modeRef.current === LAND
      )
        setMode(LAND_FAST);
      if (!now.b && before.b) {
        landSince = 0;
        if (modeRef.current === LAND_FAST) setMode(LAND);
      }

      // Mode 2, as on RC transmitters: the left stick climbs and yaws, the right stick tilts or flies.
      const [yawX, climbY, tiltX, tiltY] = [0, 1, 2, 3].map((index) =>
        stick(pad.axes[index] ?? 0)
      );
      const round = (value: number) => Math.round(value * 2) / 2 || 0;
      const sizes = keySizes.current;
      const hovering = modeRef.current !== MANUAL;
      const yaw_rate = round(-yawX * sizes.yaw);
      // Without the tag but with the GPS hold the right stick flies a speed, up to hover_nudge_speed, through the hold's velocity loop: let go and it slows to a stop where it is. Flown as a tilt it sped up as long as it was held and the hold then braked at full tilt the other way (2026-11-09 05:50 UTC: 15 degrees for 1.5 s reached 3 m/s, then -15 on release).
      const speed = freeRef.current && gpsHoldRef.current;
      if (hovering && freeRef.current && !speed)
        send({
          roll: round(tiltX * sizes.tilt),
          pitch: round(-tiltY * sizes.tilt),
          yaw_rate,
        });
      else send({ roll: 0, pitch: 0, yaw_rate });
      if (!hovering) return;
      const seconds = GAMEPAD_TICK_MS / 1000;
      const forward =
        freeRef.current && !speed ? 0 : -tiltY * sizes.move * seconds;
      const left =
        freeRef.current && !speed ? 0 : -tiltX * sizes.move * seconds;
      const up = -climbY * sizes.climb * seconds;
      if (forward || left || up) onNudge(forward, left, up, speed);
    }

    const timer = setInterval(tick, GAMEPAD_TICK_MS);
    return () => clearInterval(timer);
  }, [available, open, onChange, setMode, onNudge, onStop, onHover]);

  return (
    <Panel
      title="Flight control"
      header={
        <div className="ml-auto flex flex-wrap items-center justify-end gap-2">
          <Dialog
            open={open}
            onOpenChange={(value) => {
              setOpen(value);
              if (value)
                setGains(
                  Object.fromEntries(
                    gainKeys
                      .filter((key) => setting(key) !== undefined)
                      .map((key) => [
                        key,
                        String(Number(setting(key)!.toPrecision(5))),
                      ])
                  )
                );
            }}
          >
            <EscCalibration
              disabled={!available || settings.enabled}
              error={escError}
              onStage={onEscStage}
            />
            <DialogTrigger
              render={
                <Button
                  variant="ghost"
                  size="icon"
                  aria-label="Control gains"
                  disabled={!available || settings.enabled}
                />
              }
            >
              <Settings2 />
            </DialogTrigger>
            <DialogContent className="max-h-[90dvh] overflow-y-auto sm:max-w-6xl">
              <DialogHeader>
                <DialogTitle>Control gains</DialogTitle>
              </DialogHeader>
              <form
                className="space-y-4"
                onSubmit={(event) => {
                  event.preventDefault();
                  onChange(
                    Object.fromEntries(
                      Object.entries(gains).map(([key, value]) => [
                        key,
                        Number(value),
                      ])
                    )
                  );
                  setOpen(false);
                }}
              >
                <div className="grid gap-x-8 gap-y-5 md:grid-cols-2 xl:grid-cols-3">
                  {gainSections
                    .filter(
                      ({ fields, needs }) =>
                        (!needs || setting(needs) !== undefined) &&
                        fields.some(({ key }) => setting(key) !== undefined)
                    )
                    .map(({ title, fields }) => (
                      <section key={title} className="space-y-2">
                        <h3 className="border-b pb-1 font-medium">{title}</h3>
                        {fields
                          .filter(({ key }) => setting(key) !== undefined)
                          .map(({ key, label, min, max }) => (
                            <label
                              key={key}
                              className="flex items-center justify-between gap-3"
                            >
                              <span className="text-sm text-muted-foreground">
                                {label}
                              </span>
                              <Input
                                className="w-28 shrink-0 tabular-nums"
                                aria-label={`${title}: ${label}`}
                                type="number"
                                min={min}
                                max={max}
                                step="any"
                                required
                                value={gains[key] ?? ''}
                                onChange={(event) =>
                                  setGains({
                                    ...gains,
                                    [key]: event.target.value,
                                  })
                                }
                              />
                            </label>
                          ))}
                      </section>
                    ))}
                </div>
                <Button
                  type="submit"
                  variant="outline"
                  disabled={!available || settings.enabled}
                >
                  Apply
                </Button>
              </form>
            </DialogContent>
          </Dialog>
          <span
            className={`text-sm tabular-nums ${mode === MANUAL ? 'text-muted-foreground' : 'font-medium text-blue-700 dark:text-blue-400'}`}
            aria-label="Flight mode"
          >
            {FLIGHT_MODES[mode] ?? `Mode ${mode}`}
          </span>
          <Button
            variant={settings.enabled || arming ? 'destructive' : 'outline'}
            disabled={!connected || arming || (!settings.enabled && !ready)}
            aria-pressed={settings.enabled}
            onClick={() =>
              settings.enabled
                ? onStop()
                : onChange({
                    enabled: true,
                    base_throttle: 0,
                    roll: 0,
                    pitch: 0,
                    yaw_rate: 0,
                  })
            }
          >
            {settings.enabled ? 'Disable' : arming ? 'Arming…' : 'Enable'}
          </Button>
          {settings.flip_enable !== undefined && (
            <Button
              variant="outline"
              disabled={
                !connected ||
                !settings.enabled ||
                settings.flip_enable < 1 ||
                mode !== HOVER ||
                !!status?.flip
              }
              onClick={onFlip}
            >
              Flip
            </Button>
          )}
          <Button
            variant="outline"
            disabled={!connected || !settings.enabled}
            onClick={onLevel}
          >
            Level
          </Button>
          <Button
            variant="outline"
            size="icon"
            aria-label="Stop motors"
            onClick={onStop}
          >
            <Square />
          </Button>
        </div>
      }
    >
      <div className="grid grid-cols-2 gap-x-4 gap-y-1.5 text-sm text-muted-foreground">
        {[
          { keys: ['T', 'H'], label: 'Take off' },
          { keys: ['L'], label: 'Land (hold: fast)' },
          { keys: ['W', 'S'], label: 'Up / down' },
          { keys: ['A', 'D'], label: 'Yaw' },
          {
            keys: ['ArrowUp', 'ArrowDown'],
            label: free ? 'Tilt fwd / back' : 'Target closer / farther',
          },
          {
            keys: ['ArrowLeft', 'ArrowRight'],
            label: free ? 'Tilt left / right' : 'Target left / right',
          },
          { keys: ['F'], label: free ? 'Switch to tag' : 'Switch to free' },
          { keys: ['Space'], label: 'Stop' },
        ].map(({ keys, label }) => (
          <div key={keys.join()} className="flex items-center gap-2">
            <KbdGroup>
              {keys.map((key) => {
                const Icon = keyIcons[key];
                return (
                  <Kbd key={key} aria-label={key}>
                    {Icon ? <Icon aria-hidden="true" /> : key}
                  </Kbd>
                );
              })}
            </KbdGroup>
            <span className="truncate">{label}</span>
          </div>
        ))}
        {(mode !== MANUAL || nudge.some(Boolean)) && (
          <span className="col-span-2 tabular-nums">{nudgeText(nudge)}</span>
        )}
      </div>
      <p
        role="status"
        className={`h-6 truncate ${status?.recovery || status?.flip ? 'font-medium text-blue-700 dark:text-blue-400' : problem ? 'text-red-700' : 'text-green-700'}`}
      >
        {status?.recovery
          ? `Recovering (${status.recovery_cause ? RECOVERY_CAUSES[status.recovery_cause] : 'tipped'}): ${status.recovery}`
          : status?.flip
            ? `Flip: ${FLIP_PHASES[status.flip]}`
            : problem || 'Good to fly.'}
      </p>
    </Panel>
  );
});
