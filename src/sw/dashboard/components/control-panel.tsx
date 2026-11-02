'use client';

import { FLIGHT_MODES } from '@/lib/telemetry';

import { memo, useEffect, useRef, useState } from 'react';
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
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';
import type { ControlSettings, Reading } from '@/lib/telemetry';

const keyIcons: Record<string, LucideIcon> = {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Space,
};

// L hands the drone its controlled landing (a steady descent, then motors off at touchdown), from hover or manual flight; holding it past LAND_FAST_MS descends faster.
const LAND_FAST_MS = 300;
const MANUAL = 0;
const LAND = 2;
const LAND_FAST = 3;

// Keys typed into a field or a dialog aren't flight commands.
export function typing(event: KeyboardEvent) {
  return (
    event.ctrlKey ||
    event.metaKey ||
    event.altKey ||
    (event.target instanceof HTMLElement &&
      !!event.target.closest(
        'input, textarea, select, [contenteditable="true"], [role="dialog"]'
      ))
  );
}
// Positive pitch is nose down, so the up arrow tilts forward; A and D roll, and the left and right arrows yaw (yawKeys).
const tiltKeys = {
  ArrowUp: { axis: 'pitch', direction: 1 },
  ArrowDown: { axis: 'pitch', direction: -1 },
  KeyA: { axis: 'roll', direction: -1 },
  KeyD: { axis: 'roll', direction: 1 },
} as const;
const yawKeys = ['ArrowLeft', 'ArrowRight'];
// In hover and landing W and S raise and lower the hold point while held, at the drone's hover_climb_rate. On the tag the arrows and A and D move it too (toward the tag and to the drone's left: [forward, left]), at hover_nudge_speed, and the map shows it as the target; without the tag they tilt the drone directly (fw/drone/main/hover.c stick).
const NUDGE_TICK_MS = 50;
const nudgeKeys = { KeyW: 1, KeyS: -1 } as const;
const moveKeys = {
  ArrowUp: [1, 0],
  ArrowDown: [-1, 0],
  KeyA: [0, 1],
  KeyD: [0, -1],
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
type Field = { key: string; label: string; min?: number; max?: number };
const gainSections: { title: string; needs?: string; fields: Field[] }[] = [
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
      { key: 'hover_center', label: 'Centre on tag (1 on, 0 off)', min: 0 },
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
    ],
  },
  {
    title: 'Model',
    needs: 'arm',
    fields: [
      { key: 'pivot_height', label: 'Pivot height (m)', min: 0 },
      { key: 'com_x', label: 'CoM x (m)', min: -0.02, max: 0.02 },
      { key: 'roll_inertia', label: 'Roll inertia (kg·m²)', min: 0 },
      { key: 'pitch_inertia', label: 'Pitch inertia (kg·m²)', min: 0 },
      { key: 'yaw_inertia', label: 'Yaw inertia (kg·m²)', min: 0 },
      {
        key: 'reaction_torque_ratio',
        label: 'Reaction torque ratio',
        min: -0.1,
        max: 0.1,
      },
      { key: 'arm', label: 'Arm (m)', min: 0 },
    ],
  },
];
const gainKeys = gainSections.flatMap(({ fields }) =>
  fields.map(({ key }) => key)
);

export const ControlPanel = memo(function ControlPanel({
  settings,
  recorderReady,
  recorderError,
  imu,
  status,
  connected,
  mode,
  nudge,
  onChange,
  onMode,
  onHover,
  onNudge,
  onStop,
}: {
  settings: ControlSettings;
  recorderReady: boolean;
  recorderError?: string;
  imu: 'offline' | 'calibrating' | 'uncalibrated' | 'calibrated';
  status: Reading['control_status'];
  connected: boolean;
  mode: number;
  nudge: [number, number, number];
  onChange: (settings: Partial<ControlSettings>) => void;
  onMode: (mode: number) => void;
  onHover: () => void;
  onNudge: (forward: number, left: number, up: number) => void;
  onStop: () => void;
}) {
  const available = connected && settings.pitch_p !== undefined;
  const ready = available && recorderReady && imu === 'calibrated';
  const problem = !connected
    ? 'Disconnected'
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
  const setting = (key: string) =>
    (settings as unknown as Record<string, number | undefined>)[key];
  const [open, setOpen] = useState(false);
  // Every key's size is a drone setting, so it can be tuned over the bridge like the rest: the arrows and A and D tilt as far as the hold itself may (hover_tilt; a per-browser step had been left at 5 degrees, too little to bring back a drifting drone, 2026-11-02), on the tag they move the hold point at hover_nudge_speed, the left and right arrows turn at hover_yaw_rate and W and S climb at hover_climb_rate.
  const keySizes = useRef({ tilt: 15, yaw: 30, climb: 0.3, move: 0.5 });
  keySizes.current = {
    tilt: setting('hover_tilt') ?? 15,
    yaw: setting('hover_yaw_rate') ?? 30,
    climb: setting('hover_climb_rate') ?? 0.3,
    move: setting('hover_nudge_speed') ?? 0.5,
  };
  const free = settings.hover_camera === 0;
  const freeRef = useRef(free);
  freeRef.current = free;
  const throttle = useRef(settings.base_throttle);
  // Key handlers read the mode through a ref so a mode change doesn't re-register them and forget held keys.
  const modeRef = useRef(mode);
  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);

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

    function setMode(next: number) {
      modeRef.current = next;
      onMode(next);
    }

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
        (Number(held.has('ArrowLeft')) - Number(held.has('ArrowRight')));
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
      if (forward || left || up) onNudge(forward, left, up);
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
      // There is no manual flight: outside hover only L and yaw act. In hover W and S, and on the tag the arrows and A and D, move the hold point while held, and are not held flight keys, so their keyup and a blur leave hover alone; without the tag the arrows and A and D are held tilt.
      const hovering = modeRef.current !== MANUAL;
      if (!hovering && (event.code in nudgeKeys || event.code in tiltKeys))
        return;
      if (
        event.code in nudgeKeys ||
        (event.code in moveKeys && !freeRef.current)
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
      changeTilt('KeyA');
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
  }, [available, settings.enabled, open, onChange, onMode, onNudge, onStop]);

  // H (or T) works disarmed too: it arms like Enable, then hovers, taking off.
  useEffect(() => {
    if (!available || open) return;
    function keydown(event: KeyboardEvent) {
      if ((event.code !== 'KeyH' && event.code !== 'KeyT') || typing(event))
        return;
      event.preventDefault();
      event.stopPropagation();
      if (event.repeat) return;
      if (!settings.enabled && !ready) return;
      onHover();
    }
    window.addEventListener('keydown', keydown, true);
    return () => window.removeEventListener('keydown', keydown, true);
  }, [available, ready, settings.enabled, open, onHover]);

  return (
    <section
      className="space-y-4 border-b px-4 py-4"
      aria-labelledby="control-heading"
    >
      <div className="flex items-center gap-2">
        <h2 id="control-heading" className="mr-auto font-semibold">
          Flight control
        </h2>
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
                            <span className="text-muted-foreground">
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
          variant="outline"
          disabled={!connected || (!settings.enabled && !ready)}
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
          {settings.enabled ? 'Disable' : 'Enable'}
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
      <div className="grid grid-cols-2 gap-x-4 gap-y-1.5 text-sm text-muted-foreground">
        {[
          { keys: ['T', 'H'], label: 'Take off' },
          { keys: ['L'], label: 'Land (hold: fast)' },
          { keys: ['W', 'S'], label: 'Up / down' },
          { keys: ['ArrowLeft', 'ArrowRight'], label: 'Yaw' },
          {
            keys: ['ArrowUp', 'ArrowDown'],
            label: free ? 'Tilt fwd / back' : 'Target closer / farther',
          },
          {
            keys: ['A', 'D'],
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
        className={`h-6 truncate ${problem ? 'text-red-700' : 'text-green-700'}`}
      >
        {problem || 'Good to fly.'}
      </p>
    </section>
  );
});
