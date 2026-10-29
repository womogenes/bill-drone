'use client';

import { FLIGHT_MODES, THROTTLE_MAX } from '@/lib/telemetry';

import { memo, useCallback, useEffect, useRef, useState } from 'react';
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Check,
  Space,
  Settings2,
  Square,
  type LucideIcon,
} from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Kbd, KbdGroup } from '@/components/ui/kbd';
import { Slider } from '@/components/ui/slider';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';
import type { ControlSettings, Reading } from '@/lib/telemetry';
import { signed } from '@/lib/format';

const keyIcons: Record<string, LucideIcon> = {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Space,
};

const THROTTLE_STEP = 10;
// T jumps collective straight to about hover, so liftoff doesn't linger half-supported on the feet.
const TAKEOFF_DEFAULT = 690;
const TAKEOFF_KEY = 'takeoffThrottle';

// The takeoff throttle the T key sets, remembered per browser.
function savedTakeoff() {
  try {
    const value = Number(localStorage.getItem(TAKEOFF_KEY));
    return value > 0 && value <= THROTTLE_MAX ? value : TAKEOFF_DEFAULT;
  } catch {
    return TAKEOFF_DEFAULT;
  }
}
const KEY_REPEAT_MS = 100;
// L hands the drone its controlled landing (a steady descent, then motors off at touchdown), from hover or manual flight; holding it past LAND_FAST_MS descends faster.
const LAND_FAST_MS = 300;
const MANUAL = 0;
const LAND = 2;
const LAND_FAST = 3;

// Keys typed into a field or a dialog aren't flight commands.
function typing(event: KeyboardEvent) {
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
// Keyboard setpoint sizes are a local preference, not drone settings.
const MANUAL_KEY = 'manual-inputs';
const MANUAL_DEFAULT = { tilt: 15, yaw: 20 };
function readManual() {
  try {
    return {
      ...MANUAL_DEFAULT,
      ...JSON.parse(localStorage.getItem(MANUAL_KEY) ?? '{}'),
    };
  } catch {
    return MANUAL_DEFAULT;
  }
}
// Positive pitch is nose down, so the up arrow tilts forward.
const tiltKeys = {
  ArrowUp: { axis: 'pitch', direction: 1 },
  ArrowDown: { axis: 'pitch', direction: -1 },
  ArrowLeft: { axis: 'roll', direction: -1 },
  ArrowRight: { axis: 'roll', direction: 1 },
} as const;
// In hover and landing the arrows move the hold point instead while held (m/s): up toward the tag, left to the drone's left; W and S raise and lower it.
const NUDGE_SPEED = 0.2;
const NUDGE_CLIMB = 0.15;
const NUDGE_TICK_MS = 50;
const nudgeKeys = {
  ArrowUp: [NUDGE_SPEED, 0, 0],
  ArrowDown: [-NUDGE_SPEED, 0, 0],
  ArrowLeft: [0, NUDGE_SPEED, 0],
  ArrowRight: [0, -NUDGE_SPEED, 0],
  KeyW: [0, 0, NUDGE_CLIMB],
  KeyS: [0, 0, -NUDGE_CLIMB],
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

const controls = [
  { key: 'roll', label: 'Roll', min: -15, max: 15, suffix: '°' },
  { key: 'pitch', label: 'Pitch', min: -15, max: 15, suffix: '°' },
  { key: 'yaw_rate', label: 'Yaw rate', min: -45, max: 45, suffix: '°/s' },
  {
    key: 'base_throttle',
    label: 'Collective throttle',
    min: 0,
    max: THROTTLE_MAX,
    suffix: '',
  },
] as const;
type Setpoint = (typeof controls)[number]['key'];
const tuningFields = [
  { key: 'roll_p', label: 'Roll P', min: 0 },
  { key: 'roll_i', label: 'Roll I', min: 0 },
  { key: 'roll_damping', label: 'Roll damping', min: 0 },
  { key: 'pitch_p', label: 'Pitch P', min: 0 },
  { key: 'pitch_i', label: 'Pitch I', min: 0 },
  { key: 'pitch_damping', label: 'Pitch damping', min: 0 },
  { key: 'yaw_damping', label: 'Yaw damping', min: 0 },
  { key: 'integral_limit', label: 'Integral limit', min: 0 },
  { key: 'pitch_trim', label: 'Pitch trim (°)', min: -15 },
  { key: 'left_trim', label: 'Left servo trim (°)', min: -15 },
  { key: 'right_trim', label: 'Right servo trim (°)', min: -15 },
  { key: 'servo_compensation', label: 'Servo compensation (°)', min: 0 },
  { key: 'integrate_throttle', label: 'Integrate above throttle', min: 0 },
  { key: 'target_rate', label: 'Target ramp (°/s)', min: 1 },
  { key: 'velocity_damping', label: 'Velocity damping (°/(m/s))', min: 0 },
  { key: 'velocity_leak', label: 'Velocity leak (s)', min: 0.1 },
  { key: 'thrust_voltage', label: 'Thrust reference (V, 0 off)', min: 0 },
  { key: 'yaw_i', label: 'Yaw I', min: 0 },
  { key: 'hover_center', label: 'Hover centres on tag (1 on, 0 off)', min: 0 },
  { key: 'hover_distance', label: 'Hover distance from tag (m, 0 = takeoff range)', min: 0 },
  { key: 'hover_approach', label: 'Hover approach speed (m/s)', min: 0.02 },
  { key: 'hover_ceiling', label: 'Hover ceiling above takeoff (m)', min: 0.2 },
  { key: 'hover_square', label: 'Hover moves square to tag (1 on, 0 off)', min: 0 },
  { key: 'hover_kside', label: 'Hover pull to square-on (m/s² per m)', min: 0 },
  { key: 'hover_cam_pitch', label: 'Camera tilt up from body (°)', min: -30 },
  { key: 'hover_cam_roll', label: 'Camera roll about its axis (°, 180 upside down)', min: -180 },
  { key: 'hover_drag', label: 'Hover rotor drag (1/s, 0 off)', min: 0 },
  { key: 'hover_drag_tilt', label: 'Hover drag thrust tilt offset (°)', min: -10 },
  { key: 'hover_drag_tau', label: 'Hover drag correction time (s)', min: 0.1 },
  { key: 'hover_trim_tau', label: 'Hover trim build-up time (s)', min: 0.1 },
  { key: 'hover_trim_max', label: 'Hover trim limit (m/s²)', min: 0 },
  { key: 'auto_trim', label: 'Self-trim per hop (0 off - 1)', min: 0, max: 1 },
] as const;
const modelFields = [
  { key: 'pivot_height', label: 'Pivot height (m)', min: 0, max: undefined },
  { key: 'com_x', label: 'CoM x (m)', min: -0.02, max: 0.02 },
  {
    key: 'roll_inertia',
    label: 'Roll inertia (kg·m²)',
    min: 0,
    max: undefined,
  },
  {
    key: 'pitch_inertia',
    label: 'Pitch inertia (kg·m²)',
    min: 0,
    max: undefined,
  },
  { key: 'yaw_inertia', label: 'Yaw inertia (kg·m²)', min: 0, max: undefined },
  {
    key: 'reaction_torque_ratio',
    label: 'Reaction torque ratio',
    min: -0.1,
    max: 0.1,
  },
  { key: 'arm', label: 'Arm (m)', min: 0, max: undefined },
] as const;
const biasFields = [
  { key: 'roll_bias', label: 'Roll bias (rad/s²)', min: -20, max: 20 },
  { key: 'pitch_bias', label: 'Pitch bias (rad/s²)', min: -20, max: 20 },
  { key: 'yaw_bias', label: 'Yaw bias (rad/s²)', min: -10, max: 10 },
  { key: 'left_thrust_scale', label: 'Left thrust scale', min: 0.5, max: 1.5 },
  {
    key: 'right_thrust_scale',
    label: 'Right thrust scale',
    min: 0.5,
    max: 1.5,
  },
] as const;
const gainKeys = [...tuningFields, ...modelFields, ...biasFields].map(
  ({ key }) => key
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
  const [open, setOpen] = useState(false);
  const [manual, setManual] = useState(MANUAL_DEFAULT);
  const [manualDraft, setManualDraft] = useState(MANUAL_DEFAULT);
  const [takeoff, setTakeoff] = useState(TAKEOFF_DEFAULT);
  const [takeoffDraft, setTakeoffDraft] = useState(String(TAKEOFF_DEFAULT));
  const takeoffRef = useRef(TAKEOFF_DEFAULT);
  useEffect(() => {
    const value = savedTakeoff();
    takeoffRef.current = value;
    setTakeoff(value);
    setTakeoffDraft(String(value));
  }, []);
  const draftTakeoff = Math.round(Number(takeoffDraft));
  const takeoffValid =
    takeoffDraft.trim() !== '' &&
    draftTakeoff > 0 &&
    draftTakeoff <= THROTTLE_MAX;
  const confirmTakeoff = () => {
    if (!takeoffValid) return;
    takeoffRef.current = draftTakeoff;
    setTakeoff(draftTakeoff);
    setTakeoffDraft(String(draftTakeoff));
    try {
      localStorage.setItem(TAKEOFF_KEY, String(draftTakeoff));
    } catch {}
  };
  useEffect(() => setManual(readManual()), []);
  const throttle = useRef(settings.base_throttle);
  // Key handlers read the mode through a ref so a mode change doesn't re-register them and forget held keys.
  const modeRef = useRef(mode);
  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);

  const changeSetpoint = useCallback(
    (key: Setpoint, value: number) => {
      if (key === 'base_throttle') throttle.current = value;
      setDraft((previous) => ({ ...previous, [key]: value }));
      onChange({ [key]: value });
    },
    [onChange]
  );

  useEffect(() => {
    throttle.current = draft.base_throttle ?? settings.base_throttle;
  }, [draft.base_throttle, settings.base_throttle]);

  useEffect(() => {
    if (!available || !settings.enabled || open) return;

    const held = new Set<string>();
    let timer: ReturnType<typeof setInterval> | undefined;
    // Arrows held for a hover nudge, and the timer moving the hold point while any is.
    const nudging = new Set<keyof typeof nudgeKeys>();
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

    function changeThrottle() {
      const direction = Number(held.has('KeyW')) - Number(held.has('KeyS'));
      if (!direction) return;

      const next = Math.max(
        0,
        Math.min(THROTTLE_MAX, throttle.current + direction * THROTTLE_STEP)
      );
      throttle.current = next;
      setDraft((previous) => ({ ...previous, base_throttle: next }));
      onChange({ base_throttle: next });
    }

    function changeYaw() {
      const yaw_rate =
        manual.yaw * (Number(held.has('KeyA')) - Number(held.has('KeyD')));
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
      const value = manual.tilt * direction;
      setDraft((previous) => ({ ...previous, [axis]: value }));
      onChange({ [axis]: value });
    }

    function nudge() {
      const tick = NUDGE_TICK_MS / 1000;
      let forward = 0;
      let left = 0;
      let up = 0;
      for (const code of nudging) {
        forward += nudgeKeys[code][0] * tick;
        left += nudgeKeys[code][1] * tick;
        up += nudgeKeys[code][2] * tick;
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
        clearInterval(timer);
        clearTimeout(landFast);
        return;
      }
      if (
        !['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyT', 'KeyL'].includes(
          event.code
        ) &&
        !(event.code in tiltKeys)
      )
        return;
      if (typing(event)) return;

      event.preventDefault();
      event.stopPropagation();
      if (event.repeat || held.has(event.code)) return;
      // A nudge key moves the hold point while held; it is not a held flight key, so its keyup and a blur leave hover alone.
      if (event.code in nudgeKeys && modeRef.current !== MANUAL) {
        nudging.add(event.code as keyof typeof nudgeKeys);
        nudge();
        if (nudgeTimer === undefined)
          nudgeTimer = setInterval(nudge, NUDGE_TICK_MS);
        return;
      }
      held.add(event.code);

      if (event.code === 'KeyL') {
        land();
      } else if (event.code === 'KeyT') {
        held.delete('KeyT');
        const value = takeoffRef.current;
        throttle.current = value;
        setDraft((previous) => ({ ...previous, base_throttle: value }));
        onChange({ base_throttle: value });
      } else if (event.code === 'KeyW' || event.code === 'KeyS') {
        changeThrottle();
        if (timer === undefined)
          timer = setInterval(changeThrottle, KEY_REPEAT_MS);
      } else if (event.code in tiltKeys) {
        changeTilt(event.code as keyof typeof tiltKeys);
      } else {
        changeYaw();
      }
    }

    function keyup(event: KeyboardEvent) {
      if (nudging.delete(event.code as keyof typeof nudgeKeys)) {
        if (!nudging.size) stopNudging();
        return;
      }
      if (!held.delete(event.code)) return;
      event.preventDefault();
      if (event.code === 'KeyL') releaseLand();
      if (event.code === 'KeyA' || event.code === 'KeyD') changeYaw();
      if (event.code in tiltKeys)
        changeTilt(event.code as keyof typeof tiltKeys);
      if (!held.has('KeyW') && !held.has('KeyS')) {
        clearInterval(timer);
        timer = undefined;
      }
    }

    // A lost keyup can't be trusted, so held keys stop the motors; L alone just drops back from land fast, and landing carries on.
    function blur() {
      stopNudging();
      if (held.has('KeyL')) releaseLand();
      held.delete('KeyL');
      if (held.size) onStop();
      held.clear();
      clearInterval(timer);
      timer = undefined;
    }

    window.addEventListener('keydown', keydown, true);
    window.addEventListener('keyup', keyup);
    window.addEventListener('blur', blur);
    return () => {
      clearInterval(timer);
      clearInterval(nudgeTimer);
      clearTimeout(landFast);
      window.removeEventListener('keydown', keydown, true);
      window.removeEventListener('keyup', keyup);
      window.removeEventListener('blur', blur);
    };
  }, [
    available,
    settings.enabled,
    open,
    onChange,
    onMode,
    onNudge,
    onStop,
    manual,
  ]);

  // H works disarmed too: it arms like Enable, then hovers.
  useEffect(() => {
    if (!available || open) return;
    function keydown(event: KeyboardEvent) {
      if (event.code !== 'KeyH' || typing(event)) return;
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
            if (value) setManualDraft(manual);
            if (value)
              setGains(
                Object.fromEntries(
                  gainKeys
                    .filter((key) => settings[key] !== undefined)
                    .map((key) => [
                      key,
                      String(Number(settings[key].toPrecision(5))),
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
          <DialogContent className="sm:max-w-3xl">
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
                const next = {
                  tilt: Number(manualDraft.tilt),
                  yaw: Number(manualDraft.yaw),
                };
                setManual(next);
                try {
                  localStorage.setItem(MANUAL_KEY, JSON.stringify(next));
                } catch {}
                setOpen(false);
              }}
            >
              <div className="grid grid-cols-2 gap-3">
                {tuningFields.map(({ key, label, min }) => (
                  <label
                    key={key}
                    className="flex items-center justify-between gap-3"
                  >
                    {label}
                    <Input
                      className="w-32"
                      aria-label={label}
                      type="number"
                      min={min}
                      max={key.endsWith('_trim') ? 15 : undefined}
                      step="any"
                      required
                      value={gains[key] ?? ''}
                      onChange={(event) =>
                        setGains({ ...gains, [key]: event.target.value })
                      }
                    />
                  </label>
                ))}
              </div>
              {settings.arm !== undefined && (
                <section className="space-y-3">
                  <h3 className="font-medium">Model</h3>
                  <div className="grid grid-cols-2 gap-3">
                    {modelFields.map(({ key, label, min, max }) => (
                      <label
                        key={key}
                        className="flex items-center justify-between gap-3"
                      >
                        {label}
                        <Input
                          className="w-32"
                          aria-label={label}
                          type="number"
                          min={min}
                          max={max}
                          step="any"
                          required
                          value={gains[key] ?? ''}
                          onChange={(event) =>
                            setGains({ ...gains, [key]: event.target.value })
                          }
                        />
                      </label>
                    ))}
                  </div>
                </section>
              )}
              {settings.roll_bias !== undefined && (
                <section className="space-y-3">
                  <h3 className="font-medium">Bias and thrust scale</h3>
                  <div className="grid grid-cols-2 gap-3">
                    {biasFields.map(({ key, label, min, max }) => (
                      <label
                        key={key}
                        className="flex items-center justify-between gap-3"
                      >
                        {label}
                        <Input
                          className="w-32"
                          aria-label={label}
                          type="number"
                          min={min}
                          max={max}
                          step="any"
                          required
                          value={gains[key] ?? ''}
                          onChange={(event) =>
                            setGains({ ...gains, [key]: event.target.value })
                          }
                        />
                      </label>
                    ))}
                  </div>
                </section>
              )}
              <section className="space-y-3">
                <h3 className="font-medium">Manual inputs</h3>
                <div className="grid grid-cols-2 gap-3">
                  <label className="flex items-center justify-between gap-3">
                    Pitch/roll step (°)
                    <Input
                      className="w-32"
                      aria-label="Pitch/roll step (°)"
                      type="number"
                      min={0}
                      max={15}
                      step="any"
                      required
                      value={manualDraft.tilt}
                      onChange={(event) =>
                        setManualDraft({
                          ...manualDraft,
                          tilt: Number(event.target.value),
                        })
                      }
                    />
                  </label>
                  <label className="flex items-center justify-between gap-3">
                    Yaw rate (°/s)
                    <Input
                      className="w-32"
                      aria-label="Yaw rate (°/s)"
                      type="number"
                      min={0}
                      max={45}
                      step="any"
                      required
                      value={manualDraft.yaw}
                      onChange={(event) =>
                        setManualDraft({
                          ...manualDraft,
                          yaw: Number(event.target.value),
                        })
                      }
                    />
                  </label>
                </div>
              </section>
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
      <div className="flex flex-col gap-2 text-muted-foreground">
        {[
          {
            keys: ['W', 'S'],
            label: 'Throttle; in hover, raise or lower the hold',
          },
          { keys: ['T'], label: 'Takeoff' },
          { keys: ['H'], label: 'Hover (arms if needed)' },
          { keys: ['L'], label: 'Land (hold for fast)' },
          { keys: ['A', 'D'], label: 'Yaw' },
          {
            keys: ['ArrowUp', 'ArrowDown'],
            label: 'Pitch; in hover, move the hold closer or farther',
          },
          {
            keys: ['ArrowLeft', 'ArrowRight'],
            label: 'Roll; in hover, move the hold left or right',
          },
          { keys: ['Space'], label: 'Stop' },
        ].map(({ keys, label }) => (
          <div key={label} className="flex items-center gap-2">
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
            <span>{label}</span>
            {label === 'Takeoff' && (
              <form
                className="flex items-center gap-1"
                noValidate
                onSubmit={(event) => {
                  event.preventDefault();
                  confirmTakeoff();
                }}
              >
                <Input
                  type="number"
                  min={0}
                  max={THROTTLE_MAX}
                  step={10}
                  value={takeoffDraft}
                  onChange={(event) => setTakeoffDraft(event.target.value)}
                  aria-label="Takeoff throttle"
                  className="h-7 w-20 tabular-nums"
                />
                <Button
                  type="submit"
                  variant="outline"
                  size="icon"
                  className="size-7"
                  aria-label="Set takeoff throttle"
                  disabled={!takeoffValid || draftTakeoff === takeoff}
                >
                  <Check />
                </Button>
              </form>
            )}
          </div>
        ))}
        {(mode !== MANUAL || nudge.some(Boolean)) && (
          <span className="tabular-nums">{nudgeText(nudge)}</span>
        )}
      </div>
      <p
        role="status"
        className={`h-6 truncate ${problem ? 'text-red-700' : 'text-green-700'}`}
      >
        {problem || 'Good to fly.'}
      </p>
      {controls.map((control) => (
        <SetpointSlider
          key={control.key}
          control={control}
          value={draft[control.key] ?? settings[control.key]}
          disabled={!available || !settings.enabled}
          onChange={changeSetpoint}
        />
      ))}
      <Button
        variant="outline"
        disabled={!available || !settings.enabled}
        onClick={() => {
          setDraft((previous) => ({ ...previous, roll: 0, pitch: 0 }));
          onChange({ roll: 0, pitch: 0 });
        }}
      >
        Level
      </Button>
    </section>
  );
});

const SetpointSlider = memo(function SetpointSlider({
  control: { key, label, min, max, suffix },
  value,
  disabled,
  onChange,
}: {
  control: (typeof controls)[number];
  value: number;
  disabled: boolean;
  onChange: (key: Setpoint, value: number) => void;
}) {
  return (
    <div className="space-y-2">
      <div className="flex justify-between tabular-nums">
        <label id={`control-${key}`}>{label}</label>
        <span>
          {key === 'base_throttle' ? value : signed(value, 0)}
          {suffix}
        </span>
      </div>
      <Slider
        aria-labelledby={`control-${key}`}
        min={min}
        max={max}
        step={1}
        value={[value]}
        disabled={disabled}
        onValueChange={(values) =>
          onChange(key, Array.isArray(values) ? values[0] : values)
        }
      />
    </div>
  );
});
