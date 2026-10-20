'use client';

import { THROTTLE_MAX } from '@/lib/telemetry';

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
const KEY_REPEAT_MS = 100;
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
  { key: 'roll_bias', label: 'Roll bias (rad/s²)', min: -10, max: 10 },
  { key: 'pitch_bias', label: 'Pitch bias (rad/s²)', min: -10, max: 10 },
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
  onChange,
  onStop,
}: {
  settings: ControlSettings;
  recorderReady: boolean;
  recorderError?: string;
  imu: 'offline' | 'calibrating' | 'uncalibrated' | 'calibrated';
  status: Reading['control_status'];
  connected: boolean;
  onChange: (settings: Partial<ControlSettings>) => void;
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
  useEffect(() => setManual(readManual()), []);
  const throttle = useRef(settings.base_throttle);

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

    function keydown(event: KeyboardEvent) {
      if (event.code === 'Space') {
        held.clear();
        clearInterval(timer);
        return;
      }
      if (
        !['KeyW', 'KeyS', 'KeyA', 'KeyD'].includes(event.code) &&
        !(event.code in tiltKeys)
      )
        return;
      if (event.ctrlKey || event.metaKey || event.altKey) return;
      if (
        event.target instanceof HTMLElement &&
        event.target.closest(
          'input, textarea, select, [contenteditable="true"], [role="dialog"]'
        )
      )
        return;

      event.preventDefault();
      event.stopPropagation();
      if (event.repeat || held.has(event.code)) return;
      held.add(event.code);

      if (event.code === 'KeyW' || event.code === 'KeyS') {
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
      if (!held.delete(event.code)) return;
      event.preventDefault();
      if (event.code === 'KeyA' || event.code === 'KeyD') changeYaw();
      if (event.code in tiltKeys)
        changeTilt(event.code as keyof typeof tiltKeys);
      if (!held.has('KeyW') && !held.has('KeyS')) {
        clearInterval(timer);
        timer = undefined;
      }
    }

    function blur() {
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
      window.removeEventListener('keydown', keydown, true);
      window.removeEventListener('keyup', keyup);
      window.removeEventListener('blur', blur);
    };
  }, [available, settings.enabled, open, onChange, onStop, manual]);

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
          { keys: ['W', 'S'], label: 'Throttle' },
          { keys: ['A', 'D'], label: 'Yaw' },
          { keys: ['ArrowUp', 'ArrowDown'], label: 'Pitch' },
          { keys: ['ArrowLeft', 'ArrowRight'], label: 'Roll' },
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
          </div>
        ))}
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
