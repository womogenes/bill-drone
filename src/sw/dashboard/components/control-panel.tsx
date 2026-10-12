'use client';

import { useState } from 'react';
import { Power } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Slider } from '@/components/ui/slider';
import type { ControlSettings } from '@/lib/telemetry';

const ANGLE_LIMIT = 60;

export function ControlPanel({
  settings,
  connected,
  onChange,
}: {
  settings: ControlSettings;
  connected: boolean;
  onChange: (settings: Partial<ControlSettings>) => void;
}) {
  const [target, setTarget] = useState<number | null>(null);
  const [gains, setGains] = useState({
    kp: String(settings.kp),
    ki: String(settings.ki),
    kd: String(settings.kd),
    base_throttle: String(settings.base_throttle),
  });
  return (
    <section
      aria-labelledby="control-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <div className="flex items-center justify-between">
        <h2 id="control-heading" className="font-semibold">
          Roll control
        </h2>
        <Button
          variant="outline"
          size="sm"
          aria-pressed={settings.enabled}
          disabled={!connected}
          onClick={() => onChange({ enabled: !settings.enabled })}
        >
          <Power />
          {settings.enabled ? 'On' : 'Off'}
        </Button>
      </div>
      <div className="flex justify-between text-sm">
        <label id="target-angle-label">Set angle</label>
        <span className="tabular-nums">{target ?? settings.target}°</span>
      </div>
      <Slider
        aria-labelledby="target-angle-label"
        min={-ANGLE_LIMIT}
        max={ANGLE_LIMIT}
        step={1}
        disabled={!connected}
        value={[target ?? settings.target]}
        onValueChange={(value) => {
          const angle = Array.isArray(value) ? value[0] : value;
          setTarget(angle);
          onChange({ target: angle });
        }}
        onValueCommitted={() => setTarget(null)}
      />
      <form
        className="space-y-3"
        onSubmit={(event) => {
          event.preventDefault();
          onChange({
            kp: Number(gains.kp),
            ki: Number(gains.ki),
            kd: Number(gains.kd),
            base_throttle: Number(gains.base_throttle),
          });
        }}
      >
        <div className="grid grid-cols-3 gap-2">
          {(['kp', 'ki', 'kd'] as const).map((gain) => (
            <div key={gain} className="space-y-1">
              <label
                htmlFor={`pid-${gain}`}
                className="text-sm text-muted-foreground"
              >
                {gain.toUpperCase()}
              </label>
              <Input
                id={`pid-${gain}`}
                type="number"
                step="any"
                min={0}
                required
                disabled={!connected}
                value={gains[gain]}
                onChange={(event) =>
                  setGains({ ...gains, [gain]: event.target.value })
                }
              />
            </div>
          ))}
        </div>
        <div className="space-y-1">
          <label
            htmlFor="base-throttle"
            className="text-sm text-muted-foreground"
          >
            Base throttle
          </label>
          <Input
            id="base-throttle"
            type="number"
            min={0}
            max={1000}
            step={1}
            required
            disabled={!connected}
            value={gains.base_throttle}
            onChange={(event) =>
              setGains({ ...gains, base_throttle: event.target.value })
            }
          />
        </div>
        <Button type="submit" variant="outline" size="sm" disabled={!connected}>
          Apply PID
        </Button>
      </form>
    </section>
  );
}
