'use client';

import { THROTTLE_MAX } from '@/lib/telemetry';

import { memo, useState } from 'react';
import { Plug, RotateCcw, Square, Unplug } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Slider } from '@/components/ui/slider';
import type { Side } from '@/lib/telemetry';

const SERVO_LIMIT = 60;

export const ActuatorPanel = memo(function ActuatorPanel({
  side,
  kind,
  value: angle,
  connected,
  locked = false,
  onChange,
  onZero,
  zeroDisabled,
}: {
  side: Side;
  kind: 'servo' | 'throttle';
  value: number | undefined;
  connected: boolean;
  locked?: boolean;
  onZero: (side: Side) => void;
  zeroDisabled: boolean;
  onChange: (kind: 'servo' | 'throttle', side: Side, value: number) => void;
}) {
  const isServo = kind === 'servo';
  const title = `${side === 'left' ? 'Left' : 'Right'} ${isServo ? 'servo' : 'ESC'}`;
  const heading = `${side}-${kind}-heading`;
  const min = isServo ? -SERVO_LIMIT : 0;
  const max = isServo ? SERVO_LIMIT : THROTTLE_MAX;
  const suffix = isServo ? '°' : '';
  const ResetIcon = isServo ? RotateCcw : Square;
  const [draft, setDraft] = useState<number | null>(null);
  const [dragging, setDragging] = useState(false);

  // Keep the released position visible until the ESP acknowledges it.
  if (!dragging && draft !== null && (angle === draft || !connected)) {
    setDraft(null);
  }

  function commit(value: number) {
    setDraft(value);
    setDragging(false);
    onChange(kind, side, value);
  }
  const disabled = !connected || angle === undefined;
  const StatusIcon = disabled ? Unplug : Plug;

  return (
    <section
      aria-labelledby={heading}
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <div className="flex items-center gap-4">
        <h2 id={heading} className="font-semibold">
          {title}
        </h2>
        <span
          role="status"
          className={`flex items-center gap-2 text-sm ${disabled ? 'text-red-700' : 'text-green-700'}`}
        >
          <StatusIcon aria-hidden="true" className="size-4" />
          {disabled ? 'Offline' : 'Online'}
        </span>
        <div className="ml-auto flex items-center gap-2">
          {isServo && (
            <Button
              variant="outline"
              className="cursor-pointer"
              aria-label={`Set ${title} zero`}
              disabled={disabled || locked || zeroDisabled || dragging}
              onClick={() => {
                onZero(side);
                setDraft(null);
              }}
            >
              Set zero
            </Button>
          )}
          <Button
            className="cursor-pointer"
            variant="ghost"
            size="icon"
            aria-label={isServo ? `Reset ${title}` : `Stop ${title}`}
            title={isServo ? `Reset ${title}` : `Stop ${title}`}
            disabled={disabled || locked}
            onClick={() => commit(0)}
          >
            <ResetIcon />
          </Button>
        </div>
      </div>
      <div>
        <p className="text-sm text-muted-foreground">
          {isServo ? 'Angle' : 'Throttle'}
        </p>
        <p className="mt-1 text-3xl font-medium tabular-nums">
          {disabled ? '—' : `${draft ?? angle}${suffix}`}
        </p>
      </div>
      <div className="space-y-3">
        <Slider
          aria-label={title}
          min={min}
          max={max}
          step={1}
          value={[draft ?? angle ?? 0]}
          disabled={disabled || locked}
          onValueChange={(value) => {
            const next = Array.isArray(value) ? value[0] : value;
            setDraft(next);
            setDragging(true);
            onChange(kind, side, next);
          }}
          onValueCommitted={() => setDragging(false)}
        />
        <div className="flex justify-between text-sm text-muted-foreground tabular-nums">
          <span>
            {min}
            {suffix}
          </span>
          {isServo && <span>0°</span>}
          <span>
            {max}
            {suffix}
          </span>
        </div>
      </div>
    </section>
  );
});
