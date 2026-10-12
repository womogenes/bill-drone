'use client';

import { memo, useEffect, useRef, useState } from 'react';
import { Plug, RotateCcw, Unplug } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Slider } from '@/components/ui/slider';

const SERVO_LIMIT = 60;

export const ServoPanel = memo(function ServoPanel({
  angle,
  connected,
  onChange,
}: {
  angle: number | undefined;
  connected: boolean;
  onChange: (angle: number) => void;
}) {
  const [draft, setDraft] = useState<number | null>(null);
  const [dragging, setDragging] = useState(false);
  const lastSent = useRef(-Infinity);
  const pending = useRef<number | null>(null);
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

  useEffect(() => () => clearTimeout(timer.current), []);

  // Keep the released position visible until the ESP acknowledges it.
  if (!dragging && draft !== null && (angle === draft || !connected)) {
    setDraft(null);
  }

  function sendLatest() {
    clearTimeout(timer.current);
    timer.current = undefined;
    if (pending.current === null) return;
    onChange(pending.current);
    pending.current = null;
    lastSent.current = performance.now();
  }

  function commit(value: number) {
    setDraft(value);
    setDragging(false);
    pending.current = value;
    sendLatest();
  }
  const disabled = !connected || angle === undefined;
  const StatusIcon = disabled ? Unplug : Plug;

  return (
    <section
      aria-labelledby="servo-heading"
      className="w-full max-w-md space-y-6 px-6 py-8"
    >
      <div className="flex items-center gap-4">
        <h2 id="servo-heading" className="font-semibold">
          Servo
        </h2>
        <span
          role="status"
          className={`flex items-center gap-2 text-sm ${disabled ? 'text-red-700' : 'text-green-700'}`}
        >
          <StatusIcon aria-hidden="true" className="size-4" />
          {disabled ? 'Offline' : 'Online'}
        </span>
        <Button
          className="ml-auto"
          variant="ghost"
          size="icon"
          aria-label="Center servo"
          title="Center servo"
          disabled={disabled}
          onClick={() => commit(0)}
        >
          <RotateCcw />
        </Button>
      </div>
      <div>
        <p className="text-sm text-muted-foreground">Angle</p>
        <p className="mt-1 text-3xl font-medium tabular-nums">
          {angle === undefined ? '—' : `${draft ?? angle}°`}
        </p>
      </div>
      <div className="space-y-3">
        <Slider
          aria-label="Servo angle"
          min={-SERVO_LIMIT}
          max={SERVO_LIMIT}
          step={1}
          value={[draft ?? angle ?? 0]}
          disabled={disabled}
          onValueChange={(value) => {
            const next = Array.isArray(value) ? value[0] : value;
            setDraft(next);
            setDragging(true);
            pending.current = next;
            const delay = 50 - (performance.now() - lastSent.current);
            if (delay <= 0) sendLatest();
            else if (timer.current === undefined) {
              timer.current = setTimeout(sendLatest, delay);
            }
          }}
          onValueCommitted={(value) => {
            commit(Array.isArray(value) ? value[0] : value);
          }}
        />
        <div className="flex justify-between text-sm text-muted-foreground tabular-nums">
          <span>−{SERVO_LIMIT}°</span>
          <span>0°</span>
          <span>{SERVO_LIMIT}°</span>
        </div>
      </div>
    </section>
  );
});
