'use client';

import { useState } from 'react';
import { Gauge } from 'lucide-react';
import { Button } from '@/components/ui/button';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';

// The ESCs learn their throttle range from the drone (fw/drone/main/flight_rt.c ESC calibration): the top pulse while the battery is plugged in, then the bottom. Stage 0 is off, 1 the top, 2 the bottom; the drone ends it by itself after 2 minutes.
const steps = [
  {
    text: 'Take both props off and unplug the battery. The ESCs will be sent full throttle.',
    action: 'Props are off, start',
    stage: 1,
  },
  {
    text: 'Plug in the battery. Wait for the ESCs to beep that they have the top of the range.',
    action: 'Beeped, next',
    stage: 2,
  },
  {
    text: 'Wait for the ESCs to beep again for the bottom of the range.',
    action: 'Beeped, finish',
    stage: 0,
  },
] as const;

export function EscCalibration({
  disabled,
  error,
  onStage,
}: {
  disabled: boolean;
  // The drone's answer to the last stage sent, if it refused it.
  error: string | null;
  onStage: (stage: 0 | 1 | 2) => void;
}) {
  const [open, setOpen] = useState(false);
  const [step, setStep] = useState(0);
  const current = steps[step];

  return (
    <Dialog
      open={open}
      onOpenChange={(value) => {
        // Closing at any step ends the calibration on the drone.
        if (!value && step > 0) onStage(0);
        setStep(0);
        setOpen(value);
      }}
    >
      <DialogTrigger
        render={
          <Button
            variant="ghost"
            size="icon"
            aria-label="Calibrate ESCs"
            disabled={disabled}
          />
        }
      >
        <Gauge />
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Calibrate ESCs</DialogTitle>
        </DialogHeader>
        <p className="text-muted-foreground">
          Step {step + 1} of {steps.length}
        </p>
        <p>{current.text}</p>
        {error && <p className="text-red-700">{error}</p>}
        <Button
          onClick={() => {
            onStage(current.stage);
            if (step + 1 < steps.length) setStep(step + 1);
            else {
              setStep(0);
              setOpen(false);
            }
          }}
        >
          {current.action}
        </Button>
      </DialogContent>
    </Dialog>
  );
}
