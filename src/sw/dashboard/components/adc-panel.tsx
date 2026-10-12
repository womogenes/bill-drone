'use client';

import { memo, useState } from 'react';
import { Settings2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';

const STORAGE_KEY = 'adc-calibration';

export const ADCPanel = memo(function ADCPanel({
  raw,
  voltage,
  connected,
}: {
  raw: number | undefined;
  voltage: number | undefined;
  connected: boolean;
}) {
  const [editing, setEditing] = useState(false);
  const [zero, setZero] = useState('0');
  const [scale, setScale] = useState('');
  const available = connected && raw !== undefined;

  function openCalibration() {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      const calibration = JSON.parse(saved);
      setZero(String(calibration.zero));
      setScale(String(calibration.unitsPerAmp));
    }
    setEditing(true);
  }

  return (
    <Dialog
      open={editing}
      onOpenChange={(open) => (open ? openCalibration() : setEditing(false))}
    >
      <section
        aria-labelledby="adc-heading"
        className="w-full max-w-md space-y-4 px-4 py-4"
      >
        <div className="flex items-center justify-between">
          <h2 id="adc-heading" className="font-semibold">
            ADC
          </h2>
          <DialogTrigger
            render={<Button variant="ghost" size="icon" />}
            aria-label="Calibrate ADC"
            title="Calibrate ADC"
          >
            <Settings2 />
          </DialogTrigger>
        </div>
        <div className="grid grid-cols-2 gap-4 tabular-nums">
          <div>
            <p className="text-sm text-muted-foreground">Raw ADC</p>
            <p className="mt-1 text-3xl font-medium">
              {available ? raw.toLocaleString() : '—'}
            </p>
          </div>
          <div>
            <p className="text-sm text-muted-foreground">Voltage</p>
            <p className="mt-1 text-3xl font-medium">
              {connected && voltage !== undefined
                ? `${voltage.toFixed(2)} V`
                : '—'}
            </p>
          </div>
        </div>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>ADC calibration</DialogTitle>
          </DialogHeader>
          <form
            id="adc-calibration"
            className="space-y-4"
            onSubmit={(event) => {
              event.preventDefault();
              localStorage.setItem(
                STORAGE_KEY,
                JSON.stringify({
                  zero: Number(zero),
                  unitsPerAmp: Number(scale),
                })
              );
              setEditing(false);
            }}
          >
            <div className="space-y-2">
              <label htmlFor="adc-zero" className="text-sm">
                Zero reading
              </label>
              <div className="flex gap-2">
                <Input
                  id="adc-zero"
                  type="number"
                  min={0}
                  max={4095}
                  step="any"
                  required
                  value={zero}
                  onChange={(event) => setZero(event.target.value)}
                />
                <Button
                  type="button"
                  variant="outline"
                  disabled={!available}
                  onClick={() => setZero(String(raw))}
                >
                  Use current reading
                </Button>
              </div>
            </div>
            <div className="space-y-2">
              <label htmlFor="adc-scale" className="text-sm">
                ADC units / A
              </label>
              <Input
                id="adc-scale"
                type="number"
                min={0.000001}
                step="any"
                required
                value={scale}
                onChange={(event) => setScale(event.target.value)}
              />
            </div>
            <Button type="submit">Save calibration</Button>
          </form>
        </DialogContent>
      </section>
    </Dialog>
  );
});
