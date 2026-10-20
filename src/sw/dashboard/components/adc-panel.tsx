'use client';

import { memo } from 'react';

export const ADCPanel = memo(function ADCPanel({
  raw,
  voltage,
  connected,
}: {
  raw: number | undefined;
  voltage: number | undefined;
  connected: boolean;
}) {
  return (
    <section
      aria-labelledby="adc-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      <h2 id="adc-heading" className="font-semibold">
        ADC
      </h2>
      <div className="grid grid-cols-2 gap-4 tabular-nums">
        <div>
          <p className="text-sm text-muted-foreground">Raw ADC</p>
          <p className="mt-1 text-3xl font-medium">
            {connected && raw !== undefined ? raw.toLocaleString() : '—'}
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
    </section>
  );
});
