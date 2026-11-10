'use client';

import { memo } from 'react';
import { clock } from '@/lib/format';
import { useFlightReport } from '@/hooks/use-flight-report';

// The flight that just ended, from its recording once the bridge has closed it (stop record written): how it ended and what went wrong, with the full report in its own tab so this one keeps the drone link.
export const LastFlight = memo(function LastFlight({ name }: { name: string }) {
  const { report: shown, error } = useFlightReport(name);
  return (
    <section
      className="space-y-2 px-4 py-3"
      aria-labelledby="last-flight-heading"
    >
      <div className="flex items-center gap-2">
        <h2 id="last-flight-heading" className="mr-auto font-semibold">
          Last flight
        </h2>
        <a
          href={`/flights?name=${encodeURIComponent(name)}`}
          target="_blank"
          rel="noopener"
          className="text-sm underline"
        >
          Report
        </a>
      </div>
      {error ? (
        <p className="text-sm text-red-700">{error}</p>
      ) : shown ? (
        <>
          <p className="text-sm tabular-nums">
            {shown.stop_reason ?? 'No stop record'} ·{' '}
            {shown.airborne
              ? `${clock(shown.airborne[1] - shown.airborne[0])} airborne`
              : 'never lifted off'}
            {shown.max_height !== null &&
              ` · up to ${shown.max_height.toFixed(1)} m`}
            {shown.shake_rms !== null &&
              ` · shake ${shown.shake_rms.toFixed(0)}°/s`}
          </p>
          {shown.warnings.length > 0 && (
            <ul className="space-y-0.5 text-sm text-red-700 tabular-nums">
              {shown.warnings.map((warning, index) => (
                <li key={index}>
                  {warning.time !== null && `${clock(warning.time)} `}
                  {warning.text}
                </li>
              ))}
            </ul>
          )}
        </>
      ) : (
        <p className="text-sm text-muted-foreground">Loading</p>
      )}
    </section>
  );
});
