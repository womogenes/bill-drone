'use client';
import type { Reading } from '@/lib/telemetry';
import { Activity, CircleHelp, Unplug } from 'lucide-react';
const format = new Intl.NumberFormat(undefined, { maximumFractionDigits: 0 })
  .format;
export function LoadCellPanel({
  reading,
  connected,
}: {
  reading: Reading | null;
  connected: boolean;
}) {
  const age = reading?.force_age_ms ?? null;
  const state = !connected
    ? {
        label: 'Offline',
        Icon: Unplug,
        color: 'text-red-700',
      }
    : !reading || reading.force === null || reading.force_age_ms === null
      ? {
          label: 'Waiting for sample',
          Icon: Activity,
          color: 'text-yellow-600/70 dark:text-yellow-400/70',
        }
      : age === null
        ? {
            label: 'Freshness unavailable',
            Icon: CircleHelp,
            color: 'text-yellow-600/70 dark:text-yellow-400/70',
          }
        : age > 1000
          ? {
              label: 'Stale · check sensor',
              Icon: Unplug,
              color: 'text-red-700',
            }
          : {
              label: 'Sampling',
              Icon: Activity,
              color: 'text-green-700',
            };

  return (
    <section
      aria-labelledby="load-cell-heading"
      className="w-full max-w-md space-y-4 px-4 py-4"
    >
      {' '}
      <div className="flex flex-wrap items-center gap-4">
        <h2 id="load-cell-heading" className="font-semibold">
          Load cell
        </h2>
        <span
          role="status"
          className={`flex items-center gap-2 text-sm ${state.color}`}
        >
          <state.Icon aria-hidden="true" className="size-4" />
          {state.label}
        </span>
      </div>
      <dl className="tabular-nums">
        <div>
          <dt className="text-sm text-muted-foreground">
            {connected && age !== null && age <= 1000
              ? 'Latest reading'
              : 'Last reported reading'}
          </dt>
          <dd className="mt-1 text-3xl font-medium">
            {reading?.force != null ? format(reading.force) : '—'}
          </dd>
        </div>
      </dl>
    </section>
  );
}
