'use client';

import { Suspense, useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import { FlightReportView, clock, startTime } from '@/components/flight-report';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import type { FlightReport, RecordingEntry } from '@/lib/flight-report';

// Past flights from the recordings on this laptop. It never opens the drone link, so it can sit in its own tab beside the flying dashboard, which the bridge serves to one client only.
function Flights() {
  const chosen = useSearchParams().get('name');
  const [entries, setEntries] = useState<RecordingEntry[] | null>(null);
  const [report, setReport] = useState<FlightReport | null>(null);
  const [error, setError] = useState('');
  const name = chosen ?? entries?.[0]?.name ?? null;

  useEffect(() => {
    fetch('/api/recordings')
      .then((response) => response.json())
      .then((value) => {
        if (!Array.isArray(value)) throw new Error(value.error);
        setEntries(value);
      })
      .catch((failure: Error) => setError(failure.message));
  }, []);

  useEffect(() => {
    if (!name) return;
    const controller = new AbortController();
    fetch(`/api/recordings?name=${encodeURIComponent(name)}`, {
      signal: controller.signal,
    })
      .then((response) => response.json())
      .then((value) => {
        if (value.error) throw new Error(value.error);
        setReport(value);
        setError('');
      })
      .catch((failure: Error) => {
        if (!controller.signal.aborted) setError(failure.message);
      });
    return () => controller.abort();
  }, [name]);

  const items = (entries ?? []).map((entry) => ({
    value: entry.name,
    label: `${startTime(entry.started)} · ${clock(entry.seconds)}`,
  }));
  return (
    <div className="flex min-h-dvh flex-col">
      <header className="flex h-14 shrink-0 items-center gap-6 border-b px-6">
        <h1 className="font-semibold">Flights</h1>
        <Select
          value={name}
          items={items}
          onValueChange={(value) => {
            const url = new URL(window.location.href);
            url.searchParams.set('name', String(value));
            window.history.replaceState(null, '', url);
          }}
        >
          <SelectTrigger aria-label="Recording" className="w-64 cursor-pointer">
            <SelectValue />
          </SelectTrigger>
          <SelectContent align="start" alignItemWithTrigger={false}>
            <SelectGroup>
              {items.map((item) => (
                <SelectItem key={item.value} value={item.value}>
                  {item.label}
                </SelectItem>
              ))}
            </SelectGroup>
          </SelectContent>
        </Select>
        {error && <span className="text-sm text-red-700">{error}</span>}
      </header>
      {report && report.name === name && <FlightReportView report={report} />}
    </div>
  );
}

export default function Page() {
  return (
    <Suspense>
      <Flights />
    </Suspense>
  );
}
