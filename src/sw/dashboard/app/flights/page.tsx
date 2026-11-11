'use client';

import { Suspense, useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import { FlightReportView } from '@/components/flight-report';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import type { RecordingEntry } from '@/lib/flight-report';
import { clock, startTime } from '@/lib/format';
import { useFlightReport } from '@/hooks/use-flight-report';

// Past flights from the recordings on this laptop. It never opens the drone link, so it can sit in its own tab beside the flying dashboard, which the bridge serves to one client only.
function Flights() {
  const chosen = useSearchParams().get('name');
  const [entries, setEntries] = useState<RecordingEntry[] | null>(null);
  const [listError, setListError] = useState('');
  const name = chosen ?? entries?.[0]?.name ?? null;
  const { report, error } = useFlightReport(name);

  useEffect(() => {
    fetch('/api/recordings')
      .then((response) => response.json())
      .then((value) => {
        if (!Array.isArray(value)) throw new Error(value.error);
        setEntries(value);
      })
      .catch((failure: Error) => setListError(failure.message));
  }, []);

  const items = (entries ?? []).map((entry) => ({
    value: entry.name,
    label: `${startTime(entry.started)} · ${clock(entry.seconds)}`,
  }));
  return (
    <div className="flex min-h-dvh flex-col lg:h-dvh lg:overflow-hidden">
      <header className="flex h-14 shrink-0 items-center gap-6 border-b px-4">
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
          <SelectTrigger aria-label="Recording" className="w-64">
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
        {(listError || error) && (
          <span className="text-sm text-red-700">{listError || error}</span>
        )}
      </header>
      {report && <FlightReportView report={report} />}
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
