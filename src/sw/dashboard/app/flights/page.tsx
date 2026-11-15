'use client';

import { Suspense, useEffect, useState } from 'react';
import { useSearchParams } from 'next/navigation';
import { paletteStyle } from '@/components/replay/palette';
import { Replay } from '@/components/replay/replay';
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

// Past flights from the recordings on this laptop, replayed. It never opens the drone link, so it can sit in its own tab beside the flying dashboard, which the bridge serves to one client only.
function Flights() {
  const chosen = useSearchParams().get('name');
  const [entries, setEntries] = useState<RecordingEntry[] | null>(null);
  const [listError, setListError] = useState('');
  const [name, setName] = useState<string | null>(chosen);
  const shown = name ?? entries?.[0]?.name ?? null;
  const { report, error } = useFlightReport(shown);

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
  const problem = listError || error;
  return (
    <div className="flex h-dvh flex-col bg-[var(--replay-paper)] text-[var(--replay-ink)]" style={paletteStyle}>
      <header className="flex h-12 shrink-0 items-center gap-4 border-b border-[var(--replay-hairline)] px-4">
        <Select
          value={shown}
          items={items}
          onValueChange={(value) => {
            setName(String(value));
            const url = new URL(window.location.href);
            url.searchParams.set('name', String(value));
            window.history.replaceState(null, '', url);
          }}
        >
          <SelectTrigger aria-label="Flight" className="w-60 border-[var(--replay-hairline)] bg-transparent">
            <SelectValue placeholder="No flights yet" />
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
        {report && (
          <span className="text-sm text-[var(--replay-gray)] tabular-nums">
            {report.max_height !== null && `${report.max_height.toFixed(1)} m up`}
            {report.stop_reason && ` · ${report.stop_reason.toLowerCase()}`}
          </span>
        )}
        {problem && <span className="text-sm text-[var(--replay-signal)]">{problem}</span>}
      </header>
      <main className="min-h-0 flex-1">
        {report ? <Replay key={report.name} trajectory={report.trajectory} /> : <div className="h-full" />}
      </main>
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
