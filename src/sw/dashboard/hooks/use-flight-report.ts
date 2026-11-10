'use client';

import { useEffect, useState } from 'react';
import type { FlightReport } from '@/lib/flight-report';

// The report of the recording called name, from /api/recordings: null until it has loaded, error if it could not be read.
export function useFlightReport(name: string | null) {
  const [report, setReport] = useState<FlightReport | null>(null);
  const [error, setError] = useState('');
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
  return { report: report?.name === name ? report : null, error };
}
