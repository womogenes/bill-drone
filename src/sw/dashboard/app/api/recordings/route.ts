import { readdir, readFile, stat } from 'node:fs/promises';
import { resolve } from 'node:path';
import {
  analyzeFlight,
  estimateSeconds,
  recordingStart,
  type RecordingEntry,
} from '@/lib/flight-report';

export const runtime = 'nodejs';

// The bridge's recordings (src/fw/controller/bridge.py RECORDINGS), beside this dashboard.
const RECORDINGS = resolve(process.cwd(), '../recordings');
// Recordings shorter than this (s, by size) are arm-and-disarm checks, not flights.
const SHORTEST_S = 1;

// GET lists the recordings, newest first; ?name=<stem> returns that flight's report (lib/flight-report.ts).
export async function GET(request: Request) {
  const name = new URL(request.url).searchParams.get('name');
  try {
    if (name === null) {
      const files = await readdir(RECORDINGS);
      const stems = new Set(
        files
          .filter((file) => file.endsWith('.jsonl'))
          .map((file) => file.slice(0, -6))
      );
      const entries: RecordingEntry[] = [];
      for (const file of files) {
        const stem = file.slice(0, -4);
        const started = recordingStart(stem);
        if (!file.endsWith('.bin') || !started || !stems.has(stem)) continue;
        const { size } = await stat(resolve(RECORDINGS, file));
        const seconds = estimateSeconds(size);
        if (seconds >= SHORTEST_S)
          entries.push({ name: stem, started, bytes: size, seconds });
      }
      entries.sort((a, b) => b.started.localeCompare(a.started));
      return Response.json(entries, {
        headers: { 'Cache-Control': 'no-store' },
      });
    }
    // Only a recording's own name, so the query cannot reach other files.
    if (!recordingStart(name))
      return Response.json({ error: 'Unknown recording' }, { status: 404 });
    const [bin, jsonl] = await Promise.all([
      readFile(resolve(RECORDINGS, `${name}.bin`)),
      readFile(resolve(RECORDINGS, `${name}.jsonl`), 'utf8'),
    ]);
    return Response.json(analyzeFlight(name, bin, jsonl), {
      headers: { 'Cache-Control': 'no-store' },
    });
  } catch (error) {
    return Response.json(
      { error: error instanceof Error ? error.message : String(error) },
      { status: 500 }
    );
  }
}
