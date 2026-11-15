import { analyzeFlight, RECORD_COLUMNS } from '@/lib/flight-report';

export const dynamic = 'force-dynamic';

// The drone service's recordings (src/fw/controller/bridge.py GET /recordings).
const RECORDINGS = 'http://127.0.0.1:8765/recordings';

// GET lists the recordings, newest first; ?name=<stem> returns that flight's report (lib/flight-report.ts) from the events and columns the service decodes.
export async function GET(request: Request) {
  const name = new URL(request.url).searchParams.get('name');
  const headers = { 'Cache-Control': 'no-store' };
  try {
    const response = await fetch(
      name === null
        ? RECORDINGS
        : `${RECORDINGS}?${new URLSearchParams({ name, columns: RECORD_COLUMNS.join(',') })}`,
      { cache: 'no-store' }
    ).catch(() => {
      throw new Error('The drone service does not answer');
    });
    const body = await response.json();
    if (!response.ok || name === null)
      return Response.json(body, { status: response.status, headers });
    return Response.json(analyzeFlight(name, body), { headers });
  } catch (error) {
    return Response.json(
      { error: error instanceof Error ? error.message : String(error) },
      { status: 500 }
    );
  }
}
