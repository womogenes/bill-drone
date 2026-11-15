export const dynamic = 'force-dynamic';

// The drone's settings with their ranges and defaults (src/fw/controller/bridge.py GET /schema), for the settings dialog.
export async function GET() {
  try {
    const response = await fetch('http://127.0.0.1:8765/schema', {
      cache: 'no-store',
    });
    return Response.json(await response.json(), { status: response.status });
  } catch {
    return Response.json(
      { error: 'The drone service does not answer' },
      { status: 502 }
    );
  }
}
