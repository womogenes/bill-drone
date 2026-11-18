export const dynamic = 'force-dynamic';

// The drone service's own account of the chain (src/fw/controller/bridge.py GET /status, without its reading), for the page to say what is missing while it cannot connect; {state: 'bridge_down'} when the service does not answer.
export async function GET() {
  try {
    const response = await fetch('http://127.0.0.1:8765/status', {
      signal: AbortSignal.timeout(500),
      cache: 'no-store',
    });
    const { state, heard_s, in_use, errors } = await response.json();
    return Response.json({ state, heard_s, in_use, errors });
  } catch {
    return Response.json({ state: 'bridge_down' });
  }
}
