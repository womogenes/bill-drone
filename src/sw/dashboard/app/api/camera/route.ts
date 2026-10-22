import { execFile } from 'node:child_process';
import { resolve } from 'node:path';

export const runtime = 'nodejs';

function cameraUrl(host: string, path: string) {
  if (!/^[a-zA-Z0-9.-]+(:\d+)?$/.test(host))
    throw new Error('Enter a camera hostname or IP address.');
  const url = new URL(`http://${host}${path}`);
  if (path.startsWith('/stream')) url.port = String(Number(url.port || 80) + 1);
  return url;
}

export async function GET(request: Request) {
  const query = new URL(request.url).searchParams;
  const host = query.get('host') || '192.168.86.28';
  const path =
    query.get('mode') === 'status'
      ? '/status'
      : query.get('full') === '1'
        ? '/stream?full=1'
        : '/stream';
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 5000);
  try {
    const response = await fetch(cameraUrl(host, path), {
      cache: 'no-store',
      signal: AbortSignal.any([request.signal, controller.signal]),
    });
    clearTimeout(timeout); // Streaming continues until the browser disconnects.
    if (!response.ok) throw new Error(`Camera returned ${response.status}.`);
    return new Response(response.body, {
      headers: {
        'Content-Type': response.headers.get('content-type')!,
        'Cache-Control': 'no-store',
      },
    });
  } catch (error) {
    const cause = error instanceof Error ? error.cause : undefined;
    const dnsFailure =
      cause instanceof Error &&
      'code' in cause &&
      (cause.code === 'ENOTFOUND' || cause.code === 'EAI_AGAIN');
    return Response.json(
      {
        error: dnsFailure
          ? `Cannot resolve ${host}. Enter the camera's IP address.`
          : `Cannot connect to ${host}. Check the camera address and power.`,
      },
      { status: 502 }
    );
  } finally {
    clearTimeout(timeout);
  }
}

export async function POST(request: Request) {
  const input = await request.json();
  const source = resolve(process.cwd(), '../..');
  try {
    const result = await new Promise<Record<string, unknown>>(
      (done, reject) => {
        const child = execFile(
          resolve(source, '.venv/bin/python'),
          [resolve(source, 'fw/esp32cam/calibration.py')],
          { timeout: 30000, maxBuffer: 1024 * 1024 },
          (error, stdout) => {
            if (!stdout.trim()) {
              reject(error || new Error('Calibration failed.'));
              return;
            }
            const result = JSON.parse(stdout);
            if (error) reject(new Error(result.error));
            else done(result);
          }
        );
        child.stdin?.on('error', () => {});
        child.stdin?.end(JSON.stringify(input));
      }
    );
    if (result.camera) {
      const response = await fetch(cameraUrl(input.host, '/config'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(result.camera),
        signal: AbortSignal.timeout(5000),
      });
      if (!response.ok)
        throw new Error('Fit saved locally, but the camera rejected it.');
    }
    return Response.json(result);
  } catch (error) {
    return Response.json(
      { error: error instanceof Error ? error.message : String(error) },
      { status: 400 }
    );
  }
}
