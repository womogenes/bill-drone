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

function indexOf(bytes: Uint8Array, pattern: number[], from = 0) {
  outer: for (let i = from; i <= bytes.length - pattern.length; i++) {
    for (let j = 0; j < pattern.length; j++)
      if (bytes[i + j] !== pattern[j]) continue outer;
    return i;
  }
  return -1;
}

// The camera's stream server handles one connection at a time, so every viewer of a host shares one upstream stream, which reconnects whenever it goes quiet. Viewers get whole parts, so they can join at any time.
type Hub = {
  key: string;
  host: string;
  viewers: Set<ReadableStreamDefaultController<Uint8Array>>;
  closed: AbortController;
  idle?: ReturnType<typeof setTimeout>;
};
const hubs: Map<string, Hub> = ((
  globalThis as { cameraHubs?: Map<string, Hub> }
).cameraHubs ??= new Map());
const quietMs = 4000;

function closeHub(hub: Hub) {
  hub.closed.abort();
  clearTimeout(hub.idle);
  hubs.delete(hub.key);
  for (const viewer of hub.viewers)
    try {
      viewer.close();
    } catch {}
}

async function runHub(hub: Hub, url: URL) {
  let backoff = 250;
  while (!hub.closed.signal.aborted) {
    const connection = new AbortController();
    let watchdog = setTimeout(() => connection.abort(), quietMs);
    const alive = () => {
      clearTimeout(watchdog);
      watchdog = setTimeout(() => connection.abort(), quietMs);
    };
    try {
      const response = await fetch(url, {
        cache: 'no-store',
        signal: AbortSignal.any([hub.closed.signal, connection.signal]),
      });
      if (!response.ok || !response.body)
        throw new Error(`Camera returned ${response.status}.`);
      const reader = response.body.getReader();
      let buffer = new Uint8Array(0);
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        alive();
        const joined = new Uint8Array(buffer.length + value.length);
        joined.set(buffer);
        joined.set(value, buffer.length);
        buffer = joined;
        for (;;) {
          const headerEnd = indexOf(buffer, [13, 10, 13, 10]);
          if (headerEnd < 0) break;
          const header = new TextDecoder().decode(
            buffer.subarray(0, headerEnd)
          );
          const length = Number(
            /Content-Length: *(\d+)/i.exec(header)?.[1] ?? 0
          );
          const end = headerEnd + 4 + length + 2;
          if (buffer.length < end) break;
          const part = buffer.slice(0, end);
          buffer = buffer.slice(end);
          backoff = 250;
          // A viewer that falls behind skips parts rather than queueing them.
          for (const viewer of hub.viewers)
            if ((viewer.desiredSize ?? 1) > 0)
              try {
                viewer.enqueue(part);
              } catch {}
        }
      }
    } catch {}
    clearTimeout(watchdog);
    await new Promise((done) => setTimeout(done, backoff));
    backoff = Math.min(backoff * 2, 4000);
  }
}

function joinHub(host: string, full: boolean, signal: AbortSignal) {
  const url = cameraUrl(host, full ? '/stream?full=1' : '/stream');
  const key = url.href;
  for (const other of hubs.values())
    if (other.host === host && other.key !== key) closeHub(other);
  let hub = hubs.get(key);
  if (!hub) {
    hub = { key, host, viewers: new Set(), closed: new AbortController() };
    hubs.set(key, hub);
    void runHub(hub, url);
  }
  const joined = hub;
  clearTimeout(joined.idle);
  let viewer: ReadableStreamDefaultController<Uint8Array>;
  const leave = () => {
    if (!joined.viewers.delete(viewer)) return;
    // A viewer that reconnects or remounts within a few seconds keeps the camera connection.
    if (!joined.viewers.size)
      joined.idle = setTimeout(() => closeHub(joined), 3000);
  };
  return new ReadableStream<Uint8Array>({
    start(controller) {
      viewer = controller;
      joined.viewers.add(controller);
      signal.addEventListener('abort', leave);
    },
    cancel: leave,
  });
}

export async function GET(request: Request) {
  const query = new URL(request.url).searchParams;
  const host = query.get('host') || '192.168.86.28';
  try {
    if (query.get('mode') !== 'status')
      return new Response(
        joinHub(host, query.get('full') === '1', request.signal),
        {
          headers: {
            'Content-Type': 'multipart/x-mixed-replace;boundary=frame',
            'Cache-Control': 'no-store',
          },
        }
      );
    const response = await fetch(cameraUrl(host, '/status'), {
      cache: 'no-store',
      signal: AbortSignal.any([request.signal, AbortSignal.timeout(3000)]),
    });
    if (!response.ok) throw new Error(`Camera returned ${response.status}.`);
    return new Response(await response.text(), {
      headers: {
        'Content-Type': 'application/json',
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
