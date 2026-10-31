import { execFile, execFileSync } from 'node:child_process';
import dgram from 'node:dgram';
import { constants, existsSync, openSync } from 'node:fs';
import tty from 'node:tty';
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
  // FPV frames delivered, for the status.
  received?: number;
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

// FPV chunks (fw/esp32cam/src/fpv.h) carry a 16-byte header (FPV1, frame number, index and count, capture and send times). With the receiver plugged in they arrive on its port as [length u16][rssi i8][chunk] at 1 Mbaud, or from the monitor-mode adapter as UDP datagrams, while a K V request every 500 ms keeps the camera sending over the radio; otherwise over UDP, kept coming by "FPVW" to the camera's port 5601. A frame still missing a chunk when a newer one starts is dropped.
// op: V for FPV, P for tag mode's preview over the radio (tracking on).
async function runFpv(hub: Hub, op: 'V' | 'P') {
  let current = -1;
  let parts: Buffer[] = [];
  let got = 0;
  hub.received = 0;
  const chunk = (message: Buffer) => {
    if (message.length < 16 || message.toString('latin1', 0, 4) !== 'FPV1')
      return;
    const frame = message.readUInt16LE(4);
    const index = message[6];
    const count = message[7];
    if (frame !== current) {
      // A late chunk of a frame a few back; a frame far behind is a restarted camera counting from 0 again, whose frames were all dropped as late until the count caught up (2026-10-30).
      if (current >= 0 && ((current - frame) & 0xffff) < 64) return;
      current = frame;
      parts = new Array(count);
      got = 0;
    }
    if (index >= parts.length || parts[index]) return;
    parts[index] = message.subarray(16);
    if (++got < parts.length) return;
    hub.received!++;
    const jpeg = Buffer.concat(parts);
    const part = new Uint8Array(
      Buffer.concat([
        Buffer.from(
          `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\nX-Time: ${message.readUInt32LE(8)} ${message.readUInt32LE(12)}\r\n\r\n`
        ),
        jpeg,
        Buffer.from('\r\n'),
      ])
    );
    for (const viewer of hub.viewers)
      if ((viewer.desiredSize ?? 1) > 0)
        try {
          viewer.enqueue(part);
        } catch {}
  };
  let keepalive: () => void;
  let close: () => void;
  const radio = receiver();
  if (radioUp()) {
    radio.listeners.add(chunk);
    let asking = false;
    keepalive = () => {
      if (asking) return;
      asking = true;
      cameraRequest(op, op === 'V' ? radioBudget() : [])
        .catch(() => {})
        .finally(() => (asking = false));
    };
    close = () => radio.listeners.delete(chunk);
  } else {
    const socket = dgram.createSocket('udp4');
    socket.on('message', chunk);
    socket.on('error', () => {});
    keepalive = () => socket.send('FPVW', 5601, hub.host);
    socket.bind(() => keepalive());
    close = () => socket.close();
  }
  const timer = setInterval(keepalive, 500);
  await new Promise((done) =>
    hub.closed.signal.addEventListener('abort', done)
  );
  clearInterval(timer);
  close();
}

function joinHub(host: string, path: string, signal: AbortSignal) {
  // FPV, or tag mode's preview over the radio: both chunked through runFpv.
  const fpv = path === '/fpv' || path === '/radio-preview';
  const url = cameraUrl(host, fpv ? '/' : path);
  const key = fpv ? `fpv://${host}` : url.href;
  for (const other of hubs.values())
    if (other.host === host && other.key !== key) closeHub(other);
  let hub = hubs.get(key);
  if (!hub) {
    hub = { key, host, viewers: new Set(), closed: new AbortController() };
    hubs.set(key, hub);
    void (fpv ? runFpv(hub, path === '/fpv' ? 'V' : 'P') : runHub(hub, url));
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

// Requests to the camera through the drone's wire to it, over the bridge's update socket, one short connection each so tools/ota_direct.py can still take the socket: K S reports the camera's network, address, channel and state ("joined miso 192.168.86.248 ch6 running ...", "ap bill-fpv 192.168.4.1 ch11 ...", "radio - 0.0.0.0 ch11 fpv ..."), K V asks for FPV over the radio (with the receiver's byte budget).
function cameraRequest(op: string, body: number[] = []): Promise<string> {
  return new Promise((done, fail) => {
    const socket = new WebSocket('ws://127.0.0.1:8765/update');
    socket.binaryType = 'arraybuffer';
    const id = crypto.getRandomValues(new Uint8Array(4));
    const timer = setTimeout(() => {
      socket.close();
      fail(new Error('The drone did not answer for the camera.'));
    }, 3000);
    socket.onopen = () =>
      socket.send(new Uint8Array([...id, 75, op.charCodeAt(0), ...body]));
    socket.onerror = () => {};
    socket.onmessage = (event) => {
      const reply = new Uint8Array(event.data as ArrayBuffer);
      if (!id.every((byte, i) => reply[i] === byte)) return;
      clearTimeout(timer);
      socket.close();
      const text = new TextDecoder().decode(reply.subarray(5));
      if (reply[4] !== 0) fail(new Error(text));
      else done(text);
    };
  });
}

// The camera's address on its network, kept for 10 s.
let found: { host: string; time: number } | undefined;

async function reportedHost() {
  if (found && Date.now() - found.time < 10_000) return found.host;
  const [mode, , host] = (await cameraRequest('S')).split(' ');
  if (mode !== 'joined' && mode !== 'ap')
    throw new Error('The camera is not on Wi-Fi.');
  found = { host, time: Date.now() };
  return host;
}

// FPV over the radio, needing no network, comes from the monitor-mode Wi-Fi adapter's reader (sw/fpv_monitor.py) as UDP datagrams to 127.0.0.1:5602, with FPVM every 500 ms while it runs, or from the ESP32 receiver (fw/fpv_rx) on the serial port FPV_RX names. That port is never guessed: the receiver's CP2102 has the same USB serial number as the drone's and the controller's, and a port path once picked the controller after the boards moved. Both are opened once and kept for as long as the server runs, their chunks going to whichever FPV stream is running: closing the serial port after each stream left it open in the process, and the stale readers took a share of the bytes.
const RECEIVER = process.env.FPV_RX;
// chunks: every one heard, for the status.
// copies: every chunk copy heard, chunks: the first of each, signal: the monitor's last (dBm), for the status.
type Receiver = {
  listeners: Set<(chunk: Buffer) => void>;
  heard: number;
  copies: number;
  chunks: number;
  signal?: number;
};
const store = globalThis as { fpvReceiver?: Receiver };

function receiver() {
  if (store.fpvReceiver) return store.fpvReceiver;
  const opened: Receiver = {
    listeners: new Set(),
    heard: 0,
    copies: 0,
    chunks: 0,
  };
  // The monitor passes every copy (the camera sends each frame several times); the first of each goes on. The capture time tells a restarted camera's frame numbers from the same numbers before it.
  const recent = new Set<string>();
  const deliver = (chunk: Buffer) => {
    opened.copies++;
    const key = `${chunk.readUInt16LE(4)}:${chunk[6]}:${chunk.readUInt32LE(8)}`;
    if (recent.has(key)) return;
    recent.add(key);
    if (recent.size > 512) recent.delete(recent.values().next().value!);
    opened.chunks++;
    for (const listener of opened.listeners) listener(chunk);
  };
  const monitor = dgram.createSocket('udp4');
  monitor.on('message', (message) => {
    opened.heard = Date.now();
    if (message.toString('latin1', 0, 4) !== 'FPVC') return;
    opened.signal = message.readInt8(4);
    deliver(message.subarray(5));
  });
  monitor.on('error', () => {});
  monitor.bind(5602, '127.0.0.1');
  if (RECEIVER && existsSync(RECEIVER)) {
    execFileSync('stty', ['-F', RECEIVER, '1000000', 'raw', '-echo']);
    const port = new tty.ReadStream(
      openSync(RECEIVER, constants.O_RDONLY | constants.O_NOCTTY)
    );
    let buffer = Buffer.alloc(0);
    port.on('data', (data: Buffer) => {
      buffer = Buffer.concat([buffer, data]);
      for (;;) {
        const at = buffer.indexOf('FPV1', 3);
        if (at < 0) {
          buffer = buffer.subarray(Math.max(0, buffer.length - 3));
          break;
        }
        const length = buffer.readUInt16LE(at - 3);
        if (length < 16 || length > 1500) {
          buffer = buffer.subarray(at + 1);
          continue;
        }
        if (buffer.length < at + length) break;
        deliver(buffer.subarray(at, at + length));
        buffer = buffer.subarray(at + length);
      }
    });
    port.on('error', () => port.destroy());
  }
  return (store.fpvReceiver = opened);
}

// The camera's byte budget for the receiver, KB/s little-endian in its K V request: the ESP32 receiver's USB link carries about 98 KB/s, the monitor-mode adapter far more.
function radioBudget() {
  const kilobytes = Date.now() - receiver().heard < 2000 ? 250 : 90;
  return [kilobytes & 0xff, kilobytes >> 8];
}

// A receiver is there: the monitor heard from lately, or the ESP32 receiver's port.
function radioUp() {
  return (
    Date.now() - receiver().heard < 2000 || !!(RECEIVER && existsSync(RECEIVER))
  );
}

export async function GET(request: Request) {
  const query = new URL(request.url).searchParams;
  let host = query.get('host') || 'the camera';
  const stream = query.get('mode') !== 'status' && query.get('full') !== '1';
  // The monitor's first hello comes up to 500 ms after the listener opens on the first request.
  if (stream && !radioUp()) await new Promise((done) => setTimeout(done, 700));
  // With a radio receiver both views come over the radio: FPV, and tag mode's preview with tracking on.
  const radio = stream && radioUp();
  try {
    const status = query.get('mode') === 'status';
    if (radio) host = 'radio';
    else if (!status && !query.get('host')) host = await reportedHost();
    if (!status)
      return new Response(
        joinHub(
          host,
          query.get('fpv') === '1'
            ? '/fpv'
            : radio
              ? '/radio-preview'
              : query.get('full') === '1'
                ? '/stream?full=1'
                : '/stream',
          request.signal
        ),
        {
          headers: {
            'Content-Type': 'multipart/x-mixed-replace;boundary=frame',
            'Cache-Control': 'no-store',
          },
        }
      );
    // The camera's state through the drone, and the FPV frames delivered when FPV runs.
    const received = [...hubs.values()].find(
      (hub) => hub.received !== undefined
    )?.received;
    const state = (await cameraRequest('S')).split(' ')[4];
    return Response.json(
      {
        camera: state,
        received,
        copies: store.fpvReceiver?.copies,
        chunks: store.fpvReceiver?.chunks,
        signal: store.fpvReceiver?.signal,
      },
      { headers: { 'Cache-Control': 'no-store' } }
    );
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
