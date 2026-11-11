import { execFile, execFileSync } from 'node:child_process';
import dgram from 'node:dgram';
import {
  closeSync,
  constants,
  existsSync,
  openSync,
  readdirSync,
  realpathSync,
  statSync,
  writeSync,
} from 'node:fs';
import tty from 'node:tty';
import { resolve } from 'node:path';
import { decode } from '@/lib/fec';
import { indexOf } from '@/lib/multipart';

export const runtime = 'nodejs';

function cameraUrl(host: string, path: string) {
  if (!/^[a-zA-Z0-9.-]+(:\d+)?$/.test(host))
    throw new Error('Enter a camera hostname or IP address.');
  const url = new URL(`http://${host}${path}`);
  if (path.startsWith('/stream')) url.port = String(Number(url.port || 80) + 1);
  return url;
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

// A viewer that falls behind skips parts rather than queueing them.
function send(hub: Hub, part: Uint8Array) {
  for (const viewer of hub.viewers)
    if ((viewer.desiredSize ?? 1) > 0)
      try {
        viewer.enqueue(part);
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
          send(hub, part);
        }
      }
    } catch {}
    clearTimeout(watchdog);
    await new Promise((done) => setTimeout(done, backoff));
    backoff = Math.min(backoff * 2, 4000);
  }
}

// FPV shards (fw/esp32cam/src/fpv.h) carry a 20-byte header (FPV2, frame number, shard index, shard count, data shards, the JPEG's length, capture and send times); any data of a frame's shards rebuild it (lib/fec.ts). With the receiver plugged in they arrive on its port as [length u16][rssi i8][chunk] at 1 Mbaud, or from the monitor-mode adapter as UDP datagrams, while a K V request every 500 ms keeps the camera sending over the radio; otherwise over UDP, kept coming by "FPVW" to the camera's port 5601. A frame still short of shards when a newer one starts is dropped.
// op: V for FPV, P for tag mode's preview over the radio (tracking on).
// A tag-mode preview frame ends in its tag (fw/esp32cam/src/web.h PreviewTag, 52 bytes: "TAGP", the state, three spare bytes, the four corners in 320x240 pixels and the tag center in the camera frame in meters, little-endian floats), passed on as the X-Tag header the camera's own stream uses ("none", or "valid"/"tracked", the corners as x,y and the position as x,y,z).
const PREVIEW_TAG_BYTES = 52;
function splitTag(frame: Buffer) {
  const at = frame.length - PREVIEW_TAG_BYTES;
  if (at < 0 || frame.toString('latin1', at, at + 4) !== 'TAGP')
    return { jpeg: frame, tag: undefined };
  const state = frame[at + 4];
  const value = (k: number) => frame.readFloatLE(at + 8 + 4 * k);
  const corners = [0, 1, 2, 3]
    .map((i) => `${value(2 * i).toFixed(1)},${value(2 * i + 1).toFixed(1)}`)
    .join(' ');
  const position = [8, 9, 10].map((k) => value(k).toFixed(3)).join(',');
  return {
    jpeg: frame.subarray(0, at),
    tag: state
      ? `${state === 2 ? 'valid' : 'tracked'} ${corners} ${position}`
      : 'none',
  };
}

async function runFpv(hub: Hub, op: 'V' | 'P') {
  let current = -1;
  let shards = new Map<number, Buffer>();
  let done = false;
  hub.received = 0;
  const chunk = (message: Buffer) => {
    if (message.length < 20 || message.toString('latin1', 0, 4) !== 'FPV2')
      return;
    const frame = message.readUInt16LE(4);
    const index = message[6];
    const data = message[8];
    const length = message.readUInt16LE(10);
    if (frame !== current) {
      // A late chunk of a frame a few back; a frame far behind is a restarted camera counting from 0 again, whose frames were all dropped as late until the count caught up (2026-10-30).
      if (current >= 0 && ((current - frame) & 0xffff) < 64) return;
      current = frame;
      shards = new Map();
      done = false;
    }
    if (done || index >= message[7] || shards.has(index)) return;
    shards.set(index, message.subarray(20));
    if (shards.size < data) return;
    done = true;
    const frameBytes = decode(shards, data, length);
    if (!frameBytes) return;
    hub.received!++;
    const { jpeg, tag } = splitTag(frameBytes);
    const part = new Uint8Array(
      Buffer.concat([
        Buffer.from(
          `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\nX-Time: ${message.readUInt32LE(12)} ${message.readUInt32LE(16)}\r\n${tag ? `X-Tag: ${tag}\r\n` : ''}\r\n`
        ),
        jpeg,
        Buffer.from('\r\n'),
      ])
    );
    send(hub, part);
  };
  let keepalive: () => void;
  let close: () => void;
  const radio = receiver();
  if (radioUp()) {
    radio.listeners.add(chunk);
    // A camera without the N request (before 2026-11-06) is not told apart.
    cameraRequest('N')
      .then((text) => radio.choose(text.split(' ')[0]))
      .catch(() => {});
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
  // Each view its own hub: with one key for both, asking for FPV joined the running preview and the camera stayed in tag mode (2026-10-30).
  const key = fpv ? `fpv://${host}${path}` : url.href;
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
// A drone without a camera (or with one off) holds its single request slot for 2 s on each camera request, and drops every request that arrives meanwhile: the FPV view's keepalive every 500 ms left drone 2 without a camera answering "No reply" to arming and tuning (2026-11-07). So after the drone answers that its camera did not, camera requests fail at once for CAMERA_BACKOFF_MS.
const CAMERA_BACKOFF_MS = 10000;
let cameraFailed: { error: Error; time: number } | undefined;

function cameraRequest(op: string, body: number[] = []): Promise<string> {
  if (cameraFailed && Date.now() - cameraFailed.time < CAMERA_BACKOFF_MS)
    return Promise.reject(cameraFailed.error);
  return askCamera(op, body).then(
    (text) => {
      cameraFailed = undefined;
      return text;
    },
    (error: Error) => {
      // Only the drone's own answer that its camera is silent: a lost radio packet is retried at the next call as usual.
      if (error.message === 'The camera did not answer')
        cameraFailed = { error, time: Date.now() };
      throw error;
    }
  );
}

function askCamera(op: string, body: number[]): Promise<string> {
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

// FPV over the radio, needing no network, comes from the monitor-mode Wi-Fi adapter's reader (sw/fpv_monitor.py) as UDP datagrams to 127.0.0.1:5602, with FPVM every 500 ms while it runs, or from the ESP32 receiver (fw/fpv_rx) on a USB serial port: one FPV_RX names (space separated, where it has been plugged in before) or, failing those, any other /dev/ttyUSB*. USB cannot tell the boards apart (the receiver's CP2102 has the same serial number as the drone's and the controller's), so a port is the receiver's once it sends what a receiver sends, video or the FPVR it sends twice a second without any (firmware from 2026-11-10): while nothing is heard the ports are tried in turn for 5 s each, the listed ones for as long as it takes and any other once per plug-in, so a drone on USB is opened once and then left alone. The bridge holds the controller's port with an exclusive lock, which rules it out wherever the boards are. Both are opened once and kept for as long as the server runs, their chunks going to whichever FPV stream is running: closing the serial port after each stream left it open in the process, and the stale readers took a share of the bytes.
const RECEIVERS = (process.env.FPV_RX ?? '').split(/\s+/).filter(Boolean);
// The USB serial ports present, FPV_RX's first, each once.
function serialPorts() {
  const ports = new Map<string, string>();
  for (const path of [
    ...RECEIVERS,
    ...readdirSync('/dev')
      .filter((name) => name.startsWith('ttyUSB'))
      .map((name) => `/dev/${name}`),
  ]) {
    if (!existsSync(path)) continue;
    const device = realpathSync(path);
    if (!ports.has(device)) ports.set(device, path);
  }
  return [...ports.values()];
}
// When each unlisted port was tried, as its device node's creation time: one try per plug-in.
const tried = new Map<string, number>();
let turn = 0;
// The ports to try now, starting one further round than last time.
function candidates() {
  const ports = serialPorts().filter(
    (path) =>
      RECEIVERS.includes(path) || tried.get(path) !== statSync(path).ctimeMs
  );
  turn++;
  return ports.map((_, index) => ports[(index + turn) % ports.length]);
}
// The receiver's port raw at 1 Mbaud: input and output flushed, output resumed, settings applied now (TCSANOW). Exits with 3, touching nothing, if another process (the bridge, on the controller) holds the port's lock.
const TERMIOS = `import fcntl, os, sys, termios
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    sys.exit(3)
a = termios.tcgetattr(fd)
a[0:4] = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0]
a[4] = a[5] = termios.B1000000
a[6][termios.VMIN], a[6][termios.VTIME] = 1, 0
termios.tcflush(fd, termios.TCIOFLUSH)
termios.tcsetattr(fd, termios.TCSANOW, a)
termios.tcflow(fd, termios.TCOON)`;
// heard: when the monitor last said anything, serial: when the ESP32 receiver did; copies: every chunk copy heard, chunks: the first of each, signal: the monitor's last (dBm), for the status.
// choose: takes only this camera's shards from now on (its MAC, as K N reports it).
type Receiver = {
  listeners: Set<(chunk: Buffer) => void>;
  choose: (mac: string) => void;
  heard: number;
  serial: number;
  copies: number;
  chunks: number;
  signal?: number;
};
const store = globalThis as { fpvReceiver?: Receiver };

function receiver() {
  if (store.fpvReceiver) return store.fpvReceiver;
  // The camera whose shards go on: every camera sends with the same BSSID and its own MAC as the sender (fw/esp32cam/src/fpv.h), so with two drones about each dashboard takes its own drone's camera's. The monitor passes each shard's sender and they are dropped here; the ESP32 receiver is told (FPVF and the MAC) and drops them itself. Until the camera has answered, every camera's shards go on.
  let camera: Buffer | undefined;
  const tell = () => {
    if (!camera || !portPath) return;
    try {
      const fd = openSync(
        portPath,
        constants.O_WRONLY | constants.O_NOCTTY | constants.O_NONBLOCK
      );
      try {
        writeSync(fd, Buffer.concat([Buffer.from('FPVF'), camera]));
      } finally {
        closeSync(fd);
      }
    } catch (error) {
      console.error(`FPV receiver ${portPath}: ${error}`);
    }
  };
  const opened: Receiver = {
    choose: (mac) => {
      const chosen = Buffer.from(mac.replace(/:/g, ''), 'hex');
      if (chosen.length !== 6 || camera?.equals(chosen)) return;
      camera = chosen;
      tell();
    },
    listeners: new Set(),
    heard: 0,
    serial: 0,
    copies: 0,
    chunks: 0,
  };
  // The monitor passes every shard it hears; the first of each goes on. The capture time tells a restarted camera's frame numbers from the same numbers before it.
  const recent = new Set<string>();
  const deliver = (chunk: Buffer) => {
    opened.copies++;
    const key = `${chunk.readUInt16LE(4)}:${chunk[6]}:${chunk.readUInt32LE(12)}`;
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
    if (camera && !message.subarray(5, 11).equals(camera)) return;
    opened.signal = message.readInt8(4);
    deliver(message.subarray(11));
  });
  monitor.on('error', () => {});
  monitor.bind(5602, '127.0.0.1');
  // The port is opened again, with its settings, after an error or 5 s without a whole shard: another program opening the port (a flash, a serial monitor) can reset the receiver or leave the port at another baud, and an error ended the old stream for good.
  let port: tty.ReadStream | undefined;
  let portPath: string | undefined;
  let heard = 0;
  const open = () => {
    port?.destroy();
    port = undefined;
    portPath = undefined;
    for (const path of candidates()) {
      try {
        // Not stty: it applies settings only once the output has drained, and a port left at its defaults echoes the receiver's bytes back while a 0x13 among them (XOFF) holds the output, so it waited forever and froze the server (2026-10-31). This flushes and applies at once, and the timeout bounds it anyway.
        execFileSync('python3', ['-c', TERMIOS, path], { timeout: 3000 });
        port = new tty.ReadStream(
          openSync(path, constants.O_RDONLY | constants.O_NOCTTY)
        );
        portPath = path;
        if (!RECEIVERS.includes(path)) tried.set(path, statSync(path).ctimeMs);
        break;
      } catch (error) {
        // 3: the bridge has it.
        if ((error as { status?: number }).status !== 3)
          console.error(`FPV receiver ${path}: ${error}`);
      }
    }
    if (!port) return;
    // The receiver forgets its camera when it restarts, which the port's reopening follows.
    tell();
    heard = Date.now();
    let buffer = Buffer.alloc(0);
    port.on('data', (data: Buffer) => {
      buffer = Buffer.concat([buffer, data]);
      if (buffer.includes('FPVR')) opened.serial = heard = Date.now();
      for (;;) {
        const at = buffer.indexOf('FPV2', 3);
        if (at < 0) {
          buffer = buffer.subarray(Math.max(0, buffer.length - 3));
          break;
        }
        const length = buffer.readUInt16LE(at - 3);
        if (length < 20 || length > 1500) {
          buffer = buffer.subarray(at + 1);
          continue;
        }
        if (buffer.length < at + length) break;
        opened.serial = heard = Date.now();
        deliver(buffer.subarray(at, at + length));
        buffer = buffer.subarray(at + length);
      }
    });
    port.on('error', () => port?.destroy());
  };
  open();
  setInterval(() => {
    if (Date.now() - heard > 5000) open();
  }, 5000);
  return (store.fpvReceiver = opened);
}

// The camera's byte budget for the receiver, KB/s little-endian in its K V request: the ESP32 receiver's USB link carries about 98 KB/s; the monitor-mode adapter takes in about 210 shards/s whatever is sent (2026-10-31: 12 KB frames, 15 shards each at 32 fps, reached the dashboard at 16 fps, where 190 KB/s, about 5 KB frames, gave 28 fps), so frames stay near 5 KB (the camera aims at 85%).
function radioBudget() {
  const kilobytes = Date.now() - receiver().heard < 2000 ? 190 : 90;
  return [kilobytes & 0xff, kilobytes >> 8];
}

// A receiver is there: the monitor or the ESP32 receiver heard from lately, or a port a receiver has been on (one from before FPVR is silent until video comes).
function radioUp() {
  return (
    Date.now() - receiver().heard < 2000 ||
    Date.now() - receiver().serial < 2000 ||
    RECEIVERS.some((path) => existsSync(path))
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
