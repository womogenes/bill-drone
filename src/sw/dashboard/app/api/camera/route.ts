import {
  execFile,
  execFileSync,
  spawn,
  type ChildProcess,
} from 'node:child_process';
import dgram from 'node:dgram';
import {
  closeSync,
  constants,
  existsSync,
  openSync,
  readdirSync,
  readFileSync,
  realpathSync,
  statSync,
  writeFileSync,
  writeSync,
} from 'node:fs';
import { resolve } from 'node:path';
import { decode } from '@/lib/fec';
import { fpvRecording, recordFpvFrame } from '@/lib/fpv-recorder';
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
  // FPV frames delivered, for the status, and where the others went: frames begun (a shard heard), those a newer frame overtook short of shards, and those with enough shards that did not decode.
  received?: number;
  started?: number;
  short?: number;
  undecoded?: number;
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
  hub.received = hub.started = hub.short = hub.undecoded = 0;
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
      if (current >= 0 && !done) hub.short!++;
      hub.started!++;
      current = frame;
      shards = new Map();
      done = false;
    }
    if (done || index >= message[7] || shards.has(index)) return;
    shards.set(index, message.subarray(20));
    if (shards.size < data) return;
    done = true;
    const frameBytes = decode(shards, data, length);
    if (!frameBytes) {
      hub.undecoded!++;
      return;
    }
    hub.received!++;
    const { jpeg, tag } = splitTag(frameBytes);
    recordFpvFrame(jpeg, message.readUInt32LE(12), frame);
    const part = new Uint8Array(
      Buffer.concat([
        Buffer.from(
          `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\nX-Time: ${message.readUInt32LE(12)} ${message.readUInt32LE(16)} ${message.readUInt16LE(4)}\r\n${tag ? `X-Tag: ${tag}\r\n` : ''}\r\n`
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
    let asking = false;
    let ticks = 0;
    keepalive = () => {
      if (asking) return;
      asking = true;
      // The camera's MAC, asked for at the start, every 10 s (a camera swapped) and at every keepalive until one answers; one request at a time, as the drone holds a single slot. A camera without the N request (before 2026-11-06) is not told apart.
      (ticks++ % 20 === 0 || !radio.camera
        ? cameraRequest('N')
            .then((text) => radio.choose(text.split(' ')[0]))
            .catch(() => {})
        : Promise.resolve()
      )
        .then(() => cameraRequest(op, op === 'V' ? radioBudget() : []))
        .catch(() => {})
        .finally(() => (asking = false));
    };
    keepalive();
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

// FPV over the radio, needing no network, comes from the monitor-mode Wi-Fi adapter's reader (sw/fpv_monitor.py) as UDP datagrams to 127.0.0.1:5602, with FPVM every 500 ms while it runs, or from FPV receivers (fw/fpv_rx) on USB serial ports, every one plugged in at once: ports FPV_RX names (space separated) and any other /dev/ttyUSB* or ttyACM*. USB cannot tell the boards apart (the receiver's CP2102 has the same serial number as the drone's and the controller's), so a port is a receiver's once it sends what a receiver sends, video or the FPVR it sends twice a second without any (firmware from 2026-11-10): every port not yet known is opened for 5 s, each again 15 s after it was last found silent (opening a drone's port this way does not reset it). A port another process holds locked (the bridge's controller, a camera update) is passed over by /proc/locks without being opened. Receivers are kept open for as long as the server runs, their chunks going to whichever FPV stream is running: closing the serial port after each stream left it open in the process, and the stale readers took a share of the bytes.
const RECEIVERS = (process.env.FPV_RX ?? '').split(/\s+/).filter(Boolean);
// The USB serial ports present, FPV_RX's first, each once.
function serialPorts() {
  const ports = new Map<string, string>();
  for (const path of [
    ...RECEIVERS,
    ...readdirSync('/dev')
      .filter((name) => name.startsWith('ttyUSB') || name.startsWith('ttyACM'))
      .map((name) => `/dev/${name}`),
  ]) {
    if (!existsSync(path)) continue;
    const device = realpathSync(path);
    if (!ports.has(device)) ports.set(device, path);
  }
  return [...ports.values()];
}
// A port another process holds a flock on, which /proc/locks names by its device and inode (major:minor:inode), so the controller's port is never opened: every receiver is looked for all the time now, not only until one is found.
function locked(path: string) {
  try {
    const { dev, ino } = statSync(path);
    const major = ((dev >> 8) & 0xfff).toString(16);
    const minor = ((dev & 0xff) | ((dev >> 12) & 0xfff00)).toString(16);
    return readFileSync('/proc/locks', 'latin1')
      .split('\n')
      .some((line) => {
        const at = /\bFLOCK\b.*\s([0-9a-f]+):([0-9a-f]+):(\d+)\s/.exec(line);
        return (
          at !== null &&
          Number(at[3]) === ino &&
          parseInt(at[1], 16) === parseInt(major, 16) &&
          parseInt(at[2], 16) === parseInt(minor, 16)
        );
      });
  } catch {
    return false;
  }
}
// When each port was last opened and found silent. A silent port is opened again every RETRY_MS: one try per plug-in left the receiver unread for good when its try came while it was restarting or before the camera sent (2026-11-11).
const RETRY_MS = 15000;
const tried = new Map<string, number>();
// A receiver's port raw at 1 Mbaud: input and output flushed, output resumed, settings applied now (TCSANOW). Exits with 3, touching nothing, if another process holds the port's lock.
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
// A receiver's port read in blocks as they come, to stdout (TERMIOS has set it raw).
const READER = `import os, sys
fd = os.open(sys.argv[1], os.O_RDONLY | os.O_NOCTTY)
out = sys.stdout.buffer
while True:
    data = os.read(fd, 65536)
    if not data:
        break
    out.write(data)
    out.flush()`;
// The S3 receiver's FPVT report (fw/fpv_rx), since its boot: frames heard, declined for buffer room, short of shards (the air), shards its buffer refused, its longest USB write in the last second (us) and bytes written, and the heard frames' shards sent and heard (receiver firmware from 2026-11-14 on; 28 bytes before, 36 now).
type ReceiverStats = {
  heard: number;
  declined: number;
  short: number;
  refused: number;
  usbLongestUs: number;
  usbBytes: number;
  shardsSent: number;
  shardsHeard: number;
};
// One receiver's port. serial: when it last sent a beacon, report or shard (a receiver once it has), video: when it last sent a shard; fast: an S3 (its beacon is FPVS, fw/fpv_rx make s3), whose own USB carries far more than the ESP32's 1 Mbaud; skipped: bytes the parser passed over to find a shard (beacons, reports, and any stream that lost bytes); shards: those it passed on, new to the stream or not; share: the shards it was told to pass (FPVK's first, count and modulus) and taken: those it acknowledged (FPVA, firmware from 2026-11-14); rssi: each shard's RSSI (dBm) with when it came, for the last second's in the status.
type Port = {
  path: string;
  reader: ChildProcess;
  opened: number;
  serial: number;
  video: number;
  fast: boolean;
  skipped: number;
  shards: number;
  fresh: number;
  stats?: ReceiverStats;
  share?: string;
  taken?: string;
  rssi: [number, number][];
};
// heard: when the monitor last said anything, signal: its last RSSI (dBm); copies: every chunk copy heard, chunks: the first of each; camera: the camera whose shards go on, as K N names it, kept on disk across restarts and drone-link outages (sw/recordings/fpv-camera.json).
// choose: takes only this camera's shards from now on (its MAC, as K N reports it).
type Receiver = {
  listeners: Set<(chunk: Buffer) => void>;
  choose: (mac: string) => void;
  ports: Map<string, Port>;
  camera?: Buffer;
  heard: number;
  signal?: number;
  copies: number;
  chunks: number;
};
const store = globalThis as { fpvReceiver?: Receiver };
const REPORT_BYTES = 36;
const CAMERA_FILE = resolve(process.cwd(), '../recordings/fpv-camera.json');
const LIVE_MS = 2000;

function live(port: Port) {
  return Date.now() - port.serial < LIVE_MS;
}

// Bytes to a receiver: its commands (fw/fpv_rx commands), FPVF and FPVK.
function tell(port: Port, bytes: Buffer) {
  try {
    const fd = openSync(
      port.path,
      constants.O_WRONLY | constants.O_NOCTTY | constants.O_NONBLOCK
    );
    try {
      writeSync(fd, bytes);
    } finally {
      closeSync(fd);
    }
  } catch (error) {
    console.error(`FPV receiver ${port.path}: ${error}`);
  }
}

function receiver() {
  if (store.fpvReceiver) return store.fpvReceiver;
  // Every camera sends with the same BSSID and its own MAC as the sender (fw/esp32cam/src/fpv.h), so with two drones about each dashboard takes its own drone's camera's. The monitor passes each shard's sender and they are dropped here; the receivers are told (FPVF and the MAC) and drop them themselves. The MAC is kept on disk: with the drone link down K N cannot answer, and the receivers, forgetting their camera when they restart, passed another camera's frames (2026-11-14, a second, damaged camera board's frames, upside down). Until a camera has ever answered, every camera's shards go on.
  let saved: { camera?: string } = {};
  try {
    saved = JSON.parse(readFileSync(CAMERA_FILE, 'utf8'));
  } catch {}
  const opened: Receiver = {
    choose: (mac) => {
      const chosen = Buffer.from(mac.replace(/:/g, ''), 'hex');
      if (chosen.length !== 6 || opened.camera?.equals(chosen)) return;
      opened.camera = chosen;
      try {
        writeFileSync(
          CAMERA_FILE,
          JSON.stringify({ camera: chosen.toString('hex') })
        );
      } catch {}
    },
    listeners: new Set(),
    ports: new Map(),
    camera: saved.camera ? Buffer.from(saved.camera, 'hex') : undefined,
    heard: 0,
    copies: 0,
    chunks: 0,
  };
  // Each receiver's shards and the monitor's go into one stream, the first of each shard going on: several receivers hear the same shards, and pass the same ones whenever they fill in for each other. The capture time tells a restarted camera's frame numbers from the same numbers before it.
  const recent = new Set<string>();
  const deliver = (chunk: Buffer) => {
    opened.copies++;
    const key = `${chunk.readUInt16LE(4)}:${chunk[6]}:${chunk.readUInt32LE(12)}`;
    if (recent.has(key)) return false;
    recent.add(key);
    if (recent.size > 1024) recent.delete(recent.values().next().value!);
    opened.chunks++;
    for (const listener of opened.listeners) listener(chunk);
    return true;
  };
  const monitor = dgram.createSocket('udp4');
  monitor.on('message', (message) => {
    opened.heard = Date.now();
    if (message.toString('latin1', 0, 4) !== 'FPVC') return;
    if (opened.camera && !message.subarray(5, 11).equals(opened.camera)) return;
    opened.signal = message.readInt8(4);
    deliver(message.subarray(11));
  });
  monitor.on('error', () => {});
  monitor.bind(5602, '127.0.0.1');
  const open = (path: string) => {
    try {
      // Not stty: it applies settings only once the output has drained, and a port left at its defaults echoes the receiver's bytes back while a 0x13 among them (XOFF) holds the output, so it waited forever and froze the server (2026-10-31). This flushes and applies at once, and the timeout bounds it anyway.
      execFileSync('python3', ['-c', TERMIOS, path], { timeout: 3000 });
    } catch (error) {
      // 3: another process has it.
      if ((error as { status?: number }).status !== 3)
        console.error(`FPV receiver ${path}: ${error}`);
      tried.set(path, Date.now());
      return;
    }
    // Read by its own process, whose blocking reads keep up with the S3 receiver's 400 KB/s whatever this server is doing: read here, on the event loop, 8% of the bytes were lost while it was busy (2026-11-14), and torn shards cost a quarter of the VGA frames. Should this server fall behind, the pipe fills and the reader waits, holding the bytes back instead.
    const reader = spawn('python3', ['-c', READER, path], {
      stdio: ['ignore', 'pipe', 'ignore'],
    });
    const port: Port = {
      path,
      reader,
      opened: Date.now(),
      serial: 0,
      video: 0,
      fast: false,
      skipped: 0,
      shards: 0,
      fresh: 0,
      rssi: [],
    };
    opened.ports.set(path, port);
    let buffer = Buffer.alloc(0);
    reader.stdout!.on('data', (data: Buffer) => {
      buffer = Buffer.concat([buffer, data]);
      if (buffer.includes('FPVR')) port.serial = Date.now();
      if (buffer.includes('FPVS')) {
        port.serial = Date.now();
        port.fast = true;
      }
      for (;;) {
        // A report, or the acknowledgment of its share (FPVA), ahead of the next shard is taken out whole once all of it has come.
        const at = buffer.indexOf('FPV2', 3);
        const report = buffer.indexOf('FPVT');
        if (report >= 0 && (at < 0 || report < at - 3)) {
          if (buffer.length < report + REPORT_BYTES) break;
          const value = (index: number) =>
            buffer.readUInt32LE(report + 4 + 4 * index);
          port.stats = {
            heard: value(0),
            declined: value(1),
            short: value(2),
            refused: value(3),
            usbLongestUs: value(4),
            usbBytes: value(5),
            shardsSent: value(6),
            shardsHeard: value(7),
          };
          port.serial = Date.now();
          port.skipped += report;
          buffer = buffer.subarray(report + REPORT_BYTES);
          continue;
        }
        const ack = buffer.indexOf('FPVA');
        if (ack >= 0 && (at < 0 || ack < at - 3)) {
          if (buffer.length < ack + 7) break;
          port.taken = [...buffer.subarray(ack + 4, ack + 7)].join(' ');
          port.skipped += ack;
          buffer = buffer.subarray(ack + 7);
          continue;
        }
        // No shard's start yet: the last 6 bytes stay, which may be the next shard's length and RSSI and the first three of its FPV2. Keeping only 3 tore the shard whenever a read ended inside its marker, about 2% of the S3 receiver's shards, a quarter of the VGA frames (2026-11-14).
        if (at < 0) {
          port.skipped += Math.max(0, buffer.length - 6);
          buffer = buffer.subarray(Math.max(0, buffer.length - 6));
          break;
        }
        port.skipped += at - 3;
        const length = buffer.readUInt16LE(at - 3);
        if (length < 20 || length > 1500) {
          port.skipped += 4;
          buffer = buffer.subarray(at + 1);
          continue;
        }
        if (buffer.length < at + length) break;
        port.serial = port.video = Date.now();
        port.rssi.push([port.video, buffer.readInt8(at - 1)]);
        if (port.rssi.length > 4096) port.rssi.splice(0, 2048);
        port.shards++;
        if (deliver(buffer.subarray(at, at + length))) port.fresh++;
        buffer = buffer.subarray(at + length);
      }
    });
    reader.on('exit', () => {
      if (opened.ports.get(path) === port) opened.ports.delete(path);
    });
  };
  // Every 500 ms: each receiver is told its camera and its share, which it forgets when it restarts. Every 5 s: a port silent for 5 s is closed (another program opening it, a flash or a serial monitor, can reset the receiver or leave the port at another baud), and every port not open, not locked and not tried in the last RETRY_MS is opened.
  setInterval(() => {
    const receivers = [...opened.ports.values()].filter(live);
    // Each receiver's share in proportion to its USB link (an S3's about 9 ESP32s'), so the links add up: shards whose index modulo the weights' sum falls in its range are its own (fw/fpv_rx FPVK).
    const modulus = receivers.reduce((sum, port) => sum + weight(port), 0);
    let first = 0;
    for (const port of receivers) {
      port.share = `${first} ${weight(port)} ${modulus}`;
      tell(
        port,
        Buffer.concat([
          opened.camera
            ? Buffer.concat([Buffer.from('FPVF'), opened.camera])
            : Buffer.alloc(0),
          Buffer.from([...Buffer.from('FPVK'), first, weight(port), modulus]),
        ])
      );
      first += weight(port);
    }
  }, 500);
  const scan = () => {
    for (const port of opened.ports.values())
      if (Date.now() - Math.max(port.serial, port.opened) > 5000) {
        tried.set(port.path, Date.now());
        port.reader.kill();
        opened.ports.delete(port.path);
      }
    for (const path of serialPorts())
      if (
        !opened.ports.has(path) &&
        Date.now() - (tried.get(path) ?? 0) > RETRY_MS &&
        !locked(path)
      )
        open(path);
  };
  scan();
  setInterval(scan, 5000);
  return (store.fpvReceiver = opened);
}

// The camera's byte budget for the receivers, KB/s little-endian in its K V request: an ESP32 receiver's USB link carries about 98 KB/s; the monitor-mode adapter takes in about 210 shards/s whatever is sent (2026-10-31: 12 KB frames, 15 shards each at 32 fps, reached the dashboard at 16 fps, where 190 KB/s, about 5 KB frames, gave 28 fps), so frames stay near 5 KB (the camera aims at 85%). The S3 receiver gets FAST_BUDGET and, after it, a shard budget (shards/s, little-endian; camera firmware from 2026-11-14, which takes the monitor's 190 when none comes), so the camera's frames grow toward its fpvRadioMaxBytes.
// Several receivers that took their shares carry the sum of their links, each its share of every frame; one that has not acknowledged its share (older firmware) passes the first shards as before and caps the budget at what one link carries. Every receiver hears every shard, so the shard budget is the smallest receiver's: SLOW_SHARDS for an ESP32 once several share (alone it is sent none, as before).
const FAST_BUDGET = { kilobytes: 800, shards: 2000 };
const SLOW_BUDGET = 90;
const SLOW_SHARDS = 1000;
function weight(port: Port) {
  return port.fast ? 9 : 1;
}
function radioBudget() {
  const receivers = [...receiver().ports.values()].filter(live);
  if (!receivers.length) {
    const kilobytes = Date.now() - receiver().heard < 2000 ? 190 : SLOW_BUDGET;
    return [kilobytes & 0xff, kilobytes >> 8];
  }
  const each = receivers.map((port) =>
    port.fast ? FAST_BUDGET.kilobytes : SLOW_BUDGET
  );
  const shared =
    receivers.length > 1 &&
    receivers.every((port) => port.taken === port.share);
  const kilobytes = shared
    ? each.reduce((a, b) => a + b, 0)
    : Math.max(...each);
  const shards = Math.min(
    ...receivers.map((port) => (port.fast ? FAST_BUDGET.shards : SLOW_SHARDS))
  );
  const budget = [kilobytes & 0xff, kilobytes >> 8];
  return shared || receivers.some((port) => port.fast)
    ? [...budget, shards & 0xff, shards >> 8]
    : budget;
}

// A receiver is there: the monitor or a serial receiver heard from lately, or a port a receiver has been on (one from before FPVR is silent until video comes).
function radioUp() {
  return (
    Date.now() - receiver().heard < 2000 ||
    [...receiver().ports.values()].some(live) ||
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
    const fpvHub = [...hubs.values()].find((hub) => hub.received !== undefined);
    const received = fpvHub?.received;
    const now = Date.now();
    const state = (await cameraRequest('S')).split(' ')[4];
    return Response.json(
      {
        camera: state,
        received,
        started: fpvHub?.started,
        short: fpvHub?.short,
        undecoded: fpvHub?.undecoded,
        copies: store.fpvReceiver?.copies,
        chunks: store.fpvReceiver?.chunks,
        signal: store.fpvReceiver?.signal,
        camera_mac: store.fpvReceiver?.camera?.toString('hex'),
        budget: store.fpvReceiver ? radioBudget() : undefined,
        recording: fpvRecording(),
        // Each serial receiver: its share and whether it took it, shards passed (all, and new to the stream) and their RSSI (dBm) over the last second, and its own report.
        receivers: [...(store.fpvReceiver?.ports.values() ?? [])].map(
          (port) => {
            const rssi = port.rssi
              .filter(([time]) => now - time < 1000)
              .map(([, value]) => value);
            return {
              path: port.path,
              fast: port.fast,
              live: live(port),
              share: port.share,
              taken: port.taken,
              shards: port.shards,
              fresh: port.fresh,
              skipped: port.skipped,
              rssi: rssi.length
                ? {
                    shards: rssi.length,
                    mean: Math.round(
                      rssi.reduce((a, b) => a + b, 0) / rssi.length
                    ),
                    min: Math.min(...rssi),
                  }
                : undefined,
              report: port.stats,
            };
          }
        ),
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
