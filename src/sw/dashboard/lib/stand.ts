import dgram from 'node:dgram';
import { readFile } from 'node:fs/promises';
import net from 'node:net';
import { homedir } from 'node:os';
import { resolve } from 'node:path';

// The thrust stand (src/fw/stand/main/stand.c) as the dashboard's server sees it: one connection over Wi-Fi, found by the stand's UDP broadcast, shared by every open stand page. The stand takes several clients, so a run from sw/stand.py keeps going beside it.
const BEACON_PORT = 3334;
// Readings older than this are dropped.
const KEEP_MS = 10 * 60_000;

export type StandSample =
  | { seq: number; kind: 'cell'; time: number; cell: number; raw: number }
  | { seq: number; kind: 'supply'; time: number; volts: number }
  | { seq: number; kind: 'esc'; time: number; pulse: number; hz: number };

export type StandState = {
  state: 'searching' | 'connecting' | 'connected' | 'offline';
  host: string | null;
  // Laptop time (ms) of the last line from the stand.
  heard: number | null;
  // Times the stand's clock went back: it restarted (power, brownout, an update).
  restarts: number;
  // Grams per count of each cell, from sw/stand.py scale (~/.config/bill-drone/stand.json).
  scale: Record<string, number>;
};

type Stand = StandState & {
  socket: net.Socket | null;
  samples: StandSample[];
  seq: number;
  // Laptop ms minus stand ms, from the least-delayed lines: maps the stand's clock onto the laptop's.
  offset: number | null;
  offsetAt: number;
  lastUs: number;
};

const globals = globalThis as unknown as { stand?: Stand };
const stand: Stand = (globals.stand ??= {
  state: 'offline',
  host: null,
  heard: null,
  restarts: 0,
  scale: {},
  socket: null,
  samples: [],
  seq: 0,
  offset: null,
  offsetAt: 0,
  lastUs: 0,
});

function findStand(seconds: number) {
  return new Promise<{ host: string; port: number } | null>((done) => {
    const socket = dgram.createSocket({ type: 'udp4', reuseAddr: true });
    const timer = setTimeout(() => finish(null), seconds * 1000);
    const finish = (found: { host: string; port: number } | null) => {
      clearTimeout(timer);
      socket.close();
      done(found);
    };
    socket.on('message', (message) => {
      const [kind, host, port] = message.toString().trim().split(' ');
      if (kind === 'STAND' && host && port)
        finish({ host, port: Number(port) });
    });
    socket.on('error', () => finish(null));
    socket.bind(BEACON_PORT);
  });
}

function receive(line: string, now: number) {
  const parts = line.trim().split(' ');
  const us = Number(parts[1]);
  if (!['F', 'V', 'E', 'H'].includes(parts[0]) || !Number.isFinite(us)) return;
  if (us < stand.lastUs - 1_000_000) {
    stand.restarts += 1;
    stand.offset = null;
  }
  stand.lastUs = us;
  // Wi-Fi delays lines by varying amounts; the smallest offset is the truest. It may creep up by 10 ppm, the stand crystal's drift.
  const candidate = now - us / 1000;
  stand.offset =
    stand.offset === null
      ? candidate
      : Math.min(stand.offset + (now - stand.offsetAt) * 1e-5, candidate);
  stand.offsetAt = now;
  const time = us / 1000 + stand.offset;
  const seq = ++stand.seq;
  if (parts[0] === 'F')
    stand.samples.push({
      seq,
      kind: 'cell',
      time,
      cell: Number(parts[2]),
      raw: Number(parts[3]),
    });
  else if (parts[0] === 'V')
    stand.samples.push({
      seq,
      kind: 'supply',
      time,
      volts: Number(parts[2]) / 1000,
    });
  else if (parts[0] === 'E')
    stand.samples.push({
      seq,
      kind: 'esc',
      time,
      pulse: Number(parts[2]),
      hz: Number(parts[3]),
    });
}

async function connect() {
  stand.state = 'searching';
  const found = await findStand(3);
  if (!found) {
    stand.state = 'offline';
    return;
  }
  stand.state = 'connecting';
  stand.host = found.host;
  stand.scale = await readFile(
    resolve(homedir(), '.config/bill-drone/stand.json'),
    'utf8'
  )
    .then((text) => JSON.parse(text).grams_per_count ?? {})
    .catch(() => ({}));
  const socket = net.connect(found.port, found.host);
  stand.socket = socket;
  let buffer = '';
  socket.setNoDelay(true);
  socket.setTimeout(3000);
  socket.on('connect', () => (stand.state = 'connected'));
  socket.on('data', (chunk) => {
    const now = Date.now();
    stand.heard = now;
    buffer += chunk.toString();
    const lines = buffer.split('\n');
    buffer = lines.pop() ?? '';
    for (const line of lines) receive(line, now);
    const cutoff = now - KEEP_MS;
    if (stand.samples.length && stand.samples[0].time < cutoff)
      stand.samples = stand.samples.filter((sample) => sample.time >= cutoff);
  });
  // A stand that stops talking (power, Wi-Fi) is dropped and searched for again on the next request.
  socket.on('timeout', () => socket.destroy());
  socket.on('close', () => {
    if (stand.socket === socket) {
      stand.socket = null;
      stand.state = 'offline';
    }
  });
  socket.on('error', () => socket.destroy());
}

// The stand's state and its readings after seq, connecting first if it is not.
export function standSince(seq: number) {
  if (!stand.socket && stand.state === 'offline') void connect();
  const { state, host, heard, restarts, scale } = stand;
  return {
    status: { state, host, heard, restarts, scale } satisfies StandState,
    samples: stand.samples.filter((sample) => sample.seq > seq),
    seq: stand.seq,
  };
}

// Stand runs (sw/stand.py run) beside this dashboard.
export const STAND_RUNS = resolve(process.cwd(), '../recordings/stand');
