import WebSocket from 'ws';
import {
  appendFileSync,
  closeSync,
  fsyncSync,
  mkdirSync,
  openSync,
} from 'node:fs';
import path from 'node:path';

// Runs on the laptop's Next server; the browser never relays sample packets.
type Recorder = {
  address: string;
  socket?: WebSocket;
  ready: boolean;
  file?: string;
  dataFd?: number;
  eventsFd?: number;
  error?: string;
  reconnect?: ReturnType<typeof setTimeout>;
  sync?: ReturnType<typeof setInterval>;
  bytes: number;
  session?: number;
};
const host = globalThis as typeof globalThis & { flightRecorder?: Recorder };
const state = (host.flightRecorder ??= { address: '', ready: false, bytes: 0 });
const directory = path.resolve(process.cwd(), '../recordings');

export function recordingStatus() {
  return {
    ready: state.ready,
    address: state.address,
    file: state.file,
    error: state.error,
    bytes: state.bytes,
    writing: state.dataFd !== undefined,
  };
}

function finish(reason: string) {
  const { dataFd, eventsFd } = state;
  state.dataFd = state.eventsFd = undefined;
  clearInterval(state.sync);
  try {
    if (eventsFd !== undefined) {
      appendFileSync(
        eventsFd,
        JSON.stringify({
          type: 'closed',
          reason,
          received_at: new Date().toISOString(),
        }) + '\n'
      );
      fsyncSync(eventsFd);
    }
    if (dataFd !== undefined) fsyncSync(dataFd);
  } finally {
    if (dataFd !== undefined) closeSync(dataFd);
    if (eventsFd !== undefined) closeSync(eventsFd);
  }
}

export function connectRecorder(address: string) {
  const url = new URL(`ws://${address}/log`);
  // The dashboard connects to a local ESP, including loopback for local development.
  if (
    !/^(10\.|192\.168\.|172\.(1[6-9]|2\d|3[01])\.|127\.)\d/.test(
      url.hostname
    ) ||
    !/^\d+\.\d+\.\d+\.\d+$/.test(url.hostname)
  ) {
    throw new Error('Recorder address must be a local IPv4 address');
  }
  if (
    state.address === address &&
    state.socket &&
    state.socket.readyState < WebSocket.CLOSING
  )
    return;

  clearTimeout(state.reconnect);
  const previous = state.socket;
  state.socket = undefined;
  previous?.terminate();
  finish('Connection changed');
  mkdirSync(directory, { recursive: true });
  state.address = address;
  state.ready = false;
  const ws = new WebSocket(url);
  state.socket = ws;
  ws.binaryType = 'arraybuffer';

  let lastMessage = Date.now();
  const watchdog = setInterval(() => {
    if (state.socket !== ws) {
      clearInterval(watchdog);
      return;
    }
    if (ws.readyState === WebSocket.OPEN) ws.send('H');
    if (Date.now() - lastMessage < 3000) return;
    clearInterval(watchdog);
    state.socket = undefined;
    state.ready = false;
    state.error = 'Flight recorder timed out';
    try {
      finish('Logging timed out');
    } catch (error) {
      state.error = String(error);
    }
    ws.terminate();
    state.reconnect = setTimeout(() => connectRecorder(address), 1000);
  }, 1000);

  function fail(error: unknown) {
    state.error = String(error);
    state.ready = false;
    console.error('Flight recorder:', error);
    ws.terminate();
  }

  ws.addEventListener('message', (event) => {
    if (state.socket !== ws) return;
    lastMessage = Date.now();
    try {
      if (typeof event.data === 'string') {
        const message = JSON.parse(event.data);
        if (message.type === 'ready') {
          state.ready = true;
          state.error = undefined;
          return;
        }
        if (message.type === 'start') {
          finish('New flight or resumed stream');
          const stamp = new Date().toISOString().replace(/[:.]/g, '-');
          const stem = path.join(directory, `${stamp}_${message.session}`);
          state.dataFd = openSync(`${stem}.bin`, 'wx');
          state.eventsFd = openSync(`${stem}.jsonl`, 'wx');
          state.file = path.basename(`${stem}.bin`);
          state.bytes = 0;
          state.session = message.session;
          state.sync = setInterval(() => {
            try {
              if (state.dataFd !== undefined) fsyncSync(state.dataFd);
              if (state.eventsFd !== undefined) fsyncSync(state.eventsFd);
            } catch (error) {
              fail(error);
            }
          }, 1000);
        }
        if (state.eventsFd !== undefined) {
          appendFileSync(
            state.eventsFd,
            JSON.stringify({
              ...message,
              received_at: new Date().toISOString(),
            }) + '\n'
          );
        }
        if (message.type === 'stop') finish(message.reason);
      } else {
        const packet = Buffer.from(event.data as ArrayBuffer);
        const count = packet.readUInt16LE(8);
        const size = packet.readUInt16LE(10);
        if (
          packet.toString('ascii', 0, 4) !== 'FLG1' ||
          packet.readUInt32LE(4) !== state.session ||
          size !== 48 ||
          count < 1 ||
          count > 16 ||
          packet.length !== 12 + count * size
        ) {
          throw new Error('Invalid flight-log packet');
        }
        if (state.dataFd === undefined)
          throw new Error('Flight-log packet arrived before metadata');
        appendFileSync(state.dataFd, packet);
        state.bytes += packet.length;
      }
    } catch (error) {
      fail(error);
    }
  });
  ws.addEventListener('error', () => {
    if (state.socket === ws) {
      state.error = 'Flight recorder disconnected';
      state.ready = false;
    }
  });
  ws.addEventListener('close', () => {
    clearInterval(watchdog);
    if (state.socket !== ws) return;
    state.ready = false;
    state.socket = undefined;
    try {
      finish('Logging connection lost');
    } catch (error) {
      state.error = String(error);
    }
    // A resumed flight gets another file with the same ESP session ID.
    state.reconnect = setTimeout(() => connectRecorder(address), 1000);
  });
}
