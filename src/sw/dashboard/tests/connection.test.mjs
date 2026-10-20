import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { runInNewContext } from 'node:vm';
import ts from 'typescript';

const source = ts.transpileModule(
  readFileSync(
    new URL('../lib/esp-connection.worker.ts', import.meta.url),
    'utf8'
  ),
  { compilerOptions: { module: ts.ModuleKind.CommonJS } }
).outputText;
const zero = {
  base_throttle: 0,
  roll: 0,
  pitch: 0,
  yaw_rate: 0,
  servo: [0, 0],
  throttle: [0, 0],
};

function connection() {
  let now = 0;
  let tick;
  const sockets = [];
  const events = [];
  const sent = [];
  class Socket {
    static OPEN = 1;
    readyState = 0;
    bufferedAmount = 0;
    constructor() {
      sockets.push(this);
    }
    send(text) {
      sent.push({ at: now, ...JSON.parse(text) });
    }
    close() {
      this.readyState = 3;
    }
  }
  const worker = {
    exports: {},
    WebSocket: Socket,
    performance: { now: () => now },
    setInterval(callback, delay) {
      assert.equal(delay, 50);
      tick = callback;
    },
    clearInterval() {
      tick = undefined;
    },
    postMessage(message) {
      events.push(message);
    },
  };
  runInNewContext(source, worker);
  const command = (data) => worker.onmessage({ data });
  command({ type: 'connect', url: 'ws://test/ws', targets: zero });
  const socket = sockets[0];
  socket.readyState = Socket.OPEN;
  socket.onopen();
  return {
    command,
    socket,
    sent,
    events,
    advance(time) {
      now = time;
      tick?.();
    },
  };
}

test('changed targets send immediately; unchanged targets keep the 50 ms heartbeat', () => {
  const link = connection();
  link.command({ type: 'targets', targets: { ...zero, pitch: 4 } });
  assert.equal(link.sent.at(-1).pitch, 4);
  assert.equal(link.sent.at(-1).at, 0);
  link.advance(50);
  assert.equal(link.sent.at(-1).pitch, 4);
  assert.equal(link.sent.at(-1).at, 50);
});

test('a blocked socket keeps only the latest target and sends zero before stop', () => {
  const link = connection();
  link.socket.bufferedAmount = 100;
  for (let throttle = 1; throttle <= 500; throttle++) {
    link.command({
      type: 'targets',
      targets: { ...zero, base_throttle: throttle },
    });
  }
  assert.equal(link.sent.length, 1);
  link.command({ type: 'targets', targets: zero });
  link.socket.bufferedAmount = 0;
  link.command({ type: 'send', message: { type: 'stop' } });
  assert.equal(link.sent.at(-2).base_throttle, 0);
  assert.equal(link.sent.at(-1).type, 'stop');
  link.advance(50);
  assert.equal(link.sent.at(-1).base_throttle, 0);
});

test('arm is ordered after current zero targets even with buffered traffic', () => {
  const link = connection();
  link.command({ type: 'targets', targets: { ...zero, base_throttle: 500 } });
  link.socket.bufferedAmount = 100;
  link.command({ type: 'targets', targets: zero });
  link.command({ type: 'send', message: { type: 'arm' } });
  assert.equal(link.sent.at(-2).base_throttle, 0);
  assert.equal(link.sent.at(-1).type, 'arm');
});

test('status timeout closes once and prevents later targets from being sent', () => {
  const link = connection();
  link.advance(3000);
  const sent = link.sent.length;
  link.command({ type: 'targets', targets: { ...zero, base_throttle: 500 } });
  link.socket.onclose();
  assert.equal(link.sent.length, sent);
  assert.equal(link.events.filter(({ type }) => type === 'close').length, 1);
  const restarted = connection();
  assert.equal(restarted.sent[0].base_throttle, 0);
});

test('incoming status renews the connection deadline', () => {
  const link = connection();
  link.advance(2500);
  link.socket.onmessage({ data: JSON.stringify({ type: 'reading' }) });
  link.advance(3000);
  assert.equal(link.socket.readyState, 1);
  link.advance(5500);
  assert.equal(link.socket.readyState, 3);
});
