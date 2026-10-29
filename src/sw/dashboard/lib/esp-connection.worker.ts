import type { EspWorkerCommand, EspWorkerMessage } from './esp-connection';
import type { Targets } from './telemetry';

// Networking has its own event loop, independent of React and chart rendering, so targets keep streaming.
const worker = globalThis as unknown as {
  onmessage: ((event: MessageEvent<EspWorkerCommand>) => void) | null;
  postMessage: (message: EspWorkerMessage) => void;
};
let socket: WebSocket;
let deadline: number;
let timer: ReturnType<typeof setInterval>;
let closed = false;
let targets: Targets;

function close(type: 'close' | 'error' = 'close') {
  if (closed) return;
  closed = true;
  clearInterval(timer);
  socket.close();
  worker.postMessage({ type });
}

function stream(beforeRequest = false) {
  // A congested socket gets the latest snapshot when it drains, without accumulating old targets.
  if (
    closed ||
    socket.readyState !== WebSocket.OPEN ||
    (!beforeRequest && socket.bufferedAmount)
  )
    return;
  socket.send(JSON.stringify({ type: 'targets', ...targets }));
}

worker.onmessage = ({ data }) => {
  if (data.type === 'targets') {
    targets = data.targets;
    stream();
    return;
  }
  if (data.type === 'send') {
    if (!closed && socket.readyState === WebSocket.OPEN) {
      // Current targets first, so an arm request never overtakes the zero throttle it needs.
      stream(true);
      socket.send(JSON.stringify(data.message));
    }
    return;
  }

  targets = data.targets;
  socket = new WebSocket(data.url);
  deadline = performance.now() + 10000;
  timer = setInterval(() => {
    if (performance.now() >= deadline) {
      close();
    } else if (socket.readyState === WebSocket.OPEN) {
      stream();
    }
  }, 50);

  socket.onopen = () => {
    deadline = performance.now() + 3000;
    stream();
    worker.postMessage({ type: 'open' });
  };
  socket.onmessage = ({ data: text }: MessageEvent<string>) => {
    deadline = performance.now() + 3000;
    worker.postMessage({ type: 'message', message: JSON.parse(text) });
  };
  socket.onerror = () => close('error');
  socket.onclose = () => close();
};
