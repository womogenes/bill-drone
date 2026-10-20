import type { EspWorkerCommand, EspWorkerMessage } from './esp-connection';
import { encodeCommand } from './protocol';

// Networking has its own event loop, independent of React and chart rendering.
const worker = globalThis as unknown as {
  onmessage: ((event: MessageEvent<EspWorkerCommand>) => void) | null;
  postMessage: (message: EspWorkerMessage, transfer?: Transferable[]) => void;
};
let socket: WebSocket;
let deadline: number;
let timer: ReturnType<typeof setInterval>;
let closed = false;
const heartbeat = encodeCommand({ command: 'heartbeat' });

function close(type: 'close' | 'error' = 'close') {
  if (closed) return;
  closed = true;
  clearInterval(timer);
  socket.close();
  worker.postMessage({ type });
}

worker.onmessage = ({ data }) => {
  if (data.type === 'send') {
    if (!closed && socket.readyState === WebSocket.OPEN) {
      socket.send(data.packet);
    }
    return;
  }

  socket = new WebSocket(data.url);
  socket.binaryType = 'arraybuffer';
  deadline = performance.now() + 10000;
  timer = setInterval(() => {
    if (performance.now() >= deadline) {
      close();
    } else if (socket.readyState === WebSocket.OPEN) {
      socket.send(heartbeat);
    }
  }, 50);

  socket.onopen = () => {
    deadline = performance.now() + 3000;
    socket.send(heartbeat);
    worker.postMessage({ type: 'open' });
  };
  socket.onmessage = ({ data: packet }: MessageEvent<ArrayBuffer>) => {
    deadline = performance.now() + 3000;
    worker.postMessage({ type: 'packet', packet }, [packet]);
  };
  socket.onerror = () => close('error');
  socket.onclose = () => close();
};
