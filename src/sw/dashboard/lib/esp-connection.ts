import type { Side, Targets } from './telemetry';

// Requests the drone service (src/fw/controller/bridge.py) forwards; stop repeats until the drone confirms it.
export type DroneRequest =
  | { type: 'stop' | 'arm' | 'calibrate' }
  | { type: 'zero'; side: 'left' | 'right' }
  | { type: 'tune'; settings: Record<string, number> };

export type ServiceMessage =
  | ({ type: 'reading' } & Record<string, unknown>)
  | {
      type: 'result';
      request: Exclude<DroneRequest['type'], 'zero'>;
      error: string | null;
    }
  | { type: 'result'; request: 'zero'; side: Side; error: string | null };

export type EspWorkerMessage =
  | { type: 'open' | 'close' | 'error' }
  | { type: 'message'; message: ServiceMessage };

export type EspWorkerCommand =
  | { type: 'connect'; url: string; targets: Targets }
  | { type: 'targets'; targets: Targets }
  | { type: 'send'; message: DroneRequest };

export class EspConnection {
  connected = false;
  private worker: Worker;

  constructor(
    address: string,
    targets: Targets,
    onOpen: () => void,
    onMessage: (message: ServiceMessage) => void,
    onClose: () => void
  ) {
    this.worker = new Worker(
      new URL('./esp-connection.worker.ts', import.meta.url)
    );
    this.worker.onmessage = ({ data }: MessageEvent<EspWorkerMessage>) => {
      if (data.type === 'open') {
        this.connected = true;
        onOpen();
      } else if (data.type === 'message') {
        onMessage(data.message);
      } else {
        this.close();
        onClose();
      }
    };
    this.worker.onerror = () => {
      this.close();
      onClose();
    };
    this.worker.postMessage({
      type: 'connect',
      url: `ws://${address}/ws`,
      targets,
    } satisfies EspWorkerCommand);
  }

  targets(targets: Targets) {
    this.worker.postMessage({
      type: 'targets',
      targets,
    } satisfies EspWorkerCommand);
  }

  send(message: DroneRequest) {
    if (this.connected) {
      this.worker.postMessage({
        type: 'send',
        message,
      } satisfies EspWorkerCommand);
    }
  }

  close() {
    this.connected = false;
    this.worker.terminate();
  }
}
