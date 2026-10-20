export type EspWorkerMessage =
  | { type: 'open' | 'close' | 'error' }
  | { type: 'packet'; packet: ArrayBuffer };

export type EspWorkerCommand =
  { type: 'connect'; url: string } | { type: 'send'; packet: ArrayBuffer };

export class EspConnection {
  connected = false;
  private worker: Worker;

  constructor(
    address: string,
    onOpen: () => void,
    onPacket: (packet: ArrayBuffer) => void,
    onClose: () => void
  ) {
    this.worker = new Worker(
      new URL('./esp-connection.worker.ts', import.meta.url)
    );
    this.worker.onmessage = ({ data }: MessageEvent<EspWorkerMessage>) => {
      if (data.type === 'open') {
        this.connected = true;
        onOpen();
      } else if (data.type === 'packet') {
        onPacket(data.packet);
      } else {
        this.close();
        onClose();
      }
    };
    this.worker.onerror = () => {
      this.close();
      onClose();
    };
    this.worker.postMessage({ type: 'connect', url: `ws://${address}/ws` });
  }

  send(packet: ArrayBuffer) {
    if (this.connected) {
      this.worker.postMessage({ type: 'send', packet }, [packet]);
    }
  }

  close() {
    this.connected = false;
    this.worker.terminate();
  }
}
