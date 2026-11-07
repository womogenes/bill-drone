import { createReadStream, readFileSync, type ReadStream } from 'node:fs';

export const runtime = 'nodejs';

// The gamepad read on this laptop from its Linux input device rather than through the browser, whose layout for the 8BitDo Ultimate 2 Wireless (Bluetooth, 2dc8:6012) depends on the browser: Firefox swapped X and Y and lost the right stick. Its evdev codes (recorded 2026-11-07): A, B, X, Y 0x130, 0x131, 0x133, 0x134; LB, RB 0x136, 0x137; LT, RT 0x138, 0x139 with ABS_BRAKE and ABS_GAS 0-255; D-pad ABS_HAT0X/Y; left stick ABS_X/Y and right stick ABS_Z/RZ, 0-255 centered on 127.5. Select, start and home sent nothing. Buttons go out by the browser's standard index (components/gamepad.tsx BUTTONS).
const KEYS: Record<number, number> = {
  0x130: 0,
  0x131: 1,
  0x133: 2,
  0x134: 3,
  0x136: 4,
  0x137: 5,
  0x13a: 8,
  0x13b: 9,
  0x13d: 10,
  0x13e: 11,
  0x13c: 16,
};
const ABS_X = 0;
const ABS_Y = 1;
const ABS_Z = 2;
const ABS_RZ = 5;
const ABS_GAS = 9;
const ABS_BRAKE = 10;
const ABS_HAT0X = 16;
const ABS_HAT0Y = 17;
const EVENT_SIZE = 24;

type Pad = {
  name: string | null;
  buttons: number[];
  axes: number[];
};

type Reader = {
  pad: Pad;
  // Trigger keys and their analog axes, combined into one value each.
  triggers: { key: [number, number]; axis: [number, number] };
  hat: [number, number];
  stream?: ReadStream;
  clients: Set<ReadableStreamDefaultController<Uint8Array>>;
  dirty: boolean;
};

const globalReader = globalThis as unknown as { gamepadReader?: Reader };

// The first input device with a joystick handler, by /proc/bus/input/devices.
function findDevice() {
  for (const block of readFileSync('/proc/bus/input/devices', 'utf8').split(
    '\n\n'
  )) {
    const handlers = block.match(/^H: Handlers=(.*)$/m)?.[1] ?? '';
    const event = handlers.match(/\bevent\d+\b/)?.[0];
    if (!/\bjs\d+\b/.test(handlers) || !event) continue;
    return {
      path: `/dev/input/${event}`,
      name: block.match(/^N: Name="(.*)"$/m)?.[1] ?? 'Gamepad',
    };
  }
  return null;
}

function blank(): Pad {
  return { name: null, buttons: Array(17).fill(0), axes: [0, 0, 0, 0] };
}

function centered(value: number) {
  return Math.max(-1, Math.min(1, (value - 127.5) / 127.5));
}

function apply(reader: Reader, type: number, code: number, value: number) {
  const { pad, triggers, hat } = reader;
  if (type === 1) {
    if (code === 0x138) triggers.key[0] = value ? 1 : 0;
    else if (code === 0x139) triggers.key[1] = value ? 1 : 0;
    else if (code in KEYS) pad.buttons[KEYS[code]] = value ? 1 : 0;
  } else if (type === 3) {
    if (code === ABS_X) pad.axes[0] = centered(value);
    else if (code === ABS_Y) pad.axes[1] = centered(value);
    else if (code === ABS_Z) pad.axes[2] = centered(value);
    else if (code === ABS_RZ) pad.axes[3] = centered(value);
    else if (code === ABS_BRAKE) triggers.axis[0] = value / 255;
    else if (code === ABS_GAS) triggers.axis[1] = value / 255;
    else if (code === ABS_HAT0X) hat[0] = value;
    else if (code === ABS_HAT0Y) hat[1] = value;
  }
  pad.buttons[6] = Math.max(triggers.key[0], triggers.axis[0]);
  pad.buttons[7] = Math.max(triggers.key[1], triggers.axis[1]);
  pad.buttons[12] = hat[1] < 0 ? 1 : 0;
  pad.buttons[13] = hat[1] > 0 ? 1 : 0;
  pad.buttons[14] = hat[0] < 0 ? 1 : 0;
  pad.buttons[15] = hat[0] > 0 ? 1 : 0;
}

// Opens the device when there is one; a closed or failed stream (the controller off or out of range) leaves the pad empty until the next look.
function open(reader: Reader) {
  if (reader.stream) return;
  let device;
  try {
    device = findDevice();
  } catch {
    device = null;
  }
  if (!device) return;
  const stream = createReadStream(device.path, {
    highWaterMark: EVENT_SIZE * 64,
  });
  reader.stream = stream;
  reader.pad = { ...blank(), name: device.name };
  reader.triggers = { key: [0, 0], axis: [0, 0] };
  reader.hat = [0, 0];
  reader.dirty = true;
  let rest = Buffer.alloc(0);
  stream.on('data', (chunk: Buffer | string) => {
    let data = Buffer.concat([rest, Buffer.from(chunk)]);
    while (data.length >= EVENT_SIZE) {
      apply(
        reader,
        data.readUInt16LE(16),
        data.readUInt16LE(18),
        data.readInt32LE(20)
      );
      data = data.subarray(EVENT_SIZE);
    }
    rest = data;
    reader.dirty = true;
  });
  const close = () => {
    if (reader.stream !== stream) return;
    reader.stream = undefined;
    reader.pad = blank();
    reader.dirty = true;
    stream.destroy();
  };
  stream.on('error', close);
  stream.on('close', close);
}

function reader() {
  if (globalReader.gamepadReader) return globalReader.gamepadReader;
  const created: Reader = {
    pad: blank(),
    triggers: { key: [0, 0], axis: [0, 0] },
    hat: [0, 0],
    clients: new Set(),
    dirty: true,
  };
  globalReader.gamepadReader = created;
  const encoder = new TextEncoder();
  let ticks = 0;
  // Changes go out every 20 ms, and the state once a second regardless so a client can tell a live stream from a stalled one.
  setInterval(() => {
    ticks++;
    if (ticks % 50 === 0) open(created);
    if (!created.clients.size || (!created.dirty && ticks % 50)) return;
    created.dirty = false;
    const message = encoder.encode(`data: ${JSON.stringify(created.pad)}\n\n`);
    for (const client of created.clients) {
      try {
        client.enqueue(message);
      } catch {
        created.clients.delete(client);
      }
    }
  }, 20);
  open(created);
  return created;
}

export async function GET(request: Request) {
  const shared = reader();
  open(shared);
  let client: ReadableStreamDefaultController<Uint8Array>;
  const leave = () => {
    shared.clients.delete(client);
  };
  return new Response(
    new ReadableStream<Uint8Array>({
      start(controller) {
        client = controller;
        shared.clients.add(controller);
        shared.dirty = true;
        request.signal.addEventListener('abort', leave);
      },
      cancel: leave,
    }),
    {
      headers: {
        'Content-Type': 'text/event-stream',
        'Cache-Control': 'no-cache, no-transform',
      },
    }
  );
}
