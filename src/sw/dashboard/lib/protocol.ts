import type { ControlSettings, Reading, Side } from './telemetry';

export const CONTROL_KEYS = [
  'enabled',
  'roll',
  'pitch',
  'yaw_rate',
  'base_throttle',
  'roll_p',
  'roll_i',
  'roll_damping',
  'pitch_p',
  'pitch_i',
  'pitch_damping',
  'yaw_damping',
  'integral_limit',
  'pitch_trim',
  'left_trim',
  'right_trim',
  'pivot_height',
  'com_x',
  'roll_inertia',
  'pitch_inertia',
  'yaw_inertia',
  'reaction_torque_ratio',
  'arm',
  'roll_bias',
  'pitch_bias',
  'yaw_bias',
  'left_thrust_scale',
  'right_thrust_scale',
] as const;

type Command =
  | { command: 'heartbeat' | 'stop' | 'calibrate_imu' }
  | { command: 'servo' | 'throttle'; side: Side; value: number }
  | { command: 'zero_servo'; side: Side }
  | { command: 'control'; settings: Partial<ControlSettings> };

export function encodeCommand(command: Command): ArrayBuffer {
  if (command.command === 'zero_servo') {
    return new Uint8Array([90, command.side === 'left' ? 0 : 1]).buffer; // Z
  }

  if (command.command === 'control') {
    const fields = Object.entries(command.settings) as [
      keyof ControlSettings,
      number | boolean,
    ][];
    const packet = new DataView(new ArrayBuffer(1 + fields.length * 5));
    packet.setUint8(0, 80); // P
    fields.forEach(([key, value], index) => {
      packet.setUint8(1 + index * 5, CONTROL_KEYS.indexOf(key));
      packet.setFloat32(2 + index * 5, Number(value), true);
    });
    return packet.buffer;
  }

  if (command.command === 'servo' || command.command === 'throttle') {
    const packet = new DataView(new ArrayBuffer(6));
    packet.setUint8(0, command.command === 'servo' ? 83 : 84); // S / T
    packet.setUint8(1, command.side === 'left' ? 0 : 1);
    packet.setFloat32(2, command.value, true);
    return packet.buffer;
  }

  const code = { heartbeat: 72, stop: 88, calibrate_imu: 67 }[command.command];
  return new Uint8Array([code]).buffer;
}

export function decodeReading(buffer: ArrayBuffer, reason: string): Reading {
  const packet = new DataView(buffer);
  // 152-byte packets predate the model parameters; 180-byte packets predate biases and thrust scales.
  if (
    ![152, 180, 200].includes(buffer.byteLength) ||
    packet.getUint32(0, true) !== 0x314c4554
  ) {
    throw new Error('Invalid TEL1 packet');
  }
  const flags = packet.getUint32(8, true);
  const value = (index: number) => packet.getFloat32(36 + index * 4, true);
  const control = Object.fromEntries(
    CONTROL_KEYS.slice(1, (buffer.byteLength - 36) / 4 - 13).map(
      (key, index) => [key, value(14 + index)]
    )
  ) as Omit<ControlSettings, 'enabled'>;

  return {
    time: Date.now(),
    force: flags & 64 ? packet.getInt32(12, true) : null,
    force_age_ms: flags & 64 ? packet.getUint32(16, true) : null,
    force_sequence: packet.getUint32(20, true),
    adc_raw: packet.getUint32(24, true),
    adc_voltage: value(0),
    actuators: {
      left: { servo_angle: value(1), throttle: value(2) },
      right: { servo_angle: value(3), throttle: value(4) },
    },
    imu:
      flags & 1
        ? {
            calibrated: !!(flags & 256),
            calibrating: !!(flags & 2),
            accel: [value(5), value(6), value(7)],
            gyro: [value(8), value(9), value(10)],
            roll: value(11),
            pitch: value(12),
            temperature: value(13),
          }
        : null,
    control: { ...control, enabled: !!(flags & 32) },
    control_status: { reason, mapping_confirmed: !!(flags & 128) },
    logging: {
      connected: !!(flags & 4),
      active: !!(flags & 8),
      ready: !!(flags & 16),
      samples: packet.getUint32(28, true),
      dropped: packet.getUint32(32, true),
    },
  };
}
