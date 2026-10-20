export const THROTTLE_MAX = 1200;

export type ImuReading = {
  calibrated: boolean;
  calibrating: boolean;
  roll: number;
  pitch: number;
  accel: [number, number, number];
  gyro: [number, number, number];
  temperature: number;
};

export type Side = 'left' | 'right';

export type ControlSettings = {
  enabled: boolean;
  roll: number;
  pitch: number;
  yaw_rate: number;
  base_throttle: number;
  roll_p: number;
  roll_i: number;
  roll_damping: number;
  pitch_p: number;
  pitch_i: number;
  pitch_damping: number;
  yaw_damping: number;
  integral_limit: number;
  pitch_trim: number;
  left_trim: number;
  right_trim: number;
  pivot_height: number;
  com_x: number;
  roll_inertia: number;
  pitch_inertia: number;
  yaw_inertia: number;
  reaction_torque_ratio: number;
  arm: number;
  roll_bias: number;
  pitch_bias: number;
  yaw_bias: number;
  left_thrust_scale: number;
  right_thrust_scale: number;
};

export type RecorderStatus = {
  ready: boolean;
  address?: string;
  file?: string;
  error?: string;
  bytes?: number;
  writing?: boolean;
};

export type Reading = {
  logging?: {
    connected: boolean;
    active: boolean;
    ready: boolean;
    samples: number;
    dropped: number;
  };
  control?: ControlSettings;
  control_status?: { reason: string; mapping_confirmed: boolean };
  imu?: ImuReading | null;
  adc_raw?: number;
  adc_voltage?: number;
  actuators?: Record<Side, { servo_angle: number; throttle: number }>;
  time: number;
  force: number | null;
  force_age_ms?: number | null;
  force_sequence?: number;
};

export type RollSample = { time: number; roll: number; session: number };

export type ImuSample = ImuReading & { time: number; session: number };
