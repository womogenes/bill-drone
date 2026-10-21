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

// The drone service's recorder (src/fw/controller/bridge.py).
export type RecorderStatus = {
  file: string | null;
  error: string | null;
  bytes: number;
  writing: boolean;
};

// Streamed to the drone every 50 ms; each packet is the controller heartbeat. Servos and throttles apply while disarmed.
export type Targets = {
  base_throttle: number;
  roll: number;
  pitch: number;
  yaw_rate: number;
  servo: [number, number];
  throttle: [number, number];
};

export type Reading = {
  logging?: {
    active: boolean;
    ready: boolean;
    samples: number;
    dropped: number;
  };
  control?: ControlSettings;
  control_status?: {
    reason: string;
    throttle_latched?: boolean;
  };
  recorder?: RecorderStatus;
  imu?: ImuReading | null;
  adc_raw?: number;
  adc_voltage?: number;
  // Computed in the dashboard from adc_voltage.
  voltage_ema?: number;
  actuators?: Record<Side, { servo_angle: number; throttle: number }>;
  time: number;
  force: number | null;
  force_age_ms?: number | null;
  force_sequence?: number;
};

export type ImuSample = ImuReading & { time: number; session: number };

export type VoltageSample = {
  time: number;
  session: number;
  voltage: number;
  ema: number;
};
