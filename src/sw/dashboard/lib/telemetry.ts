export type ImuReading = {
  roll: number;
  accel: [number, number, number];
  gyro: [number, number, number];
  temperature: number;
};

export type Side = 'left' | 'right';

export type ControlSettings = {
  base_throttle: number;
  enabled: boolean;
  target: number;
  kp: number;
  ki: number;
  kd: number;
};

export type Reading = {
  control?: ControlSettings;
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
