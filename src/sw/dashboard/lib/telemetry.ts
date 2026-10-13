export const THROTTLE_MAX = 1200;

export type ImuReading = {
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
  attitude_kp: number;
  roll_kp: number;
  roll_ki: number;
  roll_kd: number;
  pitch_kp: number;
  pitch_ki: number;
  pitch_kd: number;
  yaw_kp: number;
  yaw_ki: number;
  yaw_kd: number;
};

export type Reading = {
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
