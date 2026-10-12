export type ImuReading = {
  accel: [number, number, number];
  gyro: [number, number, number];
  temperature: number;
};

export type Side = 'left' | 'right';

export type Reading = {
  imu?: ImuReading | null;
  adc_raw?: number;
  actuators?: Record<Side, { servo_angle: number; throttle: number }>;
  time: number;
  force: number | null;
  force_age_ms?: number | null;
  force_sequence?: number;
};

export type LoadCellSample = { time: number; force: number; session: number };
