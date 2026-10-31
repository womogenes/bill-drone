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
  // 1 flies camera hover on the tag; 0 is free flight, the camera streaming FPV instead (components/fpv-panel.tsx).
  hover_camera: number;
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
  servo_compensation: number;
  integrate_throttle: number;
  target_rate: number;
  velocity_damping: number;
  velocity_leak: number;
  thrust_voltage: number;
  yaw_i: number;
  hover_center: number;
  hover_distance: number;
  hover_approach: number;
  hover_ceiling: number;
  hover_square: number;
  hover_kside: number;
  hover_cam_pitch: number;
  hover_cam_roll: number;
  hover_drag: number;
  hover_drag_tilt: number;
  hover_drag_tau: number;
  hover_trim_tau: number;
  hover_trim_max: number;
  auto_trim: number;
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
  // FLIGHT_MODES index. In hover and landing the drone flies itself from the camera and ignores the setpoints above.
  mode: number;
  // Moves the hover hold point (m): [0] toward the tag, [1] to the drone's left, [2] up. Hover and landing only.
  nudge: [number, number, number];
};

export const FLIGHT_MODES = ['Manual', 'Hover', 'Landing', 'Landing fast'];

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

// The drone service's hover push (src/fw/controller/bridge.py push_hover), in the hover loop's yaw-turned level frame (m; forward, left, up).
export type HoverState = {
  elapsed_us: number;
  mode: number;
  tracking: boolean;
  // The tag relative to the drone; null without a sighting.
  tag: [number, number, number] | null;
  tag_age_ms: number | null;
  velocity: [number, number];
  height: number;
  height_target: number;
  // The hold as nudged: range_target from the tag, on the line whose heading from drone to tag is arc_deg.
  range_target: number | null;
  arc_deg: number | null;
};

export type HoverSample = HoverState & { time: number; session: number };

export type ImuSample = ImuReading & {
  time: number;
  session: number;
  // The drone's roll and pitch targets (degrees) while armed.
  target?: [number, number];
};

export type VoltageSample = {
  time: number;
  session: number;
  voltage: number;
  ema: number;
};
