// A drone's motion over time, from whatever made it (a recording, a simulation, a rollout), as the replay viewer (components/replay) plays it. Frame i is at i / rate seconds; every array holds its values frame after frame.

// Something that happened, for the timeline: signal ones ended or endangered the flight (a stop, a recovery, a tip cut), quiet ones only mark the way (a mode change, liftoff).
export type TrajectoryEvent = {
  time: number;
  label: string;
  signal?: boolean;
};

// A named series for the strips (one strip per group, its unit on the strip); null samples are gaps; dashed for targets.
export type TrajectoryChannel = {
  name: string;
  group: string;
  unit: string;
  values: (number | null)[];
  dashed?: boolean;
};

export type Trajectory = {
  rate: number;
  // Position in meters, north, west and up of where it started (the firmware's level frame), 3 per frame; null when the producer has none, and the drone replays in place.
  position: number[] | null;
  // Attitude, body (forward, left, up) to that frame, as a unit quaternion w, x, y, z, 4 per frame.
  attitude: number[];
  // Each rotor's thrust as a share of the most it gives (0-1) and its forward tilt (degrees), left then right, 2 per frame each.
  thrust: number[];
  tilt: number[];
  channels: TrajectoryChannel[];
  events: TrajectoryEvent[];
};

export type Pose = {
  position: [number, number, number];
  attitude: [number, number, number, number];
  thrust: [number, number];
  tilt: [number, number];
};

export function frameCount(trajectory: Trajectory) {
  return trajectory.attitude.length / 4;
}

export function duration(trajectory: Trajectory) {
  return Math.max(0, frameCount(trajectory) - 1) / trajectory.rate;
}

// The quaternion of roll, then pitch, then yaw (radians) as the firmware has them: positive roll is right side down (about forward), positive pitch nose down (about left), positive yaw a turn left (about up).
export function quaternionOf(roll: number, pitch: number, yaw: number) {
  const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
  const [cp, sp] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
  const [cy, sy] = [Math.cos(yaw / 2), Math.sin(yaw / 2)];
  return [
    cy * cp * cr + sy * sp * sr,
    cy * cp * sr - sy * sp * cr,
    cy * sp * cr + sy * cp * sr,
    sy * cp * cr - cy * sp * sr,
  ];
}

// The pose at a time (s): linear between frames, the attitude by normalized interpolation along the shorter way round.
export function poseAt(trajectory: Trajectory, time: number): Pose {
  const last = frameCount(trajectory) - 1;
  const at = Math.min(Math.max(time * trajectory.rate, 0), Math.max(last, 0));
  const index = Math.min(Math.floor(at), Math.max(last - 1, 0));
  const share = last > 0 ? at - index : 0;
  const mix = (values: number[], width: number, slot: number) =>
    values[index * width + slot] * (1 - share) +
    values[Math.min(index + 1, last) * width + slot] * share;
  const q = trajectory.attitude;
  const flip =
    q[index * 4] * q[(index + 1) * 4] +
      q[index * 4 + 1] * q[(index + 1) * 4 + 1] +
      q[index * 4 + 2] * q[(index + 1) * 4 + 2] +
      q[index * 4 + 3] * q[(index + 1) * 4 + 3] <
    0
      ? -1
      : 1;
  const attitude = [0, 1, 2, 3].map(
    (slot) =>
      q[index * 4 + slot] * (1 - share) +
      flip * q[Math.min(index + 1, last) * 4 + slot] * share
  );
  const norm = Math.hypot(...attitude) || 1;
  return {
    position: trajectory.position
      ? [
          mix(trajectory.position, 3, 0),
          mix(trajectory.position, 3, 1),
          mix(trajectory.position, 3, 2),
        ]
      : [0, 0, 0],
    attitude: attitude.map((value) => value / norm) as Pose['attitude'],
    thrust: [mix(trajectory.thrust, 2, 0), mix(trajectory.thrust, 2, 1)],
    tilt: [mix(trajectory.tilt, 2, 0), mix(trajectory.tilt, 2, 1)],
  };
}
