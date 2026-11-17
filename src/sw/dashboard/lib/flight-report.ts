// A recorded flight (src/sw/recordings/<UTC start>_<session>.bin and .jsonl, written by src/fw/controller/bridge.py) as the flights page replays it, from its events and its records' columns as the bridge's GET /recordings decodes them (src/fw/controller/log_format.py decode_records).

import { quaternionOf, type Trajectory } from '@/lib/trajectory';

// The columns the report reads (log_format.py COLUMNS): time, the IMU and attitude, the motors, servos and targets, the hover state, the flip's phase and the recovery's stage, and the Kalman filter's heading and position.
export const RECORD_COLUMNS = [
  'esp_time_us',
  'ax_g',
  'ay_g',
  'az_g',
  'temperature_c',
  'gx_deg_s',
  'gy_deg_s',
  'gz_deg_s',
  'roll_deg',
  'pitch_deg',
  'left_throttle',
  'right_throttle',
  'left_servo_deg',
  'right_servo_deg',
  'target_roll_deg',
  'target_pitch_deg',
  'base_throttle',
  'hover_height_m',
  'hover_height_target_m',
  'hover_yaw_deg',
  'flip',
  'recovery',
  'ekf_yaw_deg',
  'ekf_north_m',
  'ekf_west_m',
  'ekf_height_m',
] as const;
type RecordColumn = (typeof RECORD_COLUMNS)[number];

// A recording as the bridge serves it: its events, its records' columns (each record once, in sequence order; null where a record has no value), whether the .bin ends mid-batch, and the flip's phases by number (log_format.py FLIP_PHASES).
type Recording = {
  events: Event[];
  columns: Record<RecordColumn, (number | null)[]>;
  truncated: boolean;
  flip_phases: (string | null)[];
};

// The drone's esp_timer microseconds in the start record wrap at 2^32.
const WRAP_MS = 2 ** 32 / 1000;
// The replay's frame rate (Hz): half the control rate, enough for a flip's 300 deg/s at 6 degrees a frame, which the viewer interpolates.
const REPLAY_HZ = 50;
// The firmware's highest throttle (fw/drone/main/flight_rt.c THROTTLE_MAX), the rotors' full thrust in the replay.
const THROTTLE_MAX = 1200;
const MODES = ['manual', 'hold', 'land', 'land fast'];
// Liftoff: vertical specific force past 1.12 g once the collective reaches 600, plus 3 ticks.
const LIFT_BASE = 600;
const LIFT_G = 1.12;
// A fly-away: GPS ground speed past FLYAWAY_MS on FLYAWAY_FIXES fixes in a row without a tilt command in the TILT_COMMAND_S before. While the pilot flies the hold point (a forward or left nudge change in the command events) and while the hold brakes after it (hover_nudge_speed / hover_nudge_accel, at least TILT_COMMAND_S), the limit is FLYAWAY_MS past hover_nudge_speed, the fastest the nudge asks for.
const FLYAWAY_MS = 2;
const FLYAWAY_FIXES = 5;
const TILT_COMMAND_S = 2;
// The bridge drops its dashboard after 0.5 s without a status message (bridge.py monitor); GPS events ride the status, so a longer gap between them (with a fix) is a link timeout.
const LINK_TIMEOUT_MS = 500;
// A tip-over at landing: pitch or roll past TIP_DEG within TIP_WINDOW_S after the motors stop.
const TIP_DEG = 30;
const TIP_WINDOW_S = 3;

type Event = Record<string, unknown> & { type: string; received_at?: string };

type GpsEvent = {
  drone_ms: number;
  age_ms: number;
  fix: number;
  ok: boolean;
  satellites: number;
  latitude: number;
  longitude: number;
  height: number;
  velocity: [number, number, number];
  horizontal_accuracy: number;
  received_at: string;
};

type CommandEvent = {
  received_at: string;
  stop?: boolean;
  mode?: number;
  roll?: number;
  pitch?: number;
  yaw_rate?: number;
  nudge?: number[];
};

// One recording in the list (bridge.py recording_list): its file stem, start (from the name) and length (s, from its size).
export type RecordingEntry = {
  name: string;
  started: string;
  seconds: number;
};

type FlightWarning = { time: number | null; text: string };

export type FlightReport = {
  name: string;
  // The stop record's reason; null when the recording has none (cut short or still open).
  stop_reason: string | null;
  max_height: number | null;
  // The flight for the replay viewer (lib/trajectory.ts).
  trajectory: Trajectory;
};

function round(value: number, digits: number) {
  if (!Number.isFinite(value)) return null;
  const scale = 10 ** digits;
  return Math.round(value * scale) / scale;
}

export function analyzeFlight(name: string, recording: Recording): FlightReport {
  const { events, truncated } = recording;
  const start = events.find((event) => event.type === 'start');
  if (!start) throw new Error('The recording has no start record');
  // The start record wins over later metadata, as in flight_log.py read_flight.
  const metadata: Record<string, unknown> = {};
  for (const event of events)
    if (event.type === 'start' || event.type === 'metadata')
      for (const [key, value] of Object.entries(event))
        if (!(key in metadata)) metadata[key] = value;
  const settings = (metadata.settings ?? {}) as Record<string, number>;
  const stop = events.find((event) => event.type === 'stop') as
    (Event & { reason: string; time_us: number; dropped: number }) | undefined;

  const column = (key: RecordColumn) =>
    Float64Array.from(recording.columns[key], (value) => value ?? NaN);
  const time = column('esp_time_us').map((value) => value / 1e6);
  const count = time.length;
  const az = column('az_g');
  const roll = column('roll_deg');
  const pitch = column('pitch_deg');
  const rollTarget = column('target_roll_deg');
  const pitchTarget = column('target_pitch_deg');
  const base = column('base_throttle');
  // NaN in records from before the hover state or the Kalman filter.
  const height = column('hover_height_m');
  const heightTarget = column('hover_height_target_m');
  const yaw = column('hover_yaw_deg');
  // For the replay: the body rates (deg/s), each motor's throttle and each servo's forward tilt from its trim (degrees; the left servo tilts forward with positive angles, the right with negative), the flip's phase and the recovery's stage (0 none), and the Kalman filter's heading (degrees, left from north; the hover's gyro heading before the filter) and position (m north, west and up).
  const rates = [column('gx_deg_s'), column('gy_deg_s'), column('gz_deg_s')];
  const motors = [column('left_throttle'), column('right_throttle')];
  const servos = [column('left_servo_deg'), column('right_servo_deg')];
  const throttle = motors[0].map((value, index) => value + motors[1][index]);
  const tilts = [
    servos[0].map((value) => value - (settings.left_trim ?? 0)),
    servos[1].map((value) => -(value - (settings.right_trim ?? 0))),
  ];
  const flip = column('flip').map((value) => value || 0);
  const recovery = column('recovery').map((value) => value || 0);
  const ekfYaw = column('ekf_yaw_deg');
  const heading = ekfYaw.map((value, index) =>
    Number.isFinite(value) ? value : yaw[index]
  );
  const place = [
    column('ekf_north_m'),
    column('ekf_west_m'),
    column('ekf_height_m'),
  ];
  // Runs of records whose seven IMU values repeat the one before: a read that failed and left the last sample in place.
  const imu = (
    [
      'ax_g',
      'ay_g',
      'az_g',
      'temperature_c',
      'gx_deg_s',
      'gy_deg_s',
      'gz_deg_s',
    ] as const
  ).map(column);
  let repeats = 0;
  let longestRepeat = 0;
  let repeatAt = NaN;
  let run = 0;
  for (let index = 1; index < count; index++) {
    const same = imu.every((values) => values[index] === values[index - 1]);
    run = same ? run + 1 : 0;
    if (run === 1) {
      repeats++;
      if (Number.isNaN(repeatAt)) repeatAt = time[index];
    }
    longestRepeat = Math.max(longestRepeat, run);
  }

  // Liftoff, then the motors stopping (the collective at zero) after it.
  let lift = -1;
  const powered = base.findIndex((value) => value >= LIFT_BASE);
  if (powered >= 0) {
    const rising = az.findIndex(
      (value, index) => index >= powered && value > LIFT_G
    );
    if (rising >= 0) lift = Math.min(count - 1, rising + 3);
  }
  let stopped = count;
  if (lift >= 0) {
    const off = base.findIndex(
      (value, index) => index > lift && value === 0 && throttle[index] === 0
    );
    if (off >= 0) stopped = off;
  }
  const airborne: [number, number] | null =
    lift >= 0 ? [time[lift], time[Math.min(stopped, count - 1)]] : null;

  // Event times on the recording's clock: GPS events carry the drone's clock (drone_ms, on started_ticks_us's esp_timer); commands only the laptop's, so they are placed by the smallest laptop-minus-drone offset among the GPS events' statuses (the least delayed one).
  const startedMs = Number(metadata.started_ticks_us ?? 0) / 1000;
  const startedAt = Date.parse(String(start.received_at));
  const sinceStart = (droneMs: number) =>
    ((((droneMs - startedMs) % WRAP_MS) + WRAP_MS) % WRAP_MS) / 1000;
  const gpsEvents = events.filter(
    (event) => event.type === 'gps'
  ) as unknown as GpsEvent[];
  let offset = Infinity;
  for (const gps of gpsEvents)
    offset = Math.min(
      offset,
      Date.parse(gps.received_at) - sinceStart(gps.drone_ms + gps.age_ms) * 1000
    );
  if (!Number.isFinite(offset)) offset = startedAt;
  const laptopTime = (receivedAt: string) =>
    (Date.parse(receivedAt) - offset) / 1000;

  // GPS fixes (10 Hz), seconds from the recording's start, ground speed in m/s.
  const track = gpsEvents
    .filter((gps) => gps.ok && gps.fix >= 2 && gps.fix <= 4)
    .map((gps) => ({
      time: sinceStart(gps.drone_ms),
      speed: Math.hypot(gps.velocity[0], gps.velocity[1]),
    }));

  // The pilot's commands as they changed, on the recording's clock.
  const commands = (
    events.filter(
      (event) => event.type === 'command'
    ) as unknown as CommandEvent[]
  ).map((command) => ({ ...command, time: laptopTime(command.received_at) }));
  const tilted = commands
    .filter((command) => command.roll || command.pitch)
    .map((command) => command.time);
  const nudged: number[] = [];
  let lastNudge: number[] = [];
  for (const command of commands) {
    if (!command.nudge) continue;
    if (
      (command.nudge[0] ?? 0) !== (lastNudge[0] ?? 0) ||
      (command.nudge[1] ?? 0) !== (lastNudge[1] ?? 0)
    )
      nudged.push(command.time);
    lastNudge = command.nudge;
  }
  // The nudge's settings as flown: the start's and every change during the recording, the fastest speed and the longest brake.
  const flownSettings = events
    .filter((event) => event.settings)
    .map((event) => event.settings as Record<string, number>);
  const nudgeSpeed = Math.max(
    settings.hover_nudge_speed ?? 0.5,
    ...flownSettings.map((value) => value.hover_nudge_speed ?? 0)
  );
  const nudgeBrake = Math.max(
    TILT_COMMAND_S,
    nudgeSpeed /
      Math.min(
        ...flownSettings
          .map((value) => value.hover_nudge_accel ?? 0)
          .filter((value) => value > 0)
      )
  );
  const before = (times: number[], at: number, within: number) =>
    times.some((value) => value <= at && value > at - within);

  const warnings: FlightWarning[] = [];
  // Fly-aways: fast fixes in a row with no tilt asked for, while airborne (a drone carried or tumbling after the motors stop is not one).
  const flown = airborne
    ? track.filter(
        (point) => point.time >= airborne[0] && point.time <= airborne[1]
      )
    : [];
  const uncommanded = (point: (typeof track)[number]) =>
    !before(tilted, point.time, TILT_COMMAND_S) &&
    point.speed >
      FLYAWAY_MS + (before(nudged, point.time, nudgeBrake) ? nudgeSpeed : 0);
  for (let index = 0; index < flown.length;) {
    if (!uncommanded(flown[index])) {
      index++;
      continue;
    }
    let end = index;
    let top = 0;
    while (end < flown.length && uncommanded(flown[end]))
      top = Math.max(top, flown[end++].speed);
    if (end - index >= FLYAWAY_FIXES)
      warnings.push({
        time: flown[index].time,
        text: `Fly-away: ${top.toFixed(1)} m/s uncommanded for ${(flown[end - 1].time - flown[index].time).toFixed(1)} s`,
      });
    index = end;
  }
  // Link timeouts: the statuses (and their GPS events) stopped reaching the laptop.
  for (let index = 1; index < gpsEvents.length; index++) {
    const gap =
      Date.parse(gpsEvents[index].received_at) -
      Date.parse(gpsEvents[index - 1].received_at);
    if (gap > LINK_TIMEOUT_MS)
      warnings.push({
        time: sinceStart(gpsEvents[index - 1].drone_ms),
        text: `Link timeout: no drone status for ${(gap / 1000).toFixed(1)} s`,
      });
  }
  const gaps = events.filter((event) => event.type === 'gap');
  if (gaps.length)
    warnings.push({
      time: null,
      text: `${gaps.reduce((sum, event) => sum + Number(event.missing), 0)} flight records lost over the radio in ${gaps.length} gaps`,
    });
  for (const event of events) {
    if (event.type === 'error')
      warnings.push({
        time: laptopTime(String(event.received_at)),
        text: `Recorder: ${event.error}`,
      });
    if (event.type === 'closed')
      warnings.push({
        time: laptopTime(String(event.received_at)),
        text: `Recording closed: ${event.reason}`,
      });
  }
  for (const command of commands)
    if (command.stop && airborne && command.time < airborne[1] + 1)
      warnings.push({ time: command.time, text: 'Stop sent from the laptop' });
  if (stop && stop.reason !== 'Landed' && stop.reason !== 'Disarmed')
    warnings.push({
      time: airborne ? airborne[1] : stop.time_us / 1e6,
      text: `Stopped: ${stop.reason}${lift >= 0 && Number.isFinite(height[stopped - 1]) ? ` at ${height[stopped - 1].toFixed(2)} m` : ''}`,
    });
  if (repeats)
    warnings.push({
      time: repeatAt,
      text: `IMU read failures: ${repeats} repeated samples, the longest ${longestRepeat} ticks`,
    });
  if (stop?.dropped)
    warnings.push({
      time: null,
      text: `${stop.dropped} records dropped on the drone`,
    });
  if (truncated || !stop)
    warnings.push({
      time: null,
      text: stop
        ? 'The recording ends mid-record'
        : 'The recording has no stop record',
    });
  // Tip-over at landing: the attitude in the seconds after the motors stopped.
  if (lift >= 0 && stopped < count) {
    let worst = 0;
    let worstAt = NaN;
    for (
      let index = stopped;
      index < count && time[index] <= time[stopped] + TIP_WINDOW_S;
      index++
    ) {
      const tilt = Math.max(Math.abs(pitch[index]), Math.abs(roll[index]));
      if (tilt > worst) {
        worst = tilt;
        worstAt = time[index];
      }
    }
    if (worst > TIP_DEG)
      warnings.push({
        time: worstAt,
        text: `Tipped over at landing: ${worst.toFixed(0)}° within ${TIP_WINDOW_S} s of the motors stopping`,
      });
  }
  warnings.sort((a, b) => (a.time ?? Infinity) - (b.time ?? Infinity));

  let maxHeight = -Infinity;
  for (let index = Math.max(0, lift); index < stopped; index++)
    if (lift >= 0 && Number.isFinite(height[index]))
      maxHeight = Math.max(maxHeight, height[index]);

  const trajectory = replayOf(
    { time, roll, pitch, rollTarget, pitchTarget, height, heightTarget, heading, flip, recovery, rates, motors, tilts, place },
    warnings,
    commands,
    recording.flip_phases
  );

  return {
    name,
    stop_reason: stop?.reason ?? null,
    max_height: Number.isFinite(maxHeight) ? maxHeight : null,
    trajectory,
  };
}

type Columns = Record<'time' | 'roll' | 'pitch' | 'rollTarget' | 'pitchTarget' | 'height' | 'heightTarget' | 'heading' | 'flip' | 'recovery', Float64Array> &
  Record<'rates' | 'motors' | 'tilts' | 'place', Float64Array[]>;

// The decoded columns as a replay (lib/trajectory.ts) at REPLAY_HZ from the first record, each frame the latest record at its time, so dropped records hold the last pose instead of stretching the clock. Its events: the report's warnings (signal), the pilot's mode changes, and the flip's and the recovery's phases as the guards' word logs them.
function replayOf(c: Columns, warnings: FlightWarning[], commands: { time: number; mode?: number }[], flipPhases: (string | null)[]): Trajectory {
  const count = c.time.length;
  const start = count ? c.time[0] : 0;
  const frames = count ? Math.floor((c.time[count - 1] - start) * REPLAY_HZ) + 1 : 0;
  const picks: number[] = [];
  for (let frame = 0, index = 0; frame < frames; frame++) {
    const at = start + frame / REPLAY_HZ;
    while (index + 1 < count && c.time[index + 1] <= at) index++;
    picks.push(index);
  }
  const fixed = (value: number, digits: number) => (Number.isFinite(value) ? round(value, digits) : null);
  const anyPlace = picks.some((index) => Number.isFinite(c.place[0][index]));
  const position: number[] = [];
  const attitude: number[] = [];
  const thrust: number[] = [];
  const tilt: number[] = [];
  let last = [0, 0, 0];
  for (const index of picks) {
    const yaw = Number.isFinite(c.heading[index]) ? c.heading[index] : 0;
    const q = quaternionOf((c.roll[index] * Math.PI) / 180, (c.pitch[index] * Math.PI) / 180, (yaw * Math.PI) / 180);
    attitude.push(...q.map((value) => round(value, 4)!));
    if (anyPlace) {
      if (Number.isFinite(c.place[0][index])) last = c.place.map((values) => values[index]);
      position.push(...last.map((value) => round(value, 3)!));
    }
    thrust.push(...c.motors.map((values) => round(values[index] / THROTTLE_MAX, 3)!));
    tilt.push(...c.tilts.map((values) => round(values[index], 2)!));
  }
  const series = (name: string, group: string, unit: string, value: (index: number) => number, digits: number, dashed = false) => ({
    name,
    group,
    unit,
    dashed,
    values: picks.map((index) => fixed(value(index), digits)),
  });
  const channels = [
    series('height', 'height', 'm', (index) => (anyPlace ? c.place[2][index] : c.height[index]), 3),
    series('target', 'height', 'm', (index) => c.heightTarget[index], 3, true),
    series('roll', 'attitude', '°', (index) => c.roll[index], 2),
    series('pitch', 'attitude', '°', (index) => c.pitch[index], 2),
    series('roll target', 'attitude', '°', (index) => c.rollTarget[index], 2, true),
    series('pitch target', 'attitude', '°', (index) => c.pitchTarget[index], 2, true),
    series('roll', 'rates', '°/s', (index) => c.rates[0][index], 1),
    series('pitch', 'rates', '°/s', (index) => c.rates[1][index], 1),
    series('yaw', 'rates', '°/s', (index) => c.rates[2][index], 1),
    series('left', 'throttle', '', (index) => c.motors[0][index], 0),
    series('right', 'throttle', '', (index) => c.motors[1][index], 0),
    series('left', 'tilt', '°', (index) => c.tilts[0][index], 1),
    series('right', 'tilt', '°', (index) => c.tilts[1][index], 1),
  ];
  const events: Trajectory['events'] = [];
  for (const warning of warnings)
    if (warning.time !== null) events.push({ time: warning.time - start, label: warning.text, signal: true });
  let mode: number | undefined;
  for (const command of commands)
    if (command.mode !== undefined && command.mode !== null && command.mode !== mode) {
      if (mode !== undefined) events.push({ time: command.time - start, label: MODES[command.mode] ?? `mode ${command.mode}` });
      mode = command.mode;
    }
  let phase = 0;
  let stage = 0;
  for (let index = 0; index < count; index++) {
    const nextPhase = c.flip[index];
    const nextStage = c.recovery[index];
    if (nextPhase !== phase && nextPhase) events.push({ time: c.time[index] - start, label: `flip ${flipPhases[nextPhase]}` });
    if (nextStage !== stage && nextStage) events.push({ time: c.time[index] - start, label: 'recovery', signal: true });
    phase = nextPhase;
    stage = nextStage;
  }
  events.sort((a, b) => a.time - b.time);
  return { rate: REPLAY_HZ, position: anyPlace ? position : null, attitude, thrust, tilt, channels, events };
}
