// A recorded flight (src/sw/recordings/<UTC start>_<session>.bin and .jsonl, written by src/fw/controller/bridge.py) decoded and summarized for the post-flight report. The decoding follows src/fw/controller/log_format.py and src/sw/flight_log.py read_flight.

import { distance } from '@/lib/gps';
import { rotorTilt } from '@/lib/telemetry';

// Each FLG1 batch: magic, session, record count, record size (log_format.py HEADER_FORMAT "<4sIHH").
export const HEADER_SIZE = 12;
// The record sizes log_format.py FORMATS knows: the 48-byte base record, plus the hover state (80), plus the barometer (84), plus the guards' word (86: fly-away failsafe phase and pattern stop, fw/drone/main/guard.h), plus the Kalman filter's estimate (108, fw/drone/main/ekf.h; the report does not read it yet).
const BASE_SIZE = 48;
const HOVER_SIZE = 80;
const BARO_SIZE = 84;
const GUARD_SIZE = 86;
const RECORD_SIZE = 108;
const MAX_BATCH = 8;
const BARO_NONE = -(2 ** 31);
// The drone's esp_timer microseconds in the start record wrap at 2^32.
const WRAP_MS = 2 ** 32 / 1000;
// The control rate; the charts take every CHART_STRIDE-th record (25 Hz), which is plenty for a screen and keeps a 3-minute flight's JSON small.
const RATE_HZ = 100;
const CHART_STRIDE = 4;
// Liftoff as sw/sim/identify_airframe.py load finds it: vertical specific force past 1.12 g once the collective reaches 600, plus 3 ticks.
const LIFT_BASE = 600;
const LIFT_G = 1.12;
// The shake as sw/sim/pitch_check.py measures it: the pitch rate's RMS in 2.2-3.5 Hz through the same zero-phase band-pass, over the airborne window (its first and last half second left out) and as a moving RMS over SHAKE_WINDOW_S.
const SHAKE_BAND: [number, number] = [2.2, 3.5];
const SHAKE_WINDOW_S = 1;
// Hands-off: hover hold, no tilt setpoint and no command change (a held stick sends one every 50 ms) for HANDS_ON_S, from SPAN_SETTLE_S after liftoff; spans shorter than SPAN_MIN_S are left out.
const HANDS_ON_S = 1;
const SPAN_SETTLE_S = 2;
const SPAN_MIN_S = 3;
// A fly-away: GPS ground speed past FLYAWAY_MS on FLYAWAY_FIXES fixes in a row without a tilt command in the TILT_COMMAND_S before (the hold's own speed commands stop at hover_nudge_speed, 1 m/s).
const FLYAWAY_MS = 2;
const FLYAWAY_FIXES = 5;
const TILT_COMMAND_S = 2;
// The firmware's speed offset comes from this long airborne without the tag (fw/drone/main/hover.c SPEED_TAKEOFF_S).
const SPEED_TAKEOFF_S = 2;
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

// One recording in the list: its file stem, start (from the name) and length (estimateSeconds).
export type RecordingEntry = {
  name: string;
  started: string;
  seconds: number;
};

export type FlightWarning = { time: number | null; text: string };

// A hands-off stretch of hover (s from the recording's start) and how still it held: attitude spread (degrees), the fused height's spread and the barometer's range (m), how far the GPS position moved from the span's start (m) and the rough wind (m/s, from the drag model's airspeed).
export type HandsOffSpan = {
  start: number;
  end: number;
  pitch_sd: number;
  roll_sd: number;
  height_sd: number;
  baro_range: number | null;
  drift_m: number | null;
  wander_m: number | null;
  wind: number | null;
};

// Chart columns at 25 Hz: seconds from the recording's start; heights in m (fused estimate and target above the engage point, barometer above its takeoff reading); attitude and targets in degrees; the pitch rate's moving shake RMS in deg/s. Null where there is no value.
export type FlightSeries = {
  time: number[];
  height: (number | null)[];
  height_target: (number | null)[];
  baro: (number | null)[];
  roll: number[];
  pitch: number[];
  roll_target: number[];
  pitch_target: number[];
  shake: (number | null)[];
};

// GPS fixes in the recording (10 Hz), seconds from its start, ground speed in m/s.
export type FlightTrackPoint = {
  time: number;
  latitude: number;
  longitude: number;
  speed: number;
  horizontal_accuracy: number;
};

export type FlightReport = {
  name: string;
  started: string;
  drone: string | null;
  duration: number;
  // Liftoff and motors stopped (s); null when it never lifted.
  airborne: [number, number] | null;
  // The stop record's reason; null when the recording has none (cut short or still open).
  stop_reason: string | null;
  samples: number;
  missing: number;
  max_height: number | null;
  top_speed: number | null;
  shake_rms: number | null;
  wind: number | null;
  gps_hold: boolean;
  series: FlightSeries;
  track: FlightTrackPoint[];
  spans: HandsOffSpan[];
  warnings: FlightWarning[];
};

const NAME = /^(\d{4}-\d{2}-\d{2})T(\d{2})-(\d{2})-(\d{2})-(\d{3})Z_\d+$/;

// The UTC start in a recording's name (bridge.py Recorder.begin), as an ISO time, or null for another file.
export function recordingStart(name: string) {
  const match = NAME.exec(name);
  return match
    ? `${match[1]}T${match[2]}:${match[3]}:${match[4]}.${match[5]}Z`
    : null;
}

// A recording's length from its size and its first batch header (its records' size; batches run about 4 records).
export function estimateSeconds(bytes: number, header: Uint8Array) {
  const size =
    header.length >= HEADER_SIZE ? header[10] | (header[11] << 8) : 0;
  return bytes / (((size || RECORD_SIZE) + HEADER_SIZE / 4) * RATE_HZ);
}

function round(value: number, digits: number) {
  if (!Number.isFinite(value)) return null;
  const scale = 10 ** digits;
  return Math.round(value * scale) / scale;
}

function median(values: number[]) {
  if (!values.length) return NaN;
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)];
}

function spread(values: ArrayLike<number>, from: number, to: number) {
  let sum = 0;
  let squares = 0;
  let count = 0;
  for (let index = from; index < to; index++) {
    const value = values[index];
    if (!Number.isFinite(value)) continue;
    sum += value;
    squares += value * value;
    count++;
  }
  if (count < 2) return NaN;
  const mean = sum / count;
  return Math.sqrt(Math.max(0, squares / count - mean * mean));
}

// The second-order Butterworth band-pass scipy's butter(2, band, btype="band") designs, as two sections [gain, pole radius², 2 × pole real part], each (1 - z⁻²) over (1 - 2 Re z⁻¹ + |p|² z⁻²): the low-pass prototype's poles moved to the band and through the bilinear transform.
function bandPass([low, high]: [number, number]) {
  const warp = (frequency: number) =>
    2 * RATE_HZ * Math.tan((Math.PI * frequency) / RATE_HZ);
  const center = Math.sqrt(warp(low) * warp(high));
  const width = warp(high) - warp(low);
  // The prototype pole at exp(3πi/4) times the bandwidth gives s = (p ± sqrt(p² - 4w0²)) / 2; each with its conjugate (from the prototype's other pole) is one section.
  const pr = -Math.SQRT1_2 * width;
  const pi = Math.SQRT1_2 * width;
  const dr = pr * pr - pi * pi - 4 * center * center;
  const di = 2 * pr * pi;
  const modulus = Math.hypot(dr, di);
  const rootReal = Math.sqrt((modulus + dr) / 2);
  const rootImag = Math.sign(di) * Math.sqrt((modulus - dr) / 2);
  const sections = [1, -1].map((sign) => {
    const sReal = (pr + sign * rootReal) / 2;
    const sImag = (pi + sign * rootImag) / 2;
    // z = (2fs + s) / (2fs - s)
    const k = 2 * RATE_HZ;
    const denominator = (k - sReal) ** 2 + sImag ** 2;
    const zr = ((k + sReal) * (k - sReal) - sImag * sImag) / denominator;
    const zi = (2 * k * sImag) / denominator;
    return { re: zr, radius2: zr * zr + zi * zi };
  });
  // Unity gain at the band's center, as the Butterworth band-pass has.
  const omega = 2 * Math.atan(center / (2 * RATE_HZ));
  let gain = 1;
  for (const { re, radius2 } of sections) {
    const [c1, s1, c2, s2] = [
      Math.cos(omega),
      Math.sin(omega),
      Math.cos(2 * omega),
      Math.sin(2 * omega),
    ];
    const numerator = Math.hypot(1 - c2, s2);
    const denominator = Math.hypot(
      1 - 2 * re * c1 + radius2 * c2,
      2 * re * s1 - radius2 * s2
    );
    gain *= numerator / denominator;
  }
  return sections.map((section, index) => ({
    ...section,
    gain: index ? 1 : 1 / gain,
  }));
}

const SHAKE_SECTIONS = bandPass(SHAKE_BAND);
// sosfiltfilt's default pad: odd extension at each end.
const PAD = 30;

// The pitch rate's shake band, zero phase like sosfiltfilt: from the mean, padded, through both sections forward and then back.
function shakeBand(rate: Float64Array) {
  const count = rate.length;
  let mean = 0;
  for (const value of rate) mean += value / count;
  const pad = Math.min(PAD, count - 1);
  const values = new Float64Array(count + 2 * pad);
  for (let index = 0; index < count; index++)
    values[pad + index] = rate[index] - mean;
  for (let index = 1; index <= pad; index++) {
    values[pad - index] = 2 * values[pad] - values[pad + index];
    values[pad + count - 1 + index] =
      2 * values[pad + count - 1] - values[pad + count - 1 - index];
  }
  for (const reverse of [false, true])
    for (const { gain, re, radius2 } of SHAKE_SECTIONS) {
      let x1 = 0;
      let x2 = 0;
      let y1 = 0;
      let y2 = 0;
      for (let step = 0; step < values.length; step++) {
        const index = reverse ? values.length - 1 - step : step;
        const x = values[index];
        const y = gain * (x - x2) + 2 * re * y1 - radius2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        values[index] = y;
      }
    }
  return values.subarray(pad, pad + count);
}

function parseEvents(jsonl: string) {
  const events: Event[] = [];
  for (const line of jsonl.split('\n')) {
    if (!line.trim()) continue;
    try {
      events.push(JSON.parse(line));
    } catch {
      // A line cut off by a crash mid-write: the rest still reads.
    }
  }
  return events;
}

// The records of a .bin as columns, each record once (the drone resends unacknowledged batches), in sequence order.
function decodeRecords(bin: Uint8Array, session: number) {
  const view = new DataView(bin.buffer, bin.byteOffset, bin.byteLength);
  const offsets: number[] = [];
  const sizes: number[] = [];
  const sequences: number[] = [];
  const seen = new Set<number>();
  let truncated = false;
  let position = 0;
  while (position < bin.length) {
    if (position + HEADER_SIZE > bin.length) {
      truncated = true;
      break;
    }
    const magic = String.fromCharCode(...bin.subarray(position, position + 4));
    const batchSession = view.getUint32(position + 4, true);
    const count = view.getUint16(position + 8, true);
    const size = view.getUint16(position + 10, true);
    if (
      magic !== 'FLG1' ||
      batchSession !== session ||
      ![BASE_SIZE, HOVER_SIZE, BARO_SIZE, GUARD_SIZE, RECORD_SIZE].includes(
        size
      ) ||
      count < 1 ||
      count > MAX_BATCH
    )
      throw new Error('Invalid flight-log packet');
    position += HEADER_SIZE;
    for (let index = 0; index < count; index++) {
      if (position + size > bin.length) {
        truncated = true;
        break;
      }
      const sequence = view.getUint32(position + 4, true);
      if (!seen.has(sequence)) {
        seen.add(sequence);
        offsets.push(position);
        sizes.push(size);
        sequences.push(sequence);
      }
      position += size;
    }
    if (truncated) break;
  }
  const order = sequences
    .map((_, index) => index)
    .sort((a, b) => sequences[a] - sequences[b]);
  return {
    view,
    offsets: order.map((index) => offsets[index]),
    sizes: order.map((index) => sizes[index]),
    sequences: order.map((index) => sequences[index]),
    truncated,
  };
}

export function analyzeFlight(
  name: string,
  bin: Uint8Array,
  jsonl: string
): FlightReport {
  const events = parseEvents(jsonl);
  const start = events.find((event) => event.type === 'start');
  if (!start) throw new Error('The recording has no start record');
  // The start record wins over later metadata, as in flight_log.py read_flight.
  const metadata: Record<string, unknown> = {};
  for (const event of events)
    if (event.type === 'start' || event.type === 'metadata')
      for (const [key, value] of Object.entries(event))
        if (!(key in metadata)) metadata[key] = value;
  const session = Number(metadata.session);
  const signs = (metadata.sensor_to_body as number[] | undefined) ?? [1, 1, 1];
  // The gyro's counts per deg/s: 131 (±250 deg/s) in recordings from before the metadata said.
  const gyroCounts = Number(metadata.gyro_counts_per_deg_s ?? 131);
  const bias = (
    (metadata.gyro_bias_rad_s as number[] | undefined) ?? [0, 0, 0]
  ).map((value) => (value * 180) / Math.PI);
  const settings = (metadata.settings ?? {}) as Record<string, number>;
  const stop = events.find((event) => event.type === 'stop') as
    (Event & { reason: string; time_us: number; dropped: number }) | undefined;

  const { view, offsets, sizes, sequences, truncated } = decodeRecords(
    bin,
    session
  );
  const count = offsets.length;
  const time = new Float64Array(count);
  const az = new Float64Array(count);
  const pitchRate = new Float64Array(count);
  const roll = new Float64Array(count);
  const pitch = new Float64Array(count);
  const rollTarget = new Float64Array(count);
  const pitchTarget = new Float64Array(count);
  const base = new Float64Array(count);
  const throttle = new Float64Array(count);
  const flags = new Uint16Array(count);
  const height = new Float64Array(count).fill(NaN);
  const heightTarget = new Float64Array(count).fill(NaN);
  const yaw = new Float64Array(count).fill(NaN);
  // Each servo's forward tilt from its trim (degrees; the left servo tilts forward with positive angles, the right with negative).
  const servoTilt = new Float64Array(count);
  const velocityX = new Float64Array(count).fill(NaN);
  const velocityY = new Float64Array(count).fill(NaN);
  const baro = new Float64Array(count).fill(NaN);
  // Runs of records whose seven raw IMU values repeat the one before: a read that failed and left the last sample in place.
  let repeats = 0;
  let longestRepeat = 0;
  let repeatAt = NaN;
  let run = 0;
  for (let index = 0; index < count; index++) {
    const at = offsets[index];
    const int16 = (offset: number) => view.getInt16(at + offset, true);
    const uint16 = (offset: number) => view.getUint16(at + offset, true);
    time[index] = view.getUint32(at, true) / 1e6;
    az[index] = (int16(12) * signs[2]) / 16384;
    pitchRate[index] = (int16(18) * signs[1]) / gyroCounts - bias[1];
    roll[index] = int16(22) / 100;
    pitch[index] = int16(24) / 100;
    throttle[index] = uint16(26) + uint16(28);
    servoTilt[index] = rotorTilt(
      int16(30) / 100,
      int16(32) / 100,
      settings.left_trim ?? 0,
      settings.right_trim ?? 0
    );
    rollTarget[index] = int16(34) / 100;
    pitchTarget[index] = int16(36) / 100;
    base[index] = uint16(40);
    flags[index] = uint16(44);
    if (index > 0) {
      const previous = offsets[index - 1];
      let same = true;
      for (let offset = 8; offset < 22 && same; offset += 2)
        same = view.getInt16(previous + offset, true) === int16(offset);
      run = same ? run + 1 : 0;
      if (run === 1) {
        repeats++;
        if (Number.isNaN(repeatAt)) repeatAt = time[index];
      }
      longestRepeat = Math.max(longestRepeat, run);
    }
    if (sizes[index] >= HOVER_SIZE) {
      height[index] = int16(60) / 1000;
      heightTarget[index] = int16(64) / 1000;
      yaw[index] = int16(70) / 100;
      velocityX[index] = int16(56) / 1000;
      velocityY[index] = int16(58) / 1000;
    }
    if (sizes[index] >= BARO_SIZE) {
      const value = view.getInt32(at + 80, true);
      if (value !== BARO_NONE) baro[index] = value / 1000;
    }
  }
  let missing = 0;
  for (let index = 1; index < count; index++)
    missing += Math.max(0, sequences[index] - sequences[index - 1] - 1);

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

  // The servo speed balance (hover.h speed_tilt): tilt less speed_pitch times pitch, over speed_tilt degrees per m/s, from its takeoff offset.
  const speedTilt = settings.hover_speed_tilt ?? 0;
  const speedRaw = (index: number) =>
    servoTilt[index] - (settings.hover_speed_pitch ?? 0) * pitch[index];
  let speedOffset = NaN;
  if (lift >= 0 && stopped - lift > SPEED_TAKEOFF_S * RATE_HZ) {
    let sum = 0;
    for (let index = lift; index < lift + SPEED_TAKEOFF_S * RATE_HZ; index++)
      sum += speedRaw(index);
    speedOffset =
      sum / (SPEED_TAKEOFF_S * RATE_HZ) + (settings.hover_speed_takeoff ?? 0);
  }

  // The barometer from its reading at liftoff (the last second before it), else its first.
  let baroZero = NaN;
  if (lift >= 0) {
    const before: number[] = [];
    for (let index = Math.max(0, lift - RATE_HZ); index <= lift; index++)
      if (Number.isFinite(baro[index])) before.push(baro[index]);
    baroZero = median(before);
  }
  if (!Number.isFinite(baroZero))
    baroZero = baro.find((value) => Number.isFinite(value)) ?? NaN;

  // The shake over the airborne window, leaving out half a second at each end where the filter starts up.
  const shake = new Float64Array(count).fill(NaN);
  let shakeRms: number | null = null;
  if (lift >= 0 && stopped - lift > 2 * RATE_HZ) {
    const band = shakeBand(pitchRate.subarray(lift, stopped));
    const edge = RATE_HZ / 2;
    let sum = 0;
    for (let index = edge; index < band.length - edge; index++)
      sum += band[index] ** 2;
    shakeRms = Math.sqrt(sum / (band.length - 2 * edge));
    const window = SHAKE_WINDOW_S * RATE_HZ;
    let moving = 0;
    for (let index = 0; index < band.length; index++) {
      moving += band[index] ** 2;
      if (index >= window) moving -= band[index - window] ** 2;
      if (index >= window - 1)
        shake[lift + index - Math.floor(window / 2)] = Math.sqrt(
          Math.max(0, moving) / window
        );
    }
  }

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

  const track: FlightTrackPoint[] = gpsEvents
    .filter((gps) => gps.ok && gps.fix >= 2 && gps.fix <= 4)
    .map((gps) => ({
      time: sinceStart(gps.drone_ms),
      latitude: gps.latitude,
      longitude: gps.longitude,
      speed: Math.hypot(gps.velocity[0], gps.velocity[1]),
      horizontal_accuracy: gps.horizontal_accuracy,
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
  const changed = commands.map((command) => command.time);
  const before = (times: number[], at: number, within: number) =>
    times.some((value) => value <= at && value > at - within);
  // A command held: the last one at or before a time (manual and level before the first).
  const heldAt = (at: number) => {
    let held: (typeof commands)[number] | undefined;
    for (const command of commands)
      if (command.time <= at && !command.stop) held = command;
      else if (command.time > at) break;
    return held;
  };

  // Hands-off spans: airborne in hover hold (flags bits 1-2), no tilt or yaw setpoint and no command change for HANDS_ON_S.
  const spans: HandsOffSpan[] = [];
  if (lift >= 0) {
    let from = -1;
    const close = (to: number) => {
      if (from >= 0 && time[to - 1] - time[from] >= SPAN_MIN_S)
        spans.push(spanStats(from, to));
      from = -1;
    };
    for (let index = lift; index < stopped; index++) {
      const at = time[index];
      const held = heldAt(at);
      const off =
        at - time[lift] >= SPAN_SETTLE_S &&
        ((flags[index] >> 1) & 3) === 1 &&
        !held?.roll &&
        !held?.pitch &&
        !held?.yaw_rate &&
        !before(changed, at, HANDS_ON_S);
      if (off && from < 0) from = index;
      if (!off) close(index);
    }
    close(stopped);
  }

  function spanStats(from: number, to: number): HandsOffSpan {
    let low = Infinity;
    let high = -Infinity;
    for (let index = from; index < to; index++)
      if (Number.isFinite(baro[index])) {
        low = Math.min(low, baro[index]);
        high = Math.max(high, baro[index]);
      }
    const fixes = track.filter(
      (point) => point.time >= time[from] && point.time <= time[to - 1]
    );
    return {
      start: time[from],
      end: time[to - 1],
      pitch_sd: spread(pitch, from, to),
      roll_sd: spread(roll, from, to),
      height_sd: spread(height, from, to),
      baro_range: high >= low ? high - low : null,
      drift_m: fixes.length > 1 ? distance(fixes[0], fixes.at(-1)!) : null,
      wander_m:
        fixes.length > 1
          ? Math.max(...fixes.map((point) => distance(fixes[0], point)))
          : null,
      wind: windEstimate(from, to),
    };
  }

  // A rough wind along the drone's heading (m/s, positive from behind): the forward ground speed (the hover velocity, GPS-corrected with the GPS hold, turned to body axes by the gyro heading) less the forward airspeed the servos' tilt explains by the firmware's speed balance (fw/drone/main/hover.h speed_tilt), with its offset taken as the firmware does without the tag: the first SPEED_TAKEOFF_S airborne plus speed_takeoff. Only the fore-aft component, and any wind at takeoff is in the offset; replace with a better airspeed model when there is one.
  function windEstimate(from: number, to: number) {
    if (!speedTilt || !Number.isFinite(speedOffset)) return null;
    let sum = 0;
    let used = 0;
    for (let index = from; index < to; index++) {
      if (!Number.isFinite(yaw[index]) || !Number.isFinite(velocityX[index]))
        continue;
      const heading = (yaw[index] * Math.PI) / 180;
      const ground =
        velocityX[index] * Math.cos(heading) +
        velocityY[index] * Math.sin(heading);
      sum += ground - (speedRaw(index) - speedOffset) / speedTilt;
      used++;
    }
    return used > (to - from) / 2 ? sum / used : null;
  }

  const warnings: FlightWarning[] = [];
  // Fly-aways: fast fixes in a row with no tilt asked for, while airborne (a drone carried or tumbling after the motors stop is not one).
  const flown = airborne
    ? track.filter(
        (point) => point.time >= airborne[0] && point.time <= airborne[1]
      )
    : [];
  for (let index = 0; index < flown.length;) {
    if (
      flown[index].speed <= FLYAWAY_MS ||
      before(tilted, flown[index].time, TILT_COMMAND_S)
    ) {
      index++;
      continue;
    }
    let end = index;
    let top = 0;
    while (
      end < flown.length &&
      flown[end].speed > FLYAWAY_MS &&
      !before(tilted, flown[end].time, TILT_COMMAND_S)
    )
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
  const winds = spans
    .map((span) => span.wind)
    .filter((value): value is number => value !== null);

  const series: FlightSeries = {
    time: [],
    height: [],
    height_target: [],
    baro: [],
    roll: [],
    pitch: [],
    roll_target: [],
    pitch_target: [],
    shake: [],
  };
  for (let index = 0; index < count; index += CHART_STRIDE) {
    series.time.push(round(time[index], 2)!);
    series.height.push(round(height[index], 3));
    series.height_target.push(round(heightTarget[index], 3));
    series.baro.push(round(baro[index] - baroZero, 2));
    series.roll.push(round(roll[index], 2)!);
    series.pitch.push(round(pitch[index], 2)!);
    series.roll_target.push(round(rollTarget[index], 2)!);
    series.pitch_target.push(round(pitchTarget[index], 2)!);
    series.shake.push(round(shake[index], 1));
  }

  return {
    name,
    started: String(start.received_at),
    drone: (start.drone as string | null | undefined) ?? null,
    duration: stop ? stop.time_us / 1e6 : count ? time[count - 1] : 0,
    airborne,
    stop_reason: stop?.reason ?? null,
    samples: count,
    missing,
    max_height: Number.isFinite(maxHeight) ? maxHeight : null,
    top_speed: flown.length
      ? Math.max(...flown.map((point) => point.speed))
      : null,
    shake_rms: shakeRms,
    wind: winds.length ? winds.reduce((a, b) => a + b) / winds.length : null,
    gps_hold: (settings.hover_gps_tau ?? 0) > 0,
    series,
    track,
    spans,
    warnings,
  };
}
