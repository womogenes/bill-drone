"""
Drone 1's sim scorecard: how well a plant predicts every flight we have recorded, as one number and per flight: uv run -m sw.sim.scorecard [--plant airframe_2026-11-12_raised.json ...] [--set k=v,...] [--flights NAME ...] [--kinds hover_calm,...] [--workers 8] [--json out.json] [--compare base.json] [--plot NAME[:START:END] ...]

The standard (the user's): at every time step we have all the sensor readings and the log remembers the commands; predict the future. So for start points t every --every s (0.1) in each flight's airborne time:
- The hidden state (the servo horns' and assemblies' angles and rates, the motors' thrusts, the command history the dead times read) is reconstructed by running the plant over the --warmup s (1) before t on the logged throttles and servo commands, the body's rates, attitude, velocity and height set to the log's every tick, so at t the actuators are already moving as the commands moved them.
- From the log's state at t (gyro, roll and pitch, the Kalman filter's velocity turned into the heading-free frame, its height) the plant flies on alone on the logged commands, and is compared with the log at the horizons 0.05, 0.2, 0.5 and 1 s: the gyro (deg/s, each tick's mean rotation as rigid.py observes it, clipped at the recording's full scale), roll and pitch (deg, against the firmware's estimate), the specific force (g, rigid.py's accelerometer with the configuration's bias) and the height change (m, the filter's, where its height is good).
- The log's gyro and accelerometer are compared through a zero-phase 15 Hz low-pass (the rotors' vibration, which no plant models, would otherwise set a floor under every short horizon); the start state takes the raw gyro. Targets where the gyro clipped, and the attitude for 2 s after a clip, are left out.
All start points of a flight fly at once as the rigid.Sim's lanes (the C backend), so a plant scores in a few minutes on drone-v0 (--workers per flight).

Per flight, per configuration (scorecard_flights.json): the bands (preload, tilt offset and accelerometer bias) and the thrust table (the old ESCs' firmware table or the 2026-11-14 stand's) come from its configuration; the thrust trim is the flight's own (each lane's: the median table thrust of the 5 s of airborne ticks before it carries the weight, so the pack's sag is followed; the plant's level and split are the flight's median, as identify_airframe takes each flight's) and so are the steady pushes (roll, pitch, yaw: the plant's plus the mean one-tick angular acceleration error over the flight's hovering ticks; --push plant keeps the plant's, which then has to know each flight's CoM). The belly's hull points are the forward legs' feet (takeoff_test.LEG_FEET) and the height is set as the filter's above where it last rested, so the touchdowns land on the legs. A plant's JSON may carry "configurations": {name: {term: value}} for terms it has per configuration.

The score: each channel's RMS error at each horizon over a flight's start points, over the RMS of the do-nothing prediction on the same start points (the log's own change over the horizon), averaged within the channel groups (rates, attitude, specific force, height where logged), then over the groups and the horizons. 1 is as bad as predicting nothing changes; 0 is perfect. A kind's score is the mean of its flights'; takeoffs (the first 2.5 s airborne) and landings (the last 2.5 s with the motors on) of the whole-flight entries are pooled into two kinds of their own, and their starts are left out of the flights' scores. The total is the mean over the kinds, so many hovers do not outweigh the few bursts. Each flight's noise is its score's standard error by a block jackknife over 2 s blocks of start points; --compare takes the paired difference from a saved run's per-start errors (the .npz beside its --json).

The rule for plant changes: a plant change must lower the total without worsening any single flight's score beyond noise (--compare flags a flight whose score rises by more than twice the paired standard error).

What it does not model: the firmware's attitude estimator (the plant's true roll and pitch are compared with the logged estimate; in a steady bank the complementary filter's pull toward the accelerometer makes them differ), the IMU's offset from the CoM (the specific force is the CoM's), the battery's sag within a lane's 2 s, wind (the windy kinds measure it as error).
"""

import argparse
import json
import math
import time as clock
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

from sw.flight_log import parse_settings, read_flight, recording, settings
from sw.sim import airframe as A
from sw.sim import flight as F
from sw.sim import rigid as R
from sw.sim.burst_replay import BAND_TILT, BANDS
from sw.sim.flip_esc import STAND
from sw.sim.flip_replay import quaternion
from sw.sim.takeoff_test import LEG_FEET

FLIGHTS = Path(__file__).with_name("scorecard_flights.json")
PLANTS = ("airframe_2026-11-12_raised.json", "airframe_2026-11-15_bursts.json")
HORIZONS = (0.05, 0.2, 0.5, 1.0)
CHANNELS = ("roll_rate", "pitch_rate", "yaw_rate", "roll", "pitch", "accel_x", "accel_y", "accel_z", "height")
UNITS = ("deg/s", "deg/s", "deg/s", "deg", "deg", "g", "g", "g", "m")
GROUPS = {"rates": (0, 1, 2), "attitude": (3, 4), "accel": (5, 6, 7), "height": (8,)}
# The bands' accelerometer error (g, body forward) as flip.py and sweep.py have it.
BAND_ACCEL = 0.025
# The accelerometer's full scale (g; flight_log reads 16384 counts per g of 16-bit samples).
ACCEL_RANGE = 2.0
# Both motors over this throttle: the motors on.
ON_THROTTLE = 300
# A drone with the motors on is airborne from this height (m, the filter's) up; without a good height, once the motors are on.
AIRBORNE_HEIGHT = 0.1
# The filter's height and velocity are used while their standard deviations are under these (m, m/s).
HEIGHT_SD, VELOCITY_SD = 0.5, 0.5
# Takeoffs and landings: the first and last this many seconds with the motors on.
PHASE = 2.5
# Kinds scored over all their airborne start points (windows and short flights), with no takeoff or landing split off.
WHOLE = ("burst", "divergence")
# The log's gyro and accelerometer are compared through this low-pass (Hz, zero phase).
TARGET_HZ = 15.0
# The trim is the median table thrust over this many seconds of airborne ticks before each start (at least 1 s of them, or the flight's).
TRIM_S = 5.0
# The flight's push zeroes the mean rate error this far ahead (s).
PUSH_S = 0.1
# Jackknife blocks (s of start points) and the --compare flag's threshold (paired standard errors).
BLOCK_S, NOISE = 2.0, 2.0
# The belly's four hull points become the forward legs' feet (relative to the CoM); the height at rest puts them on the ground.
HULL = np.array(R.HULL)
HULL[:4] = LEG_FEET[:4]
REST_Z = -min(z for _, _, z in LEG_FEET[:4])


def flight_list(path=FLIGHTS):
    return json.loads(Path(path).read_text())


class Flight:
    """A whole recording as arrays at the log's 100 Hz: commands, the start states the warm-up pins and the targets the predictions are scored against."""

    def __init__(self, entry, configuration):
        self.entry, self.name, self.kind = entry, entry["name"], entry["kind"]
        self.configuration = configuration
        path = recording(self.name)
        df = read_flight(path)[0].reset_index(drop=True)
        meta_counts = json.loads(path.with_suffix(".jsonl").read_text().splitlines()[0]).get("gyro_counts_per_deg_s", 131.0)
        self.gyro_range = 32768 / meta_counts * A.DEG
        self.time = (df["esp_time_us"] - df["esp_time_us"].iloc[0]).to_numpy() / 1e6
        self.gyro = df[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].to_numpy() * A.DEG
        self.throttle = df[["left_throttle", "right_throttle"]].to_numpy(float)
        s = settings(path)
        trims = np.array([s.get("left_trim", 0.0), s.get("right_trim", 0.0)])
        directions = np.array([F.MODEL["left_servo_direction"], F.MODEL["right_servo_direction"]])
        self.tilt = np.radians(directions * (df[["left_servo_deg", "right_servo_deg"]].to_numpy(float) - trims))
        self.attitude = df[["roll_deg", "pitch_deg"]].to_numpy() * A.DEG
        self.quat = np.array([quaternion(*a) for a in self.attitude])
        self.accel = df[["ax_g", "ay_g", "az_g"]].to_numpy()
        self.flip = df["flip"].fillna(0).to_numpy()
        # Samples the link lost: a start's warm-up and horizon must have none (the commands and the log would slip against each other).
        self.gaps = np.concatenate([[0], np.cumsum(np.diff(df["sequence"].to_numpy()) != 1)])
        lowpass = butter(2, TARGET_HZ / 50)
        self.gyro_smooth = filtfilt(*lowpass, self.gyro, axis=0)
        self.accel_smooth = filtfilt(*lowpass, self.accel, axis=0)
        # The filter's height above where it last rested, and its velocity north, west and up turned into the heading-free frame the quaternions are in (x along the nose's heading), where good.
        height = df["ekf_height_m"].to_numpy(float)
        self.height = np.where(df["ekf_height_sd_m"].to_numpy(float) < HEIGHT_SD, height, np.nan)
        yaw = np.radians(df["ekf_yaw_deg"].to_numpy(float))
        north, west, up = (df[f"ekf_{c}_m_s"].to_numpy(float) for c in ("north", "west", "up"))
        good = df["ekf_velocity_sd_m_s"].to_numpy(float) < VELOCITY_SD
        self.velocity = np.column_stack([np.where(good, np.cos(yaw) * north + np.sin(yaw) * west, 0.0), np.where(good, -np.sin(yaw) * north + np.cos(yaw) * west, 0.0), np.nan_to_num(up)])
        self.velocity = np.nan_to_num(self.velocity)
        # The tilt from upright (rad) and the body-plane speed (m/s) the log flew, against the plant's clipped ranges (rigid.py ATTITUDE_RANGE, SPEED_RANGE).
        self.tilt_angle = np.arccos(np.clip(np.cos(self.attitude[:, 0]) * np.cos(self.attitude[:, 1]), -1, 1))
        body = np.einsum("nji,nj->ni", R.rotation(self.quat), self.velocity)
        self.plane_speed = np.hypot(body[:, 0], body[:, 1])
        self.on = (self.throttle > ON_THROTTLE).all(1)
        self.airborne = self.on & ~(self.height < AIRBORNE_HEIGHT)
        self.clipped = (np.abs(self.gyro) >= 0.97 * self.gyro_range).any(1)
        self.accel_clipped = (np.abs(self.accel) >= 0.97 * ACCEL_RANGE).any(1)
        # The attitude estimate is left out for 2 s after the gyro clipped.
        after = np.convolve(self.clipped.astype(float), np.ones(200), "full")[: len(self.time)] > 0
        self.attitude_good = ~after
        on = np.flatnonzero(self.on)
        self.first_on, self.last_on = (self.time[on[0]], self.time[on[-1]]) if len(on) else (math.inf, -math.inf)
        # The table thrusts (N, before the plant's right gain) of the hovering ticks: the motors on, airborne, no flip.
        self.table = STAND if configuration["table"] == "stand" else A.TABLE
        self.hovering = self.airborne & (self.flip == 0)

    def starts(self, every, warmup, horizon, windowed=True):
        """The start indices with each one's phase (the flight's kind, "takeoff" or "landing"); windowed False ignores the entry's window."""
        step = max(1, round(every / A.DT))
        first = round(warmup / A.DT) + 1
        index = np.arange(first, len(self.time) - 1, step)
        index = index[self.airborne[index] & ~self.clipped[index] & ~self.clipped[np.maximum(index - 10, 0)]]
        index = index[self.gaps[np.minimum(index + horizon, len(self.time) - 1)] == self.gaps[index - first + 1]]
        window = self.entry.get("window") if windowed else None
        if window:
            index = index[(self.time[index] >= window[0]) & (self.time[index] <= window[1])]
        if self.kind in WHOLE or window:
            return index, np.array([self.kind] * len(index), dtype=object)
        t = self.time[index]
        phase = np.where(t - self.first_on < PHASE, "takeoff", np.where(self.last_on - t < PHASE, "landing", self.kind)).astype(object)
        return index, phase

    def targets(self, index, steps):
        """The log's values at index + steps [start, step, channel] (NaN where not scored) and at index itself [start, channel]."""
        k = np.clip(index[:, None] + steps[None], 0, len(self.time) - 1)
        beyond = (index[:, None] + steps[None]) >= len(self.time)
        values = self.values(k)
        values[beyond] = np.nan
        start = self.values(index)
        values[..., 8] -= start[:, None, 8]
        start[:, 8] = 0.0
        return values, start

    def values(self, k):
        gyro = np.where(self.clipped[k][..., None], np.nan, self.gyro_smooth[k] / A.DEG)
        attitude = np.where(self.attitude_good[k][..., None], self.attitude[k] / A.DEG, np.nan)
        accel = np.where(self.accel_clipped[k][..., None], np.nan, self.accel_smooth[k])
        # The ground is at zero: the filter's height below it (after a crash it integrates on) is not a height.
        height = np.where(self.height[k] < -AIRBORNE_HEIGHT, np.nan, self.height[k])
        return np.concatenate([gyro, attitude, accel, height[..., None]], -1)


def plant_for(flight, name, overrides):
    """The rigid.Sim plant for a flight: the airframe (a models/ file or a path) with the configuration's bands and table, the plant's own per-configuration terms, the overrides, and the flight's thrust trim."""
    params = json.loads(Path(name).read_text()) if Path(name).exists() else A.model(name)
    configuration = flight.configuration
    bands = configuration["bands"]
    own = {"left_band": params.get("left_band", 0.0), "right_band": params.get("right_band", 0.0)}
    preload = (own if any(own.values()) else BANDS) if bands else {"left_band": 0.0, "right_band": 0.0}
    params = params | preload | params.get("configurations", {}).get(flight.entry["configuration"], {}) | overrides
    geometry = A.geometry_of(params)
    hover = A.thrust(flight.throttle[flight.hovering], flight.table) * np.array([1.0, params.get("right_thrust_gain", 1.0)])
    level, mean = A.trim_for(hover, geometry)
    params |= {"thrust_level": float(level), "hover_thrust": [float(m) for m in mean]}
    return {"airframe": params, "geometry": geometry, "disturbances": [], "rotor_drag": tuple(params.get("rotor_drag", (0.07, 0.2))), "tilt_offset": BAND_TILT if bands else 0.0,
            "accel_bias": (BAND_ACCEL if bands else 0.0, 0.0, 0.0), "table": flight.table}


def local_throttle(flight, plant, index, steps):
    """Each lane's throttles over its warm-up and horizon [lane, step, side], rescaled so the thrust they ask for carries the weight at the trim of the TRIM_S s before the lane's start (the pack's sag) rather than the flight's median."""
    p = plant["airframe"]
    gain = np.array([1.0, p.get("right_thrust_gain", 1.0)])
    total = (A.thrust(flight.throttle, flight.table) * gain).sum(1)
    window = round(TRIM_S / A.DT)
    ratio = np.ones(len(index))
    for i, k in enumerate(index):
        chosen = flight.hovering[max(0, k - window): k + 1]
        if chosen.sum() >= 100:
            ratio[i] = np.median(total[max(0, k - window): k + 1][chosen])
    flight_median = np.median(total[flight.hovering]) if flight.hovering.any() else 1.0
    ratio = np.where(ratio == 1.0, 1.0, flight_median / ratio)
    k = np.clip(index[:, None] + steps[None], 0, len(flight.time) - 1)
    thrust = A.thrust(flight.throttle[k], flight.table) * ratio[:, None, None]
    return np.where(flight.throttle[k] > 0, R.throttle_for(thrust, flight.table), 0.0)


def make_sim(plant, n):
    R.HULL = HULL
    sim = R.Sim(n, plant, seed=0)
    sim.gyro_bias[:] = 0.0
    return sim


def flight_push(flight, plant, warmup, every=0.2, rounds=2):
    """The flight's steady angular acceleration the plant lacks (rad/s², roll, pitch, yaw: a CoM off the geometry's, a trim): over the flight's hovering start points (no takeoff, landing or flip), the push that zeroes the mean error of the rate PUSH_S s ahead, found by rounds of the plant's own response (a steady push a moves the rate about a times the horizon)."""
    index, _ = flight.starts(every, warmup * A.DT, 0, windowed=False)
    t = flight.time[index]
    index = index[(flight.flip[index] == 0) & (t - flight.first_on > PHASE) & (flight.last_on - t > PHASE)]
    push = np.zeros(3)
    if len(index) < 20:
        return push
    ticks = round(PUSH_S / A.DT)
    for _ in range(rounds):
        predicted = predict(flight, plant, index, warmup, ticks, push=push)
        target, _ = flight.targets(index, np.array([ticks]))
        bias = np.nanmean(predicted[:, ticks - 1, :3] - target[:, 0, :3], 0) * A.DEG
        push = push - bias / ((ticks - 0.5) * A.DT)
    return push


def predict(flight, plant, index, warmup, horizon, push=None):
    """The plant's predictions [lane, step, channel] for steps 1..horizon ticks after each start index, its hidden state warmed up over the warmup ticks before it with the body pinned to the log; push (rad/s², roll, pitch, yaw) is added to the plant's steady pushes."""
    n = len(index)
    steps = np.arange(-warmup + 1, horizon + 1)
    throttle = local_throttle(flight, plant, index, steps)
    k = np.clip(index[:, None] + steps[None], 0, len(flight.time) - 1)
    tilt = flight.tilt[k]
    sim = make_sim(plant, n)
    p = sim.p
    if push is not None:
        sim.push[:] += push
    gain = np.tile([1.0, p["right_thrust_gain"]], (n, 1))
    pin = k[:, 0] - 1
    motor = A.actual_thrust(p, A.table_thrust(p, throttle[:, 0], np.ones(n), table=sim.table, gain=gain), sim.trim)
    position = np.column_stack([np.zeros((n, 2)), np.nan_to_num(flight.height[pin], nan=1.0) + REST_Z])
    sim.reset(position, flight.velocity[pin], flight.quat[pin], flight.gyro[pin], motor=motor, tilt=tilt[:, 0] - sim.tilt_offset)
    sim.throttles[:] = throttle[:, :1]
    sim.bind()
    out = np.full((n, horizon, len(CHANNELS)), np.nan)
    for j, step in enumerate(steps):
        if step <= 1:
            # Pinned to the log's state before this tick's commands (the warm-up, and the start itself).
            before = k[:, j] - 1
            sim.omega[:] = flight.gyro[before]
            sim.quat[:] = flight.quat[before]
            sim.vel[:] = flight.velocity[before]
            sim.pos[:, 2] = np.where(np.isnan(flight.height[before]), sim.pos[:, 2], flight.height[before] + REST_Z)
            if step == 1:
                z0 = sim.pos[:, 2].copy()
        sim.step(throttle[:, j], tilt[:, j])
        if step >= 1:
            up = R.rotation(sim.quat)[:, 2, :]
            roll, pitch = np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))
            rates = np.clip(sim.turned, -flight.gyro_range, flight.gyro_range)
            out[:, step - 1] = np.column_stack([rates / A.DEG, roll / A.DEG, pitch / A.DEG, np.clip((sim.accel + sim.accel_bias) / A.G, -ACCEL_RANGE, ACCEL_RANGE), sim.pos[:, 2] - z0])
    return out


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def score_flight(task):
    """One flight on one plant: per start, the errors at the horizons [start, horizon, channel] and the do-nothing prediction's, with the starts' phases and times."""
    entry, configuration, plant_name, overrides, every, warmup, push = task
    began = clock.time()
    flight = Flight(entry, configuration)
    horizon = round(max(HORIZONS) / A.DT)
    index, phase = flight.starts(every, warmup, horizon)
    ticks = np.array([round(h / A.DT) for h in HORIZONS])
    if len(index) == 0:
        return {"name": flight.name, "kind": flight.kind, "error": np.zeros((0, len(HORIZONS), len(CHANNELS))), "naive": np.zeros((0, len(HORIZONS), len(CHANNELS))), "phase": phase, "time": np.zeros(0), "reach": np.zeros((0, 2)), "push": None, "seconds": 0.0}
    plant = plant_for(flight, plant_name, overrides)
    extra = flight_push(flight, plant, round(warmup / A.DT)) if push else None
    predicted = predict(flight, plant, index, round(warmup / A.DT), horizon, extra)
    target, start = flight.targets(index, ticks)
    error = predicted[:, ticks - 1] - target
    naive = start[:, None] - target
    for c in (3, 4):
        error[..., c], naive[..., c] = wrap(error[..., c]), wrap(naive[..., c])
    # How far each start's horizon goes past the plant's clipped ranges: the largest tilt (deg) and body-plane speed (m/s) the log flew in it.
    k = np.clip(index[:, None] + np.arange(horizon + 1)[None], 0, len(flight.time) - 1)
    reach = np.column_stack([np.degrees(flight.tilt_angle[k].max(1)), flight.plane_speed[k].max(1)])
    return {"name": flight.name, "kind": flight.kind, "error": error, "naive": naive, "phase": phase, "time": flight.time[index], "reach": reach, "push": extra, "seconds": clock.time() - began}


def rms(x, axis=0):
    with np.errstate(invalid="ignore"):
        count = np.sum(~np.isnan(x), axis)
        return np.where(count > 0, np.sqrt(np.nansum(x ** 2, axis) / np.maximum(count, 1)), np.nan)


def normalized(error, naive):
    """The score of a set of start points' errors [start, horizon, channel]: each channel's RMS over the do-nothing prediction's RMS on the same starts (the log's own change over the horizon), averaged within the groups, then over the groups present and the horizons. 1 is as good as predicting that nothing changes, 0 is perfect."""
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio = rms(error) / rms(naive)
    ratio[~np.isfinite(ratio)] = np.nan
    groups = [np.nanmean(ratio[:, list(c)], 1) for c in GROUPS.values() if not np.all(np.isnan(ratio[:, list(c)]))]
    return float(np.nanmean(groups))


def jackknife(statistic, time):
    """A statistic's (a function of a mask of start points) standard error by a block jackknife over BLOCK_S s blocks of start times."""
    blocks = np.floor((time - time.min()) / BLOCK_S).astype(int) if len(time) else np.zeros(0, int)
    names = np.unique(blocks)
    if len(names) < 3:
        return math.nan
    values = np.array([statistic(blocks != b) for b in names])
    return float(math.sqrt((len(names) - 1) / len(names) * np.sum((values - values.mean()) ** 2)))


def units(results):
    """The scored units: each flight's own starts (its kind's phase), and the pooled takeoffs and landings."""
    out = []
    for r in results:
        own = r["phase"] == r["kind"]
        if own.any():
            out.append({"name": r["name"], "kind": r["kind"], "error": r["error"][own], "naive": r["naive"][own], "time": r["time"][own], "reach": r["reach"][own], "push": r.get("push")})
    for kind in ("takeoff", "landing"):
        parts = [(r["error"][r["phase"] == kind], r["naive"][r["phase"] == kind], r["time"][r["phase"] == kind] + 1000.0 * i) for i, r in enumerate(results) if (r["phase"] == kind).any()]
        if parts:
            out.append({"name": f"all {kind}s", "kind": kind, "error": np.concatenate([p[0] for p in parts]), "naive": np.concatenate([p[1] for p in parts]), "time": np.concatenate([p[2] for p in parts]),
                        "reach": np.concatenate([r["reach"][r["phase"] == kind] for r in results if (r["phase"] == kind).any()]), "push": None})
    return out


def summarize(results):
    """Scores per unit, per kind and the total."""
    scored = units(results)
    for u in scored:
        u["score"] = normalized(u["error"], u["naive"])
        u["noise"] = jackknife(lambda keep, u=u: normalized(u["error"][keep], u["naive"][keep]), u["time"])
        u["rms"] = rms(u["error"])
        u["count"] = len(u["error"])
    kinds = {}
    for u in scored:
        kinds.setdefault(u["kind"], []).append(u["score"])
    kind_scores = {k: float(np.mean(v)) for k, v in kinds.items()}
    return {"total": float(np.mean(list(kind_scores.values()))), "kinds": kind_scores, "units": scored}


def trained(name, plant, listing):
    """Whether the plant was fitted on this recording (scorecard_flights.json "training", by the plant file's stem and recording-name prefixes)."""
    return any(name.startswith(prefix) for prefix in listing.get("training", {}).get(Path(plant).stem, []))


def held_out(results, plant, listing):
    """The summary over the flights the plant was not fitted on."""
    kept = [r for r in results if not trained(r["name"], plant, listing)]
    return summarize(kept) if kept else None


def table(summary, label):
    """The printed table: per unit its score (and noise), and its RMS errors at 0.2 and 1 s of the pitch rate, roll rate, pitch and roll and the vertical specific force."""
    lines = [f"{label}: total {summary['total']:.3f}" + (f", held out {summary['held_out']['total']:.3f}" if summary.get("held_out") else "") + " (* fitted on)", "  kinds: " + ", ".join(f"{k} {v:.3f}" for k, v in summary["kinds"].items()),
             f"  {'flight':24} {'kind':14} {'n':>4} {'score':>6} {'±':>5} | at 0.2 s / 1 s: {'p rate':>11} {'r rate':>11} {'y rate':>11} {'pitch':>9} {'roll':>9} {'az g':>11} {'height':>11}"]
    h02, h1 = HORIZONS.index(0.2), HORIZONS.index(1.0)
    for u in summary["units"]:
        cells = " ".join(f"{u['rms'][h02, c]:5.1f}/{u['rms'][h1, c]:5.1f}" if c < 5 else f"{u['rms'][h02, c]:5.2f}/{u['rms'][h1, c]:5.2f}" for c in (1, 0, 2, 4, 3, 7, 8))
        lines.append(f"  {u['name']:24} {u['kind']:14} {u['count']:4d} {u['score']:6.3f} {u['noise']:5.3f}{'*' if u.get('trained') else ' '}|                 {cells}")
    return "\n".join(lines)


def run(plant, overrides, entries, configurations, every, warmup, push, workers):
    tasks = [(e, configurations[e["configuration"]], plant, overrides, every, warmup, push) for e in entries]
    if workers > 1:
        with ProcessPoolExecutor(workers) as pool:
            return list(pool.map(score_flight, tasks))
    return [score_flight(t) for t in tasks]


def save(summary, results, plant, overrides, path):
    """The JSON summary and, beside it (.npz), every start's errors for --compare."""
    out = {"plant": plant, "overrides": overrides, "total": summary["total"], "kinds": summary["kinds"], "held_out": summary.get("held_out"), "horizons": HORIZONS, "channels": CHANNELS, "units_of_channels": UNITS,
           "flights": [{"name": u["name"], "kind": u["kind"], "starts": u["count"], "score": u["score"], "noise": u["noise"], "rms": np.round(u["rms"], 4).tolist(), "trained": u.get("trained", False),
                        "push": None if u["push"] is None else np.round(u["push"], 3).tolist(), "past_attitude_range": float(np.mean(u["reach"][:, 0] > math.degrees(R.ATTITUDE_RANGE))) if len(u["reach"]) else 0.0,
                        "past_speed_range": float(np.mean(u["reach"][:, 1] > R.SPEED_RANGE)) if len(u["reach"]) else 0.0, "largest_tilt_deg": float(u["reach"][:, 0].max()) if len(u["reach"]) else 0.0,
                        "largest_speed_m_s": float(u["reach"][:, 1].max()) if len(u["reach"]) else 0.0} for u in summary["units"]]}
    Path(path).write_text(json.dumps(out, indent=1))
    arrays = {}
    for r in results:
        arrays[f"{r['name']}/error"], arrays[f"{r['name']}/naive"], arrays[f"{r['name']}/time"], arrays[f"{r['name']}/phase"] = r["error"], r["naive"], r["time"], r["phase"].astype(str)
    np.savez_compressed(Path(path).with_suffix(".npz"), **arrays)


def load_results(path, entries):
    data = np.load(Path(path).with_suffix(".npz"))
    kinds = {e["name"]: e["kind"] for e in entries}
    return [{"name": n, "kind": kinds[n], "error": data[f"{n}/error"], "naive": data[f"{n}/naive"], "time": data[f"{n}/time"], "phase": data[f"{n}/phase"].astype(object)} for n in kinds if f"{n}/error" in data]


def compare(results, base_results):
    """Per unit the score's change from the base run on the same start points, with its paired block-jackknife standard error; flags a rise past NOISE of them."""
    new, old = {u["name"]: u for u in units(results)}, {u["name"]: u for u in units(base_results)}
    lines, worse = [], []
    for name, u in new.items():
        if name not in old or len(old[name]["error"]) != len(u["error"]):
            continue
        b = old[name]
        change = lambda keep: normalized(u["error"][keep], u["naive"][keep]) - normalized(b["error"][keep], b["naive"][keep])
        delta = change(np.ones(len(u["error"]), bool))
        se = jackknife(change, u["time"])
        flag = delta > NOISE * se if not math.isnan(se) else delta > 0.05
        if flag:
            worse.append(name)
        lines.append(f"  {name:24} {u['kind']:14} {delta:+.3f} ± {se:.3f}{'  WORSE' if flag else ''}")
    return lines, worse


def plot(entry, configuration, plants, overrides, warmup, out, every=0.02):
    """The log against each plant's predictions made 0.2 s and 1 s earlier (dense starts), over the entry's window: the rates, roll and pitch, the vertical specific force."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    flight = Flight(entry, configuration)
    index, _ = flight.starts(every, warmup, 100)
    window = entry.get("window") or (flight.time[index[0]], flight.time[index[-1]])
    index = index[(flight.time[index] >= window[0]) & (flight.time[index] <= window[1])]
    rows = [(1, "pitch rate (deg/s)"), (0, "roll rate (deg/s)"), (2, "yaw rate (deg/s)"), (4, "pitch (deg)"), (3, "roll (deg)"), (7, "specific force up (g)")]
    fig, axes = plt.subplots(len(rows), 1, figsize=(13, 2.2 * len(rows)), sharex=True)
    shown = (flight.time >= window[0]) & (flight.time <= window[1] + 1.0)
    log_values = flight.values(np.flatnonzero(shown))
    colors = ("#1f77b4", "#d62728", "#2ca02c", "#9467bd")
    predictions = {name: predict(flight, plant_for(flight, name, overrides), index, round(warmup / A.DT), 100) for name in plants}
    for ax, (c, label) in zip(axes, rows):
        ax.plot(flight.time[shown], log_values[:, c], color="black", lw=1.2, label="log")
        for p, name in enumerate(plants):
            for h, style in ((0.2, "-"), (1.0, ":")):
                tick = round(h / A.DT)
                ax.plot(flight.time[index] + h, predictions[name][:, tick - 1, c], style, color=colors[p % len(colors)], lw=1, label=f"{Path(name).stem.replace('airframe_', '')} {h} s ahead")
        ax.set_ylabel(label, fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, ncol=3, loc="upper left")
    axes[0].set_title(f"{flight.name} ({entry['kind']}): the log and the plants' predictions made 0.2 s (solid) and 1 s (dotted) earlier", fontsize=9)
    axes[-1].set_xlabel("s from the recording's start")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plant", nargs="+", default=list(PLANTS), help="airframe models (sw/sim/models names or paths)")
    parser.add_argument("--set", default="", help="airframe terms over every plant's, name=value,...")
    parser.add_argument("--flights", nargs="*", help="only these recordings (name prefixes)")
    parser.add_argument("--kinds", default="", help="only these kinds, comma-separated")
    parser.add_argument("--list", default=str(FLIGHTS))
    parser.add_argument("--every", type=float, default=0.1)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--push", choices=("flight", "plant"), default="flight")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--json", help="write each plant's summary here (PLANT in the name is replaced by the plant's stem when scoring several)")
    parser.add_argument("--compare", help="a saved --json run (its .npz beside it) to compare the first plant with")
    parser.add_argument("--plot", nargs="*", help="flights to plot, NAME or NAME:START:END, instead of scoring")
    parser.add_argument("--plot-dir", default=".")
    args = parser.parse_args()
    listing = flight_list(args.list)
    configurations = listing["configurations"]
    entries = listing["flights"]
    if args.flights:
        entries = [e for e in entries if any(e["name"].startswith(f) for f in args.flights)]
    if args.kinds:
        entries = [e for e in entries if e["kind"] in args.kinds.split(",")]
    overrides = parse_settings(args.set) if args.set else {}
    if args.plot is not None:
        everything = {e["name"]: e for e in listing["flights"]}
        for spec in args.plot:
            name, *window = spec.split(":")
            entry = dict(next(e for n, e in everything.items() if n.startswith(name)))
            if window:
                entry["window"] = [float(window[0]), float(window[1])]
            out = Path(args.plot_dir) / f"scorecard_{entry['name']}.png"
            plot(entry, configurations[entry["configuration"]], args.plant, overrides, args.warmup, out)
            print(f"wrote {out}")
        return
    for plant in args.plant:
        began = clock.time()
        results = run(plant, overrides, entries, configurations, args.every, args.warmup, args.push == "flight", args.workers)
        summary = summarize(results)
        for u in summary["units"]:
            u["trained"] = trained(u["name"], plant, listing)
        unseen = held_out(results, plant, listing)
        summary["held_out"] = {"total": unseen["total"], "kinds": unseen["kinds"]} if unseen else None
        print(table(summary, f"{plant} ({clock.time() - began:.0f} s)"))
        if args.json:
            path = args.json.replace("PLANT", Path(plant).stem) if len(args.plant) > 1 else args.json
            save(summary, results, plant, overrides, path)
        if args.compare and plant == args.plant[0]:
            base = load_results(args.compare, entries)
            lines, worse = compare(results, base)
            print(f"against {args.compare}: total {summary['total']:.3f} vs {summarize(base)['total']:.3f}")
            print("\n".join(lines))
            print("worse beyond noise: " + (", ".join(worse) if worse else "none"))


if __name__ == "__main__":
    main()
