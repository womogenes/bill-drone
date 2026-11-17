"""
Drone 1's sim scorecard: how well a plant predicts every flight we have recorded, as one number and per flight: uv run -m sw.sim.scorecard [--plant airframe_2026-11-12_raised.json ...] [--set k=v,...] [--flights NAME ...] [--kinds hover_calm,...] [--workers 8] [--json out.json] [--compare base.json] [--figures DIR] [--history] [--inventory] [--firmware]

The rule for plant changes: a plant change must lower the total without worsening any single flight's score beyond noise (--compare base.json flags a flight whose score rises by more than twice its paired block-jackknife standard error, or by 0.05 where it is too short for one). Score held-out flights apart from the ones a plant was fitted on (the "held out" total; scorecard_flights.json "training").

The standard (the user's): at every time step we have all the sensor readings and the log remembers the commands; predict the future. So for start points t every --every s (0.1) of each flight:
- The hidden state (the servo horns' and assemblies' angles and rates, the motors' thrusts, the command history the dead times read) is reconstructed by running the plant over the --warmup s (1) before t on the logged throttles and servo commands, the body's rates, attitude, velocity and height set to the log's every tick, so at t the actuators are already moving as the commands moved them.
- From the log's state at t (gyro, roll and pitch, the Kalman filter's velocity turned into the heading-free frame, its height above where it last rested) the plant flies on alone on the logged commands, and is compared with the log at the horizons 0.05, 0.2, 0.5 and 1 s: the gyro (deg/s, each tick's mean rotation as rigid.py observes it, clipped at the recording's full scale), roll and pitch (deg, against the firmware's estimate), the specific force (g, rigid.py's accelerometer with the configuration's bias, clipped at the accelerometer's ±2 g) and the height change (m, the filter's, where it is good and not below the ground).
- The log's gyro and accelerometer are compared through a zero-phase 15 Hz low-pass (the rotors' vibration, which no plant models, would otherwise set a floor under every short horizon); the start state takes the raw gyro. Targets where the gyro or the accelerometer clipped, and the attitude for 2 s after a gyro clip, are left out, and so is any start whose warm-up or horizon spans samples the link lost.
With --firmware the firmware flies the horizon (the scorecard to judge plants by): past the start tick the plant flies the logged commands plus what the firmware's own attitude controller (flight.Controller, with INDI and the servos' predictor as the flight's settings set them) changes in them for the plant's departure from the log, on the logged targets and collective, so a prediction answers its own errors as the drone's controller answered the drone's instead of letting the airframe's unstable pitch-speed mode grow them (open loop every plant is worse than do-nothing past 0.5 s). The controller's state at each start (integrators, filters, INDI's actuator history) comes from one replay of the firmware over the whole log on the logged gyro and attitude; flying the change against that replay cancels what it does not reproduce (the pack's voltage compensation, older firmware: the replay's throttles sit 25-40 below the logged, its servos within 0.4-1.6 degrees), and a plant that tracks the log flies the logged commands.
All start points of a flight fly at once as the rigid.Sim's lanes (the C backend): every flight on three plants scores in under a minute on a laptop (--workers per flight).

Which flights (--inventory writes models/scorecard/inventory.json): every drone 1 recording in sw/recordings inside a configuration's dates whose motors ran (both throttles over 300) and that was airborne for 0.3 s or more, unless scorecard_flights.json excludes it with a reason; the rest are listed there with theirs. A configuration is what changed physically between sessions: the bands (preload, tilt offset and accelerometer bias), the thrust table (the old ESCs' firmware table, flight.thrust_table, or the 2026-11-14 stand's for the Nuofany ESCs), what it rests on, and for the drone before its pivots were raised the geometry (--history scores those apart: their configurations' geometry, --plant era each one's model of its time).

Each flight's own terms, the same for every plant: the thrust trim (each lane's: the median table thrust of the 5 s of airborne ticks before it carries the weight, so the pack's sag is followed; the plant's level and split are the flight's median), and its CoM forward and left of the plant's and a steady yaw push (flight_terms: the battery is reseated between flights; a CoM's torque scales with the thrust as the flights climb and descend). A flight with too little hover for them takes the nearest flight's in its configuration. A plant's JSON may also carry "configurations": {name: {term: value}} for terms it has per configuration.

The score: each channel's RMS error at each horizon over a flight's start points, over the RMS of the do-nothing prediction on the same start points (the log's own change over the horizon), averaged within the channel groups (rates, attitude, specific force, height where logged), then over the groups and the horizons. 1 is as good as predicting that nothing changes; 0 is perfect. Hovers held by the controller change little, so open-loop predictions of them score near 1 even when they follow the shake well. A kind's score is the mean of its flights'; takeoffs (the first 2.5 s with the motors on), landings and cuts (the last 2.5 s, as the flight ended by landing or by a stop or the tip cut) of the whole-flight entries are pooled into kinds of their own, and their starts are left out of the flights' scores; hops (under 5 s airborne), divergences and the windowed bursts are scored whole. The total is the mean over the kinds, so many hovers do not outweigh the few bursts. Each flight's noise is its score's standard error by a block jackknife over 2 s blocks of start points.

--figures DIR draws every scored flight (plot: the log against each plant's predictions made 0.2 s and 1 s earlier, from start points every 0.02 s) and writes DIR/index.html and DIR/contact_sheet.png. sw/sim/scorecard_closed.py is the closed-loop check (the firmware flying the flights' settings on the plant).

What it does not model: the firmware's attitude estimator (the plant's true roll and pitch are compared with the logged estimate; in a steady bank the complementary filter's pull toward the accelerometer makes them differ), the IMU's offset from the CoM (the specific force is the CoM's), wind (the windy kinds measure it as error).
"""

import argparse
import ctypes
import json
import math
import time as clock
from array import array
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

from sw.flight_log import FIRST_DRONE, RECORDINGS, drone, events, parse_settings, read_flight, recording, settings
from sw.sim import airframe as A
from sw.sim import flight as F
from sw.sim import rigid as R
from sw.sim.flip_esc import STAND
from sw.sim.sweep import FEET
from sw.sim.takeoff_test import LEG_FEET

FLIGHTS = Path(__file__).with_name("scorecard_flights.json")
# The bands' preload as the raised pivots' fit has it (N·m) and their tilt offset (degrees), for plants that do not carry their own.
BANDS = {"left_band": 0.07189, "right_band": 0.003407}
BAND_TILT = -2.5
PLANTS = ("airframe_2026-11-16b_plant.json", "airframe_2026-11-16_plant.json", "airframe_2026-11-15_plant.json", "airframe_2026-11-12_raised.json")
HORIZONS = (0.05, 0.2, 0.5, 1.0)
CHANNELS = ("roll_rate", "pitch_rate", "yaw_rate", "roll", "pitch", "accel_x", "accel_y", "accel_z", "height")
UNITS = ("deg/s", "deg/s", "deg/s", "deg", "deg", "g", "g", "g", "m")
GROUPS = {"rates": (0, 1, 2), "attitude": (3, 4), "accel": (5, 6, 7), "height": (8,)}
# The bands' accelerometer error (g, body forward) as flip_firmware.py has it.
BAND_ACCEL = 0.025
# Both motors over this throttle: the motors on.
ON_THROTTLE = 300
# A drone with the motors on is airborne from this height (m, the filter's) up; without a good height, once the motors are on.
AIRBORNE_HEIGHT = 0.1
# The filter's height and velocity are used while their standard deviations are under these (m, m/s).
HEIGHT_SD, VELOCITY_SD = 0.5, 0.5
# Takeoffs and landings: the first and last this many seconds with the motors on.
PHASE = 2.5
# Kinds scored over all their airborne start points (windows and short flights), with no takeoff or landing split off.
WHOLE = ("burst", "divergence", "hop")
# The log's gyro and accelerometer are compared through this low-pass (Hz, zero phase).
TARGET_HZ = 15.0
# The trim is the median table thrust over this many seconds of airborne ticks before each start (at least 1 s of them, or the flight's).
TRIM_S = 5.0
# The flight's own CoM and yaw push zero the mean rate error this far ahead (s), over at least this many hovering start points (0.2 s apart).
TERMS_S = 0.1
TERMS_STARTS = 10
# Plots predict from start points this often (s).
PLOT_EVERY = 0.02
# Jackknife blocks (s of start points) and the --compare flag's threshold (paired standard errors).
BLOCK_S, NOISE = 2.0, 2.0
# What the drone rests on by configuration: rigid.HULL's four belly points become these (relative to the CoM), and the height at rest puts them on the ground. legs: the forward legs since 2026-11-11 (takeoff_test.LEG_FEET); belly: legless, 2026-11-07 to 11-10 (rigid.HULL as it is); feet: the first legs (sweep.FEET).
HULL = np.array(R.HULL)
RESTS = {"legs": LEG_FEET[:4], "belly": [tuple(point) for point in HULL[:4]], "feet": FEET}
# Where a flight without a good height flies (m): clear of the ground.
NO_HEIGHT_Z = 10.0
# Stops that are landings; any other ending with the motors on is a cut (the pilot's stop, the tilt limit).
LANDED = ("Landed", "Landed after link loss")
INVENTORY = A.MODELS / "scorecard" / "inventory.json"
# A flight shorter than this airborne (s) is a hop unless the list gives its kind.
HOP_S = 5.0


def flight_list(path=FLIGHTS):
    return json.loads(Path(path).read_text())


def airborne_mask(on, height, vertical):
    """Airborne ticks: the motors on and, with the filter's height, above AIRBORNE_HEIGHT; without one (before 2026-11-09), from the liftoff (the vertical specific force past 1.12 g with the motors on) to the motors' stop."""
    if np.isfinite(height[on]).any():
        return on & ~(height < AIRBORNE_HEIGHT)
    lifted = on & (vertical > 1.12)
    if not lifted.any():
        return np.zeros_like(on)
    out = on.copy()
    out[: np.argmax(lifted)] = False
    return out


def configuration_of(name, listing):
    """The configuration a recording name falls in (by from-until), or None."""
    for key, c in listing["configurations"].items():
        if c["from"] <= name < c["until"]:
            return key
    return None


def classify(path, listing):
    """One recording's inventory row: what it is, whether it is scored (scored, history or excluded) and why not."""
    name = path.name[:19]
    row = {"name": name, "configuration": configuration_of(name, listing)}
    try:
        flown_by = drone(path)
        df = read_flight(path)[0]
    except Exception as error:
        return row | {"status": "excluded", "reason": f"unreadable ({type(error).__name__})"}
    if len(df) < 10:
        return row | {"status": "excluded", "reason": "empty log"}
    if flown_by != FIRST_DRONE:
        return row | {"status": "excluded", "reason": f"another drone ({flown_by})"}
    time = (df["esp_time_us"] - df["esp_time_us"].iloc[0]).to_numpy() / 1e6
    on = ((df["left_throttle"] > ON_THROTTLE) & (df["right_throttle"] > ON_THROTTLE)).to_numpy()
    height = np.where(df["ekf_height_sd_m"].to_numpy(float) < HEIGHT_SD, df["ekf_height_m"].to_numpy(float), np.nan)
    airborne = airborne_mask(on, height, df["az_g"].to_numpy())
    lost = int(np.clip(np.diff(df["sequence"].to_numpy()) - 1, 0, None).sum())
    stops = [event.get("reason") for event in events(path) if event["type"] == "stop"]
    row |= {"seconds": round(float(time[-1]), 1), "motors_on_s": round(on.sum() * A.DT, 1), "airborne_s": round(airborne.sum() * A.DT, 1), "height_known": bool(np.isfinite(height[on]).any()) if on.any() else False,
            "max_height_m": round(float(np.nanmax(height)), 2) if np.isfinite(height).any() else None, "lost_samples": lost, "stop": stops[-1] if stops else None,
            "max_tilt_deg": round(float(np.degrees(np.arccos(np.clip(np.cos(np.radians(df["roll_deg"])) * np.cos(np.radians(df["pitch_deg"])), -1, 1))).max()), 1)}
    if np.median(np.diff(time)) > 1.5 * A.DT:
        return row | {"status": "excluded", "reason": f"logged at {1 / np.median(np.diff(time)):.0f} Hz, not the control rate"}
    if name in listing.get("excluded", {}):
        return row | {"status": "excluded", "reason": listing["excluded"][name]}
    if row["configuration"] is None:
        return row | {"status": "excluded", "reason": "outside every configuration's dates"}
    if listing["configurations"][row["configuration"]].get("excluded"):
        return row | {"status": "excluded", "reason": listing["configurations"][row["configuration"]]["excluded"]}
    if not on.any():
        return row | {"status": "excluded", "reason": "motors never both above 300 (arming, bench or link test)"}
    if on.sum() * A.DT < 1.0:
        return row | {"status": "excluded", "reason": f"motors on {on.sum() * A.DT:.1f} s (stopped or tipped at takeoff)"}
    if airborne.sum() * A.DT < 0.3:
        return row | {"status": "excluded", "reason": f"airborne under 0.3 s (motors on {on.sum() * A.DT:.1f} s, at most {row['max_height_m']} m up)"}
    history = listing["configurations"][row["configuration"]].get("history", False)
    return row | {"status": "history" if history else "scored", "reason": ""}


def inventory(listing, workers=1, path=INVENTORY):
    """Every recording in sw/recordings classified (classify), written to models/scorecard/inventory.json."""
    paths = sorted(RECORDINGS.glob("2026-*.bin"))
    if workers > 1:
        with ProcessPoolExecutor(workers) as pool:
            rows = list(pool.map(classify, paths, [listing] * len(paths)))
    else:
        rows = [classify(p, listing) for p in paths]
    others = sorted(p.name for p in RECORDINGS.iterdir() if not p.name.startswith("2026-"))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"recordings": rows, "not_flight_logs": others}, indent=1))
    return rows


def entries(listing, history=False, path=INVENTORY):
    """The flights to score: every inventory row scored (or history), with its configuration's kind, a hop's if short, or the list's own entry (kind, window, note)."""
    rows = json.loads(Path(path).read_text())["recordings"]
    special = {f["name"]: f for f in listing["flights"]}
    out = []
    for row in rows:
        if row["status"] != ("history" if history else "scored"):
            continue
        configuration = listing["configurations"][row["configuration"]]
        kind = configuration["kind"] if row["airborne_s"] >= HOP_S or history else "hop"
        out.append({"name": row["name"], "kind": kind, "configuration": row["configuration"]} | special.get(row["name"], {}))
    return out


def named(name):
    """The scored flight whose recording name starts with name: its entry and its Flight."""
    listing = flight_list()
    entry = next(e for e in entries(listing) if e["name"].startswith(name))
    return entry, Flight(entry, listing["configurations"][entry["configuration"]])


class Flight:
    """A whole recording as arrays at the log's 100 Hz: commands, the start states the warm-up pins and the targets the predictions are scored against."""

    def __init__(self, entry, configuration):
        self.entry, self.name, self.kind = entry, entry["name"], entry["kind"]
        self.configuration = configuration
        path = recording(self.name)
        df = read_flight(path)[0].reset_index(drop=True)
        # The IMU's full scales as the recording was flown (its metadata; ±250 deg/s and ±2 g before it said).
        meta = json.loads(path.with_suffix(".jsonl").read_text().splitlines()[0])
        self.gyro_range = 32768 / meta.get("gyro_counts_per_deg_s", 131.0) * A.DEG
        self.accel_range = 32768 / meta.get("accel_counts_per_g", 16384.0)
        self.time = (df["esp_time_us"] - df["esp_time_us"].iloc[0]).to_numpy() / 1e6
        self.gyro = df[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].to_numpy() * A.DEG
        self.throttle = df[["left_throttle", "right_throttle"]].to_numpy(float)
        s = settings(path)
        trims = np.array([s.get("left_trim", 0.0), s.get("right_trim", 0.0)])
        directions = np.array([F.MODEL["left_servo_direction"], F.MODEL["right_servo_direction"]])
        self.tilt = np.radians(directions * (df[["left_servo_deg", "right_servo_deg"]].to_numpy(float) - trims))
        self.attitude = df[["roll_deg", "pitch_deg"]].to_numpy() * A.DEG
        self.quat = np.array([F.from_euler(*a) for a in self.attitude])
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
        self.airborne = airborne_mask(self.on, self.height, self.accel[:, 2])
        self.clipped = (np.abs(self.gyro) >= 0.97 * self.gyro_range).any(1)
        self.accel_clipped = (np.abs(self.accel) >= 0.97 * self.accel_range).any(1)
        # The attitude estimate is left out for 2 s after the gyro clipped.
        after = np.convolve(self.clipped.astype(float), np.ones(200), "full")[: len(self.time)] > 0
        self.attitude_good = ~after
        stops = [event.get("reason") for event in events(path) if event["type"] == "stop"]
        self.landed = bool(stops) and stops[-1] in LANDED
        self.rest = np.array(RESTS[configuration.get("hull", "legs")])
        self.rest_z = -self.rest[:, 2].min()
        on = np.flatnonzero(self.on)
        self.first_on, self.last_on = (self.time[on[0]], self.time[on[-1]]) if len(on) else (math.inf, -math.inf)
        # The table thrusts (N, before the plant's right gain) of the hovering ticks: the motors on, airborne, no flip.
        self.table = STAND if configuration["table"] == "stand" else np.array(F.thrust_table(path), float)
        self.hovering = self.airborne & (self.flip == 0)

    def starts(self, every, warmup, horizon, windowed=True):
        """The start indices with each one's phase (the flight's kind, "takeoff" or "landing"); windowed False ignores the entry's window."""
        step = max(1, round(every / A.DT))
        first = round(warmup / A.DT) + 1
        index = np.arange(first, len(self.time) - 1, step)
        # Short flights are scored from the motors' start (their liftoff is the test), the rest while airborne.
        flying = self.on if self.kind in WHOLE and not self.entry.get("window") else self.airborne
        index = index[flying[index] & ~self.clipped[index] & ~self.clipped[np.maximum(index - 10, 0)]]
        index = index[self.gaps[np.minimum(index + horizon, len(self.time) - 1)] == self.gaps[index - first + 1]]
        window = self.entry.get("window") if windowed else None
        if window:
            index = index[(self.time[index] >= window[0]) & (self.time[index] <= window[1])]
        if self.kind in WHOLE or window:
            return index, np.array([self.kind] * len(index), dtype=object)
        t = self.time[index]
        phase = np.where(t - self.first_on < PHASE, "takeoff", np.where(self.last_on - t < PHASE, "landing" if self.landed else "cut", self.kind)).astype(object)
        # Takeoffs, landings and cuts need the height (the ground is in their horizon).
        keep = (phase == self.kind) | ~np.isnan(self.height[index])
        return index[keep], phase[keep]

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


def airframe_for(name, bands, overrides, configuration=None):
    """An airframe (a models/ file, a path or the parameters themselves) with the bands' preload (the plant's own, else BANDS; none with the bands off; as the plant has it for "plant"), the plant's own terms for the named configuration and the overrides, a side-paired term set alone setting both sides."""
    params = dict(name) if isinstance(name, dict) else A.model(name)
    own = {"left_band": params.get("left_band", 0.0), "right_band": params.get("right_band", 0.0)}
    preload = {} if bands == "plant" else (own if any(own.values()) else BANDS) if bands else {"left_band": 0.0, "right_band": 0.0}
    params = params | preload | params.get("configurations", {}).get(configuration, {}) | overrides
    for key, other in (("servo_delay", "right_servo_delay"), ("servo_wn", "right_servo_wn")):
        if key in overrides and other not in overrides:
            params[other] = overrides[key]
    return params


def plant_for(flight, name, overrides):
    """The rigid.Sim plant for a flight: the airframe (airframe_for) with the configuration's bands and table, the plant's own per-configuration terms, the overrides, and the flight's thrust trim."""
    if name == "era":
        name = flight.configuration.get("plant", PLANTS[0])
    configuration = flight.configuration
    bands = configuration["bands"]
    params = airframe_for(name, bands, overrides, flight.entry["configuration"])
    if configuration.get("geometry"):
        params["geometry"] = configuration["geometry"]
    geometry = A.geometry_of(params)
    hover = A.thrust(flight.throttle[flight.hovering], flight.table) * np.array([1.0, params.get("right_thrust_gain", 1.0)])
    level, mean = A.trim_for(hover, geometry)
    params |= {"thrust_level": float(level), "hover_thrust": [float(m) for m in mean]}
    on = bands is True
    return {"airframe": params, "geometry": geometry, "observer": params.get("observer"), "disturbances": [], "rotor_drag": tuple(params.get("rotor_drag", (0.07, 0.2))), "tilt_offset": BAND_TILT if on else 0.0,
            "accel_bias": (BAND_ACCEL if on else 0.0, 0.0, 0.0), "table": flight.table}


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


def make_sim(plant, n, rest):
    """A rigid.Sim of n lanes resting on these four points (rigid.HULL's belly replaced), no gyro bias."""
    R.HULL = np.concatenate([rest, HULL[4:]])
    sim = R.Sim(n, plant, seed=0)
    sim.gyro_bias[:] = 0.0
    return sim


def with_terms(plant, terms):
    """The plant with a flight's own terms: its CoM moved forward and left by terms[0] and terms[1] (m; the pitch and roll torques then scale with the thrust, as a CoM's do) and terms[2] added to its steady yaw push (rad/s²)."""
    out = dict(plant)
    out["airframe"] = dict(plant["airframe"]) | {"com_x": plant["airframe"].get("com_x", A.PARAMS["com_x"][2]) + terms[0], "yaw_push": plant["airframe"].get("yaw_push", 0.0) + terms[2]}
    com = list(plant["geometry"]["com"])
    com[1] += terms[1]
    out["geometry"] = dict(plant["geometry"]) | {"com": tuple(com)}
    return out


def flight_terms(flight, plant, warmup, every=0.2, rounds=3):
    """
    The flight's own CoM (forward and left of the plant's, m) and steady yaw push (rad/s²): the battery and the deck move between flights, and a CoM's torque scales with the thrust where a constant push would not (the flights climb and descend). Over the flight's hovering start points (no takeoff, landing or flip) they zero the mean error of the rates TERMS_S s ahead, by rounds of the CoM's own lever: a CoM dx forward pitches the nose down by dx T / Iyy, dy left rolls it left by dy T / Ixx. None for a flight with under TERMS_STARTS such start points: run gives it the nearest flight's in time within its configuration (the battery was reseated between flights: 2026-11-15 06:29 and 06:30 UTC hovered with the CoM about centered, 06:32 on about 7 mm left).
    """
    index, _ = flight.starts(every, warmup * A.DT, 0, windowed=False)
    t = flight.time[index]
    index = index[(flight.flip[index] == 0) & (t - flight.first_on > PHASE) & (flight.last_on - t > PHASE)]
    terms = np.zeros(3)
    if len(index) < TERMS_STARTS:
        return None
    ticks = round(TERMS_S / A.DT)
    weight = plant["geometry"]["mass"] * A.G
    resolved = A.resolve(A.complete(plant["airframe"]), plant["geometry"])
    for _ in range(rounds):
        predicted = predict(flight, with_terms(plant, terms), index, warmup, ticks)
        target, _ = flight.targets(index, np.array([ticks]))
        bias = np.nanmean(predicted[:, ticks - 1, :3] - target[:, 0, :3], 0) * A.DEG / ((ticks - 0.5) * A.DT)
        # Roll torque -dy T (rigid_step.c), pitch torque +dx T: an angular acceleration bias b is taken back by dy = b Ixx / T, dx = -b Iyy / T.
        terms = terms + np.array([-bias[1] * resolved["Iyy"] / weight, bias[0] * resolved["Ixx"] / weight, -bias[2]])
    return terms


# --firmware: the firmware flies the horizon (main sets it before the workers start).
FIRMWARE = False
CONTROL = {key: 6 + F.TUNING_LAYOUT.index(key) for key in ("base_throttle", "roll", "pitch", "yaw_rate", "left_thrust_scale", "right_thrust_scale")}
DIRECTIONS = np.array([F.MODEL["left_servo_direction"], F.MODEL["right_servo_direction"]])
replays = {}


def firmware(flown):
    """The flight's attitude controller and INDI as its settings configure them, reset."""
    controller = F.Controller()
    controller.configure(flown)
    controller.command[F.YAW_I] = flown.get("yaw_i", 0.0)
    for side in ("left", "right"):
        controller.command[CONTROL[f"{side}_thrust_scale"]] = flown.get(f"{side}_thrust_scale", 1.0)
    controller.reset()
    return controller, F.Incremental(flown)


def fly(controller, indi, flown, attitude, gyro, q, targets, base, holding, height, last_base):
    """One firmware tick (flight_rt.c fly's attitude part): INDI's update, the controller, what was sent. Returns the throttles and the servos' physical forward tilts (rad)."""
    command = controller.command
    command[CONTROL["base_throttle"]], command[CONTROL["roll"]], command[CONTROL["pitch"]], command[CONTROL["yaw_rate"]] = base, *targets
    if indi.on:
        indi.update(gyro, holding, height, last_base)
        indi.apply(controller)
    left, right, left_angle, right_angle = controller.update(attitude, gyro, A.DT, q)
    throttles = np.array([round(min(max(left, 0.0), 1200.0)), round(min(max(right, 0.0), 1200.0))], float) * (base > 0)
    tilt = DIRECTIONS * (np.array([left_angle, right_angle]) - np.radians([flown.get("left_trim", 0.0), flown.get("right_trim", 0.0)]))
    if indi.on:
        indi.sent(list(tilt), [F.thrust(t) * command[CONTROL[f"{side}_thrust_scale"]] if t > 0 else 0.0 for side, t in zip(("left", "right"), throttles)])
    return throttles, tilt


def replay(flight):
    """The firmware over the whole log on its gyro, attitude, targets and collective: after each tick its controller's and INDI's state and its commands."""
    if flight.name in replays:
        return replays[flight.name]
    path = recording(flight.name)
    flown = settings(path) | {"enabled": 1}
    df = read_flight(path)[0].reset_index(drop=True)
    targets = np.column_stack([np.radians(df[c].fillna(0).to_numpy(float)) for c in ("target_roll_deg", "target_pitch_deg", "target_yaw_deg_s")])
    base = df["base_throttle"].fillna(0).to_numpy(float)
    height = np.nan_to_num(flight.height)
    controller, indi = firmware(flown)
    n = len(flight.time)
    out = {"flown": flown, "targets": targets, "base": base, "height": height, "states": np.zeros((n, len(controller.state))), "indis": [], "fades": np.zeros(n), "throttles": np.zeros((n, 2)), "tilts": np.zeros((n, 2))}
    for t in range(n):
        out["throttles"][t], out["tilts"][t] = fly(controller, indi, flown, tuple(flight.attitude[t]), tuple(flight.gyro[t]), tuple(flight.quat[t]), targets[t], base[t], bool(flight.airborne[t]), height[t], base[max(t - 1, 0)])
        out["states"][t] = controller.state
        out["indis"].append(bytes(indi.state))
        out["fades"][t] = indi.fade
    replays[flight.name] = out
    return out


def predict(flight, plant, index, warmup, horizon):
    """The plant's predictions [lane, step, channel] for steps 1..horizon ticks after each start index, its hidden state warmed up over the warmup ticks before it with the body pinned to the log."""
    n = len(index)
    steps = np.arange(-warmup + 1, horizon + 1)
    # The commands a tick flies are the ones logged the tick before: the firmware reads the gyro, computes and sends the servos and motors, then records the tick (flight_rt.c fly, then capture), so the record at k holds what acts from t_k on, after the rates it was computed from (the step that ends at k flies k - 1's; until 2026-11-15 this used k's, a tick early, and the delays fitted that way are 10 ms longer than the closed loop's).
    throttle = local_throttle(flight, plant, index, steps - 1)
    k = np.clip(index[:, None] + steps[None], 0, len(flight.time) - 1)
    tilt = flight.tilt[np.clip(k - 1, 0, None)]
    sim = make_sim(plant, n, flight.rest)
    p = sim.p
    gain = np.tile([1.0, p["right_thrust_gain"]], (n, 1))
    pin = k[:, 0] - 1
    motor = A.actual_thrust(p, A.table_thrust(p, throttle[:, 0], np.ones(n), table=sim.table, gain=gain), sim.trim)
    position = np.column_stack([np.zeros((n, 2)), np.nan_to_num(flight.height[pin] + flight.rest_z, nan=NO_HEIGHT_Z)])
    sim.reset(position, flight.velocity[pin], flight.quat[pin], flight.gyro[pin], motor=motor, tilt=tilt[:, 0] - sim.tilt_offset)
    sim.throttles[:] = throttle[:, :1]
    sim.bind()
    out = np.full((n, horizon, len(CHANNELS)), np.nan)
    # The disturbance state (plant["observer"]: {"correlation": s, "sd": rad/s², "noise": rad/s²}, or the older fixed-gain {"filter": s, "correlation": s}): a torque the plant does not explain, wandering as a first-order (Ornstein-Uhlenbeck) process with that correlation time, estimated over the warm-up from the log alone (each tick's measured angular acceleration less the plant's from the same pinned state, exponentially averaged over filter s; ticks where the gyro clipped, on the ground or touching it are skipped) and carried into the prediction decaying as exp(-t / correlation).
    observer = plant.get("observer")
    estimate, variance = np.zeros((n, 3)), np.zeros((n, 3))
    for j, step in enumerate(steps):
        if step <= 1:
            # Pinned to the log's state before this tick's commands (the warm-up, and the start itself).
            before = k[:, j] - 1
            sim.omega[:] = flight.gyro[before]
            sim.quat[:] = flight.quat[before]
            sim.vel[:] = flight.velocity[before]
            sim.pos[:, 2] = np.where(np.isnan(flight.height[before]), sim.pos[:, 2], flight.height[before] + flight.rest_z)
            if step == 1:
                z0 = sim.pos[:, 2].copy()
        if observer:
            sim.torque_bias[:] = estimate * math.exp(-(step - 1) * A.DT / observer["correlation"]) if step >= 1 else 0.0
        if FIRMWARE and step >= 2:
            # The start tick flies the logged commands (computed from the start state); from the next, each lane's firmware, as the replay left it after the start tick, answers the plant's own state.
            if step == 2:
                played = replay(flight)
                lanes = [firmware(played["flown"]) for _ in index]
                for (controller, indi), start in zip(lanes, index):
                    controller.state[:] = array("f", played["states"][start])
                    ctypes.memmove(ctypes.byref(indi.state), played["indis"][start], ctypes.sizeof(indi.state))
                    indi.fade = played["fades"][start]
            up = R.rotation(sim.quat)[:, 2, :]
            roll, pitch = np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))
            t = np.clip(index + step - 1, 0, len(flight.time) - 1)
            moved = np.zeros((n, 2)), np.zeros((n, 2))
            for i, (controller, indi) in enumerate(lanes):
                moved[0][i], moved[1][i] = fly(controller, indi, played["flown"], (roll[i], pitch[i]), tuple(sim.turned[i]), tuple(sim.quat[i]), played["targets"][t[i]], played["base"][t[i]], bool(flight.airborne[t[i]]), played["height"][t[i]], played["base"][max(t[i] - 1, 0)])
            sim.step(np.where(throttle[:, j] > 0, np.clip(throttle[:, j] + moved[0] - played["throttles"][t], 0, 1200), 0.0), tilt[:, j] + moved[1] - played["tilts"][t])
        else:
            sim.step(throttle[:, j], tilt[:, j])
        if observer and step <= 0:
            measured = (flight.gyro[k[:, j]] - flight.gyro[before]) / A.DT
            usable = ~(flight.clipped[k[:, j]] | flight.clipped[before]) & flight.airborne[k[:, j]] & flight.airborne[before] & (sim.contact_now == 0)
            residual = measured - (sim.omega - flight.gyro[before]) / A.DT
            if "sd" in observer:
                # A Kalman filter on the process: the estimate starts at its mean (zero) with its whole variance sd² when the drone is not flying, and each flying tick's residual is a measurement of it with the noise's variance (noise², per axis: the gyro's vibration differenced).
                keep = math.exp(-A.DT / observer["correlation"])
                prior = np.asarray(observer["sd"], float) ** 2
                variance = np.where(usable[:, None], keep ** 2 * variance + prior * (1 - keep ** 2), prior)
                estimate = np.where(usable[:, None], keep * estimate, 0.0)
                gain = variance / (variance + np.asarray(observer["noise"], float) ** 2)
                estimate = np.where(usable[:, None], estimate + gain * (residual - estimate), estimate)
                variance = np.where(usable[:, None], (1 - gain) * variance, variance)
            else:
                gain = A.DT / observer["filter"]
                estimate = np.where(usable[:, None], estimate + gain * (residual - estimate), estimate)
        if step >= 1:
            up = R.rotation(sim.quat)[:, 2, :]
            roll, pitch = np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))
            rates = np.clip(sim.turned, -flight.gyro_range, flight.gyro_range)
            out[:, step - 1] = np.column_stack([rates / A.DEG, roll / A.DEG, pitch / A.DEG, np.clip((sim.accel + sim.accel_bias) / A.G, -flight.accel_range, flight.accel_range), sim.pos[:, 2] - z0])
    return out


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def score_flight(task):
    """One flight on one plant: per start, the errors at the horizons [start, horizon, channel] and the do-nothing prediction's, with the starts' phases and times."""
    entry, configuration, plant_name, overrides, every, warmup, extra = task
    began = clock.time()
    flight = Flight(entry, configuration)
    horizon = round(max(HORIZONS) / A.DT)
    index, phase = flight.starts(every, warmup, horizon)
    ticks = np.array([round(h / A.DT) for h in HORIZONS])
    if len(index) == 0:
        return {"name": flight.name, "kind": flight.kind, "error": np.zeros((0, len(HORIZONS), len(CHANNELS))), "naive": np.zeros((0, len(HORIZONS), len(CHANNELS))), "phase": phase, "time": np.zeros(0), "reach": np.zeros((0, 2)), "terms": None, "seconds": 0.0}
    plant = plant_for(flight, plant_name, overrides)
    predicted = predict(flight, with_terms(plant, extra) if extra is not None else plant, index, round(warmup / A.DT), horizon)
    target, start = flight.targets(index, ticks)
    error = predicted[:, ticks - 1] - target
    naive = start[:, None] - target
    for c in (3, 4):
        error[..., c], naive[..., c] = wrap(error[..., c]), wrap(naive[..., c])
    # How far each start's horizon goes past the plant's clipped ranges: the largest tilt (deg) and body-plane speed (m/s) the log flew in it.
    k = np.clip(index[:, None] + np.arange(horizon + 1)[None], 0, len(flight.time) - 1)
    reach = np.column_stack([np.degrees(flight.tilt_angle[k].max(1)), flight.plane_speed[k].max(1)])
    return {"name": flight.name, "kind": flight.kind, "error": error, "naive": naive, "phase": phase, "time": flight.time[index], "reach": reach, "terms": extra, "seconds": clock.time() - began}


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
            out.append({"name": r["name"], "kind": r["kind"], "error": r["error"][own], "naive": r["naive"][own], "time": r["time"][own], "reach": r["reach"][own], "terms": r.get("terms")})
    for kind in ("takeoff", "landing", "cut"):
        parts = [(r["error"][r["phase"] == kind], r["naive"][r["phase"] == kind], r["time"][r["phase"] == kind] + 1000.0 * i) for i, r in enumerate(results) if (r["phase"] == kind).any()]
        if parts:
            out.append({"name": f"all {kind}s", "kind": kind, "error": np.concatenate([p[0] for p in parts]), "naive": np.concatenate([p[1] for p in parts]), "time": np.concatenate([p[2] for p in parts]),
                        "reach": np.concatenate([r["reach"][r["phase"] == kind] for r in results if (r["phase"] == kind).any()]), "terms": None})
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


def terms_of(task):
    """flight_terms for one (entry, configuration, plant, overrides, warmup)."""
    entry, configuration, plant_name, overrides, warmup = task
    flight = Flight(entry, configuration)
    return flight_terms(flight, plant_for(flight, plant_name, overrides), round(warmup / A.DT))


def nearest_terms(entries, found):
    """Each entry's own terms by name from flight_terms' results (found, in order), one too short for them (None) taking the nearest flight's in time within its configuration, or none."""
    known = [(e["name"], e["configuration"], t) for e, t in zip(entries, found) if t is not None]
    stamp = lambda name: clock.mktime(clock.strptime(name[:19], "%Y-%m-%dT%H-%M-%S"))
    out = {}
    for e, t in zip(entries, found):
        if t is None:
            near = sorted((abs(stamp(n) - stamp(e["name"])), n, own) for n, c, own in known if c == e["configuration"])
            t = near[0][2] if near else np.zeros(3)
        out[e["name"]] = t
    return out


def run(plant, overrides, entries, configurations, every, warmup, own_terms, workers):
    """Every entry scored on the plant: first each flight's own terms (flight_terms; a flight too short for them takes the nearest one's in time within its configuration, or none), then its predictions. Returns the results and the terms by flight."""
    pool = ProcessPoolExecutor(workers) if workers > 1 else None
    mapper = pool.map if pool else map
    terms = nearest_terms(entries, list(mapper(terms_of, [(e, configurations[e["configuration"]], plant, overrides, warmup) for e in entries]))) if own_terms else {}
    tasks = [(e, configurations[e["configuration"]], plant, overrides, every, warmup, terms.get(e["name"])) for e in entries]
    results = list(mapper(score_flight, tasks))
    if pool:
        pool.shutdown()
    return results, terms


def save(summary, results, plant, overrides, path):
    """The JSON summary and, beside it (.npz), every start's errors for --compare."""
    out = {"plant": plant, "overrides": overrides, "total": summary["total"], "kinds": summary["kinds"], "held_out": summary.get("held_out"), "horizons": HORIZONS, "channels": CHANNELS, "units_of_channels": UNITS,
           "flights": [{"name": u["name"], "kind": u["kind"], "starts": u["count"], "score": u["score"], "noise": u["noise"], "rms": np.round(u["rms"], 4).tolist(), "trained": u.get("trained", False),
                        "flight_terms": None if u["terms"] is None else {"com_forward_mm": round(1000 * u["terms"][0], 2), "com_left_mm": round(1000 * u["terms"][1], 2), "yaw_push": round(float(u["terms"][2]), 3)}, "past_attitude_range": float(np.mean(u["reach"][:, 0] > math.degrees(R.ATTITUDE_RANGE))) if len(u["reach"]) else 0.0,
                        "past_speed_range": float(np.mean(u["reach"][:, 1] > R.SPEED_RANGE)) if len(u["reach"]) else 0.0, "largest_tilt_deg": float(u["reach"][:, 0].max()) if len(u["reach"]) else 0.0,
                        "largest_speed_m_s": float(u["reach"][:, 1].max()) if len(u["reach"]) else 0.0} for u in summary["units"]]}
    Path(path).write_text(json.dumps(out, indent=1))
    arrays = {}
    for r in results:
        for key in ("error", "naive", "time", "reach"):
            arrays[f"{r['name']}/{key}"] = r[key]
        arrays[f"{r['name']}/phase"] = r["phase"].astype(str)
    np.savez_compressed(Path(path).with_suffix(".npz"), **arrays)


def load_results(path, entries):
    data = np.load(Path(path).with_suffix(".npz"))
    kinds = {e["name"]: e["kind"] for e in entries}
    return [{"name": n, "kind": kinds[n], "error": data[f"{n}/error"], "naive": data[f"{n}/naive"], "time": data[f"{n}/time"], "reach": data[f"{n}/reach"], "phase": data[f"{n}/phase"].astype(object)} for n in kinds if f"{n}/error" in data]


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


def label_of(plant):
    return Path(plant).stem.replace("airframe_", "")


def caption(unit):
    """A unit's score and its RMS errors at 0.2 s / 1 s for a plot's title."""
    if unit is None:
        return "not scored"
    r = unit["rms"]
    h02, h1 = HORIZONS.index(0.2), HORIZONS.index(1.0)
    pairs = ", ".join(f"{name} {r[h02, c]:.0f}/{r[h1, c]:.0f}" for name, c in (("p", 0), ("q", 1), ("r", 2), ("roll", 3), ("pitch", 4)))
    return f"score {unit['score']:.2f}" + (f" ±{unit['noise']:.2f}" if not math.isnan(unit["noise"]) else "") + (" (fitted on)" if unit.get("trained") else "") + f"\nRMS at 0.2/1 s: {pairs} (deg/s, deg), height {r[h02, 8]:.2f}/{r[h1, 8]:.2f} m"


def plot(task):
    """One flight's figure: for each channel (the gyro's roll, pitch and yaw rate, roll and pitch, the height) the log (black) against every plant's predictions (a color each) made 0.2 s earlier (upper row) and 1 s earlier (lower row), from start points every PLOT_EVERY s over the entry's window (or its whole time with the motors on). The title carries each plant's score and RMS errors from the scoring run."""
    entry, configuration, plants, overrides, warmup, out, captions, terms = task
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    flight = Flight(entry, configuration)
    index, _ = flight.starts(PLOT_EVERY, warmup, 100)
    if len(index) == 0:
        return None
    window = entry.get("window") or (flight.time[index[0]], flight.time[index[-1]])
    index = index[(flight.time[index] >= window[0]) & (flight.time[index] <= window[1])]
    shown = np.flatnonzero((flight.time >= window[0]) & (flight.time <= window[1] + 1.0))
    log_values = flight.values(shown)
    channels = [(0, "roll rate\n(deg/s)"), (1, "pitch rate\n(deg/s)"), (2, "yaw rate\n(deg/s)"), (3, "roll (deg)"), (4, "pitch (deg)"), (8, "height (m)")]
    colors = ("#1f77b4", "#d62728", "#2ca02c", "#9467bd")
    predictions = {}
    for name in plants:
        plant = plant_for(flight, name, overrides)
        own = terms[name] if terms else flight_terms(flight, plant, round(warmup / A.DT))
        predictions[name] = predict(flight, with_terms(plant, own) if own is not None else plant, index, round(warmup / A.DT), 100)
    width = min(40.0, max(9.0, (window[1] - window[0] + 1.0) * 0.6))
    fig, axes = plt.subplots(2 * len(channels), 1, figsize=(width, 1.25 * 2 * len(channels) + 0.6 + 0.35 * len(plants)), sharex=True)
    for row, (c, label) in enumerate(channels):
        values = log_values[:, c]
        low, high = np.nanpercentile(values, [0.5, 99.5]) if np.isfinite(values).any() else (-1.0, 1.0)
        span = max(high - low, 1e-3)
        for line, h in enumerate((0.2, 1.0)):
            ax = axes[2 * row + line]
            ax.plot(flight.time[shown], values, color="black", lw=1.1, label="log")
            tick = round(h / A.DT)
            for name, color in zip(plants, colors):
                guess = predictions[name][:, tick - 1, c] + (flight.height[index] if c == 8 else 0.0)
                ax.plot(flight.time[index] + h, guess, color=color, lw=0.8, alpha=0.85, label=label_of(name))
            ax.set_ylim(low - 0.5 * span, high + 0.5 * span)
            ax.set_ylabel(f"{label}\n{h:g} s ahead", fontsize=7)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=7)
    axes[0].legend(fontsize=7, loc="upper left", ncol=len(plants) + 1)
    axes[-1].set_xlabel("s from the recording's start", fontsize=8)
    lines = [f"{flight.name} UTC, {entry['kind']} ({entry['configuration']}){': ' + entry['note'] if entry.get('note') else ''}. Black: the log; colors: each plant's prediction from the logged state 0.2 s (upper rows) and 1 s (lower rows) earlier, " + ("the firmware in the loop." if FIRMWARE else "on the logged commands.")]
    lines += [f"{label_of(name)}: {captions.get(name, 'not scored').replace(chr(10), '; ')}" for name in plants]
    fig.suptitle("\n".join(lines), fontsize=8, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 1 - (0.012 * (len(plants) + 1) * 14 / (1.25 * 2 * len(channels) + 0.6 + 0.35 * len(plants)))))
    fig.savefig(out, dpi=75)
    plt.close(fig)
    return str(out)


def index_page(rows, plants, directory):
    """The figures' index.html: every scored flight with each plant's score, linking its figure, and contact_sheet.png, the figures' thumbnails in one image."""
    head = "".join(f"<th>{label_of(p)}</th>" for p in plants)
    body = []
    for row in rows:
        cells = "".join(f"<td>{row['scores'].get(p, '')}</td>" for p in plants)
        link = f"<a href='{Path(row['figure']).name}'>{row['name']}</a>" if row.get("figure") else row["name"]
        body.append(f"<tr><td>{link}</td><td>{row['kind']}</td><td>{row['configuration']}</td><td>{row['airborne_s']}</td>{cells}<td>{row.get('note', '')}</td></tr>")
    page = f"""<!doctype html><html><head><meta charset="utf-8"><title>Sim scorecard</title><style>body{{font-family:sans-serif;font-size:13px}}td,th{{padding:2px 8px;border-bottom:1px solid #ddd;text-align:left}}</style></head><body>
<h1>Drone 1 sim scorecard: every scored flight</h1><p>Score per flight (sw/sim/scorecard.py): the open-loop prediction error at 0.05-1 s over the do-nothing prediction's on the same start points (1 is no better than assuming nothing changes, 0 is perfect), with its block-jackknife standard error; * marks flights the plant was fitted on.</p>
<table><tr><th>flight (UTC)</th><th>kind</th><th>configuration</th><th>airborne s</th>{head}<th>note</th></tr>{''.join(body)}</table></body></html>"""
    (Path(directory) / "index.html").write_text(page)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as image
    import matplotlib.pyplot as plt
    figures = [r for r in rows if r.get("figure")]
    columns = 6
    lines = math.ceil(len(figures) / columns)
    fig, axes = plt.subplots(lines, columns, figsize=(4 * columns, 2.6 * lines), squeeze=False)
    for ax in axes.ravel():
        ax.axis("off")
    for ax, row in zip(axes.ravel(), figures):
        ax.imshow(image.imread(row["figure"]))
        ax.set_title(f"{row['name'][5:]} {row['kind']}\n" + "  ".join(f"{label_of(p)[11:] or label_of(p)} {row['scores'].get(p, '')}" for p in plants), fontsize=7)
    fig.tight_layout()
    fig.savefig(Path(directory) / "contact_sheet.png", dpi=70)
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
    parser.add_argument("--terms", choices=("flight", "plant"), default="flight", help="plant: no per-flight CoM and yaw push (the plant must know each flight's)")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--json", help="write each plant's summary here (PLANT in the name is replaced by the plant's stem when scoring several)")
    parser.add_argument("--compare", help="a saved --json run (its .npz beside it) to compare the first plant with")
    parser.add_argument("--plot", nargs="*", help="flights to plot, NAME or NAME:START:END, instead of scoring")
    parser.add_argument("--plot-dir", default=".")
    parser.add_argument("--figures", help="after scoring, a figure per flight (every plant a column), index.html and contact_sheet.png in this directory")
    parser.add_argument("--inventory", action="store_true", help="classify every recording into models/scorecard/inventory.json instead of scoring")
    parser.add_argument("--observer", help="the disturbance state's filter and correlation times (s), FILTER,CORRELATION, over every plant's own (scorecard.predict); 0 turns it off")
    parser.add_argument("--history", action="store_true", help="score the flights before the pivots were raised (their configurations' geometry; --plant era is each one's model of its time) instead")
    parser.add_argument("--firmware", action="store_true", help="the firmware flies each horizon from its second tick (see above)")
    args = parser.parse_args()
    global FIRMWARE
    FIRMWARE = args.firmware
    listing = flight_list(args.list)
    configurations = listing["configurations"]
    if args.inventory:
        rows = inventory(listing, args.workers)
        for status in ("scored", "history", "excluded"):
            print(f"{status}: {sum(r['status'] == status for r in rows)}")
        return
    chosen = entries(listing, args.history)
    if args.flights:
        chosen = [e for e in chosen if any(e["name"].startswith(f) for f in args.flights)]
    if args.kinds:
        chosen = [e for e in chosen if e["kind"] in args.kinds.split(",")]
    overrides = parse_settings(args.set) if args.set else {}
    if args.observer:
        values = [float(v) for v in args.observer.split(",")]
        overrides["observer"] = {"filter": values[0], "correlation": values[1]} if values[0] > 0 else None
    if args.plot is not None:
        everything = {e["name"]: e for e in entries(listing) + entries(listing, True)}
        tasks = []
        for spec in args.plot:
            name, *window = spec.split(":")
            entry = dict(next(e for n, e in everything.items() if n.startswith(name)))
            if window:
                entry["window"] = [float(window[0]), float(window[1])]
            tasks.append((entry, configurations[entry["configuration"]], args.plant, overrides, args.warmup, Path(args.plot_dir) / f"{entry['name']}.png", {}, None))
        for out in map(plot, tasks):
            print(f"wrote {out}")
        return
    captions, scores, terms_by = {}, {}, {}
    for plant in args.plant:
        began = clock.time()
        results, terms = run(plant, overrides, chosen, configurations, args.every, args.warmup, args.terms == "flight", args.workers)
        terms_by[plant] = terms
        summary = summarize(results)
        for u in summary["units"]:
            u["trained"] = trained(u["name"], plant, listing)
        unseen = held_out(results, plant, listing)
        summary["held_out"] = {"total": unseen["total"], "kinds": unseen["kinds"]} if unseen else None
        print(table(summary, f"{plant} ({clock.time() - began:.0f} s)"))
        for u in summary["units"]:
            captions.setdefault(u["name"], {})[plant] = caption(u)
            scores.setdefault(u["name"], {})[plant] = f"{u['score']:.2f}{'*' if u.get('trained') else ''}"
        if args.json:
            path = args.json.replace("PLANT", Path(plant).stem) if len(args.plant) > 1 else args.json
            save(summary, results, plant, overrides, path)
        if args.compare and plant == args.plant[0]:
            base = load_results(args.compare, chosen)
            lines, worse = compare(results, base)
            print(f"against {args.compare}: total {summary['total']:.3f} vs {summarize(base)['total']:.3f}")
            print("\n".join(lines))
            print("worse beyond noise: " + (", ".join(worse) if worse else "none"))
    if args.figures:
        directory = Path(args.figures)
        directory.mkdir(parents=True, exist_ok=True)
        rows = json.loads(INVENTORY.read_text())["recordings"]
        airborne = {r["name"]: r.get("airborne_s", "") for r in rows}
        tasks = [(e, configurations[e["configuration"]], args.plant, overrides, args.warmup, directory / f"{e['name']}.png", captions.get(e["name"], {}), {p: terms_by[p].get(e["name"]) for p in args.plant}) for e in chosen]
        if args.workers > 1:
            with ProcessPoolExecutor(args.workers) as pool:
                outputs = list(pool.map(plot, tasks))
        else:
            outputs = list(map(plot, tasks))
        pages = [{"name": e["name"], "kind": e["kind"], "configuration": e["configuration"], "airborne_s": airborne.get(e["name"], ""), "note": e.get("note", ""), "scores": scores.get(e["name"], {}), "figure": out} for e, out in zip(chosen, outputs)]
        pages += [{"name": name, "kind": name.split()[-1].rstrip("s"), "configuration": "", "airborne_s": "", "scores": value} for name, value in scores.items() if name.startswith("all ")]
        index_page(pages, args.plant, directory)
        print(f"wrote {directory / 'index.html'} and {directory / 'contact_sheet.png'}")


if __name__ == "__main__":
    main()
