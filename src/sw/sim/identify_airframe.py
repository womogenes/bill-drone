"""
Identify the airframe model (sw/sim/airframe.py) from hover recordings by closed-loop multi-step prediction: uv run --with scipy -m sw.sim.identify_airframe <recordings or globs> [--start params.json] [--freeze key,...] [--ground height|time] [--folds 5] [--workers N] [--out file]

Each recording's controller is replayed from arming on its logged attitude, gyro, references, collective and battery (it reproduces the logged throttles to rounding and servo commands to 0.04°), and the airframe is run one step ahead along the logged commands for its actuator states and the flight's constant push. From every start (every 10th airborne sample) the firmware controller, with its state at that sample, then flies the airframe for up to 2 s on the logged references alone, and the fit is least squares on the predicted rates at 50 ms to 2 s ahead (over the rates' spread) and on the attitude (over the attitude change's spread at that horizon). A start counts only at the horizons that stay airborne and clear of dropped log records, so short hops and flights that diverged count too: they hold most of what there is on the stability margin. Because the controller is in the loop, a disturbance shows only as output error: fitting the plant to logged commands instead (open loop) credits the controller's reactions to the airframe. All lanes' controllers step in one C call (sw/sim/flight_many.c), and the Jacobian's columns run in parallel.
With --ground, the tilt's pitch effectiveness grows near the floor (sw/sim/airframe.py pitch_ground, ground_scale), by the logged hover height (flights without one are left out) or by the time since liftoff.
Flights with the identification test pattern (sw/excite.py) need nothing special: its target and collective steps are in the logged references, and its servo and motor steps are added back to the replayed controller's outputs from the pattern (the recording's "pattern" event) and the segment logged in each sample's flags.
The per-flight push absorbs any steady torque, so the right motor's static thrust gain is not identifiable from the motion: it is pinned to the airborne thrust split (median left over right table thrust, taking the CoM as centred between the motors, as measured), and only the motors' dead times and lags may differ.
It starts from the newest saved model (sw/sim/models/airframe_*.json), so after a hardware change a few flights refit what changed. Then each of --folds groups of flights is predicted by a fit to the others, and the held-out frame-by-frame scores (R² of the rates, attitude error and R² of the attitude change at 10 ms to 1 s) are printed, closed loop and on the logged commands.
"""

import argparse
import json
import math
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.fit import settings as recording_settings
from sw.flight_log import read_flight
from sw.sim import airframe as A
from sw.sim.flight import MODEL, MODEL_KEYS, MODEL_OFFSET, MODEL_TUNING_KEYS, TUNING_LAYOUT, Controller, control_many

DT = A.DT
DEG = A.DEG
HORIZONS = (5, 10, 25, 50, 100, 200)
SCORE_HORIZONS = (1, 5, 10, 25, 50, 100)
INDEX = {key: 6 + TUNING_LAYOUT.index(key) for key in TUNING_LAYOUT}
THRUST_VOLTAGE = MODEL_OFFSET + len(MODEL_KEYS)
VOLTAGE_TAU = 0.3  # the runtime's battery smoothing for thrust compensation (flight_rt.c)
VOLTAGE_RANGE = 0.15
MODELS = Path(__file__).with_name("models")


def paths(arguments):
    return sorted(path for argument in arguments for path in (Path(argument).parent.glob(Path(argument).name) if "*" in argument else [Path(argument)]) if path.suffix == ".bin")


def load(path, shortest=0.5):
    """
    A whole recording at the control rate, with its liftoff (vertical specific force past 1.12 g once collective reaches 600) and the end of its airborne window (an upset past 35°, an impact, motor stop or the recording ending), or None without shortest seconds airborne.
    """
    try:
        data, _ = read_flight(path)
        setting = recording_settings(path)
    except Exception:
        return None
    # Recordings logged at 50 Hz cannot be stepped at the control rate.
    if np.mean(np.abs(np.diff(data.esp_time_us.values) - 10000) > 3000) > 0.05:
        return None
    on = np.flatnonzero(data.base_throttle.values >= 600)
    if not len(on) or not (data.az_g.values[on[0]:] > 1.12).any():
        return None
    lift = on[0] + np.argmax(data.az_g.values[on[0]:] > 1.12) + 3
    shock = np.sqrt(data.ax_g ** 2 + data.ay_g ** 2 + data.az_g ** 2).values
    end = next((i for i in range(lift + 3, len(data)) if abs(data.pitch_deg[i]) > 35 or abs(data.roll_deg[i]) > 35 or shock[i] > 2.5 or data.base_throttle[i] == 0), len(data))
    if (end - lift) * DT < shortest:
        return None
    record = {column: data[column].values.astype(float) for column in data.columns if data[column].dtype.kind in "fiub"}
    record.update(name=path.name[:19], setting=setting, lift=lift, end=end)
    record["excite"] = excitation(record, path)
    return None if record["excite"] is None else record


def excitation(record, path):
    """
    The test pattern's servo and motor steps at every sample [sample, (left, right servo tilt in degrees, left, right motor throttle)], zero where it did not play; None when it played without its pattern in the recording's events.
    """
    flags = record["flags"].astype(int)
    out = np.zeros((len(flags), 4))
    active = flags >> 8 & 1
    if not active.any():
        return out
    pattern = None
    for line in path.with_suffix(".jsonl").read_text().splitlines():
        event = json.loads(line)
        if event.get("type") == "pattern":
            pattern = event
    if not pattern or not pattern["segments"]:
        print(f"{path.name}: test pattern played but is not in the events; left out", flush=True)
        return None
    segments = pattern["segments"]
    values = np.array([[segment.get(key, 0.0) for key in ("left_servo", "right_servo", "left_motor", "right_motor")] for segment in segments])
    # The flags hold the segment modulo 128; segments play in order and wrap, so take the next one with that remainder.
    index = 0
    for k in np.flatnonzero(active):
        remainder = flags[k] >> 9 & 127
        while index % 128 != remainder:
            index = (index + 1) % len(segments)
        out[k] = values[index]
    return out


def controller(record):
    """The flight's firmware controller, configured and reset, as the state and command buffers control_many takes."""
    c = Controller()
    s = dict(record["setting"])
    for key in MODEL_TUNING_KEYS:
        s.setdefault(key, MODEL[key])
    c.configure(s)
    c.reset()
    c.command[THRUST_VOLTAGE] = 0.0  # applied through the thrust scales (Data.thrust_scales), as the runtime does
    c.command[THRUST_VOLTAGE + 1] = s.get("yaw_i", 0.0)
    return np.array(c.state, np.float32), np.array(c.command, np.float32)


class Data:
    """The recordings as padded arrays [flight, sample, ...]: the logged signals, the controller's references and voltage-compensated thrust scales, and every sample's replayed controller state."""

    def __init__(self, records, ground=None):
        self.records = records
        self.size = max(len(r["base_throttle"]) for r in records)
        self.trims = np.array([(r["setting"].get("left_trim", 0.0), r["setting"].get("right_trim", 0.0)) for r in records])
        self.throttle = self.pad(lambda r: np.column_stack([r["left_throttle"], r["right_throttle"]]))
        self.command = self.pad(lambda r: np.column_stack([r["left_servo_deg"] - r["setting"].get("left_trim", 0.0), -(r["right_servo_deg"] - r["setting"].get("right_trim", 0.0))]))
        self.rate = self.pad(lambda r: np.column_stack([r["gx_deg_s"], r["gy_deg_s"], r["gz_deg_s"]]) * DEG)
        self.att = self.pad(lambda r: np.column_stack([r["roll_deg"], r["pitch_deg"]]) * DEG)
        self.vel = self.pad(lambda r: np.nan_to_num(np.column_stack([r["hover_vx_m_s"], r["hover_vy_m_s"]])))
        self.voltage = self.pad(lambda r: np.where(r["battery_v"] > 5, r["battery_v"], self.reference(r)) / self.reference(r))
        self.targets = self.pad(lambda r: np.column_stack([r["base_throttle"] > 0, r["target_roll_deg"] * DEG, r["target_pitch_deg"] * DEG, r["target_yaw_deg_s"] * DEG, r["base_throttle"]]))
        self.scale = self.pad(self.thrust_scales)
        # Dropped log records up to each sample, to leave out windows that span one.
        # How far each sample is from the floor for the near-ground term: the logged height above the engage point (m) or the time since liftoff (s).
        self.near = None if ground is None else self.pad(lambda r: np.maximum(np.nan_to_num(r["hover_height_m"], nan=10.0), 0.0) if ground == "height" else np.maximum(np.arange(len(r["base_throttle"])) - r["lift"], 0) * DT)
        self.excite = self.pad(lambda r: r["excite"])
        self.gaps = self.pad(lambda r: np.cumsum(np.abs(np.diff(r["esp_time_us"], prepend=r["esp_time_us"][0] - 10000) - 10000) > 3000))
        buffers = [controller(r) for r in records]
        self.initial = np.array([b[0] for b in buffers])
        self.commands = np.array([b[1] for b in buffers])
        self.states = self.replay()
        self.spread = np.concatenate([self.rate[i, r["lift"]:r["end"]] for i, r in enumerate(records)]).std(0)
        self.angle_spread = {}

    @staticmethod
    def reference(record):
        return record["setting"].get("thrust_voltage") or 16.1

    def pad(self, column):
        values = [np.asarray(column(r), float) for r in self.records]
        out = np.zeros((len(values), self.size) + values[0].shape[1:])
        for i, v in enumerate(values):
            out[i, :len(v)] = v
            out[i, len(v):] = v[-1]
        return out

    @staticmethod
    def thrust_scales(record):
        """Each motor's thrust scale as the runtime sets it: the configured scale times (smoothed battery over thrust_voltage)², within ±VOLTAGE_RANGE."""
        s = record["setting"]
        smooth = np.zeros(len(record["battery_v"]))
        value = 0.0
        for k, volts in enumerate(record["battery_v"]):
            value = value + (volts - value) * (1 - math.exp(-DT / VOLTAGE_TAU)) if value else volts
            smooth[k] = value
        reference = s.get("thrust_voltage", 0.0)
        ratio = np.clip(smooth / reference, 1 - VOLTAGE_RANGE, 1 + VOLTAGE_RANGE) if reference > 0 else np.ones(len(smooth))
        ratio = np.where(smooth > 0, ratio, 1.0)
        return np.column_stack([s.get("left_thrust_scale", 1.0) * ratio ** 2, s.get("right_thrust_scale", 1.0) * ratio ** 2])

    def control(self, states, commands, flights, k, attitude, gyro):
        """
        Step the lanes' controllers in place (lane j flying flight flights[j] at its sample k[j]) on the logged references with this attitude and gyro (rad, rad/s). Returns throttles [lane, side] and physical tilt commands [lane, side] (degrees, positive forward).
        """
        commands[:, 0:2] = attitude
        commands[:, 2:5] = gyro
        commands[:, 5] = DT
        for j, key in enumerate(("enabled", "roll", "pitch", "yaw_rate", "base_throttle")):
            commands[:, INDEX[key]] = self.targets[flights, k, j]
        commands[:, INDEX["left_thrust_scale"]] = self.scale[flights, k, 0]
        commands[:, INDEX["right_thrust_scale"]] = self.scale[flights, k, 1]
        control_many(states, commands)
        throttle = np.trunc(states[:, 2:4]).astype(float)
        tilt = np.column_stack([np.degrees(states[:, 4]) - self.trims[flights, 0], -(np.degrees(states[:, 5]) - self.trims[flights, 1])])
        # The test pattern's servo and motor steps, added after the controller as the drone does (a stopped motor stays stopped).
        excite = self.excite[flights, k]
        throttle = np.where(throttle > 0, np.clip(throttle + excite[:, 2:], 1, 1200), throttle)
        return throttle, tilt + excite[:, :2]

    def replay(self):
        """The controller's state before every sample [flight, sample, state], replayed on the logged attitude and gyro."""
        flights = np.arange(len(self.records))
        states, commands = self.initial.copy(), self.commands.copy()
        out = np.zeros((len(flights), self.size, states.shape[1]), np.float32)
        for k in range(self.size):
            out[:, k] = states
            self.control(states, commands, flights, np.full(len(flights), k), self.att[:, k], self.rate[:, k])
        return out

    def lanes(self, flights, horizons, stride):
        """
        Starts every stride samples in the flights' airborne windows as (flight, start) arrays, and which of the horizons each may be scored at [horizon, lane]: those still airborne with no dropped record from 200 ms before the start.
        """
        F = np.concatenate([np.full(len(range(self.records[i]["lift"] + 5, self.records[i]["end"] - horizons[0] - 1, stride)), i) for i in flights]).astype(int)
        S = np.concatenate([np.arange(self.records[i]["lift"] + 5, self.records[i]["end"] - horizons[0] - 1, stride) for i in flights]).astype(int)
        end = np.array([self.records[i]["end"] for i in F])
        before = self.gaps[F, np.maximum(S - 20, 0)]
        valid = np.array([(S + h < end) & (self.gaps[F, np.minimum(S + h + 1, self.size - 1)] == before) for h in horizons])
        keep = valid[0]
        return F[keep], S[keep], valid[:, keep]

    def spread_of_angle(self, h):
        if h not in self.angle_spread:
            self.angle_spread[h] = np.concatenate([self.att[i, r["lift"] + h:r["end"]] - self.att[i, r["lift"]:r["end"] - h] for i, r in enumerate(self.records) if r["end"] - r["lift"] > h]).std(0)
        return self.angle_spread[h]


def teacher(p, data, flights):
    """One step ahead along the log: every flight's actuator states before each sample and its constant push (the mean one-step residual over the airborne window)."""
    n = len(flights)
    f = np.array(flights)
    state = A.State(n)
    state.motor = A.thrust(data.throttle[f, 0]) * np.array([1.0, p["right_thrust_gain"]])
    state.horn = data.command[f, 0].copy()
    state.tilt = state.horn.copy()
    keep = {key: np.zeros((n, data.size) + value.shape[1:]) for key, value in state.__dict__.items()}
    residual = np.zeros((n, data.size, 3))
    zero = np.zeros((n, 3))
    throttle, command = data.throttle[f], data.command[f]
    for k in range(data.size - 1):
        state.rate, state.att = data.rate[f, k].copy(), data.att[f, k].copy()
        v = data.vel[f, k]
        state.vel = np.where(v != 0, v, state.vel)
        for key in keep:
            keep[key][:, k] = getattr(state, key)
        state, _ = A.step(p, state, throttle, command, np.full(n, k), data.voltage[f, k], zero, near=None if data.near is None else data.near[f, k])
        residual[:, k] = data.rate[f, k + 1] - state.rate
    push = np.array([residual[j, data.records[i]["lift"]:data.records[i]["end"] - 1].mean(0) / DT for j, i in enumerate(flights)])
    return keep, push


def predict(p, data, flights, horizons, stride, closed=True):
    """
    Free runs from every start in the flights (a sorted array): closed loop (the replayed controller flies the airframe on the logged references) or on the logged commands. Returns the lanes' flights and starts, which horizons each is valid at [horizon, lane], and the predicted rates and attitudes [step, lane, axis].
    """
    keep, push = teacher(p, data, flights)
    F, S0, valid = data.lanes(flights, horizons, stride)
    J = np.searchsorted(flights, F)
    state = A.State(len(F))
    for key in keep:
        setattr(state, key, keep[key][J, S0].copy())
    state.rate, state.att, state.vel = data.rate[F, S0].copy(), data.att[F, S0].copy(), data.vel[F, S0].copy()
    throttle, command, voltage = data.throttle[F].copy(), data.command[F].copy(), data.voltage[F]
    states, commands = data.states[F, S0].copy(), data.commands[F].copy()
    rates = np.zeros((max(horizons), len(F), 3))
    attitudes = np.zeros((max(horizons), len(F), 2))
    lane = np.arange(len(F))
    for j in range(max(horizons)):
        k = np.minimum(S0 + j, data.size - 1)
        if closed:
            throttle[lane, k], command[lane, k] = data.control(states, commands, F, k, state.att, state.rate)
        state, _ = A.step(p, state, throttle, command, k, voltage[lane, k], push[J], near=None if data.near is None else data.near[F, k])
        rates[j] = state.rate
        attitudes[j] = state.att
    return F, S0, valid, rates, attitudes


DATA = None
FREE = None
BASE = None


def unpack(x):
    values = dict(BASE)
    values.update(zip(FREE, map(float, x)))
    return values


def residuals(x, flights, stride=10, closed=True):
    F, S0, valid, rates, attitudes = predict(unpack(x), DATA, flights, HORIZONS, stride, closed)
    out = []
    for j, h in enumerate(HORIZONS):
        k = np.minimum(S0 + h, DATA.size - 1)
        mask = valid[j][:, None]
        out.append((np.clip((rates[h - 1] - DATA.rate[F, k]) / DATA.spread, -30, 30) * mask).ravel())
        out.append((np.clip((attitudes[h - 1] - DATA.att[F, k]) / DATA.spread_of_angle(h), -30, 30) * mask).ravel())
    return np.concatenate(out) / math.sqrt(len(HORIZONS))


def initialise(data, free, base):
    global DATA, FREE, BASE
    DATA, FREE, BASE = data, free, base


def column(job):
    x, j, h, flights = job
    y = x.copy()
    y[j] += h
    return residuals(y, flights)


def fit(pool, x0, flights, evaluations):
    from scipy.optimize import least_squares
    low = np.array([A.PARAMS[key][0] for key in FREE])
    high = np.array([A.PARAMS[key][1] for key in FREE])
    x0 = np.clip(x0, low + 1e-9, high - 1e-9)
    flights = np.array(sorted(flights))

    def jacobian(x):
        f0 = residuals(x, flights)
        steps = [1e-3 * max(1.0, abs(v)) * (-1 if v + 1e-3 * max(1.0, abs(v)) > hi else 1) for v, hi in zip(x, high)]
        columns = pool.map(column, [(x, j, h, flights) for j, h in enumerate(steps)])
        return np.column_stack([(c - f0) / h for c, h in zip(columns, steps)])

    result = least_squares(lambda x: residuals(x, flights), x0, jac=jacobian, bounds=(low, high), max_nfev=evaluations, x_scale="jac")
    return result.x, result.cost


def score(x, flights, closed):
    F, S0, valid, rates, attitudes = predict(unpack(x), DATA, np.array(sorted(flights)), SCORE_HORIZONS, 5, closed)
    out = {}
    for j, h in enumerate(SCORE_HORIZONS):
        v = valid[j]
        f, s = F[v], S0[v]
        out[h] = (rates[h - 1][v], DATA.rate[f, s + h], attitudes[h - 1][v] - DATA.att[f, s], DATA.att[f, s + h] - DATA.att[f, s])
    return out


def report(parts, label):
    print(label, flush=True)
    for h in SCORE_HORIZONS:
        rate, real, change, real_change = [np.concatenate([part[h][j] for part in parts]) for j in range(4)]
        r2 = 1 - ((rate - real) ** 2).sum(0) / ((real - real.mean(0)) ** 2).sum(0)
        error = np.sqrt(((change - real_change) ** 2).mean(0)) / DEG
        r2_change = 1 - ((change - real_change) ** 2).sum(0) / (real_change ** 2).sum(0)
        print(f"  {h * 10:4d} ms: rate R² roll {r2[0]:.2f} pitch {r2[1]:.2f} yaw {r2[2]:.2f}; attitude error roll {error[0]:.2f}° pitch {error[1]:.2f}° (R² of the change {r2_change[0]:.2f}, {r2_change[1]:.2f})", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--start", help="parameters to start from (default: the newest sw/sim/models/airframe_*.json; 'defaults' for the plant's starting values)")
    parser.add_argument("--freeze", default="roll_lift,band_stiffness", help="parameters held at their start (default: the terms the 2026-10-27 fit did without); right_thrust_gain is always held at the thrust split")
    parser.add_argument("--ground", choices=("height", "time"), help="fit the near-ground pitch term by logged height or time since liftoff (default: off, unless the start model has one)")
    parser.add_argument("--evaluations", type=int, default=60)
    parser.add_argument("--folds", type=int, default=5, help="held-out groups of flights to score (0 to skip)")
    parser.add_argument("--shortest", type=float, default=0.5, help="seconds airborne a flight needs")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--out", help="default: sw/sim/models/airframe_<newest recording's day>.json")
    args = parser.parse_args()
    saved = sorted(MODELS.glob("airframe_*.json"))
    start_path = None if args.start == "defaults" else Path(args.start) if args.start else saved[-1] if saved else None
    loaded = json.loads(start_path.read_text()) if start_path else {}
    start = A.complete(loaded) if start_path else {key: A.PARAMS[key][2] if key not in A.OPTIONAL else 0.0 for key in A.KEYS}
    print(f"starting from {start_path or 'the plant defaults'}", flush=True)
    ground = args.ground or loaded.get("ground_by")
    if ground and not start["pitch_ground"]:
        # From zero the scale has no gradient.
        start["pitch_ground"] = 0.3
    records = [r for r in (load(p, args.shortest) for p in paths(args.recordings)) if r and (ground != "height" or not np.isnan(r["hover_height_m"]).all())]
    print(f"{len(records)} flights, {sum(r['end'] - r['lift'] for r in records) * DT:.0f} s airborne", flush=True)
    data = Data(records, ground)
    split = np.median(np.concatenate([A.thrust(r["left_throttle"][r["lift"]:r["end"]]) for r in records])) / np.median(np.concatenate([A.thrust(r["right_throttle"][r["lift"]:r["end"]]) for r in records]))
    start["right_thrust_gain"] = float(split)
    print(f"right motor static thrust gain {split:.3f} (airborne thrust split)", flush=True)
    frozen = [key for key in args.freeze.split(",") if key] + ["right_thrust_gain"] + ([] if ground else ["pitch_ground", "ground_scale"])
    free = [key for key in A.KEYS if key not in frozen]
    base = {key: start[key] for key in A.KEYS}
    initialise(data, free, base)
    everything = list(range(len(records)))
    out = Path(args.out) if args.out else MODELS / f"airframe_{max(r['name'] for r in records)[:10]}.json"
    with Pool(args.workers, initializer=initialise, initargs=(data, free, base)) as pool:
        x, cost = fit(pool, np.array([start[key] for key in free]), everything, args.evaluations)
        params = unpack(x)
        _, push = teacher(params, data, everything)
        print(f"fit cost {cost:.1f}: " + ", ".join(f"{key} {value:.4g}" for key, value in params.items()), flush=True)
        print(f"pushes (rad/s²) roll {push[:, 0].mean():+.2f} ± {push[:, 0].std():.2f}, pitch {push[:, 1].mean():+.2f} ± {push[:, 1].std():.2f}, yaw {push[:, 2].mean():+.2f} ± {push[:, 2].std():.2f}", flush=True)
        result = {key: round(value, 6) for key, value in params.items() if ground or key not in ("pitch_ground", "ground_scale")} | ({"ground_by": ground} if ground else {}) | {f"{axis}_push": round(push[:, j].mean(), 4) for j, axis in enumerate(("roll", "pitch", "yaw"))} | {f"{axis}_push_sd": round(push[:, j].std(), 4) for j, axis in enumerate(("roll", "pitch", "yaw"))}
        out.write_text(json.dumps(result, indent=1) + "\n")
        print(f"saved {out}", flush=True)
        folds = min(args.folds, len(records))
        if not folds:
            return
        groups = np.random.default_rng(0).permutation(len(records))
        closed, logged = [], []
        for g in range(folds):
            test = sorted(groups[g::folds])
            xt, _ = fit(pool, x, [i for i in everything if i not in test], args.evaluations // 3)
            closed.append(score(xt, test, True))
            logged.append(score(xt, test, False))
    report(closed, "held out by flight, closed loop (the firmware controller flying the model on the logged references):")
    report(logged, "held out by flight, on the logged commands:")


if __name__ == "__main__":
    main()
