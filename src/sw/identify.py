"""
Whole-airframe system identification from airborne recordings: uv run -m sw.identify <recordings or globs> [--group day|bands|all] [--shared] [--generations 200] [--workers 16] [--out result.json]
The logged motor throttles and servo commands go through actuator models (motors: dead time, first-order lag, thrust from the table scaled by battery voltage to a power; servos: dead time, lag, slew limit, play on each side, and a gain from command to physical tilt) to the torques the controller's model would expect about each axis. Each axis's rate change over several horizons (50 to 300 ms) is then regressed on the integrated torque, rate and attitude, with a constant push per flight, so the fit never differentiates the gyro. Actuator parameters describe parts that stay when the airframe changes, so they can be shared by every group of flights (--shared) or fitted per group; the linear coefficients (effectiveness relative to the model, rate damping, angle stiffness) are always per group, as they depend on the geometry. CMA-ES searches the actuator parameters on held-out prediction: each flight is predicted by coefficients fitted on the other flights of its group.
"""

import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.fit import settings
from sw.flight_log import read_flight
from sw.sim.flight import MODEL, THRUST_TABLE

DEG = np.pi / 180
DT = 0.01
TABLE = np.array(THRUST_TABLE, float)
HORIZONS = (5, 10, 20, 30)
AXES = ("roll", "pitch", "yaw")
# Actuator parameters, their search bounds and what they mean: motor dead time and lag (s), thrust's power of battery voltage, servo dead time and lag (s), slew limit (°/s), play left and right (total, °), and physical tilt per commanded degree.
BOUNDS = {
    "motor_delay": (0.0, 0.08), "motor_tau": (0.005, 0.12), "voltage_power": (0.0, 3.0),
    "servo_delay": (0.0, 0.12), "servo_tau": (0.002, 0.12), "servo_slew": (100.0, 1500.0),
    "left_play": (0.0, 4.0), "right_play": (0.0, 4.0), "servo_gain": (0.4, 1.4),
}
KEYS = list(BOUNDS)
# Rubber bands pulling the servos forward (against their play), from the flight notes: none before 2026-10-25 03:47 UTC, then on the right servo only, on both from 09:09 (never noted as removed).
BANDS = (("2026-10-25T03-47", "right"), ("2026-10-25T09-09", "both"))


def window(path):
    """
    The airborne part of a flight: from liftoff (vertical specific force past 1.12 g once collective reaches 600) to an upset past 35°, an impact, motor stop or the recording ending.
    """
    try:
        data, _ = read_flight(path)
        setting = settings(path)
    except Exception:
        return None
    on = np.flatnonzero(data.base_throttle.values >= 600)
    if not len(on) or not (data.az_g.values[on[0]:] > 1.12).any():
        return None
    lift = on[0] + np.argmax(data.az_g.values[on[0]:] > 1.12) + 3
    shock = np.sqrt(data.ax_g**2 + data.ay_g**2 + data.az_g**2).values
    end = next((i for i in range(lift + 3, len(data)) if abs(data.pitch_deg[i]) > 35 or abs(data.roll_deg[i]) > 35 or shock[i] > 2.5 or data.base_throttle[i] == 0), len(data))
    if end - lift < 100:
        return None
    span = slice(max(0, lift - 50), end)
    battery = data.battery_v.values[span]
    reference = setting.get("thrust_voltage") or float(np.median(battery[battery > 5])) if (battery > 5).any() else 16.0
    return {
        "name": path.name[:19], "day": path.name[:10], "bands": next((state for since, state in reversed(BANDS) if path.name[:16] >= since), "none"), "path": str(path), "start": lift - span.start, "setting": setting,
        "throttle": np.column_stack([data.left_throttle.values[span], data.right_throttle.values[span]]).astype(float),
        "voltage": np.where(battery > 5, battery, reference) / reference,
        # Commanded physical tilt, positive forward: the right servo is mirrored, and the trims are each servo's zero.
        "command": np.column_stack([data.left_servo_deg.values[span] - setting.get("left_trim", 0.0), -(data.right_servo_deg.values[span] - setting.get("right_trim", 0.0))]),
        "rate": np.column_stack([data.gx_deg_s.values[span], data.gy_deg_s.values[span], data.gz_deg_s.values[span]]) * DEG,
        "angle": np.column_stack([data.roll_deg.values[span], data.pitch_deg.values[span] - setting.get("pitch_trim", 0.0)]) * DEG,
    }


def delayed(signal, delay):
    """
    The signal delayed by delay seconds (fractional samples by linear interpolation), holding its first value.
    """
    t = np.arange(len(signal)) * DT
    return np.interp(t - delay, t, signal)


def lag(signal, tau):
    keep = np.exp(-DT / tau)
    out = np.empty_like(signal)
    value = signal[0]
    for i, target in enumerate(signal):
        value = target + (value - target) * keep
        out[i] = value
    return out


def servo(command, delay, tau, slew, play, gain):
    """
    Physical tilt (degrees) from a servo's command (degrees): dead time, a first-order lag limited to slew (°/s), then play (total, degrees) between the horn and the motor mount, then the gain.
    """
    shifted = delayed(command, delay)
    keep = np.exp(-DT / tau)
    step = slew * DT
    half = play / 2
    horn = tilt = shifted[0]
    out = np.empty_like(command)
    for i, target in enumerate(shifted):
        horn += min(step, max(-step, (target - horn) * (1 - keep)))
        if horn - tilt > half:
            tilt = horn - half
        elif tilt - horn > half:
            tilt = horn + half
        out[i] = tilt
    return out * gain


def torques(flight, p):
    """
    Each axis's angular acceleration from the actuators as the controller's model expects it (rad/s², effectiveness 1), and the pitch term a centre of mass ahead of the pivot would add per metre.
    """
    thrust = np.interp(flight["throttle"], TABLE[:, 0], TABLE[:, 1]) * flight["voltage"][:, None] ** p["voltage_power"]
    thrust = np.column_stack([lag(delayed(thrust[:, side], p["motor_delay"]), p["motor_tau"]) for side in range(2)])
    tilt = np.column_stack([servo(flight["command"][:, side], p["servo_delay"], p["servo_tau"], p["servo_slew"], p[("left_play", "right_play")[side]], p["servo_gain"]) for side in range(2)]) * DEG
    left, right = thrust[:, 0], thrust[:, 1]
    up = left * np.cos(tilt[:, 0]), right * np.cos(tilt[:, 1])
    forward = left * np.sin(tilt[:, 0]), right * np.sin(tilt[:, 1])
    model = flight["setting"] | {key: flight["setting"].get(key, MODEL[key]) for key in MODEL}
    return {
        "roll": model["arm"] * (up[0] - up[1]) / model["roll_inertia"],
        "pitch": model["pivot_height"] * (forward[0] + forward[1]) / model["pitch_inertia"],
        "yaw": (model["arm"] * (forward[1] - forward[0]) + model["reaction_torque_ratio"] * (up[1] - up[0])) / model["yaw_inertia"],
        "com": (up[0] + up[1]) / model["pitch_inertia"],
    }


def rows(flight, model, axis):
    """
    Rate change over each horizon from every airborne sample, and the same horizons' integrals of the regressors: the model torque, the rate, the attitude (roll and pitch only), the centre-of-mass term (pitch only) and time (the flight's constant push).
    """
    index = "rpy".index(axis[0]) if axis != "yaw" else 2
    rate = flight["rate"][:, index]
    columns = [model[axis], rate]
    if axis != "yaw":
        columns.append(flight["angle"][:, index])
    if axis == "pitch":
        columns.append(model["com"])
    sums = [np.concatenate([[0], np.cumsum(c) * DT]) for c in columns]
    ys, xs, ts = [], [], []
    for horizon in HORIZONS:
        start = np.arange(flight["start"], len(rate) - horizon)
        scale = 1 / np.sqrt(horizon)
        ys.append((rate[start + horizon] - rate[start]) * scale)
        xs.append(np.column_stack([(s[start + horizon] - s[start]) * scale for s in sums]))
        ts.append(np.full(len(start), horizon * DT * scale))
    return np.concatenate(ys), np.concatenate(xs), np.concatenate(ts)


def solve(blocks):
    """
    Shared coefficients and one constant per flight: blocks of (y, x, t) for each flight.
    """
    width = blocks[0][1].shape[1]
    total = sum(len(b[0]) for b in blocks)
    design = np.zeros((total, width + len(blocks)))
    target = np.empty(total)
    at = 0
    for k, (y, x, t) in enumerate(blocks):
        design[at:at + len(y), :width] = x
        design[at:at + len(y), width + k] = t
        target[at:at + len(y)] = y
        at += len(y)
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
    return coefficients[:width], coefficients[width:]


def held_out(blocks):
    """
    Each flight predicted by coefficients from the other flights of the group; its own constant push is fitted given those coefficients (it changes flight to flight). Returns the pooled held-out R² and the all-flight coefficients.
    """
    residual, total = 0.0, 0.0
    for k in range(len(blocks)):
        shared, _ = solve([b for i, b in enumerate(blocks) if i != k]) if len(blocks) > 1 else solve(blocks)
        y, x, t = blocks[k]
        rest = y - x @ shared
        push = np.dot(rest, t) / np.dot(t, t)
        residual += np.sum((rest - push * t) ** 2)
        total += np.sum((y - y.mean()) ** 2)
    shared, pushes = solve(blocks)
    return 1 - residual / total, shared, pushes


FLIGHTS = {}


def score(args):
    """
    Held-out R² per group and axis for one actuator parameter set (a dict), as {group: {axis: (r2, coefficients, pushes)}}.
    """
    p, groups = args
    result = {}
    for group in groups:
        flights = FLIGHTS[group]
        models = [torques(f, p) for f in flights]
        result[group] = {}
        for axis in AXES:
            r2, shared, pushes = held_out([rows(f, m, axis) for f, m in zip(flights, models)])
            result[group][axis] = (r2, shared.tolist(), pushes.tolist())
    return result


def cost(result):
    return -sum(r2 for group in result.values() for r2, _, _ in group.values())


def parameters(x):
    return {k: lo + min(1.0, max(0.0, v)) * (hi - lo) for k, (lo, hi), v in zip(KEYS, BOUNDS.values(), x)}


def initialise(groups):
    FLIGHTS.update(groups)


def optimise(groups, generations, workers, seed=1):
    import cma

    x0 = [0.5] * len(KEYS)
    es = cma.CMAEvolutionStrategy(x0, 0.25, {"bounds": [0, 1], "popsize": max(workers, 12), "maxiter": generations, "seed": seed, "verbose": -9})
    names = list(groups)
    with Pool(workers, initializer=initialise, initargs=(groups,)) as pool:
        while not es.stop():
            xs = es.ask()
            results = pool.map(score, [(parameters(x), names) for x in xs])
            es.tell(xs, [cost(r) for r in results])
            if es.countiter % 10 == 0:
                print(f"  generation {es.countiter}: held-out R² sum {-es.result.fbest:.3f}", flush=True)
    best = parameters(es.result.xbest)
    initialise(groups)
    return best, score((best, names))


def describe(p, result):
    lines = ["actuators: " + ", ".join(f"{k} {v:.3g}" for k, v in p.items())]
    labels = {"roll": ("effectiveness", "rate damping /s", "angle stiffness /s²"), "pitch": ("effectiveness", "rate damping /s", "angle stiffness /s²", "com_x m"), "yaw": ("effectiveness", "rate damping /s")}
    for group, axes in result.items():
        for axis, (r2, shared, pushes) in axes.items():
            terms = ", ".join(f"{label} {value:+.3g}" for label, value in zip(labels[axis], shared))
            lines.append(f"{group} {axis:5s} held-out R² {r2:.3f}: {terms}; push per flight {np.mean(pushes):+.2f} ± {np.std(pushes):.2f} rad/s²")
    return "\n".join(lines)


def paths(arguments):
    return sorted(path for argument in arguments for path in (Path(argument).parent.glob(Path(argument).name) if "*" in argument else [Path(argument)]) if path.suffix == ".bin")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--group", choices=("day", "bands", "all"), help="bands: by day and rubber-band state", default="day")
    parser.add_argument("--shared", action="store_true", help="one set of actuator parameters for every group")
    parser.add_argument("--generations", type=int, default=200)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out")
    args = parser.parse_args()
    flights = [f for f in (window(p) for p in paths(args.recordings)) if f]
    groups = {}
    for f in flights:
        groups.setdefault({"day": f["day"], "bands": f"{f['day']} {f['bands']} bands", "all": "all"}[args.group], []).append(f)
    for group, members in groups.items():
        print(f"{group}: {len(members)} flights, {sum(len(f['rate']) - f['start'] for f in members) * DT:.0f} s airborne", flush=True)
    output = {}
    sets = {"shared": groups} if args.shared else {group: {group: members} for group, members in groups.items()}
    for name, subset in sets.items():
        print(f"fitting actuators for {name}", flush=True)
        p, result = optimise(subset, args.generations, args.workers, args.seed)
        print(describe(p, result), flush=True)
        output[name] = {"actuators": p, "groups": result}
    if args.out:
        Path(args.out).write_text(json.dumps(output, indent=1))


if __name__ == "__main__":
    main()
