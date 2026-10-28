"""
Calibrate the hover sim to the real camera hovers' closed-loop behaviour: uv run --with scipy --with cma -m sw.sim.calibrate_sim [--generations 30] [--workers 90]

identify_airframe fits the airframe to short-horizon prediction, which the attitude controller's feedback dominates: it pins the plant above about 2 Hz but barely constrains its low-frequency gain, while fit_disturbances only sizes the torque residual's drift. Flown in closed loop, the sim then wobbles at the wrong frequency (0.7 Hz in pitch against the real 0.9-1 Hz). This CMA-ES search sets what both leave loose so that the sim flying each camera hover's settings (sweep.run with hover=True) matches the hover's statistics: the pitch and roll attitude spectra in three bands (0.3-0.8, 0.8-1.5, 1.5-3 Hz), the range error and sideways RMS and how fast they fade. Searched: scales on the pitch effectiveness (pitch_tilt and pitch_wake together) and roll effectiveness, and on the calibrated disturbances (roll and pitch torque drift size and correlation time, force drift size). The prediction error on the logs (identify_airframe's closed-loop free runs at 250 ms, 500 ms and 1 s, all flights) is kept within reach by a penalty on its growth. Writes the scaled airframe and disturbances next to the sources (--out-airframe, --out-disturbances).
"""

import argparse
import json
import math
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.signal import welch

from sw.fit import settings
from sw.flight_log import read_flight
from sw.sim import airframe as A
from sw.sim import identify_airframe as N
from sw.sim import plot_prediction as P
from sw.sim.sweep import IDENTIFIED_ESC, run

MODELS = Path(__file__).with_name("models")
RECORDINGS = Path(__file__).parents[1] / "recordings"
# The camera hovers (tracking the tag for 45 s) and every hover after the ESC recalibration (for the prediction penalty).
HOVERS = ("2026-10-28T06-59-38-797Z_3919631855", "2026-10-28T07-00-59-352Z_1068518903")
FLIGHTS = ("2026-10-28T06-54-20-503Z_1581871433", "2026-10-28T06-55-20-397Z_757306949") + HOVERS
FS = 100.0
SETTLE, LANDING = 4.0, 4.0
BANDS = ((0.3, 0.8), (0.8, 1.5), (1.5, 3.0))
SEEDS = 4
# Searched: name, low, high, start (log-scaled except the correlation times).
SPACE = (("pitch_gain", 0.7, 1.8, 1.0), ("roll_gain", 0.7, 2.5, 1.0), ("pitch_drift", 0.3, 3.0, 1.0), ("roll_drift", 0.3, 3.0, 1.0), ("pitch_tau", 0.1, 1.5, 0.7), ("roll_tau", 0.1, 1.5, 0.7), ("force_drift", 0.3, 3.0, 1.0))
HORIZONS = (25, 50, 100)
PENALTY = 2.0


def bands(series):
    f, power = welch(series - series.mean(), fs=FS, nperseg=min(len(series), 1024))
    return [math.sqrt(power[(f >= lo) & (f < hi)].sum() * (f[1] - f[0])) for lo, hi in BANDS]


def decorrelation(series):
    e = series - series.mean()
    ac = np.correlate(e, e, "full")[len(e) - 1:]
    return np.argmax(ac / ac[0] < 0.5) / FS


def statistics(pitch, roll, reach, side):
    """The matched statistics of one hover: pitch and roll band RMS (degrees), range and sideways RMS (m) and fade times (s)."""
    return np.array(bands(pitch) + bands(roll) + [reach.std(), side.std(), decorrelation(reach), decorrelation(side)])


def real(name):
    path = RECORDINGS / f"{name}.bin"
    d, _ = read_flight(path)
    on = np.flatnonzero(d.base_throttle.values > 0)[0]
    flags = d["flags"].values.astype(int)
    age = np.nan_to_num(d.hover_tag_age_ms.values, nan=65535)
    k = np.arange(len(d))
    ok = (k > on + int(SETTLE * FS) + 300) & ((flags >> 1 & 3) == 1) & (age < 120)
    first, last = np.flatnonzero(ok)[[0, -1]]
    span = slice(first, last - int(LANDING * FS))
    x, y = d.hover_x_m.values[span], d.hover_y_m.values[span]
    reach = np.hypot(x, y)
    side = np.radians(((np.degrees(np.arctan2(y, x)) - d.hover_arc_deg.values[span] + 180) % 360) - 180) * reach
    fill = lambda v: np.interp(np.arange(len(v)), np.flatnonzero(~np.isnan(v)), v[~np.isnan(v)])
    return statistics(fill(d.pitch_deg.values[span]), fill(d.roll_deg.values[span]), fill(reach - d.hover_range_target_m.values[span]), fill(side))


def plant(x):
    """The sim plant for a point of SPACE."""
    v = dict(zip([s[0] for s in SPACE], x))
    airframe = IDENTIFIED_ESC["airframe"]
    airframe = airframe | {"pitch_tilt": airframe["pitch_tilt"] * v["pitch_gain"], "pitch_wake": airframe.get("pitch_wake", 0.0) * v["pitch_gain"], "roll_thrust": airframe["roll_thrust"] * v["roll_gain"]}
    terms = []
    for t in IDENTIFIED_ESC["disturbances"]:
        t = dict(t)
        if t["kind"] == "drift":
            if t["channel"] in ("pitch", "roll"):
                t["sd"] *= v[f"{t['channel']}_drift"]
                t["tau"] = v[f"{t['channel']}_tau"]
            elif t["channel"] in ("forward", "left"):
                t["sd"] *= v["force_drift"]
        terms.append(t)
    return IDENTIFIED_ESC | {"airframe": airframe, "disturbances": terms}


def fly(job):
    x, name, seed, seconds = job
    path = RECORDINGS / f"{name}.bin"
    s = settings(path) | {"hover_distance": 2.0}
    log = []
    p = plant(x)
    result = run(s, p, seconds=seconds, seed=seed, log=log, hover=True)
    if not result["stable"]:
        return None
    t = np.array([e[0] for e in log]) - result["liftoff_s"]
    keep = t > SETTLE
    position = np.array([e[1] for e in log])[keep]
    estimate = np.degrees([e[3] for e in log])[keep]
    tag = np.array(p["camera"]["tag"])
    reach = np.hypot(tag[0] - position[:, 0], tag[1] - position[:, 1]) - 2.0
    return statistics(estimate[:, 1], estimate[:, 0], reach, position[:, 1] - tag[1])


DATA = None


def load_data():
    global DATA
    if DATA is None:
        records = [r for r in (N.load(RECORDINGS / f"{name}.bin") for name in FLIGHTS) if r]
        DATA = N.Data(records)
    return DATA


def prediction(x):
    """identify_airframe's closed-loop attitude-change error (degrees, roll and pitch) at each of HORIZONS, all flights."""
    data = load_data()
    params = A.complete(plant(x)["airframe"])
    split = np.median(np.concatenate([A.thrust(r["left_throttle"][r["lift"]:r["end"]]) for r in data.records])) / np.median(np.concatenate([A.thrust(r["right_throttle"][r["lift"]:r["end"]]) for r in data.records]))
    params["right_thrust_gain"] = float(split)
    out = []
    for h in HORIZONS:
        F, S0, attitudes = P.runs(params, data, h, 10)
        out.append(P.errors(data, F, S0, attitudes, h))
    return np.array(out)


def evaluate(pool, xs, target, base_error, seconds):
    jobs = [(x, name, seed, seconds) for x in xs for name in HOVERS for seed in range(SEEDS)]
    flown = pool.map(fly, jobs)
    errors = pool.map(prediction, xs)
    costs, details = [], []
    per = len(HOVERS) * SEEDS
    for i, x in enumerate(xs):
        runs = flown[i * per:(i + 1) * per]
        crashed = sum(r is None for r in runs)
        stats = [np.median([r for j, r in enumerate(runs[k * SEEDS:(k + 1) * SEEDS]) if r is not None], 0) if any(r is not None for r in runs[k * SEEDS:(k + 1) * SEEDS]) else None for k in range(len(HOVERS))]
        if any(s is None for s in stats):
            costs.append(10.0)
            details.append(None)
            continue
        mismatch = np.sqrt(np.mean([np.log(s / t) ** 2 for s, t in zip(stats, target)]))
        growth = np.log(errors[i] / base_error).clip(0, None).mean()
        costs.append(mismatch + PENALTY * growth + 0.5 * crashed / per)
        details.append((mismatch, growth, crashed, np.mean(stats, 0), errors[i]))
    return costs, details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--popsize", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--out-airframe", type=Path, default=MODELS / "airframe_2026-10-28_calibrated.json")
    parser.add_argument("--out-disturbances", type=Path, default=MODELS / "disturbances_2026-10-28_calibrated.json")
    args = parser.parse_args()
    import cma
    target = [real(name) for name in HOVERS]
    names = ["pitch 0.3-0.8", "0.8-1.5", "1.5-3", "roll 0.3-0.8", "0.8-1.5", "1.5-3", "range", "side", "range fade", "side fade"]
    print("real   " + "  ".join(f"{n} {v:.2f}" for n, v in zip(names, np.mean(target, 0))), flush=True)
    start = np.array([s[3] for s in SPACE])
    low, high = np.array([s[1] for s in SPACE]), np.array([s[2] for s in SPACE])
    to_x = lambda u: np.exp(np.log(low) + (np.log(high) - np.log(low)) * np.clip(u, 0, 1))
    to_u = lambda x: (np.log(x) - np.log(low)) / (np.log(high) - np.log(low))
    with Pool(args.workers) as pool:
        base_error = prediction(start)
        costs, details = evaluate(pool, [start], target, base_error, args.seconds)
        print(f"start: cost {costs[0]:.3f}, mismatch {details[0][0]:.3f} | " + "  ".join(f"{n} {v:.2f}" for n, v in zip(names, details[0][3])), flush=True)
        best = (costs[0], start, details[0])
        es = cma.CMAEvolutionStrategy(to_u(start), 0.2, {"bounds": [0, 1], "popsize": args.popsize, "seed": 1, "verbose": -9})
        for generation in range(args.generations):
            us = es.ask()
            xs = [to_x(u) for u in us]
            costs, details = evaluate(pool, xs, target, base_error, args.seconds)
            es.tell(us, costs)
            i = int(np.argmin(costs))
            if costs[i] < best[0]:
                best = (costs[i], xs[i], details[i])
            print(f"generation {generation + 1}: best {costs[i]:.3f} | so far {best[0]:.3f} (mismatch {best[2][0]:.3f}, prediction growth {best[2][1]:.3f}) " + " ".join(f"{s[0]} {v:.3g}" for s, v in zip(SPACE, best[1])), flush=True)
    cost, x, (mismatch, growth, crashed, stats, errors) = best
    print("best   " + "  ".join(f"{n} {v:.2f}" for n, v in zip(names, stats)), flush=True)
    print("prediction error roll/pitch (°) " + "  ".join(f"{h * 10} ms {e[0]:.2f}/{e[1]:.2f} (was {b[0]:.2f}/{b[1]:.2f})" for h, e, b in zip(HORIZONS, errors, base_error)), flush=True)
    calibrated = plant(x)
    args.out_airframe.write_text(json.dumps({key: round(value, 6) if isinstance(value, float) else value for key, value in calibrated["airframe"].items()}, indent=1) + "\n")
    args.out_disturbances.write_text(json.dumps(calibrated["disturbances"], indent=1) + "\n")
    print(f"saved {args.out_airframe} and {args.out_disturbances}: " + ", ".join(f"{s[0]} {v:.3g}" for s, v in zip(SPACE, x)), flush=True)


if __name__ == "__main__":
    main()
