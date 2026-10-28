"""
Calibrate the hover sim to the real camera hovers' closed-loop behaviour: uv run --with scipy --with cma -m sw.sim.calibrate_sim [--generations 30] [--workers 90]

identify_airframe fits the airframe to short-horizon prediction, which the attitude controller's feedback dominates: it pins the plant above about 2 Hz but barely constrains its low-frequency gain, while fit_disturbances only sizes the torque residual's drift. Flown in closed loop, the sim then wobbles at the wrong frequency (0.7 Hz in pitch against the real 0.9-1 Hz). This CMA-ES search sets what both leave loose so that the sim flying each camera hover's settings (sweep.run with hover=True) matches the hover's statistics: the pitch and roll attitude spectra in three bands (0.3-0.8, 0.8-1.5, 1.5-3 Hz) and their rates below 5 Hz, the range error and sideways RMS (from the hover's own position estimate on both sides) and how fast they fade. Searched: scales on the tilt's pitch torque (through wash_z, so its lagged share pitch_wake scales with it), on the lagged share and its lag, on the roll authority (differential_slope), on the attitude terms (roll_pendulum, pitch_pendulum), and on the calibrated disturbances (roll and pitch torque drift size and correlation time, forward force drift size, sideways force drift size and correlation time). The prediction error on the logs (identify_airframe's closed-loop free runs at 250 ms, 500 ms and 1 s, all flights) is kept within reach by a penalty on its growth. Writes the scaled airframe and disturbances next to the sources (--out-airframe, --out-disturbances).
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
SEEDS = 6
# Searched: name, low, high, start (log-scaled; the lag and correlation times start at the current plant's, see starting).
SPACE = (("pitch_gain", 0.6, 1.8, 1.0), ("wake_gain", 0.3, 3.0, 1.0), ("wake_tau", 0.2, 2.0, 1.0), ("roll_gain", 0.6, 2.5, 1.0), ("roll_pendulum_gain", 0.2, 3.0, 1.0), ("pitch_pendulum_gain", 0.2, 3.0, 1.0),
         ("pitch_drift", 0.3, 3.0, 1.0), ("roll_drift", 0.3, 3.0, 1.0), ("pitch_tau", 0.05, 1.5, 0.1), ("roll_tau", 0.05, 1.5, 0.1), ("force_drift", 0.3, 3.0, 1.0), ("side_drift", 0.3, 5.0, 1.0), ("side_tau", 0.2, 10.0, 0.47))
HORIZONS = (25, 50, 100)
PENALTY = 4.0


def bands(series):
    """Band RMS of an angle (degrees), then its rate's RMS over 0.3-5 Hz (deg/s; the gyro above that is mostly rotor vibration)."""
    f, power = welch(series - series.mean(), fs=FS, nperseg=min(len(series), 1024))
    df = f[1] - f[0]
    rate = (f >= 0.3) & (f < 5.0)
    return [math.sqrt(power[(f >= lo) & (f < hi)].sum() * df) for lo, hi in BANDS] + [math.sqrt((power[rate] * (2 * math.pi * f[rate]) ** 2).sum() * df)]


def decorrelation(series):
    e = series - series.mean()
    ac = np.correlate(e, e, "full")[len(e) - 1:]
    return np.argmax(ac / ac[0] < 0.5) / FS


def statistics(pitch, roll, reach, side):
    """The matched statistics of one hover: pitch and roll band RMS (degrees) and rate RMS (deg/s), range and sideways RMS (m) and fade times (s)."""
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


def starting():
    """The point of SPACE that leaves the current plant as it is: unit scales, and its own lag and drift correlation times."""
    taus = {t["channel"]: t["tau"] for t in IDENTIFIED_ESC["disturbances"] if t["kind"] == "drift"}
    current = {"wake_tau": A.complete(IDENTIFIED_ESC["airframe"])["wake_tau"], "pitch_tau": taus["pitch"], "roll_tau": taus["roll"], "side_tau": taus["left"]}
    return np.array([np.clip(current.get(s[0], s[3]), s[1], s[2]) for s in SPACE])


def plant(x):
    """The sim plant for a point of SPACE."""
    v = dict(zip([s[0] for s in SPACE], x))
    airframe = IDENTIFIED_ESC["airframe"]
    geometry = A.geometry_of(airframe)
    p = A.complete(airframe)
    # The tilt's pitch torque arm is dz - wash (wash_z - CoM height): scaling it moves wash_z.
    cz = geometry["com"][2]
    arm = A.resolve(p, geometry)["tilt_arm"] * v["pitch_gain"]
    airframe = airframe | {"wash_z": cz + (geometry["pivot_z"] - cz - arm) / p["wash"], "pitch_wake": p["pitch_wake"] * v["wake_gain"], "wake_tau": v["wake_tau"], "differential_slope": p["differential_slope"] * v["roll_gain"],
                           "roll_pendulum": p["roll_pendulum"] * v["roll_pendulum_gain"], "pitch_pendulum": p["pitch_pendulum"] * v["pitch_pendulum_gain"]}
    terms = []
    for t in IDENTIFIED_ESC["disturbances"]:
        t = dict(t)
        if t["kind"] == "drift":
            if t["channel"] in ("pitch", "roll"):
                t["sd"] *= v[f"{t['channel']}_drift"]
                t["tau"] = v[f"{t['channel']}_tau"]
            elif t["channel"] == "forward":
                t["sd"] *= v["force_drift"]
            elif t["channel"] == "left":
                t["sd"] *= v["side_drift"]
                t["tau"] = v["side_tau"]
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
    estimate = np.degrees([e[3] for e in log])[keep]
    # The hover's own position estimate, as the real hovers log it (sweep.run's log).
    hover = [e[8] for e, k in zip(log, keep) if k]
    x, y = np.array([h[0][0] for h in hover]), np.array([h[0][1] for h in hover])
    reach = np.hypot(x, y)
    side = np.angle(np.exp(1j * (np.arctan2(y, x) - np.array([h[1] for h in hover])))) * reach
    return statistics(estimate[:, 1], estimate[:, 0], reach - np.array([h[2] for h in hover]), side)


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
    params["right_thrust_gain"] = N.thrust_split(data.records)
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
    parser.add_argument("--start", default="", help="name=value,... to start the search from")
    parser.add_argument("--out-airframe", type=Path, default=MODELS / "airframe_2026-10-28.json")
    parser.add_argument("--out-disturbances", type=Path, default=MODELS / "disturbances_2026-10-28_calibrated.json")
    args = parser.parse_args()
    import cma
    target = [real(name) for name in HOVERS]
    names = ["pitch 0.3-0.8", "0.8-1.5", "1.5-3", "rate", "roll 0.3-0.8", "0.8-1.5", "1.5-3", "rate", "range", "side", "range fade", "side fade"]
    print("real   " + "  ".join(f"{n} {v:.2f}" for n, v in zip(names, np.mean(target, 0))), flush=True)
    given = dict((k, float(v)) for k, v in (item.split("=") for item in args.start.split(",") if item))
    start = np.array([given.get(s[0], v) for s, v in zip(SPACE, starting())])
    low, high = np.array([s[1] for s in SPACE]), np.array([s[2] for s in SPACE])
    to_x = lambda u: np.exp(np.log(low) + (np.log(high) - np.log(low)) * np.clip(u, 0, 1))
    to_u = lambda x: (np.log(x) - np.log(low)) / (np.log(high) - np.log(low))
    with Pool(args.workers) as pool:
        base_error = prediction(starting())
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
    args.out_airframe.write_text(json.dumps({key: round(value, 6) if isinstance(value, float) else value for key, value in calibrated["airframe"].items()} | {"calibrated": {s[0]: round(float(v), 4) for s, v in zip(SPACE, x)}}, indent=1) + "\n")
    args.out_disturbances.write_text(json.dumps([{key: round(value, 3) if isinstance(value, float) else value for key, value in t.items()} for t in calibrated["disturbances"]], indent=1) + "\n")
    print(f"saved {args.out_airframe} and {args.out_disturbances}: " + ", ".join(f"{s[0]} {v:.3g}" for s, v in zip(SPACE, x)), flush=True)


if __name__ == "__main__":
    main()
