"""
flip_esc's band check on the MPC harness (make_mpc.py's ffmpc.py, the sagging pack and the firmware's thrust_voltage), greedy against the MPC: python -m sw.sim.mpc.band_mpc CASE KIND REGIME --mpc '{}' [--n 32] [--workers 30] [--bands 0,2,4] [--set k=v,...] [--params '{}'] [--out results.json]

REGIME: none (the hover-fit plant), worst (noband with the flip regime at its worst throughout: authority 2.6x, servos 0.11 / 0.09 s), repro (the 2026-11-15 burst's reproducing pitch: the tilt's authority 1.8-2.6x and the servos 0.08-0.11 s left, 0.03-0.07 s right, from the flip's start to its hand-back) or noband (the drone since 2026-11-15 ~06:20 UTC: the bands off, the assemblies' swing 3x, 60 ms servos; in the flip the authority 1.8-2.6x and the servos 0.08-0.11 / 0.06-0.09 s). --mpc: omitted, flip.c's greedy allocation; a JSON dict, the MPC with those MpcConfig fields. Per band: started, clean, peak pitch rate (deg/s, median and 90th), the nose's furthest from level while the flip owns the motors (degrees, median and 90th), the hand-back (s), height lost (m, 90th) and the highest point.
"""

import argparse
import json
from multiprocessing import Pool

import numpy as np

from sw.flight_log import parse_settings
from sw.sim import flip_esc as E
from sw.sim import tune_flip as T
from sw.sim.mpc import ffmpc as FF


def jobs(case, kind, n, params, regime, band, extra, mpc, seed=95000):
    out = []
    for k, j in enumerate(E.jobs(case, kind, n, seed, band, params=params, extra=extra)):
        options = dict(j[12])
        rng = np.random.default_rng(seed + 7 * k + 1000 * band)
        if regime == "repro":
            options["flip_pitch"] = {"gain": float(rng.uniform(1.8, 2.6)), "set": {"servo_delay": float(rng.uniform(0.08, 0.11)), "right_servo_delay": float(rng.uniform(0.03, 0.07))}, "revert": True}
        if regime in ("noband", "worst"):
            options["set"] = dict(options.get("set", {})) | {"servo_inertia": 0.001175, "left_band": 0.0, "right_band": 0.0, "servo_delay": 0.06, "right_servo_delay": 0.06}
            options["flip_pitch"] = {"gain": float(rng.uniform(1.8, 2.6)), "set": {"servo_delay": float(rng.uniform(0.08, 0.11)), "right_servo_delay": float(rng.uniform(0.06, 0.09))}, "revert": True}
            if regime == "worst":
                options["flip_pitch"] = {"gain": 2.6, "set": {"servo_delay": 0.11, "right_servo_delay": 0.09}, "revert": True}
            j = (j[0], 0.0) + tuple(j[2:])
        options["sag"] = (16.196, 0.0161)
        options["thrust_voltage"] = 16.1
        options["mpc"] = mpc
        flown = j[0] | {"thrust_voltage": 16.1}
        out.append((flown,) + j[1:12] + (options,))
    return out


def summary(rows):
    s = E.stats(rows)
    started = [r for r in rows if r["started"] is not None]
    nose = np.array([max(r["nose_up"], -r["nose_down"]) for r in started])
    s["nose_median"], s["nose_90"] = float(np.median(nose)), float(np.percentile(nose, 90))
    s["nose_up_90"] = float(np.percentile([r["nose_up"] for r in started], 90))
    s["peak_max"] = float(max(r["peak"] for r in rows))
    s["mpc_iterations"] = float(np.mean([r["mpc"][0] for r in rows]))
    s["own_handback"] = float(np.mean([r["handed"] < 4.0 for r in started]))
    return s


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case")
    parser.add_argument("kind")
    parser.add_argument("regime", choices=("none", "repro", "noband", "worst"))
    parser.add_argument("--mpc", default=None, help="MpcConfig fields as JSON ({} for mpc_defaults); omitted: the greedy allocation")
    parser.add_argument("--n", type=int, default=32)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--bands", default="0,2,4")
    parser.add_argument("--set", default="")
    parser.add_argument("--params", default="{}")
    parser.add_argument("--out")
    args = parser.parse_args()
    mpc = json.loads(args.mpc) if args.mpc is not None else None
    extra, params = parse_settings(args.set), json.loads(args.params)
    res, pooled = {}, []
    with Pool(args.workers) as pool:
        for band in [int(b) for b in args.bands.split(",")]:
            rows = [r for b in pool.map(FF.fly, jobs(args.case, args.kind, args.n, params, args.regime, band, extra, mpc), chunksize=1) for r in b]
            pooled += rows
            s = summary(rows)
            low, high = T.MOMENTUM_BANDS[band]
            res[f"{low:.2f}-{high:.2f}"] = s
            print(f"{args.kind} {args.regime} {'greedy' if mpc is None else 'mpc ' + json.dumps(mpc)} rm {low:.2f}-{high:.2f}: started {s['started']}/{s['n']} clean {s['clean'] * 100:.1f}% touched {s['touched'] * 100:.1f}% "
                  f"pitch rate med {s['rates_median'][1]:.0f} 90th {s['rates_90'][1]:.0f} roll med {s['rates_median'][0]:.0f} nose med {s['nose_median']:.1f} 90th {s['nose_90']:.1f} handed {s['handed_median']:.2f}/{s['handed_90']:.2f} lost90 {s['lost_90']:.2f} "
                  f"highest {s['peak_max']:.2f} iterations {s['mpc_iterations']:.1f}", flush=True)
    s = res["all"] = summary(pooled)
    print(f"{args.kind} {args.regime} {'greedy' if mpc is None else 'mpc ' + json.dumps(mpc)} all: started {s['started']}/{s['n']} clean {s['clean'] * 100:.1f}% pitch rate med {s['rates_median'][1]:.0f} 90th {s['rates_90'][1]:.0f} nose med {s['nose_median']:.1f} 90th {s['nose_90']:.1f} "
          f"own hand-back {s['own_handback'] * 100:.0f}% handed {s['handed_median']:.2f} lost90 {s['lost_90']:.2f} highest {s['peak_max']:.2f}", flush=True)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f)


if __name__ == "__main__":
    main()
