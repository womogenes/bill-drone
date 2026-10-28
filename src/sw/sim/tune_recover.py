"""
Tune the recovery controller (sw/sim/recover.py), or with --flip the roll flip on top of it (sw/sim/flip.py), by CMA-ES over randomized runs: uv run --with cma -m sw.sim.tune_recover [--flip] [--generations 60] [--n 256] [--workers N] [--out params.json]

Every candidate flies the same n falls (heights, orientations, rates and velocities from recover.scenarios, plants randomized by recover.randomization) for 20 s; its cost is the mean over falls of how far each missed a safe landing: the tilt at first touch over 30°, the descent speed over 1.5 m/s, the contact force over 300 N, the final uprightness, and a penalty for never touching. Each generation draws new falls, so the result generalizes rather than fitting one set.
"""

import argparse
import json
import math
import os
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.sim import recover as C

# Searched parameters: (name, low, high), on a log scale except the angles.
SPACE = [
    ("kp", 1.0, 20.0), ("brake", 3.0, 60.0), ("max_rate", 1.5, 12.0), ("k_roll", 2.0, 60.0), ("k_pitch", 2.0, 80.0), ("kd_yaw", 0.5, 20.0),
    ("kv", 0.5, 8.0), ("kh", 0.2, 4.0), ("descent", 0.2, 1.0), ("flip_thrust", 0.1, 2.5), ("right_angle", 15.0, 80.0), ("tilt_max", 5.0, 35.0),
    ("servo_max", 20.0, 60.0), ("level_tau", 0.3, 5.0), ("cut_time", 0.1, 1.5), ("flare", 0.3, 3.0), ("k_alt", 0.2, 5.0), ("leak_tau", 0.5, 30.0)
]
HEIGHTS = (3.0, 8.0)
# Not searched: recover.DEFAULTS roll_over (165°) only matters near inverted, which random falls rarely reach, so the search drifted it to 179°, where 180° drops from 5 m fall from 95% to 67% safe; from 150° rolling the long way does not pay (3 m: 46% at 165°, 38% at 135°).
# The flip's parameters, searched with the recovery controller's (from models/recover_params.json unless --recovery) for the hold after it.
FLIP_SPACE = [("climb_thrust", 1.0, 2.2), ("climb_time", 0.1, 1.0), ("spin_rate", 4.0, 30.0), ("k_spin", 5.0, 80.0), ("spin_thrust", 0.1, 1.5), ("handoff", 0.3, 2.5)]
# Height a flip may climb before the cost counts it (m).
FLIP_ROOM = 1.0
TASK = {"space": SPACE, "flip": False, "recovery": {}}


def decode(x):
    return {name: math.exp(math.log(lo) + (math.log(hi) - math.log(lo)) * min(1.0, max(0.0, v))) for (name, lo, hi), v in zip(TASK["space"], x)}


def encode(params):
    return [(math.log(params[name]) - math.log(lo)) / (math.log(hi) - math.log(lo)) for name, lo, hi in TASK["space"]]


def flip_cost(result):
    """How far each flip missed a clean one: hitting the ground, the height lost (per 2 m), the height climbed beyond FLIP_ROOM (indoors there is little), not ending upright and still."""
    c = 3.0 * result["touched"] + np.minimum(result["lost"], 3) / 2 + np.maximum(result["climbed"] - FLIP_ROOM, 0) + 2 * np.clip(1 - result["up"], 0, 2) + (~result["clean"]) * 0.5
    return float(c.mean()), float(result["clean"].mean())


def cost(result):
    tilt, speed, contact, up = result["first_tilt"], result["first_speed"], result["contact"], result["final_up"]
    touched = np.isfinite(tilt)
    c = np.where(touched, np.maximum(0, np.nan_to_num(tilt) - 30) / 30 + np.maximum(0, np.nan_to_num(speed) - 1.5) / 1.5, 3.0)
    c += np.maximum(0, contact - 300) / 300 + 2 * np.clip(1 - up, 0, 2)
    return float(c.mean()), float(result["safe"].mean())


def evaluate(job):
    x, seed, n, task = job
    TASK.update(task)
    C.SPREAD = task.get("spread", 1.0)
    if task["flip"]:
        from sw.sim import flip as F
        return flip_cost(F.run(decode(x), task["recovery"], n=n, seed=seed))
    C.scenarios.__defaults__ = (HEIGHTS,)
    return cost(C.run(decode(x), n=n, seed=seed, seconds=20.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=60)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--popsize", type=int, default=16)
    parser.add_argument("--chunk", type=int, default=128)
    parser.add_argument("--spread", type=float, default=1.0, help="plant randomization width (recover.SPREAD)")
    parser.add_argument("--start", type=Path)
    parser.add_argument("--flip", action="store_true")
    parser.add_argument("--recovery", type=Path, default=Path(__file__).with_name("models") / "recover_params.json")
    parser.add_argument("--out", type=Path, default=Path("recover_params.json"))
    args = parser.parse_args()
    import cma
    if args.flip:
        from sw.sim import flip as F
        recovery = json.loads(args.recovery.read_text())
        TASK.update(space=FLIP_SPACE + SPACE, flip=True, recovery=recovery)
        start = F.DEFAULTS | C.DEFAULTS | recovery | (json.loads(args.start.read_text()) if args.start else {})
    else:
        start = C.DEFAULTS | (json.loads(args.start.read_text()) if args.start else {})
    task = dict(TASK) | {"spread": args.spread}
    es = cma.CMAEvolutionStrategy(encode(start), 0.15, {"bounds": [0, 1], "popsize": args.popsize, "maxiter": args.generations, "seed": 1, "verbose": -9})
    began = time.time()
    best = (math.inf, None, 0.0)
    with Pool(args.workers) as pool:
        while not es.stop():
            xs = es.ask()
            seed = es.countiter * 1000
            # Each candidate's falls split over workers in chunks of --chunk (the numpy controller's per-tick overhead dominates below about 128 envs).
            chunks = max(1, args.n // args.chunk)
            jobs = [(x, seed + c, args.n // chunks, task) for x in xs for c in range(chunks)]
            out = pool.map(evaluate, jobs)
            costs, safes = [], []
            for i in range(len(xs)):
                part = out[i * chunks:(i + 1) * chunks]
                costs.append(np.mean([p[0] for p in part]))
                safes.append(np.mean([p[1] for p in part]))
            es.tell(xs, costs)
            i = int(np.argmin(costs))
            if costs[i] < best[0]:
                best = (costs[i], decode(xs[i]), safes[i])
                args.out.write_text(json.dumps({k: round(v, 4) for k, v in best[1].items()}, indent=1) + "\n")
            print(f"generation {es.countiter} {time.time() - began:.0f} s: best this generation cost {costs[i]:.3f} safe {safes[i] * 100:.0f}% | best so far cost {best[0]:.3f} safe {best[2] * 100:.0f}%", flush=True)
    print("best", json.dumps({k: round(v, 3) for k, v in best[1].items()}), flush=True)


if __name__ == "__main__":
    main()
