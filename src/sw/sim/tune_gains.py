"""
Robust gain tuning on the identified airframe: uv run --with cma -m sw.sim.tune_gains <recording> [--airframe sw/sim/models/airframe_<day>.json] [--range 8] [--generations 50] [--workers N] [--seed 1] [--out result.json]

CMA-ES over the attitude and camera-hover gains, starting from the recording's settings (which also supply everything not tuned), each gain searched on a log scale from its flown value divided by --range to times it, within the firmware's limits (LIMITS, from fw/drone/main/settings.def). A candidate flies the camera hover (sweep.run with hover=True, on the ESC-recalibrated drone IDENTIFIED_ESC with its calibrated pushes and drifting forces) for 10 s on every variant below with three noise seeds, and scores as the mean of its runs plus WORST times its worst one (the worst alone is mostly set by the odd run that loses the tag): how far it strays from its hold over the last 7 s (the RMS range error from hover_distance plus the RMS sideways wander from where it was at 3 s) plus a share of the pitch error, with a run that loses the tag counting as up to 1 m and a crash as more than any flight.
The variants are what the identification cannot pin down, CoM shifts (COM_PUSH) the integrators must absorb, plus a 6 dB gain margin: the identified sim needs its pitch effectiveness ×1.5–2 before the 2026-10-27 pitch_p 30 / pitch_damping 12 hop (06-42-15) diverges as it did, while the p20/d8 and p30/d8 hops stay up, so tuning against the nominal model alone would go past what flies.
"""

import argparse
import json
import math
import os
import time
from multiprocessing import Pool
from pathlib import Path

from sw.fit import settings as recording_settings
from sw.sim import airframe as A
from sw.sim.sweep import IDENTIFIED_ESC, run

KEYS = ("pitch_p", "pitch_damping", "pitch_i", "roll_p", "roll_damping", "roll_i", "hover_kp", "hover_kd", "hover_kside", "hover_kyaw", "hover_tilt", "hover_trim_tau", "hover_trim_max")
# The settings' ranges in the firmware (settings.def), which the search stays within.
LIMITS = {"hover_kp": (0, 20), "hover_kd": (0, 20), "hover_kside": (0, 5), "hover_kyaw": (0, 10), "hover_tilt": (0, 15), "hover_trim_tau": (0.1, 20), "hover_trim_max": (0, 3)}
SEEDS = (0, 1, 2)
MODELS = Path(__file__).with_name("models")
# The steady torque of a CoM shift the gains must absorb without retrimming (rad/s²): on 2026-10-28 moving the battery changed the trim biases learn_bias found by up to ~5 in pitch and roll.
COM_PUSH = 5.0
# Share of a candidate's worst run in its score, on top of the mean of all its runs.
WORST = 0.3


def variants(a):
    """Airframe changes to be robust to, by name."""
    return {
        "nominal": {},
        "pitch gain ×2": {"tilt_gain": 2.0},
        "roll gain ×2": {"differential_slope": a["differential_slope"] * 2},
        # The motor assemblies' effective inertia ×1.5 too.
        "pitch gain ×0.7": {"tilt_gain": 0.7, "servo_inertia": a["servo_inertia"] + 0.5 * A.resolve(A.complete(a), A.geometry_of(a))["J"]},
        "roll gain ×0.7": {"differential_slope": a["differential_slope"] * 0.7},
        "servo +20 ms": {"servo_delay": a["servo_delay"] + 0.02, "right_servo_delay": a["right_servo_delay"] + 0.02},
        "motors alike": {"right_motor_delay": a["motor_delay"], "right_motor_tau": a["motor_tau"]},
        "no bands": {"left_band": 0.0, "right_band": 0.0},
        "push +1 sd": {"roll_push": a["roll_push"] + a.get("roll_push_sd", 2.6), "pitch_push": a["pitch_push"] + a.get("pitch_push_sd", 1.4)},
        # A moved battery: a steady torque the trim biases do not know about.
        **{f"CoM {name} {sign:+}": {f"{name}_push": a[f"{name}_push"] + sign * COM_PUSH} for name in ("pitch", "roll") for sign in (-1, 1)},
    }


# Horizontal pushes beyond the calibrated ones (m/s², forward and left: rotor wash off nearby walls, a level-reference error), added to the airframe variants' plants.
PUSHES = {"": (0.0, 0.0), ", wash forward": (1.0, 0.0), ", wash back": (-1.0, 0.0), ", wash left": (0.0, 0.7)}


def spread(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def fly(job):
    gains, plant, seed = job
    log = []
    r = run(gains, plant, seconds=10.0, seed=seed, log=log, hover=True)
    if not r["stable"]:
        return 10 + (10 - r["fail_s"])
    late = [e for e in log if e[0] >= 3.0]
    tag = plant["camera"]["tag"]
    hold = gains.get("hover_distance", 0.0)
    ranges = [math.hypot(tag[0] - e[1][0], tag[1] - e[1][1]) - hold for e in late]
    across = [e[1][1] - late[0][1][1] for e in late]
    # Lost tags count as drift: a run that never sees the tag scores as 1 m.
    return rms(ranges) + rms(across) + 0.02 * spread([math.degrees(e[2][1] - e[6]) for e in late]) + (1 - r["seen"])


def score(costs):
    return sum(costs) / len(costs) + WORST * max(costs)


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="a flight whose settings to start from (.bin or .jsonl)")
    parser.add_argument("--airframe", type=Path, help="default: the newest sw/sim/models/airframe_*.json")
    parser.add_argument("--range", type=float, default=8.0, help="search each gain from its flown value over this to times this")
    parser.add_argument("--generations", type=int, default=50)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    import cma

    airframe = json.loads((args.airframe or sorted(MODELS.glob("airframe_2*.json"))[-1]).read_text())
    plants = {name + extra: IDENTIFIED_ESC | {"airframe": airframe | change, "disturbances": IDENTIFIED_ESC["disturbances"] + [{"channel": c, "kind": "constant", "value": v} for c, v in zip(("forward", "left"), push) if v]}
              for name, change in variants(airframe).items() for extra, push in PUSHES.items() if name == "nominal" or not extra}
    base = recording_settings(args.recording.with_suffix(".jsonl"))
    flown = {key: base[key] for key in KEYS}
    # Gains at zero are searched from a small floor so the log scale can move them.
    centre = {key: max(value, 0.05) for key, value in flown.items()}

    def gains(x):
        return {key: min(max(centre[key] * args.range ** max(-1.0, min(1.0, v)), LIMITS.get(key, (0, math.inf))[0]), LIMITS.get(key, (0, math.inf))[1]) for key, v in zip(KEYS, x)}

    with Pool(args.workers) as pool:
        def evaluate(candidates):
            jobs = [(base | g, plant, seed) for g in candidates for plant in plants.values() for seed in SEEDS]
            costs = pool.map(fly, jobs, chunksize=2)
            n = len(plants) * len(SEEDS)
            return [costs[i * n:(i + 1) * n] for i in range(len(candidates))]

        flown_costs = evaluate([flown])[0]
        print("flown", {k: round(v, 2) for k, v in flown.items()}, f"cost {score(flown_costs):.3f}", flush=True)
        for j, name in enumerate(plants):
            print(f"  {name}: " + ", ".join(f"{c:.2f}" for c in flown_costs[j * len(SEEDS):(j + 1) * len(SEEDS)]), flush=True)
        es = cma.CMAEvolutionStrategy([0.0] * len(KEYS), 0.25, {"bounds": [-1, 1], "popsize": 14, "maxiter": args.generations, "seed": args.seed, "verbose": -9})
        start = time.time()
        while not es.stop():
            xs = es.ask()
            es.tell(xs, [score(c) for c in evaluate([gains(x) for x in xs])])
            print(f"generation {es.countiter} {time.time() - start:.0f} s: best {es.result.fbest:.3f}", {k: round(v, 2) for k, v in gains(es.result.xbest).items()}, flush=True)
        best = gains(es.result.xbest)
        best_costs = evaluate([best])[0]
    print("best", {k: round(v, 2) for k, v in best.items()}, f"cost {score(best_costs):.3f}", flush=True)
    for j, name in enumerate(plants):
        print(f"  {name}: " + ", ".join(f"{c:.2f}" for c in best_costs[j * len(SEEDS):(j + 1) * len(SEEDS)]), flush=True)
    if args.out:
        args.out.write_text(json.dumps({"best": best, "cost": score(best_costs), "flown": flown}, indent=1) + "\n")


if __name__ == "__main__":
    main()
