"""
The firmware's tumble recovery and level-and-hold (fw/drone/main/recover.c) as flight_rt.c flies them, in sw/sim/flip_firmware.py's harness: uv run -m sw.sim.recover_firmware [--scenario tumble tilt landing abort flip] [--n 64] [--heights 2 3] [--workers 2] [--wide] [--set name=value,...]

Each run takes off in the GPS hold as flip_firmware does (its FLIGHT settings, recover_enable 1 unless the scenario says otherwise) and climbs to the scenario's height, then:
  tumble   20 s after liftoff the drone is put into a tumble like 2026-11-12 02:27:15 UTC's after the flip's hand-back: 90-180 degrees from upright about a random horizontal axis, pitching at 400-600 deg/s either way, rolling and yawing at a few hundred (normal, 200 and 150 deg/s), sinking at up to 2 m/s
  tilt     60-150 degrees from upright, turning at rest rates (20 deg/s), the vertical speed near zero
  landing  a link-loss landing (at least 1 m/s down) begins 15 s after liftoff, and at a drawn height of 0.5-2.5 m the drone is put 60-150 degrees over: recovery above 1 m (recover.h RECOVER_LAND_HEIGHT), the tip cut below; after a recovery the hold flies LINK_HOLD_S, then the landing starts again
  abort    recover_enable 0: the drone is put 20-60 degrees over turning at up to 100 deg/s and level-and-hold is asked for at once
  flip     the flip as flip_firmware flies it, with recovery on
The attitude estimate is put into the drawn attitude with it (about a degree of error), as the gyro would have carried it there; the hover's height and climb rate are not, as they are not on the drone either.

Recovered: never cut, never touched the ground after the event, upright within 10 degrees and turning under 1 rad/s at the end and above 0.5 m (landing: or landed by the hold). It prints the share recovered, cut (and why), touched, the height lost from the event to the lowest point in 4 s, and the time from the event to the hand-back to the hold.

The plant terms the 2026-11-12 flip showed to be uncertain, widened with --wide (or each on its own): the roll authority (recover.randomization's 0.8-5 times the fit, --roll-thrust 0.4 5 here: the flip's spin reached 540 deg/s on the drone against 950 in the sim, so the low end is plausible), the props' gyroscopic coupling, which rigid.py leaves out (--gyroscopic: up to this much angular momentum per root newton either way; a 10-inch prop with its motor bell is roughly 1e-4 kg m² at 600 rad/s near 4.5 N, about 0.03), an ESC deadband under which a motor stops and restarts from standstill (--deadband, throttle; airframe motor_start 0.16 s), and slower motors (--motor-tau factor).
"""

import argparse
import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording, settings as recording_settings
from sw.sim import flip_firmware as F
from sw.sim import rigid as R

SCENARIOS = ("tip", "tilt", "tumble", "landing", "abort", "flip")
DEG = math.pi / 180
# The event comes this long after liftoff (s), once the hold has climbed to its height, and each run goes on AFTER s past it; the link-loss landing starts at EVENT_AT.
EVENT_AT = 14.0
AFTER = 6.0
# Tuned with --tune on these, at both heights.
TUNED = ("tip", "tilt", "tumble")
# --tune's search: RecoverParams fields with their ranges (log scale where low > 0 and the third item is True; angles in radians).
SPACE = [("kp", 1.0, 20.0, True), ("brake", 5.0, 150.0, True), ("max_rate", 4.0, 30.0, True), ("k_roll", 2.0, 60.0, True), ("k_pitch", 2.0, 80.0, True), ("kd_yaw", 0.5, 20.0, True),
         ("roll_over", 90 * DEG, 175 * DEG, False), ("servo_max", 20 * DEG, 60 * DEG, False), ("kv", 0.5, 8.0, True), ("floor", 0.02, 1.0, True), ("idle", 0.03, 0.3, True),
         ("right_angle", 10 * DEG, 60 * DEG, False), ("right_rate", 0.5, 6.0, True)]


def tilted(rng, n, low, high):
    """Quaternions low-high degrees from upright about a random horizontal axis, at a random heading."""
    angle = rng.uniform(low, high, n) * DEG
    axis = rng.uniform(0, 2 * math.pi, n)
    heading = rng.uniform(0, 2 * math.pi, n)
    tilt = np.column_stack([np.cos(angle / 2), np.sin(angle / 2) * np.cos(axis), np.sin(angle / 2) * np.sin(axis), np.zeros(n)])
    yaw = np.column_stack([np.cos(heading / 2), np.zeros(n), np.zeros(n), np.sin(heading / 2)])
    return R.quat_multiply(yaw, tilt)


def tip(rng, n):
    return {"time": np.full(n, EVENT_AT), "quat": tilted(rng, n, 45, 75), "omega": rng.normal(0, 100, (n, 3)) * DEG, "climb": rng.normal(0, 0.3, n), "abort": np.zeros(n, bool)}


def tilt(rng, n):
    return {"time": np.full(n, EVENT_AT), "quat": tilted(rng, n, 60, 150), "omega": rng.normal(0, 20, (n, 3)) * DEG, "climb": rng.normal(0, 0.3, n), "abort": np.zeros(n, bool)}


def tumble(rng, n):
    pitch = rng.choice([-1, 1], n) * rng.uniform(400, 600, n)
    omega = np.column_stack([rng.normal(0, 200, n), pitch, rng.normal(0, 150, n)]) * DEG
    return {"time": np.full(n, EVENT_AT), "quat": tilted(rng, n, 90, 180), "omega": omega, "climb": rng.uniform(-2, 0.5, n), "abort": np.zeros(n, bool)}


def landing(rng, n):
    return {"height": rng.uniform(0.5, 2.5, n), "quat": tilted(rng, n, 60, 150), "omega": rng.normal(0, 50, (n, 3)) * DEG, "climb": np.full(n, -1.0), "abort": np.zeros(n, bool)}


def abort(rng, n):
    return {"time": np.full(n, EVENT_AT), "quat": tilted(rng, n, 20, 60), "omega": rng.uniform(-100, 100, (n, 3)) * DEG, "climb": rng.normal(0, 0.3, n), "abort": np.ones(n, bool)}


EVENTS = {"tip": tip, "tilt": tilt, "tumble": tumble, "landing": landing, "abort": abort}


def recovered(row, scenario):
    """Never cut, never touched the ground after the event, upright and still above 0.5 m at the end; a landing may also have landed."""
    return (row["cut"] == 0 and not row["event_touched"] and row["upright"] and row["end_height"] > 0.5) or (scenario == "landing" and row["landed"])


def jobs(scenario, height, settings, options, n, seed, off=False):
    """flip_firmware.fly's jobs for one scenario at one hold height."""
    flown = settings | {"recover_enable": 0 if off or scenario == "abort" else 1, "hover_climb": height, "hover_ceiling": max(height + 0.5, 1.8)}
    extra = dict(options)
    if scenario in EVENTS:
        extra["event"] = EVENTS[scenario]
    if scenario == "landing":
        extra["landing"] = EVENT_AT
    seconds = 50.0 if scenario == "flip" else EVENT_AT + AFTER + (4.0 if scenario == "landing" else 0.0)
    batch = 16
    return [(flown, 0.0, seed + s, batch, seconds, math.radians(2000), 2.0, 25.0, scenario == "flip", extra) for s in range(0, n, batch)]


def summary(rows, scenario):
    n = len(rows)
    if scenario == "flip":
        started = [r for r in rows if r["started"] is not None]
        caught = [r for r in started if r["recoveries"]]
        return (f"flips started {len(started)} of {n}; clean {np.mean([r['clean'] for r in started]) * 100:.1f}%, touched {np.mean([r['touched'] for r in started]) * 100:.1f}%, cut {np.mean([r['cut'] not in (0, 8) for r in started]) * 100:.1f}% "
                f"({', '.join(f'{c}: {sum(r['cut'] == c for r in started)}' for c in sorted({r['cut'] for r in started} - {0}))}); recovery started in {len(caught)} "
                f"(causes {sorted({c for r in caught for _, c in r['recoveries']})}), holding at the end {np.mean([r['held'] for r in started]) * 100:.1f}%; height lost median {np.median([r['lost'] for r in started]):.2f} m, max {max(r['lost'] for r in started):.2f}")
    events = [r for r in rows if r["event"] is not None]
    good = [recovered(r, scenario) for r in events]
    lost = np.array([r["event_height"] - r["event_lowest"] for r in events])
    times = np.array([r["recovered_at"] - r["event"] for r in events if r["recovered_at"] is not None])
    cuts = {}
    for r in events:
        if r["cut"] not in (0, 8):
            cuts[r["cut"]] = cuts.get(r["cut"], 0) + 1
    text = (f"events {len(events)} of {n}: recovered {np.mean(good) * 100:.1f}%, cut {sum(cuts.values())} {cuts}, touched the ground {np.mean([r['event_touched'] for r in events]) * 100:.1f}%; "
            f"height lost median {np.median(lost):.2f} m (90th {np.percentile(lost, 90):.2f}, max {lost.max():.2f}); "
            f"hand-back to the hold {np.median(times) if len(times) else float('nan'):.2f} s median (90th {np.percentile(times, 90) if len(times) else float('nan'):.2f})")
    if scenario == "landing":
        high = [g for g, r in zip(good, events) if r["event_height"] >= 1.0]
        low = [r for r in events if r["event_height"] < 1.0]
        text += f"; above 1 m recovered {np.mean(high) * 100 if high else float('nan'):.1f}% of {len(high)}, below it cut {sum(r['cut'] == 4 for r in low)} of {len(low)}"
    return text


def decode(x):
    return {name: math.exp(math.log(lo) + (math.log(hi) - math.log(lo)) * min(1.0, max(0.0, v))) if log else lo + (hi - lo) * min(1.0, max(0.0, v)) for (name, lo, hi, log), v in zip(SPACE, x)}


def encode(params):
    return [(math.log(params[name]) - math.log(lo)) / (math.log(hi) - math.log(lo)) if log else (params[name] - lo) / (hi - lo) for name, lo, hi, log in SPACE]


def tune(args, settings, options, pool):
    """CMA-ES over SPACE: each candidate flies the TUNED scenarios at both heights on fresh draws each generation; its cost is the share not recovered plus a fifth of the height lost over the hold's height."""
    import cma
    defaults = {name: getattr(F.RECOVER_DEFAULTS, name) for name, *_ in SPACE}
    start = encode(defaults | parse_settings(args.params))
    strategy = cma.CMAEvolutionStrategy([min(0.95, max(0.05, v)) for v in start], 0.2, {"popsize": args.population, "bounds": [0, 1], "seed": args.seed + 1, "verbose": -9})
    best = (math.inf, None)
    for generation in range(args.generations):
        candidates = strategy.ask()
        seed = 5000 + 1000 * generation + args.seed
        work, index = [], []
        for c, x in enumerate(candidates):
            extra = options | {"params": decode(x)}
            for scenario in TUNED:
                for height in args.heights:
                    for job in jobs(scenario, height, settings, extra, args.n, seed):
                        work.append(job)
                        index.append((c, scenario, height))
        results = pool.map(F.fly, work)
        costs = [[] for _ in candidates]
        for (c, scenario, height), rows in zip(index, results):
            for r in rows:
                if r["event"] is not None:
                    costs[c].append((0.0 if recovered(r, scenario) else 1.0) + 0.2 * min(1.0, (r["event_height"] - r["event_lowest"]) / height))
        means = [float(np.mean(c)) for c in costs]
        strategy.tell(candidates, means)
        k = int(np.argmin(means))
        if means[k] < best[0]:
            best = (means[k], decode(candidates[k]))
        print(f"generation {generation}: best {means[k]:.3f} median {np.median(means):.3f}; best so far {best[0]:.3f} {json.dumps({key: round(v, 4) for key, v in best[1].items()})}", flush=True)
    if args.out:
        Path(args.out).write_text(json.dumps({key: round(v, 4) for key, v in best[1].items()}, indent=1) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", nargs="*", default=list(SCENARIOS), choices=SCENARIOS)
    parser.add_argument("--n", type=int, default=64)
    parser.add_argument("--heights", type=float, nargs="*", default=[2.0, 3.0], help="hover_climb for the hold (m)")
    parser.add_argument("--settings", default="2026-11-11T23-35")
    parser.add_argument("--set", default="", help="settings on top, name=value,...")
    parser.add_argument("--params", default="", help="recover.c RecoverParams on top of recover_defaults, name=value,... (radians and SI)")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--wide", action="store_true", help="--roll-thrust 0.4 5 --gyroscopic 0.03 --deadband 60 --motor-tau 1.5")
    parser.add_argument("--roll-thrust", type=float, nargs=2)
    parser.add_argument("--gyroscopic", type=float, default=0.0)
    parser.add_argument("--deadband", type=float, default=0.0)
    parser.add_argument("--motor-tau", type=float, default=1.0)
    parser.add_argument("--off", action="store_true", help="recover_enable 0 (today's tip cut) for comparison")
    parser.add_argument("--tune", action="store_true", help="CMA-ES over the gains (SPACE) instead; --n runs per scenario and height per candidate")
    parser.add_argument("--generations", type=int, default=40)
    parser.add_argument("--population", type=int, default=16)
    parser.add_argument("--out", help="--tune: the best gains as JSON")
    args = parser.parse_args()
    if args.wide:
        args.roll_thrust, args.gyroscopic, args.deadband, args.motor_tau = args.roll_thrust or [0.4, 5.0], args.gyroscopic or 0.03, args.deadband or 60.0, args.motor_tau if args.motor_tau != 1.0 else 1.5
    settings = recording_settings(recording(args.settings)) | F.FLIGHT | parse_settings(args.set)
    options = {"gyroscopic": args.gyroscopic, "deadband": args.deadband}
    if args.roll_thrust:
        options["roll_thrust"] = tuple(args.roll_thrust)
    if args.motor_tau != 1.0:
        options["airframe"] = {"motor_tau": args.motor_tau, "right_motor_tau": args.motor_tau}
    print(f"plant: roll authority {options.get('roll_thrust', (0.8, 5.0))} x, gyroscopic ±{args.gyroscopic:g} N·m·s/√N, ESC deadband up to {args.deadband:g}, motor lag x{args.motor_tau:g}; params {args.params or 'recover_defaults'}", flush=True)
    with Pool(args.workers) as pool:
        if args.tune:
            tune(args, settings, options, pool)
            return
        options["params"] = parse_settings(args.params)
        for scenario in args.scenario:
            for height in [3.0] if scenario == "flip" else args.heights:
                rows = [r for part in pool.map(F.fly, jobs(scenario, height, settings, options, args.n, 2000 + args.seed, args.off)) for r in part]
                print(f"{scenario:8s} hold {height:g} m: {summary(rows, scenario)}", flush=True)


if __name__ == "__main__":
    main()
