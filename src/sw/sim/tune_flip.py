"""
Tune the firmware's flip (fw/drone/main/flip.c) as flown from the GPS hold (sw/sim/flip_firmware.py) by CMA-ES, on plants that span what the first real flip left uncertain: uv run -m sw.sim.tune_flip [--generations 40] [--n 64] [--workers 40] [--out models/flip_params_<day>.json]; --check params.json [--n 512] flies a parameter set and prints flip_firmware's summary.

Every candidate flies the same n drawn plants (a quarter of them the staged 60- and 90-degree bursts, flip_enable 2 and 3) (flip_firmware.uncertain: every term the flip leans on that nothing real tests, over its whole plausible range, on models/airframe_2026-11-12_flip.json, the plant fitted to the first flip) from a 5 m hold with the bands' tilt offset drawn from 0 to -5 degrees. Its cost is the mean over runs of: 20 for touching the ground, 3 for not ending upright and still, twice the height lost over 1 m, the throttles' travel over TRAVEL, the time either motor spends at FULL throttle or more, twice the climb above the start past CLIMB_ROOM, and twice the hand-back past HANDBACK s after the start: a flip that climbs metres first or turns slowly for seconds flies a different maneuver in regimes nothing has measured. The motors' limits are not searched: each keeps at least FLOOR of the weight while the flip owns them and throttles slew the full range in no less than 0.12 s with a cap of THROTTLE_CAP (flip.c flip_limit), so no draw can win by idling a motor or by full throttle.
"""

import argparse
import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording, settings as recording_settings
from sw.sim import flip_firmware as FF

MODELS = Path(__file__).with_name("models")
AIRFRAME = "airframe_2026-11-12_flip.json"
# The hold at 5 m (settings.def now allows hover_climb 5 and hover_ceiling 5.5) and the flip from 4.5.
HIGH = {"hover_climb": 5.0, "hover_ceiling": 5.5, "flip_height": 4.5}
# The tuning climbs there faster than the drone would (the climb is not what is tuned).
FAST_CLIMB = {"hover_climb_rate": 1.0}
# Fixed motor limits (flip.c FlipParams): each motor at least FLOOR of the weight (0.2 of 10.3 N is 2.1 N, about throttle 230 on the thrust table at the 1.3 thrust scale: the ESC keeps driving it, above the coast_below 150 where the first flip's cut motor coasted, and twice what the table's foot near throttle 100 gives), the full throttle range in no less than 0.12 s (about twice the motors' 0.066 s lag), at most throttle 1100 (92%).
FLOOR, SLEW, THROTTLE_CAP = 0.2, 10000.0, 1100.0
# Searched: (name, low, high, log scale). The rate loops' gains (k_spin, k_roll, k_pitch, kd_yaw, kp, brake) stop where the motors' 0.066 s lag and the slew's 0.12 s for the full range would make them chatter: a search left free took them there and the throttles swung between 200 and 1100 every 0.1 s.
SPACE = [
    ("climb_thrust", 1.2, 2.2, False), ("climb_time", 0.05, 0.35, True), ("spin_rate", 4.0, 26.0, True), ("k_spin", 2.0, 12.0, True), ("spin_thrust", 0.4, 2.0, False), ("spin_lift", 0.0, 0.8, False), ("handoff", 1.0, 3.0, False),
    ("kp", 1.0, 4.0, True), ("brake", 1.0, 20.0, True), ("max_rate", 3.0, 14.0, True), ("k_roll", 2.0, 8.0, True), ("k_pitch", 1.0, 20.0, True), ("kd_yaw", 0.5, 8.0, True),
    ("flip_thrust", 0.8, 2.2, False), ("right_angle", 0.2, 0.6, False), ("servo_max", 0.1, 0.9, False), ("kv", 2.0, 15.0, True), ("ceiling", 0.6, 1.4, False), ("handoff_rate", 1.0, 3.0, True),
    # The levers against the rotors' momentum (2026-11-13): the spin's pitch and yaw rate gains apart from its roll gain, the gyroscopic pitch fed forward (0 to twice the physics' 0.018 / 0.0044), the motors' floor raised (a smaller speed difference between the rotors), and a later timeout.
    ("spin_pitch", 2.0, 25.0, True), ("spin_yaw", 0.5, 12.0, True), ("gyro_ff", 0.0, 8.0, False), ("floor", 0.2, 0.6, False), ("timeout", 1.5, 3.0, False),
]
FIXED = {"slew": SLEW, "throttle_cap": THROTTLE_CAP}
CLIMB_ROOM, HANDBACK = 0.8, 1.3
# Throttle travel through the flip (both motors' summed changes) that costs as much as a metre of height: the swings between low and high that load the ESCs.
TRAVEL = 5000.0
STAGED = 4
# The robust tuning (2026-11-13): rotor_momentum anywhere from 0 to MOMENTUM_TOP (the replay's upper bound; physics gives 0.015-0.02 give or take 40%), in bands of 0.01 for the cost's worst case, and the left motor at LEFT_LOW to 1 of the right's.
MOMENTUM_TOP = 0.05
MOMENTUM_BANDS = [(0.01 * k, 0.01 * (k + 1)) for k in range(5)]
LEFT_LOW = 0.8
# The attitude controller takes back a drone within right_angle (at most 0.6 rad) of upright turning under handoff_rate (at most 3 rad/s), and every 20 degrees it then tilts past AFTER_TILT costs as much as a metre of height: the controller works in Euler angles, which go ambiguous near 90 degrees of pitch (at 02:27 the estimate jumped from roll 58, pitch 87 to roll 163, pitch 93 and it flew that as 140 degrees of error), so the flip's own quaternion righting keeps the drone until it is nearly level and slow.
AFTER_TILT = 40.0
SETTINGS = "2026-11-12T02-27-04"
BANDS = (0.0, -2.5, -5.0)


def decode(x):
    out = {}
    for (name, low, high, log), value in zip(SPACE, np.clip(x, 0, 1)):
        out[name] = math.exp(math.log(low) + (math.log(high) - math.log(low)) * value) if log else low + (high - low) * value
    return out | FIXED


def encode(params):
    return np.array([(math.log(params[name] / low) / math.log(high / low)) if log else (params[name] - low) / (high - low) for name, low, high, log in SPACE]).clip(0, 1)


def jobs(params, n, seed, settings, batch=4, flip_at=7.0, seconds=17.0, randomize="flip", airframe=AIRFRAME, staged=False, robust=False, band=None):
    """
    flip_firmware.fly jobs: n runs in batches (with randomize "flip" each batch its own draw of the untested terms), the band offset cycling through BANDS.
    robust: each batch's rotor_momentum is set across 0 to MOMENTUM_TOP (a golden-ratio sequence, so any n spreads it evenly) in place of the draw, and its left motor gives LEFT_LOW to 1 of the right's thrust (a weak left ESC, as right_thrust_gain); band (an index into MOMENTUM_BANDS) keeps rotor_momentum within that band instead.
    """
    out = []
    golden = (math.sqrt(5) - 1) / 2
    for k, start in enumerate(range(0, n, batch)):
        offset = BANDS[k % len(BANDS)]
        # With staged, every STAGED-th batch flies a staged burst instead (flip_enable 2 or 3, aimed at 60 or 90 degrees), so the tests before the flip are tuned for too.
        flown = settings | {"flip_enable": 2 + k % 2} if staged and k % STAGED == STAGED - 1 else settings
        options = {}
        if robust or band is not None:
            spread = (seed * 0.001 + k * golden) % 1
            low, high = (0.0, MOMENTUM_TOP) if band is None else MOMENTUM_BANDS[band]
            left = 1 - (1 - LEFT_LOW) * ((seed * 0.001 + k * golden * golden) % 1)
            options = {"set": {"rotor_momentum": low + (high - low) * spread}, "airframe": {"right_thrust_gain": 1 / left}}
        out.append((flown, offset, seed + start, batch, seconds, math.radians(2000), 2.0, flip_at, True, airframe, randomize, params, options))
    return out


def cost(rows):
    total = []
    for r in rows:
        if r["started"] is None:
            total.append(15.0)
            continue
        total.append(20.0 * r["touched"] + 3.0 * (not r["clean"]) + 2.0 * max(0.0, r["lost"] - 1.0) + r["full"] + 2.0 * max(0.0, r["climbed"] - CLIMB_ROOM) + 2.0 * max(0.0, r["handed"] - HANDBACK) + r["travel"] / TRAVEL + max(0.0, r["after_tilt"] - AFTER_TILT) / 20)
    return float(np.mean(total))


def robust_cost(rows):
    """The worst rotor_momentum band's mean cost, plus half the mean over all: the flip has to hold across the band nothing real has pinned down, not on average."""
    bands = {}
    for r in rows:
        index = min(len(MOMENTUM_BANDS) - 1, int(r["momentum"] / (MOMENTUM_TOP / len(MOMENTUM_BANDS))))
        bands.setdefault(index, []).append(r)
    return max(cost(group) for group in bands.values()) + 0.5 * cost(rows)


def evaluate(pool, candidates, n, seed, settings, **options):
    batches = [jobs(c, n, seed, settings, **options) for c in candidates]
    flat = [job for b in batches for job in b]
    results = pool.map(FF.fly, flat, chunksize=1)
    out, i = [], 0
    for b in batches:
        rows = []
        for job, batch_rows in zip(b, results[i:i + len(b)]):
            for r in batch_rows:
                r["momentum"] = job[12].get("set", {}).get("rotor_momentum", math.nan)
            rows += batch_rows
        i += len(b)
        out.append((robust_cost(rows) if options.get("robust") else cost(rows), rows))
    return out


def summary(rows):
    started = [r for r in rows if r["started"] is not None]
    lost = np.array([r["lost"] for r in started])
    full = np.array([r["full"] for r in started])
    peak = np.array([r["peak_current"] for r in started])
    mean = np.array([r["mean_current"] for r in started])
    climbed = np.array([r["climbed"] for r in started])
    rates = np.array([r["rates"] for r in started])
    turned = np.array([r["turned"] for r in started])
    handed = np.array([r["handed"] for r in started])
    travel = np.array([r["travel"] for r in started])
    past90 = np.array([r["past90"] for r in started])
    after = np.array([r["after_tilt"] for r in started])
    return (f"past 90 deg {np.median(past90):.2f} s median (90th {np.percentile(past90, 90):.2f}); tilt after the hand-back median {np.median(after):.0f} deg (90th {np.percentile(after, 90):.0f}, max {after.max():.0f}); throttle travel {np.median(travel):.0f} median (90th {np.percentile(travel, 90):.0f}); turned {np.median(turned):.0f} deg median; peak rates roll/pitch/yaw median {np.median(rates, 0).round(0)} deg/s (90th {np.percentile(rates, 90, axis=0).round(0)}); climbed median {np.median(climbed):.2f} m (max {climbed.max():.2f}), handed back at {np.median(handed):.2f} s median (90th {np.percentile(handed, 90):.2f}); {len(started)} of {len(rows)} started; clean {np.mean([r['clean'] for r in started]) * 100:.1f}%, touched {np.mean([r['touched'] for r in started]) * 100:.1f}%, tip cut {np.mean([r['tipped'] for r in started]) * 100:.1f}%; "
            f"height lost median {np.median(lost):.2f} m, 90th {np.percentile(lost, 90):.2f}, worst {lost.max():.2f}; at full throttle median {np.median(full):.2f} s, max {full.max():.2f}; current peak median {np.median(peak):.0f} A (max {peak.max():.0f}), mean {np.median(mean):.1f} A (max {mean.max():.1f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generations", type=int, default=40)
    parser.add_argument("--n", type=int, default=64)
    parser.add_argument("--population", type=int, default=16)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--start", type=Path, help="a parameter file to start from (default flip.c's flip_defaults)")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--check", type=Path, help="fly this parameter file on --n fresh plants and print the summary instead of tuning")
    parser.add_argument("--seed", type=int, default=20000)
    parser.add_argument("--randomize", choices=("flip", "wide", "none"), default="flip", help="with --check: the plants (flip_firmware.py --randomize)")
    parser.add_argument("--airframe", default=AIRFRAME, help="with --check: the plant's airframe")
    parser.add_argument("--set", default="", help="with --check: settings on top, name=value,... (flip_enable=2 or 3 for the staged 60- and 90-degree bursts)")
    parser.add_argument("--bands", action="store_true", help="with --check: --n runs in each rotor_momentum band (MOMENTUM_BANDS), the left motor at LEFT_LOW to 1 of the right's, a summary per band")
    parser.add_argument("--robust", action="store_true", help="tune for the worst rotor_momentum band (robust_cost) over 0 to MOMENTUM_TOP with the left motor at LEFT_LOW to 1")
    args = parser.parse_args()
    settings = recording_settings(recording(SETTINGS)) | FF.FLIGHT | HIGH | FAST_CLIMB | parse_settings(args.set)
    defaults = {name: getattr(FF.DEFAULTS, name) for name, *_ in FF.Params._fields_}
    with Pool(args.workers) as pool:
        if args.check:
            params = defaults | json.loads(args.check.read_text())
            if args.bands:
                for band, (low, high) in enumerate(MOMENTUM_BANDS):
                    value, rows = evaluate(pool, [params], args.n, args.seed, settings, randomize=args.randomize, airframe=args.airframe, band=band)[0]
                    print(f"{args.check} rotor_momentum {low:.2f}-{high:.2f}: cost {value:.3f}; {summary(rows)}", flush=True)
                return
            value, rows = evaluate(pool, [params], args.n, args.seed, settings, randomize=args.randomize, airframe=args.airframe)[0]
            print(f"{args.check}: cost {value:.3f}; {summary(rows)}")
            return
        import cma
        start = defaults | (json.loads(args.start.read_text()) if args.start else {})
        for name, low, high, _ in SPACE:
            start[name] = min(max(start[name], low), high)
        strategy = cma.CMAEvolutionStrategy(encode(start), 0.2, {"bounds": [0, 1], "popsize": args.population, "seed": 1, "verbose": -9})
        best = (math.inf, None)
        for generation in range(args.generations):
            xs = strategy.ask()
            # Every generation flies fresh plants, so the result generalizes rather than fitting one set.
            results = evaluate(pool, [decode(x) for x in xs], args.n, args.seed + 1000 * generation, settings, staged=True, robust=args.robust)
            strategy.tell(xs, [r[0] for r in results])
            mean_params = decode(strategy.mean)
            print(f"generation {generation}: best {min(r[0] for r in results):.3f}, median {np.median([r[0] for r in results]):.3f}; {summary(results[int(np.argmin([r[0] for r in results]))][1])}", flush=True)
            if args.out:
                args.out.write_text(json.dumps({k: round(v, 4) for k, v in mean_params.items()}, indent=1) + "\n")


if __name__ == "__main__":
    main()
