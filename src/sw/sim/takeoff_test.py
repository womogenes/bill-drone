"""
Takeoffs from a tilted rest on drone 1's forward legs (since 2026-11-11): uv run -m sw.sim.takeoff_test [--rests -12,-8,-4,0,4,8] [--seeds 8] [--recording 2026-11-12T01-31-56] [--workers 1]

Each run flies the recording's settings (its hover takeoff, outdoors: no camera, the GPS hold and the barometer as flown) on sweep.CURRENT_AIRFRAME's plant from a rest on LEG_FEET, the floor sloped under it so it rests at the given pitch (positive nose-down), with the floor's friction drawn from 0.7-1.5 (--friction), and reports how far the pitch went past the rest (away from level) while the feet are still within 5 cm of the floor, the peak pitch rate then, the largest tilt in the first 1.5 s and whether it was lost. On 2026-11-12 (01:31:56 from -11.5 degrees, 01:05:26 from +7.6, 00:53:45 from -12.5, 01:07:43 from +4.5) the drone pitched 20-30 degrees further from its rest at up to 155 degrees/s before its feet were 4 cm up; from within 3 degrees of level it stayed within 6.
"""

import argparse
import math
import random
from concurrent.futures import ProcessPoolExecutor

# The forward legs as fitted on 2026-11-11 (not measured; from the photos and the resting attitude): feet 80 mm ahead of the CoM and the battery tray's rear edge 54.5 mm behind it, both 73.5 mm below it, so it rests level on a flat floor; then the tray's tipped-over points (sweep.TRAY_FEET).
LEG_FEET = [(0.08, 0.075, -0.0735), (0.08, -0.075, -0.0735), (-0.0545, 0.022, -0.0735), (-0.0545, -0.022, -0.0735),
            (0.117, 0.03, 0.023), (0.117, -0.03, 0.023), (-0.135, 0.03, 0.039), (-0.135, -0.03, 0.039), (0.0, 0.19, 0.003), (0.0, -0.19, 0.003)]
RECORDING = "2026-11-12T01-31-56"
# Off the floor: the feet this far up (m).
CLEAR = 0.05


def fly(job):
    rest, seed, recording_name, friction = job
    from sw.flight_log import recording, settings
    from sw.sim import sweep

    flown = settings(recording(recording_name))
    rng = random.Random(seed + 7)
    # The floor slopes down ahead for a nose-down rest; a little sideways slope too.
    plant = sweep.CURRENT | {"feet": LEG_FEET, "friction": rng.uniform(*friction), "ground_slope": (-math.tan(math.radians(rest)), math.tan(math.radians(rng.uniform(-2, 2)))),
                             "imu_range": (250.0, 2.0), "tumble": 60.0, "guard": False}
    if not flown.get("hover_camera", 1):
        plant["gps"] = sweep.GPS if hasattr(sweep, "GPS") else plant.get("gps")
        plant["baro"] = sweep.BARO
    log = []
    result = sweep.run(flown, plant, seconds=2.5, seed=seed, log=log, hover=True)
    lowest = None
    worst_ground, rate_ground, worst, lifted = 0.0, 0.0, 0.0, None
    start = None
    for k, entry in enumerate(log):
        t, position, angles = entry[0], entry[1], entry[2]
        if lowest is None:
            lowest = position[2]
        pitch, roll = math.degrees(angles[1]), math.degrees(angles[0])
        if start is None:
            start = pitch
        rate = math.degrees((log[k][2][1] - log[k - 1][2][1]) / (t - log[k - 1][0])) if k else 0.0
        up = position[2] - lowest
        if lifted is None and up > CLEAR:
            lifted = t
        if lifted is None:
            worst_ground = max(worst_ground, abs(pitch) - abs(start))
            rate_ground = max(rate_ground, abs(rate))
        if lifted is None or t - lifted < 1.5:
            worst = max(worst, abs(pitch), abs(roll))
    return rest, seed, {"ground": worst_ground, "rate": rate_ground, "worst": worst, "lost": not result["stable"], "lifted": lifted, "start": start}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--rests", default="-12,-8,-4,0,4,8")
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--recording", default=RECORDING)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--friction", default="0.7,1.5", help="the floor's friction is drawn from this range; on grass the feet grip (try 3,6)")
    args = parser.parse_args()
    rests = [float(r) for r in args.rests.split(",")]
    friction = tuple(float(f) for f in args.friction.split(","))
    jobs = [(rest, seed, args.recording, friction) for rest in rests for seed in range(args.seeds)]
    with ProcessPoolExecutor(args.workers) as pool:
        results = list(pool.map(fly, jobs))
    print("rest   tilt past the rest on the floor (median, max)   rate there (median, max)   tilt in 1.5 s (median, max)   lost")
    for rest in rests:
        rows = [r for x, _, r in results if x == rest]
        g = sorted(r["ground"] for r in rows)
        q = sorted(r["rate"] for r in rows)
        w = sorted(r["worst"] for r in rows)
        print(f"{rest:+5.1f}  {g[len(g) // 2]:5.1f} {g[-1]:5.1f}   {q[len(q) // 2]:6.0f} {q[-1]:6.0f}   {w[len(w) // 2]:5.1f} {w[-1]:5.1f}   {sum(r['lost'] for r in rows)}/{len(rows)}  (sim rest {rows[0]['start']:+.1f})")


if __name__ == "__main__":
    main()
