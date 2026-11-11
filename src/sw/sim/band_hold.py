"""
The tag hold with the rubber bands on the servos, in the hover sim: uv run -m sw.sim.band_hold [--recording 2026-11-11T23-35] [--seeds 16] [--seconds 30] [--workers 40] [--cases name,...] [--set name=value,...]

Each case flies a recording's settings (drone 1 on 2026-11-11) on sweep.CURRENT from its resting pitch with the tag ahead, as the hops did, for --seconds after liftoff, and reports over the last half: the range error from the target (median of the runs' means, and of their RMS), the camera's speed toward the tag, and at the end the hold's push and carry (what the drone logs as its trim), and runs lost. A case is a plant (bands or not) and a firmware (hover.c as it is, or with HOVER_DEFINES):
- the bands, as measured on 2026-11-11 (models/NOTES.md): the assemblies' real tilt behind the command (plant tilt_offset), and the accelerometer in flight reading forward of the attitude estimate (plant flight_accel_bias forward), which the hold's level acceleration then reads as a steady forward push;
- without them, the same plant with neither.
Run per case in a worker pool; the same seeds in every case.
"""

import argparse
import os
from multiprocessing import Pool

import numpy as np

from sw.flight_log import parse_settings, read_flight, recording, settings

# The bands (2026-11-11 23:17-23:35 UTC against 22:57-23:01): the specific force in body axes sits 2.2-2.7 degrees further back from the commanded thrust tilt than without them, and the attitude estimate 0.6-2.4 degrees nose-down of the accelerometer's tilt (0.1-0.4 m/s² of level acceleration, 0.01-0.04 g).
BAND_TILT = -2.5
BAND_ACCEL = 0.025
TAG_HEIGHT = 0.75


def rest_pitch(path):
    """The pitch the drone rested at before the motors started (degrees)."""
    data, _ = read_flight(path)
    still = data[data.left_throttle < 100]
    return float(still.pitch_deg.iloc[:100].median())


def fly(job):
    """One run: the case's name, seed and measures."""
    name, flown, plant_extra, defines, seconds, seed, rest, tag = job
    if defines:
        os.environ["HOVER_DEFINES"] = defines
    from sw.sim.sweep import CURRENT, run
    plant = CURRENT | {"ground_pitch": rest} | plant_extra
    plant["camera"] = dict(plant["camera"]) | {"tag": (tag, 0.0, TAG_HEIGHT)}
    log = []
    result = run(flown, plant, seconds=seconds + 2.0, seed=seed, log=log, hover=True)
    t = np.array([e[0] for e in log])
    height = np.array([e[1][2] for e in log])
    lifted = next((t[k] for k in range(len(t)) if height[k] > 0.05), None)
    if lifted is None:
        return name, seed, None
    late = (t >= lifted + seconds / 2) & (t <= lifted + seconds)
    # Lost before the measured half began.
    if not late.any():
        return name, seed, {"lost": True}
    position = np.array([e[1][:2] for e in log])
    distance = np.hypot(tag - position[:, 0], position[:, 1])
    target = np.array([e[8][2] if e[8] else np.nan for e in log]) - flown.get("nudge_forward", 0.0)
    closing = -np.gradient(distance, t)
    extra = log[np.flatnonzero(late)[-1]][15]
    return name, seed, {
        "error": float(np.nanmean(distance[late] - target[late])),
        "rms": float(np.sqrt(np.nanmean((distance[late] - target[late]) ** 2))),
        "closing": float(np.mean(closing[late])),
        "push": float(extra[0][0]), "carry": float(extra[1][0]), "drag_bias": float(extra[2][0]),
        "lost": not result["stable"],
    }


CASES = {
    "no bands": ({}, ""),
    "bands": ({"tilt_offset": BAND_TILT, "flight_accel_bias": (BAND_ACCEL, 0.0, 0.0)}, ""),
    "band tilt only": ({"tilt_offset": BAND_TILT}, ""),
    "band accel only": ({"flight_accel_bias": (BAND_ACCEL, 0.0, 0.0)}, ""),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recording", default="2026-11-11T23-35")
    parser.add_argument("--seeds", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--workers", type=int, default=40)
    parser.add_argument("--cases", default=",".join(CASES))
    parser.add_argument("--set", default="", help="settings flown instead, name=value,...")
    parser.add_argument("--define", default="", help="HOVER_DEFINES for every case (hover.c compile-time constants)")
    args = parser.parse_args()
    path = recording(args.recording)
    flown = settings(path) | parse_settings(args.set)
    rest = rest_pitch(path)
    tag = flown["hover_distance"]
    names = args.cases.split(",")
    jobs = [(name, flown, CASES[name][0], args.define or CASES[name][1], args.seconds, seed, rest, tag) for name in names for seed in range(args.seeds)]
    # Build the hover library once before the workers load it.
    if args.define:
        os.environ["HOVER_DEFINES"] = args.define
    import sw.sim.hover  # noqa: F401
    with Pool(min(args.workers, len(jobs))) as pool:
        runs = pool.map(fly, jobs)
    print(f"{args.recording} settings{' with ' + args.set if args.set else ''}{' and ' + args.define if args.define else ''}, {args.seeds} seeds, last {args.seconds / 2:.0f} of {args.seconds:.0f} s")
    print(f"{'case':<20} {'range error m':>14} {'RMS m':>6} {'closing m/s':>12} {'push':>6} {'carry':>6} {'drag_bias':>9} {'lost':>5}")
    for name in names:
        measured = [m for n, _, m in runs if n == name and m is not None]
        kept = [m for m in measured if not m["lost"]]
        def med(key):
            return np.median([m[key] for m in kept]) if kept else float("nan")
        print(f"{name:<20} {med('error'):>+14.2f} {med('rms'):>6.2f} {med('closing'):>+12.3f} {med('push'):>+6.2f} {med('carry'):>+6.2f} {med('drag_bias'):>+9.2f} {len(measured) - len(kept):>3}/{len(measured)}")


if __name__ == "__main__":
    main()
