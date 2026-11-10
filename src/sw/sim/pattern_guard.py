"""
The test pattern's limits (fw/drone/main/guard.c guard_pattern) in the hover sim: uv run -m sw.sim.pattern_guard [--runs 24] [--workers N] [--plot dir]

Each case flies a test pattern once, 12 s after collective first reached integrate_throttle, in the hover sim with the firmware's guards on at settings.def's defaults. On the tag (sweep.IDENTIFIED_ESC and the 2026-10-29 07:20 UTC camera hover's settings, as landing_test flies; sweep.CURRENT on drone 1's hop settings swings 10-20 degrees and loses the tag every few seconds in the sim, so it cannot tell a pattern's loss from its own) the pattern is the 2026-11-02 05:30 UTC hover's (a ±2 degree pitch chirp over 8 s) or sw/excite.py's speed pattern (the hold point 1.5 m out and back); without it, on the GPS hold outdoors (sw/sim/outdoor.py's plant, 2 m/s wind, the 2026-11-09 06:38 UTC settings), that flight's (±8 degree pitch then roll doublets, 0.3 s each way, 5 s apart).
  chirp       on the tag, undisturbed: the pattern should play to its end
  speed       on the tag, the hold point flown 1.5 m away and back by the pattern itself: it should play to its end
  pushed      the chirp with a 2 m/s² push away from the tag (more than the hold's 15 degrees can hold for long) from about 1.5 s into it (a gust): it should stop for distance
  stick       the chirp with the laptop's arrow held from about 2 s into it: it should stop for the stick
  lost        the chirp with the camera blind from about 3 s into it: it should stop for the lost tag
  gps         the doublets on the GPS hold: position from the GPS-corrected track
  still       a pattern of nothing for as long on the GPS hold: how often the hold's own wander leaves the box
  tilt        as gps with pattern_tilt 8 degrees: it should stop for tilt
It prints per case how the runs ended (played to the end, or stopped and why, with the median time into the pattern), and with --plot saves each case's first run: distance from where the pattern meant the drone to be, with the pattern's span and the stop.
"""

import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import recording, test_pattern
from sw.flight_log import settings as recording_settings

TAG_SETTINGS = "2026-10-29T07-20-47"
GPS_SETTINGS = "2026-11-09T06-38-51"
AFTER = 12.0
CHIRP_FROM = "2026-11-02T05-30-45"
DOUBLETS = {"after": AFTER, "repeat": 1, "segments": [{"ticks": 30, "pitch": 8.0}, {"ticks": 30, "pitch": -8.0}, {"ticks": 500}, {"ticks": 30, "roll": 8.0}, {"ticks": 30, "roll": -8.0}, {"ticks": 500}]}
SPEED = {"after": AFTER, "repeat": 1, "segments": [{"ticks": 300, "forward": -0.5}, {"ticks": 300}, {"ticks": 300, "forward": 0.5}, {"ticks": 300}]}
STILL = {"after": AFTER, "repeat": 1, "segments": [{"ticks": 1120}]}
CASES = ("chirp", "speed", "pushed", "stick", "lost", "gps", "tilt", "still")
SECONDS = 30.0


def plant_for(case, seed):
    from sw.sim.sweep import IDENTIFIED_ESC

    if case in ("gps", "tilt", "still"):
        from sw.sim.airframe import model
        from sw.sim.outdoor import plant

        p, _ = plant(model("airframe_2026-11-08b.json"), 2.0, seed, True, False, 1.2, speed_draw=(0.7, 1.4), baro=True)
    else:
        p = dict(IDENTIFIED_ESC)
    chirp = test_pattern(recording(CHIRP_FROM)) | {"after": AFTER, "repeat": 1}
    p = dict(p) | {"excite": SPEED if case == "speed" else STILL if case == "still" else DOUBLETS if case in ("gps", "tilt") else chirp}
    # The pattern starts about AFTER + 1 s from the run's start (liftoff, then collective reaching integrate_throttle).
    if case == "pushed":
        p["disturbances"] = list(p["disturbances"]) + [{"channel": "forward", "kind": "ramp", "rate": -20.0, "start": AFTER + 2.0, "limit": -2.0}]
    if case == "stick":
        p["nudge"] = (AFTER + 2.0, 0.0, 0.5, 2.0)
    if case == "lost":
        p["camera_lost"] = AFTER + 3.0
    return p


def fly(job):
    from sw.sim.sweep import run

    case, seed = job
    if case in ("gps", "tilt", "still"):
        settings = recording_settings(recording(GPS_SETTINGS)) | {"hover_camera": 0, "hover_center": 0, "hover_climb": 1.2, "hover_ceiling": 1.2, "hover_gps_tau": 0.5, "hover_gps_place_tau": 2.0, "hover_baro_tau": 2.0}
    else:
        settings = recording_settings(recording(TAG_SETTINGS))
    if case == "tilt":
        settings = settings | {"pattern_tilt": 8.0}
    p = plant_for(case, seed)
    log = []
    result = run(settings, p, seconds=SECONDS, seed=seed, log=log, hover=True)
    t = np.array([e[0] for e in log])
    position = np.array([e[1] for e in log])
    lifted = result.get("liftoff_s") or 0.0
    return {"case": case, "seed": seed, "stable": result["stable"], "stop": result.get("pattern_stop"), "start": result.get("pattern_start"), "lifted": lifted, "t": t.tolist(), "position": position.tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=24)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--cases", default=",".join(CASES))
    parser.add_argument("--plot", type=Path, help="directory for one plot per case")
    parser.add_argument("--out", type=Path, help="write each run's outcome as JSON")
    args = parser.parse_args()
    cases = args.cases.split(",")
    with Pool(args.workers) as pool:
        results = pool.map(fly, [(case, seed) for case in cases for seed in range(args.runs)])
    for case in cases:
        runs = [r for r in results if r["case"] == case]
        ends = {}
        for r in runs:
            key = (r["stop"][1] if r["stop"] else "played to the end") + ("" if r["stable"] else ", then lost attitude")
            ends.setdefault(key, []).append(r)
        parts = []
        for key, group in sorted(ends.items(), key=lambda item: -len(item[1])):
            when = [r["stop"][0] - r["start"] for r in group if r["stop"] and r["start"] is not None]
            parts.append(f"{key} {len(group)}" + (f" (median {np.median(when):.1f} s into the pattern)" if when else ""))
        print(f"{case:9s}: {', '.join(parts)}", flush=True)
    if args.out:
        args.out.write_text(json.dumps([{k: r[k] for k in ("case", "seed", "stable", "stop", "start", "lifted")} for r in results], indent=1))
    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        args.plot.mkdir(parents=True, exist_ok=True)
        figure, axes = plt.subplots(len(cases), 1, figsize=(10, 2.2 * len(cases)), sharex=True)
        for ax, case in zip(np.atleast_1d(axes), cases):
            for r in [r for r in results if r["case"] == case][:6]:
                t, position = np.array(r["t"]), np.array(r["position"])
                # Horizontal distance from where the drone was as the pattern began (the speed pattern moves its hold point on purpose; the guard allows for it).
                k0 = min(int(np.searchsorted(t, r["lifted"] + (r["start"] if r["start"] is not None else AFTER))), len(t) - 1)
                distance = np.hypot(*(position[:, :2] - position[k0, :2]).T)
                line, = ax.plot(t - t[k0], distance, lw=1)
                if r["stop"]:
                    ax.axvline(r["stop"][0] + r["lifted"] - t[k0], color=line.get_color(), ls=":", lw=1)
            ax.axhline(0.7, color="gray", ls="--", lw=0.8)
            ax.axvspan(0, 12.0 if case == "speed" else 11.2 if case in ("gps", "tilt", "still") else 8.0, color="tab:blue", alpha=0.07)
            ax.set_ylabel("m")
            ax.set_title(f"{case}: distance from the pattern's start (dotted: the guard stopped it; dashed: pattern_distance)", fontsize=9)
            ax.set_xlim(-2, 14)
        np.atleast_1d(axes)[-1].set_xlabel("s from the pattern's start")
        figure.tight_layout()
        figure.savefig(args.plot / "pattern_guard.png", dpi=110)
        print(f"Saved {args.plot / 'pattern_guard.png'}")


if __name__ == "__main__":
    main()
