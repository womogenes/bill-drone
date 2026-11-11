"""
A ladder of roll steps from a GPS hover, for the first measurements past the 35 degrees the airframe model was fitted within: uv run -m sw.sim.tilt_ladder [--amplitudes 15,20,25,30,35] [--width 0.5] [--rest 3] [--roll-rate 60] [--doublet] [--tag] [--distance m] [--seeds 8] [--dry]

The pattern (sw/excite.py's format, its roll channel) steps the roll target to +A for --width s, back to 0 for --rest s, to -A, back to 0, for each amplitude in turn; on the drone the hold keeps flying under it, so the step's sideways push is braked between steps. Before flying it, it is flown here on the outdoor plant (sw/sim/outdoor.py: drone 1 as of 2026-11-09 with its uncertain terms drawn, GPS, barometer, a light wind) on the firmware's own hold and guards with the test's settings (TEST): each step's peak true roll, the attitude estimate's error there (the firmware's complementary filter pulls toward the accelerometer, which in a banked hover reads the thrust, not gravity), the height lost, the sideways speed and how far to the side of the hold point it gets before the next step, and whether the pattern's tilt guard or the 45-degree tip cut (flight_rt.c TIP_LIMIT; the sim fails a run there) ended it. --dry prints the pattern for sw/excite.py file instead, and the settings the sim flew it on.

--doublet follows each step at once with its opposite (+A, -A, rest, -A, +A, rest), which takes the step's sideways speed out again at the step's own tilt instead of the hold's: about half the room for the same roll, but the swing from one side to the other overshoots the target (on the tag a 20-degree doublet of 0.4 s each way peaks at 30 degrees, a 40-degree one passes the tip cut), so the peak it reports is the larger of the two halves.

--tag flies it indoors on the tag instead: sweep.IDENTIFIED_ESC on the 2026-10-29 07:20 UTC camera hover's settings, as pattern_guard.py flies its patterns (0.5 m up, 1.9 m from the tag), with pattern_distance 0.5 unless --distance gives another. That is the drone as it was then; sweep.CURRENT on drone 1's hop settings loses the tag every few seconds in the sim. A room 2 m wide leaves the drone (0.6 m across its prop tips) 0.7 m to each side: --tag --doublet --amplitudes 20 --width 0.3 --roll-rate 360 reaches 24 degrees within 0.45 m of the hold point (0.3 m median), where single steps fit to about 10 degrees (models/NOTES.md 2026-11-10).

Today's firmware caps the test below 45 degrees: TIP_LIMIT stops the motors there, nothing raises the collective for a bank (at 60 degrees holding height needs twice the thrust), and the filter's pull toward the accelerometer grows with the time spent banked. Steps past that need those three changed first (models/NOTES.md).
"""

import argparse
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

from sw.excite import SAMPLE_HZ, segments
from sw.flight_log import recording
from sw.flight_log import settings as recording_settings
from sw.sim.outdoor import GPS, plant as outdoor_plant
from sw.sim.pattern_guard import TAG_SETTINGS
from sw.sim.sweep import CURRENT_AIRFRAME, IDENTIFIED_ESC, run

# The calm GPS and barometer hover of 2026-11-09 06:25 UTC, as flown.
SETTINGS = "2026-11-09T06-25-48"
# The test's settings: high enough to fall from a failed step, the pattern's tilt guard just under the tip cut, room for the steps' sideways travel, the takeoff wiggle that pins the GPS hold's heading (without it a still hover may never pin it, and the hold drifts on its drag model), and the thrust-direction hold (models/NOTES.md 2026-11-09: without it 1 of 16 ladder runs lost its attitude in the hold after its pattern stopped).
TEST = {"hover_climb": 3.0, "hover_ceiling": 5.0, "pattern_tilt": 42.0, "pattern_distance": 4.0, "hover_camera": 0, "hover_center": 0, "hover_gps_calibrate": 5.0, "hover_thrust_direction": 0.8, "hover_kd": 1.5}
# On the tag (--tag) the camera hover's settings stay as flown but for the pattern's guards: the tilt guard as above, and the distance guard for a small room.
TAG_TEST = {"pattern_tilt": 42.0, "pattern_distance": 0.5}
# The pattern starts this long (s) after collective first reaches integrate_throttle: the climb to 3 m and a settled hold.
AFTER = 14.0


def ladder(amplitudes, width, rest, doublet=False):
    """Roll +A, 0, -A, 0 per amplitude (degrees), or with doublet +A, -A, 0, -A, +A, 0, as sw/excite.py segments."""
    values = []
    for a in amplitudes:
        for sign in (1, -1):
            values += [{"roll": sign * a}] * round(width * SAMPLE_HZ) + ([{"roll": -sign * a}] * round(width * SAMPLE_HZ) if doublet else []) + [{}] * round(rest * SAMPLE_HZ)
    return {"after": AFTER, "repeat": 1, "segments": segments(values, 1)}


def fly(job):
    pattern, settings, wind, seed, steps, tag, doublet = job
    if tag:
        p = dict(IDENTIFIED_ESC)
    else:
        airframe = dict(CURRENT_AIRFRAME)
        p, _ = outdoor_plant(airframe, wind, seed, True, False, settings["hover_climb"], baro=True, level_sd=1.3)
        p["gps"] = GPS | {"heading": 0.0}
    p["excite"] = pattern
    log = []
    seconds = AFTER + 6 + sum(s["ticks"] for s in pattern["segments"]) / SAMPLE_HZ
    result = run(settings, p, seconds=seconds, seed=seed, log=log, hover=True)
    t = np.array([e[0] for e in log])
    position = np.array([e[1] for e in log])
    true_roll = np.degrees([e[2][0] for e in log])
    estimate = np.degrees([e[3][0] for e in log])
    excited = np.degrees([e[10] for e in log])
    velocity = np.gradient(position, t, axis=0)
    out = []
    lifted = result.get("liftoff_s")
    # The hold point: where the hover sat over the 4 s before the pattern.
    hold = None
    if lifted is not None and result.get("pattern_start") is not None:
        before = (t >= result["pattern_start"] + lifted - 4.0) & (t < result["pattern_start"] + lifted)
        hold = position[before].mean(axis=0) if before.any() else None
    for amplitude, sign, start, end, until in steps:
        if lifted is None:
            break
        # Each step from its start to 1.5 s after it ends, on the run's clock (the pattern's start is AFTER past the integrate point, which the run reports as pattern_start after liftoff).
        begin = result["pattern_start"] + lifted + start if result.get("pattern_start") is not None else None
        if begin is None or (result.get("pattern_stop") and begin >= result["pattern_stop"][0] + lifted):
            break
        k = (t >= begin) & (t <= begin + (end - start) + 1.5)
        if not k.any() or t[-1] < begin:
            break
        # A doublet's peak is the larger of its two halves (the swing back overshoots).
        toward = np.abs(true_roll[k]) if doublet else sign * true_roll[k]
        peak = int(np.argmax(toward))
        # Sideways from the hold point, until the next step starts.
        resting = (t >= begin) & (t < begin + until - start)
        out.append({"amplitude": amplitude, "sign": sign, "peak": float(toward[peak]), "estimate_error": float(estimate[k][peak] - true_roll[k][peak]),
                    "height_lost": float(position[k][0, 2] - position[k][:, 2].min()), "side_speed": float(np.abs(velocity[k][:, 1]).max()),
                    "side": float(np.abs(position[resting][:, 1] - hold[1]).max()) if hold is not None else float("nan"), "excited": float(np.abs(excited[k]).max())})
    return {"stable": result["stable"], "fail_s": result.get("fail_s"), "stop": result.get("pattern_stop"), "steps": out}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--amplitudes", default="15,20,25,30,35")
    parser.add_argument("--width", type=float, default=0.5)
    parser.add_argument("--rest", type=float, default=3.0)
    parser.add_argument("--roll-rate", type=float, default=60.0, help="roll_target_rate (degrees/s): how fast the roll setpoint may move")
    parser.add_argument("--doublet", action="store_true", help="follow each step at once with its opposite")
    parser.add_argument("--tag", action="store_true", help="indoors on the tag hold instead of the outdoor GPS hold")
    parser.add_argument("--distance", type=float, help="pattern_distance (m): how far from its start the pattern may carry the drone (default 4, with --tag 0.5)")
    parser.add_argument("--wind", type=float, default=1.0)
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--workers", type=int, default=min(4, os.cpu_count()))
    parser.add_argument("--dry", action="store_true")
    args = parser.parse_args()
    amplitudes = [float(a) for a in args.amplitudes.split(",")]
    pattern = ladder(amplitudes, args.width, args.rest, args.doublet)
    test = (TAG_TEST if args.tag else TEST) | {"roll_target_rate": args.roll_rate} | ({"pattern_distance": args.distance} if args.distance is not None else {})
    if args.dry:
        print(json.dumps(pattern))
        print("flown here on " + ",".join(f"{name}={value:g}" for name, value in test.items()), file=sys.stderr)
        return
    span = args.width * (2 if args.doublet else 1)
    steps, at = [], 0.0
    for a in amplitudes:
        for sign in (1, -1):
            steps.append((a, sign, at, at + span, at + span + args.rest))
            at += span + args.rest
    settings = recording_settings(recording(TAG_SETTINGS if args.tag else SETTINGS)) | test
    with Pool(args.workers) as pool:
        results = pool.map(fly, [(pattern, settings, args.wind, seed, steps, args.tag, args.doublet) for seed in range(args.seeds)])
    where = f"on the tag, pattern_distance {settings['pattern_distance']:g} m" if args.tag else f"wind {args.wind} m/s"
    print(f"roll ladder {amplitudes}, {args.width} s {'doublets' if args.doublet else 'steps'}, {args.rest} s rests, roll_target_rate {args.roll_rate}°/s, {where}, {args.seeds} runs")
    print(f"  lost (past 45°, the tip cut) {sum(not r['stable'] for r in results)}, pattern stopped by its guard {sum(r['stop'] is not None for r in results)}: {[r['stop'] for r in results if r['stop']]}")
    for a in amplitudes:
        rows = [s for r in results for s in r["steps"] if s["amplitude"] == a]
        if not rows:
            print(f"  {a:4.0f}°: never reached")
            continue
        col = lambda key: np.array([s[key] for s in rows])
        print(f"  {a:4.0f}°: {len(rows)} steps, peak roll {np.median(col('peak')):.1f}° (max {col('peak').max():.1f}), estimate off by {np.median(np.abs(col('estimate_error'))):.1f}° (max {np.abs(col('estimate_error')).max():.1f}), height lost {np.median(col('height_lost')):.2f} m (max {col('height_lost').max():.2f}), sideways speed {np.median(col('side_speed')):.1f} m/s (max {col('side_speed').max():.1f}), furthest to the side of the hold point {np.median(col('side')):.2f} m (max {col('side').max():.2f})")


if __name__ == "__main__":
    main()
