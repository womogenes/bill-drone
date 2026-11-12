"""
How fast the GPS hold can fly and still stop cleanly: uv run -m sw.sim.speed_limit [--speeds 2,3,4,5] [--tilts 10,15,20] [--runs 64] [--direction forward|back|left] [--set name=value,...] [--workers N] [--out file.json]

Written 2026-11-12 for drone 1 on the raised pivots. Each run takes off on the GPS hold (sw/sim/outdoor.py's plant on models/airframe_2026-11-12_raised.json with its uncertain terms drawn, GPS, barometer, the rubber bands, a level-reference error of 1.3 degrees SD, a wind drawn 0-3 m/s from a random direction), holds for SETTLE s, then flies the hold point at the commanded speed as the gamepad's stick does (sweep.run plant["nudge"], stepped every 50 ms and wrapped at NUDGE_MAX as the dashboard sends it; nothing in the sim caps the speed, the drone caps it at settings.def hover_nudge_speed's limit) for long enough to reach it and cruise CRUISE s, then lets go and watches AFTER s. On top of the draws outdoor.py makes (forward speed moment, sideways moment, rotor drag, tilt authority, thrust, servo lag, GPS latency) it draws the tilted thrust's push share rising with airspeed (airframe tilt_push_speed 0-0.04 per m/s, the 2026-11-09 fly-aways' fit) and the forward speed moment wider than outdoor.py's default (SPEED_DRAW: measured to 4.4 m/s before the raise, never at speed after it).

It prints per commanded speed and lean cap (hover_tilt): runs lost (past the tip cut), landed by the failsafe and those whose failsafe fired at all; then medians (90th percentile) of the speed reached (mean over the last second before release), the time from release to under STOPPED and the distance from the release point by then, the overshoot (how far along the track the drone got past where it came to rest, which it then flew back), the largest tilt (body, either axis) and servo command from the start of the push to the end, the body pitch in the cruise and its extremes, and the oscillation after stopping: the body pitch's peak to peak and its 0.4-1.6 Hz RMS from 1 s after the stop, and the largest ground speed regained after it.
"""

import argparse
import json
import math
import os
import random
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording
from sw.flight_log import settings as recording_settings
from sw.sim import airframe as A
from sw.sim.outdoor import plant as outdoor_plant
from sw.sim.pitch_check import SWING, band
from sw.sim.sweep import NUDGE_MAX, run

# Drone 1's GPS-hold flights since the 2026-11-11 6:14 PM Pacific one (hover_kd 1.5, hover_kp 1, hover_tilt 10, hover_thrust_direction 0.8, pitch_damping 6, failsafe_speed 2, failsafe_distance 5).
SETTINGS = "2026-11-12T02-14-11"
AIRFRAME = A.MODELS / "airframe_2026-11-12_raised.json"
CLIMB = 3.0
# The hold before the push (s after liftoff): past the takeoff and, in most runs, the heading's pinning.
SETTLE = 12.0
# Cruise at the commanded speed this long after the time it takes to reach it at ACCEL (m/s²), then watch this long after letting go (s).
CRUISE = 3.0
ACCEL = 1.0
AFTER = 15.0
# Under this ground speed (m/s) the drone has stopped.
STOPPED = 0.3
# The dashboard's gamepad sends the nudge every 50 ms (sw/dashboard/components/control-panel.tsx GAMEPAD_TICK_MS).
NUDGE_PERIOD = 0.05
# The forward speed moment's draw (times the airframe's, log-uniform), the push share's rise with airspeed (per m/s), the wind (m/s, uniform).
SPEED_DRAW = (0.7, 2.0)
PUSH_SPEED = (0.0, 0.04)
WIND = (0.0, 3.0)
# The rubber bands on the servos (sw/sim/band_hold.py): the tilt behind its command (degrees) and the accelerometer's forward bias in flight (g).
BAND_TILT = -2.5
BAND_ACCEL = 0.025
DIRECTIONS = {"forward": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0)}


def push_seconds(speed):
    return speed / ACCEL + CRUISE


def flight(airframe, flown, speed, tilt, seed, direction, speed_draw=SPEED_DRAW):
    """One run's plant drawn and flown: the sim's result, its log, the draws and how long the push lasts (s)."""
    rng = random.Random(seed + 77000)
    wind = rng.uniform(*WIND)
    p, draws = outdoor_plant(airframe, wind, seed, True, False, flown["hover_climb"], baro=True, speed_draw=speed_draw, level_sd=1.3, pushes=True)
    draws |= {"wind": wind, "push_speed": rng.uniform(*PUSH_SPEED)}
    p["airframe"] = p["airframe"] | {"tilt_push_speed": draws["push_speed"]}
    p |= {"tilt_offset": BAND_TILT, "flight_accel_bias": (BAND_ACCEL, 0.0, 0.0)}
    s = flown | {"hover_tilt": tilt, "hover_gps_heading": p["gps"]["heading"] + random.Random(seed + 1).uniform(-30, 30)}
    length = push_seconds(speed)
    forward, left = DIRECTIONS[direction]
    p |= {"nudge": [(SETTLE, forward * speed, left * speed, length)], "nudge_wrap": NUDGE_MAX, "nudge_period": NUDGE_PERIOD}
    log = []
    result = run(s, p, seconds=SETTLE + length + AFTER + 2, seed=seed, log=log, hover=True)
    return result, log, draws, length


def series(log, lifted):
    """The log as arrays on s after liftoff: true horizontal position (m), roll and pitch (degrees, pitch positive nose-down), servo tilts (degrees, positive forward), pitch target (degrees), and the hold's own place (the drone less its hold point, m, hold frame)."""
    return {"t": np.array([e[0] for e in log]) - lifted, "position": np.array([e[1] for e in log])[:, :2], "angles": np.degrees([e[2][:2] for e in log]),
            "servo": np.degrees([e[4] for e in log]), "target": np.degrees([e[6] for e in log]), "place": np.array([e[-1][5] for e in log])}


def fly(job):
    airframe, flown, speed, tilt, seed, direction, speed_draw = job
    result, log, draws, length = flight(airframe, flown, speed, tilt, seed, direction, speed_draw)
    out = {"speed": speed, "tilt": tilt, "seed": seed, "draws": draws}
    lifted = result.get("liftoff_s")
    if lifted is None:
        return out | {"fate": "no liftoff"}
    d = series(log, lifted)
    t, position, angles, servo, place = d["t"], d["position"], d["angles"], d["servo"], d["place"]
    velocity = np.gradient(position, t, axis=0)
    ground = np.hypot(*velocity.T)
    start, release = SETTLE, SETTLE + length
    failsafe = [(time, phase) for time, phase in result.get("failsafe", []) if time >= start]
    out |= {"failsafe": failsafe, "pinned": bool(log[np.searchsorted(t, start)][13][2])}
    if not result["stable"]:
        return out | {"fate": "lost", "at": float(result["fail_s"] - lifted)}
    if result.get("touched") is not None:
        return out | {"fate": "landed" if any(phase == "land" for _, phase in failsafe) else "touched", "at": result["touched"]}
    span = (t >= start) & (t <= release + AFTER)
    cruise = (t >= release - 1.0) & (t < release)
    r = np.searchsorted(t, release)
    # The overshoot past the hold point as the hold measures it: once the nudge's speed has faded (hover.c NUDGE_TAU 0.2 s), the place along the way the hold point had led it.
    way = -place[cruise].mean(axis=0)
    way = way / max(np.hypot(*way), 1e-6)
    beyond = place[(t >= release + 0.5) & (t <= release + AFTER)] @ way
    after = np.flatnonzero((t > release) & (t <= release + AFTER) & (ground < STOPPED))
    out |= {"fate": "held", "reached": float(ground[cruise].mean()), "peak": float(ground[span].max()), "lag": float(-place[cruise].mean(axis=0) @ way), "overshoot": float(max(beyond.max(), 0.0)),
            "tilt_max": float(np.abs(angles[span]).max()), "servo_max": float(np.abs(servo[span]).max()),
            "cruise_pitch": float(angles[cruise, 1].mean()), "cruise_servo": float(servo[cruise].mean()), "pitch_low": float(angles[span, 1].min()), "pitch_high": float(angles[span, 1].max())}
    if not len(after):
        return out | {"stop_time": math.nan, "stop_distance": math.nan, "ringing": math.nan, "swing": math.nan, "regained": math.nan}
    stop = after[0]
    settled = (t > t[stop] + 1.0) & (t <= release + AFTER)
    swing = band(angles[:, 1], *SWING)
    return out | {"stop_time": float(t[stop] - release), "stop_distance": float(np.hypot(*(position[stop] - position[r]))),
                  "ringing": float(np.ptp(angles[settled, 1])) if settled.any() else math.nan, "swing": float(np.sqrt(np.mean(swing[settled] ** 2))) if settled.any() else math.nan,
                  "regained": float(ground[settled].max()) if settled.any() else math.nan}


def summary(runs):
    """One line per case: fates, then medians (90th percentile) over the runs that held."""
    held = [r for r in runs if r["fate"] == "held"]
    fates = {f: sum(r["fate"] == f for r in runs) for f in ("lost", "landed", "touched", "no liftoff")}
    fired = sum(bool(r.get("failsafe")) for r in runs)
    line = f"held {len(held)}/{len(runs)}" + "".join(f", {f} {n}" for f, n in fates.items() if n) + f", failsafe fired {fired}"
    if not held:
        return line
    q = lambda key: f"{np.nanmedian([r[key] for r in held]):.2f} ({np.nanpercentile([r[key] for r in held], 90):.2f})"
    lo = lambda key: f"{np.nanmedian([r[key] for r in held]):.1f} ({np.nanpercentile([r[key] for r in held], 10):.1f})"
    unstopped = sum(math.isnan(r["stop_time"]) for r in held)
    return (line + f" | reached {q('reached')} m/s, peak {q('peak')}; stop {q('stop_time')} s, {q('stop_distance')} m" + (f" ({unstopped} never)" if unstopped else "")
            + f"; lag behind the hold point {q('lag')} m, overshoot past it {q('overshoot')} m; tilt max {q('tilt_max')}°, servo max {q('servo_max')}°, cruise pitch {lo('cruise_pitch')}° servo {q('cruise_servo')}°, pitch {lo('pitch_low')}..{q('pitch_high')}°"
            + f"; after stop: pitch p2p {q('ringing')}°, swing rms {q('swing')}°, regained {q('regained')} m/s")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speeds", default="2,3,4,5", help="commanded speeds (m/s)")
    parser.add_argument("--tilts", default="10,15,20", help="hover_tilt lean caps (degrees)")
    parser.add_argument("--runs", type=int, default=64, help="per case; the same drawn plants in every case")
    parser.add_argument("--direction", choices=sorted(DIRECTIONS), default="forward")
    parser.add_argument("--speed-draw", type=lambda text: tuple(float(v) for v in text.split(",")), default=SPEED_DRAW, help="LOW,HIGH: the forward speed moment's draw, times the airframe's (log-uniform)")
    parser.add_argument("--settings", default=SETTINGS, help="recording whose settings to fly")
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--airframe", type=Path, default=AIRFRAME)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--out", type=Path, help="write every run's result as JSON")
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text())
    flown = recording_settings(recording(args.settings)) | {"hover_camera": 0, "hover_center": 0, "hover_climb": CLIMB, "hover_ceiling": CLIMB}
    flown |= parse_settings(args.set)
    speeds = [float(v) for v in args.speeds.split(",")]
    tilts = [float(v) for v in args.tilts.split(",")]
    jobs = [(airframe, flown, speed, tilt, seed, args.direction, args.speed_draw) for speed in speeds for tilt in tilts for seed in range(args.runs)]
    with Pool(args.workers) as pool:
        results = pool.map(fly, jobs)
    print(f"{args.airframe.name}, settings {args.settings}{' + ' + args.set if args.set else ''}, {args.direction}, {args.runs} runs per case, wind {WIND[0]:g}-{WIND[1]:g} m/s")
    for speed in speeds:
        for tilt in tilts:
            runs = [r for r in results if r["speed"] == speed and r["tilt"] == tilt]
            print(f"{speed:g} m/s, hover_tilt {tilt:g}: {summary(runs)}", flush=True)
            failed = [r for r in runs if r["fate"] != "held"]
            if failed:
                print("    not held: " + ", ".join(f"{r['fate']} at {r.get('at', math.nan):.1f} s (wind {r['draws']['wind']:.1f}, moment x{r['draws']['speed_moment']:.2f}, push {r['draws']['push_speed']:.3f})" for r in failed[:8]))
    if args.out:
        args.out.write_text(json.dumps(results))


if __name__ == "__main__":
    main()
