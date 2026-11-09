"""
Outdoor hovers in the hover sim, without the tag: uv run -m sw.sim.outdoor [--runs 48] [--winds 0,2,4,6] [--seconds 40] [--gps] [--baro [TAU]] [--flow LOW,HIGH] [--set name=value,...] [--settings recording] [--airframe models/airframe_<day>.json] [--nominal] [--sticks 'start,pitch,roll,seconds;...'] [--flyaway m] [--workers N]

Each run takes off in Free mode (hover_camera 0) on a recording's settings (default: the newest 9-inch tag hover), on the 9-inch airframe as calibrated to its closed-loop hovers (calibrate_sim), climbs to --climb and holds for --seconds in the open air: a mean wind of the given speed from a random direction, with turbulence after MIL-F-8785C's low-altitude Dryden scales at the hover height (horizontal RMS 0.2 of the mean wind over 15 m, vertical 0.1 over the height; plus 0.1 m/s of still-air gusts) and the rotors' inflow damping the vertical airspeed. The room's torque drifts stay (the airframe's own noise); its force drifts, the room's recirculation, are replaced by the wind. With --gps the drone's GPS (sweep.run plant["gps"]: 10 Hz, 0.05 m/s velocity noise, 0.7 m of wandering position error) feeds the hold, its heading belief off by up to ±30 degrees. With --baro the barometer (BARO: 0.2 m of noise, 0.3 m of drift over a minute, rotor wash near the floor) feeds the height at hover_baro_tau TAU (default 1 s), which also caps the hold at hover_ceiling (here --climb). With --flow the forward camera's optical flow feeds it too (sweep.run plant["flow"]), the scene at a distance drawn per run that the drone only guesses (hover_flow_depth).

Indoors nothing measured the airframe above about 0.5 m/s of airspeed, so every run also draws the terms that matter there from wide ranges (unless --nominal): the forward speed's pitching moment ×0.5-3 of the airframe's (the rotors flapping back, linear in airspeed as fitted), the sideways one ×0.5-2, rotor drag 0.04-0.4/s forward and 0.1-0.4/s sideways, the tilt's pitch authority ×0.8-1.2, the thrust level ±8 %, the servos' dead time and response ±25 %, the GPS latency 40-120 ms (the firmware assumes 60).

The firmware's fly-away failsafe (fw/drone/main/guard.c; on at settings.def's defaults, off with --set failsafe_speed=0,failsafe_distance=0) flies with the hover. A run it lands counts as "failsafe landed" rather than touched down.

It prints, per wind: how many runs lost attitude (past 45 degrees), touched down unasked, were landed by the failsafe or flew off (beyond 10 m), and over the rest the median and 90th percentile of the horizontal distance from the takeoff point and of the height error over the hold, the worst tilt, the pitch RMS and the pitch's RMS in the swing band (0.4-1.6 Hz, as pitch_check measures the swing).
"""

import argparse
import json
import math
import os
import random
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.fit import settings as recording_settings
from sw.sim import airframe as A
from sw.sim.pitch_check import SWING, band
from sw.sim.sweep import BARO, CURRENT, run

MODELS = Path(__file__).with_name("models")
RECORDINGS = Path(__file__).parents[1] / "recordings"
SETTINGS = "2026-11-02T08-15-23"
AIRFRAME = MODELS / "airframe_2026-11-02_9in_calibrated.json"
DISTURBANCES = MODELS / "disturbances_2026-11-02_9in_calibrated.json"
GPS = {"rate": 10.0, "latency": 0.06, "velocity_noise": 0.05, "vertical_noise": 0.1, "position_noise": 0.7, "position_tau": 30.0}
SETTLE = 5.0
FLYAWAY = 10.0


def loguniform(rng, low, high):
    return math.exp(rng.uniform(math.log(low), math.log(high)))


def plant(airframe, wind, seed, gps, nominal, height, disturbances=None, headwind=False, flow=None, baro=False, speed_draw=(0.5, 3.0)):
    """The outdoor plant for one run: the airframe with its uncertain terms drawn, the wind and the GPS."""
    rng = random.Random(seed)
    p = dict(airframe) | {f"{axis}_push": 0.0 for axis in ("roll", "pitch", "yaw")}
    geometry = A.geometry_of(p)
    cz = geometry["com"][2]
    full = A.complete(p)
    draws = {"speed_moment": 1.0, "side_moment": 1.0, "drag": (0.04, 0.1), "tilt": 1.0, "thrust": 1.0, "servo": 1.0, "latency": 0.06}
    if not nominal:
        draws = {"speed_moment": loguniform(rng, *speed_draw), "side_moment": loguniform(rng, 0.5, 2.0), "drag": (loguniform(rng, 0.04, 0.4), loguniform(rng, 0.1, 0.4)),
                 "tilt": rng.uniform(0.8, 1.2), "thrust": rng.uniform(0.92, 1.08), "servo": rng.uniform(0.75, 1.25), "latency": rng.uniform(0.04, 0.12)}
    p["drag_z_x"] = cz + (full["drag_z_x"] - cz) * draws["speed_moment"]
    p["drag_z_y"] = cz + (full["drag_z_y"] - cz) * draws["side_moment"]
    p["tilt_gain"] = full.get("tilt_gain", 1.0) * draws["tilt"]
    p["thrust_level"] = p["thrust_level"] * draws["thrust"]
    for key in ("servo_delay", "right_servo_delay"):
        p[key] = full[key] * draws["servo"]
    for key in ("servo_wn", "right_servo_wn"):
        p[key] = full[key] / draws["servo"]
    direction = rng.uniform(0, 2 * math.pi)
    if headwind:
        # Taking off facing the wind: the air blows from ahead (toward the drone's tail), give or take 20 degrees.
        direction = math.pi + rng.uniform(-1, 1) * math.radians(20)
    speed = max(wind, 0.5)
    out = CURRENT | {
        "airframe": p, "rotor_drag": draws["drag"], "sensed_disturbances": True,
        "disturbances": [d for d in disturbances or CURRENT["disturbances"] if d["channel"] in ("roll", "pitch", "yaw")],
        "wind_mean": (wind * math.cos(direction), wind * math.sin(direction)), "wind": 0.2 * wind + 0.1, "wind_tau": 15.0 / speed,
        "wind_vertical": 0.1 * wind + 0.05, "wind_vertical_tau": height / speed, "vertical_drag": 0.7,
    }
    if flow:
        # The forward camera's scene: trees, buildings or open field, its distance unknown to the drone (hover_flow_depth is its guess), drawn per run with more pixel noise than indoors (grass and leaves moving, the sky's low texture).
        out["flow"] = {"depth": loguniform(random.Random(seed + 7000), *flow), "noise_px": 0.6, "fps": 28.0, "latency": 0.045, "max_px": 14.0}
    if gps:
        out["gps"] = GPS | {"latency": draws["latency"], "heading": rng.uniform(-180, 180)}
    if baro:
        out["baro"] = BARO
    return out, draws


def fly(job):
    airframe, disturbances, flown, wind, seed, args = job
    p, draws = plant(airframe, wind, seed, args.gps, args.nominal, args.climb, disturbances, args.headwind, args.flow, args.baro, args.speed_draw)
    s = dict(flown)
    if args.preload:
        # The wind's expected nose-up moment at liftoff, cancelled by pitch_bias (rad/s², ramped in once airborne): preload per m/s of the wind.
        s["pitch_bias"] = args.preload * wind
    if args.gps:
        # The drone's belief about where its nose points at takeoff: the truth give or take 30 degrees (a phone compass and eyeballing).
        s["hover_gps_heading"] = p["gps"]["heading"] + random.Random(seed + 1).uniform(-30, 30)
    if args.sticks:
        p["stick"] = [(SETTLE + start, pitch, roll, length) for start, pitch, roll, length in args.sticks]
    log = []
    result = run(s, p, seconds=args.seconds + SETTLE, seed=seed, log=log, hover=True)
    out = {"wind": wind, "seed": seed, "draws": draws}
    lifted = result.get("liftoff_s")
    if lifted is not None and log:
        t = np.array([e[0] for e in log]) - lifted
        position = np.array([e[1] for e in log])
        # The failsafe (guard.c): its phase changes, and where it first fired the true speed and distance from takeoff (a false alarm fires on a drone that is neither fast nor far).
        failsafe = [(time, phase) for time, phase in result.get("failsafe", []) if time >= 0]
        out |= {"failsafe": failsafe, "max_distance": float(np.hypot(position[t > 0, 0], position[t > 0, 1]).max())}
        if failsafe:
            k = min(np.searchsorted(t, failsafe[0][0]), len(t) - 2)
            out |= {"fired_speed": float(np.hypot(*(position[k + 1, :2] - position[k, :2])) / (t[k + 1] - t[k])), "fired_distance": float(np.hypot(*position[k, :2]))}
    if not result["stable"]:
        return out | {"fate": "lost", "at": result["fail_s"]}
    if lifted is None:
        return out | {"fate": "no liftoff"}
    angles = np.degrees([e[2][:2] for e in log])
    hold = t > SETTLE
    distance = np.hypot(position[hold, 0], position[hold, 1])
    if result.get("touched") is not None:
        landed = any(phase == "land" for _, phase in out.get("failsafe", []))
        return out | {"fate": "failsafe landed" if landed else "touched", "at": result["touched"], "landed_distance": float(np.hypot(*position[-1, :2]))}
    if distance.max() > args.flyaway:
        return out | {"fate": "flew off", "at": float(t[hold][np.argmax(distance > args.flyaway)])}
    height = position[hold, 2] - position[0, 2] - s["hover_climb"]
    held = {"fate": "held", "distance": float(np.median(distance)), "distance_max": float(distance.max()), "height": float(np.sqrt(np.mean(height ** 2))),
            "tilt": float(np.abs(angles[hold]).max()), "pitch_rms": float(np.std(angles[hold, 1])), "swing_band": float(np.sqrt(np.mean(band(angles[:, 1], *SWING)[hold] ** 2)))}
    if args.sticks:
        # After each stick is let go: the pitch's peak to peak from 0.3 s to 3 s after (the hold braking, and any swing it leaves).
        swings = [float(np.ptp(angles[(t > start + length + 0.3) & (t < start + length + 3.0), 1])) for start, _, _, length in args.sticks if ((t > start + length + 0.3) & (t < start + length + 3.0)).any()]
        held["swing"] = max(swings) if swings else 0.0
    return out | held


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=48, help="per wind")
    parser.add_argument("--winds", default="0,2,4,6", help="mean wind speeds (m/s)")
    parser.add_argument("--seconds", type=float, default=40.0, help="hold after settling")
    parser.add_argument("--climb", type=float, default=1.5, help="hold height above the takeoff point (m)")
    parser.add_argument("--gps", action="store_true")
    parser.add_argument("--baro", type=float, nargs="?", const=1.0, default=0.0, help="the barometer (BARO) feeding the height at hover_baro_tau of this (s; 1 without a value)")
    parser.add_argument("--headwind", action="store_true", help="take off facing the wind (±20 degrees) instead of at a random heading")
    parser.add_argument("--preload", type=float, default=0.0, help="pitch_bias per m/s of wind (rad/s² per m/s), for headwind takeoffs")
    parser.add_argument("--nominal", action="store_true", help="the airframe as fitted, no drawn uncertainty")
    parser.add_argument("--settings", default=SETTINGS, help="recording whose settings to fly")
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--airframe", type=Path, default=AIRFRAME)
    parser.add_argument("--disturbances", type=Path, default=DISTURBANCES, help="the airframe's torque drifts (the force drifts are left out: outdoors the wind replaces them)")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--flow", type=lambda text: tuple(float(v) for v in text.split(",")), help="the camera's flow, the scene LOW,HIGH m away (drawn log-uniform per run); hover_flow_depth (--set) is the drone's guess")
    parser.add_argument("--out", type=Path, help="write every run's result as JSON")
    parser.add_argument("--sticks", type=lambda text: [tuple(float(v) for v in item.split(",")) for item in text.split(";")], help="a pilot's stick sequence during the hold: start s after settling, pitch and roll degrees, seconds held; ';' between sticks (e.g. '5,10,0,2;12,-10,0,2'); adds each run's largest pitch peak to peak 0.3-3 s after a stick is let go")
    parser.add_argument("--flyaway", type=float, default=FLYAWAY, help="m from takeoff that counts as flown off")
    parser.add_argument("--speed-draw", type=lambda text: tuple(float(v) for v in text.split(",")), default=(0.5, 3.0), help="range the forward speed moment is drawn in, times the airframe's (log-uniform; default 0.5,3 from before any flight measured it; models fitted to the outdoor flights warrant narrower)")
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text())
    flown = recording_settings(next(RECORDINGS.glob(args.settings + "*.bin"))) | {"hover_camera": 0, "hover_center": 0, "hover_climb": args.climb, "hover_ceiling": args.climb}
    if args.gps:
        flown |= {"hover_gps_tau": 0.5, "hover_gps_place_tau": 2.0}
    if args.baro:
        flown |= {"hover_baro_tau": args.baro}
    flown |= {k: float(v) for k, v in (item.split("=") for item in args.set.split(",") if item)}
    winds = [float(w) for w in args.winds.split(",")]
    disturbances = json.loads(args.disturbances.read_text())
    jobs = [(airframe, disturbances, flown, wind, 1000 * k + seed, args) for k, wind in enumerate(winds) for seed in range(args.runs)]
    with Pool(args.workers) as pool:
        results = pool.map(fly, jobs)
    print(f"{args.airframe.name} with {args.disturbances.name}, settings {args.settings}{' + ' + args.set if args.set else ''}, {'GPS' if args.gps else 'no GPS'}, {f'baro at {args.baro:g} s' if args.baro else 'no baro'}, {'nominal' if args.nominal else 'drawn'} plant, {args.runs} runs per wind, {args.seconds:.0f} s hold at {args.climb} m")
    for wind in winds:
        runs = [r for r in results if r["wind"] == wind]
        fates = {f: sum(r["fate"] == f for r in runs) for f in ("lost", "touched", "failsafe landed", "flew off", "no liftoff")}
        held = [r for r in runs if r["fate"] == "held"]
        line = f"wind {wind:3.0f} m/s: held {len(held)}/{len(runs)}" + "".join(f", {f} {n}" for f, n in fates.items() if n)
        if held:
            q = lambda key, pct: np.percentile([r[key] for r in held], pct)
            line += f" | distance median {q('distance', 50):.2f} m (p90 {q('distance', 90):.2f}, worst max {max(r['distance_max'] for r in held):.1f}), height rms {q('height', 50):.2f} m (p90 {q('height', 90):.2f}), tilt max {q('tilt', 50):.1f}° (p90 {q('tilt', 90):.1f}), pitch rms {q('pitch_rms', 50):.1f}°, swing band (0.4-1.6 Hz) rms {q('swing_band', 50):.1f}° (p90 {q('swing_band', 90):.1f})"
            if args.sticks:
                line += f", swing after release {q('swing', 50):.1f}° (p90 {q('swing', 90):.1f})"
        print(line, flush=True)
        fired = [r for r in runs if r.get("failsafe")]
        if fired:
            # The failsafe: how many runs it fired in, by what it reached, the true speed and distance where it fired, and the furthest any run got.
            reached = {phase: sum(any(p == phase for _, p in r["failsafe"]) for r in fired) for phase in ("hold", "land")}
            print(f"    failsafe fired in {len(fired)}/{len(runs)} (held after braking {reached['hold']}, landed {reached['land']}), first at {np.median([r['failsafe'][0][0] for r in fired]):.1f} s median, true speed there {np.median([r['fired_speed'] for r in fired]):.1f} m/s median (min {min(r['fired_speed'] for r in fired):.1f}), distance {np.median([r['fired_distance'] for r in fired]):.1f} m median", flush=True)
        if any("max_distance" in r for r in runs):
            far = [r["max_distance"] for r in runs if "max_distance" in r]
            print(f"    furthest from takeoff over the run: median {np.median(far):.1f} m, p90 {np.percentile(far, 90):.1f}, worst {max(far):.1f}", flush=True)
        failed = [r for r in runs if r["fate"] != "held"]
        if failed:
            # Which drawn terms the failures had, against all runs: a failure that follows one term names what to measure.
            for key in ("speed_moment", "tilt", "servo"):
                values = [r["draws"][key] for r in failed]
                print(f"    failed runs' {key}: {', '.join(f'{v:.2f}' for v in sorted(values)[:12])}{' ...' if len(values) > 12 else ''}")
    if args.out:
        args.out.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
