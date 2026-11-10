"""
Outdoor hovers in the hover sim, without the tag: uv run -m sw.sim.outdoor [--runs 48] [--winds 0,2,4,6] [--seconds 40] [--gps] [--baro [TAU]] [--flow LOW,HIGH] [--set name=value,...] [--settings recording] [--airframe models/airframe_<day>.json] [--nominal] [--sticks 'start,pitch,roll,seconds;...'] [--speed-sticks 'start,forward,left,seconds;...'] [--flyaway m] [--gps-outage start,seconds] [--ekf] [--workers N]

Each run takes off in Free mode (hover_camera 0) on a recording's settings (default: the newest 9-inch tag hover), on the 9-inch airframe as calibrated to its closed-loop hovers (calibrate_sim), climbs to --climb and holds for --seconds in the open air: a mean wind of the given speed from a random direction, with turbulence after MIL-F-8785C's low-altitude Dryden scales at the hover height (horizontal RMS 0.2 of the mean wind over 15 m, vertical 0.1 over the height; plus 0.1 m/s of still-air gusts) and the rotors' inflow damping the vertical airspeed. The room's torque drifts stay (the airframe's own noise); its force drifts, the room's recirculation, are replaced by the wind. With --gps the drone's GPS (sweep.run plant["gps"]: 10 Hz, 0.05 m/s velocity noise (--gps-noise), 0.7 m of wandering position error) feeds the hold, its heading belief (hover_gps_heading) off by up to ±30 degrees (--heading-error); held runs also report the hold frame's heading error at the end and when hover.c pinned the heading from the GPS (versions with that gate). With --baro the barometer (BARO: 0.2 m of noise, 0.3 m of drift over a minute, rotor wash near the floor) feeds the height at hover_baro_tau TAU (default 1 s), which also caps the hold at hover_ceiling (here --climb). With --flow the forward camera's optical flow feeds it too (sweep.run plant["flow"]), the scene at a distance drawn per run that the drone only guesses (hover_flow_depth).

Indoors nothing measured the airframe above about 0.5 m/s of airspeed, so every run also draws the terms that matter there from wide ranges (unless --nominal): the forward speed's pitching moment ×0.5-3 of the airframe's (the rotors flapping back, linear in airspeed as fitted), the sideways one ×0.5-2, rotor drag 0.03-0.15/s forward and 0.1-0.25/s sideways (from 2026-11-09 the GPS measures 0.04-0.08 forward from the accelerometer, 0.13-0.19 with the attitude, and 0.12-0.22 sideways; before then 0.04-0.4 and 0.1-0.4 were drawn), the tilt's pitch authority ×0.8-1.2, the thrust level ±8 %, the servos' dead time and response ±25 %, the GPS latency 40-120 ms (the firmware assumes 60).

The firmware's fly-away failsafe (fw/drone/main/guard.c; on at settings.def's defaults, off with --set failsafe_speed=0,failsafe_distance=0) flies with the hover. A run it lands counts as "failsafe landed" rather than touched down.

It prints, per wind: how many runs lost attitude (past 45 degrees), touched down unasked, were landed by the failsafe or flew off (beyond 10 m), and over the rest the median and 90th percentile of the horizontal distance from the takeoff point and of the height error over the hold, the worst tilt, the pitch RMS and the pitch's RMS in the swing band (0.4-1.6 Hz, as pitch_check measures the swing). With --sticks (the laptop's tilt held) or --speed-sticks (the hold point flown at a speed, as the gamepad's stick is on the GPS hold) also the pitch's swing after each release and the braking: peak ground speed, and the time and distance from release until under STOPPED.
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
from sw.sim.pitch_check import SWING, band
from sw.sim.sweep import BARO, CURRENT, NUDGE_MAX, run

SETTINGS = "2026-11-02T08-15-23"
AIRFRAME = A.MODELS / "airframe_2026-11-02_9in_calibrated.json"
DISTURBANCES = A.MODELS / "disturbances_2026-11-02_9in_calibrated.json"
# At the GPS-held liftoffs of 2026-11-09 (05:45, 05:50, 06:19, 06:25 UTC) the GPS's climb rate read 0.5-1.2 m/s down for 0.3-0.5 s while the drone climbed at up to 1 m/s (sw/sim/gps_truth.py); in flight it is 0.18-0.39 m/s RMS off.
GPS = {"rate": 10.0, "latency": 0.06, "velocity_noise": 0.05, "vertical_noise": 0.25, "position_noise": 0.7, "position_tau": 30.0, "liftoff_descent": (0.5, 1.2), "liftoff_s": 0.4}
# Ground effect: 10-inch rotors (0.127 m radius) 0.115 m above the floor at rest (geometry_2026-11-08b rotor_z 0.094 m above the tube, the body's underside 0.02 m below it).
GROUND_EFFECT = {"radius": 0.127, "rotor": 0.115}
# The takeoff: the highest the drone climbs in its first TAKEOFF_S (s) after liftoff, over the climb goal and over the height it then settles at (the median over the following SETTLED_S, s), which leaves out the barometer's own error (0.07-0.13 m RMS over 4 s against the accelerometer on 2026-11-09).
TAKEOFF_S = 5.0
SETTLED_S = 5.0
SETTLE = 5.0
FLYAWAY = 10.0
# Braking after a stick: the ground speed (m/s) that counts as stopped, and how long after release it may take (s).
STOPPED = 0.3
STOP_WINDOW = 8.0
# With the Kalman filter (--ekf, hover_ekf): the rest before liftoff (s). On the drone it runs from boot.
EKF_REST = 5.0


def loguniform(rng, low, high):
    return math.exp(rng.uniform(math.log(low), math.log(high)))


def plant(airframe, wind, seed, gps, nominal, height, disturbances=None, headwind=False, flow=None, baro=False, speed_draw=(0.5, 3.0), gps_noise=None, level_sd=0.0):
    """The outdoor plant for one run: the airframe with its uncertain terms drawn, the wind and the GPS."""
    rng = random.Random(seed)
    p = dict(airframe) | {f"{axis}_push": 0.0 for axis in ("roll", "pitch", "yaw")}
    geometry = A.geometry_of(p)
    cz = geometry["com"][2]
    full = A.complete(p)
    draws = {"speed_moment": 1.0, "side_moment": 1.0, "drag": (0.04, 0.17), "tilt": 1.0, "thrust": 1.0, "servo": 1.0, "latency": 0.06}
    if not nominal:
        draws = {"speed_moment": loguniform(rng, *speed_draw), "side_moment": loguniform(rng, 0.5, 2.0), "drag": (loguniform(rng, 0.03, 0.15), loguniform(rng, 0.1, 0.25)),
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
        "wind_vertical": 0.1 * wind + 0.05, "wind_vertical_tau": height / speed, "vertical_drag": 0.7, "ground_effect": GROUND_EFFECT,
    }
    if flow:
        # The forward camera's scene: trees, buildings or open field, its distance unknown to the drone (hover_flow_depth is its guess), drawn per run with more pixel noise than indoors (grass and leaves moving, the sky's low texture).
        out["flow"] = {"depth": loguniform(random.Random(seed + 7000), *flow), "noise_px": 0.6, "fps": 28.0, "latency": 0.045, "max_px": 14.0}
    if gps:
        out["gps"] = GPS | {"latency": draws["latency"], "heading": rng.uniform(-180, 180)} | ({"velocity_noise": gps_noise} if gps_noise else {})
    if baro:
        out["baro"] = BARO
    if level_sd:
        # The level reference's error the drone flies with (degrees, roll and pitch): on 2026-11-09 each flight's constant forward acceleration against the GPS was -0.12 to +0.68 m/s² (sd about 0.22, 1.3 degrees).
        level = random.Random(seed + 9000)
        out["level_error"] = (level.gauss(0, level_sd), level.gauss(0, level_sd))
    return out, draws


def filter_errors(log, t, position, psi):
    """The Kalman filter's errors against the sim's truth from 1 s after liftoff, beside the hover's own estimates' (the hold's own are the filter's where it flew on it): the heading (degrees RMS once known, and when it was), the horizontal velocity as the hold flies it (m/s RMS, in the hold's frame: the filter's turned by its own heading as hover.c turns it), the height (m RMS above the takeoff point) and the tilt (degrees RMS of roll and pitch; the attitude filter's is the flown estimate's)."""
    flying = np.flatnonzero(t > 1.0)
    if not len(flying):
        return {}
    e = [log[i] for i in flying]
    truth = np.array([entry[2] for entry in e])
    yaw, velocity, height = np.array([entry[14][2] for entry in e]), np.array([entry[14][3] for entry in e]), np.array([entry[14][5] for entry in e])
    aligned, flown = np.array([entry[14][8] for entry in e], bool), np.array([entry[14][9] for entry in e], bool)
    own_velocity, own_height = np.array([entry[14][10] for entry in e]), np.array([entry[14][11] for entry in e])
    gyro_yaw = np.array([entry[13][1] for entry in e])
    true_velocity = np.gradient(position[:, :2], t, axis=0)[flying]
    # The hold's frame: where the nose pointed when its gyro yaw was zero, so the world's forward axis turned by the true yaw less the gyro's.
    frame = truth[:, 2] - gyro_yaw
    held = np.column_stack([true_velocity[:, 0] * np.cos(frame) + true_velocity[:, 1] * np.sin(frame), -true_velocity[:, 0] * np.sin(frame) + true_velocity[:, 1] * np.cos(frame)])
    heading = gyro_yaw - yaw
    turned = np.column_stack([velocity[:, 0] * np.cos(heading) - velocity[:, 1] * np.sin(heading), velocity[:, 0] * np.sin(heading) + velocity[:, 1] * np.cos(heading)])
    rms = lambda values: float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan
    yaw_error = np.degrees(np.remainder(yaw - (truth[:, 2] - psi) + np.pi, 2 * np.pi) - np.pi)
    above = position[flying, 2] - position[0, 2]
    tilt = np.degrees(np.column_stack([[entry[14][0] for entry in e], [entry[14][1] for entry in e]]) - truth[:, :2])
    own_tilt = np.degrees(np.array([entry[3] for entry in e]) - truth[:, :2])
    return {"ekf": {"aligned_at": float(t[flying][np.argmax(aligned)]) if aligned.any() else math.nan, "heading": rms(yaw_error[aligned]), "velocity": rms(np.hypot(*(turned - held)[aligned].T)),
                    "own_velocity": rms(np.hypot(*(own_velocity - held)[aligned].T)), "height": rms(height - above), "own_height": rms(own_height - above), "tilt": rms(np.hypot(*tilt.T)), "own_tilt": rms(np.hypot(*own_tilt.T)),
                    "flown": float(flown.mean())}}


def fly(job):
    airframe, disturbances, flown, wind, seed, args = job
    if args.source:
        from sw.sim.landing_test import install

        install(args.source, args.defines.split())
    p, draws = plant(airframe, wind, seed, args.gps, args.nominal, args.climb, disturbances, args.headwind, args.flow, args.baro, args.speed_draw, args.gps_noise, args.level_sd)
    s = dict(flown)
    if args.preload:
        # The wind's expected nose-up moment at liftoff, canceled by pitch_bias (rad/s², ramped in once airborne): preload per m/s of the wind.
        s["pitch_bias"] = args.preload * wind
    if args.gps:
        # The drone's belief about where its nose points at takeoff: the truth give or take --heading-error degrees (30: a phone compass and eyeballing; 180: hover_gps_heading left at 0 whichever way the nose points).
        s["hover_gps_heading"] = p["gps"]["heading"] + random.Random(seed + 1).uniform(-args.heading_error, args.heading_error)
    if args.sticks:
        p["stick"] = [(SETTLE + start, pitch, roll, length) for start, pitch, roll, length in args.sticks]
    if args.speed_sticks:
        # As the dashboard sends it without the tag: the nudge wrapped at its limit, only its speed counting.
        p["nudge"] = [(SETTLE + start, forward, left, length) for start, forward, left, length in args.speed_sticks]
        p["nudge_wrap"] = NUDGE_MAX
    if args.gps and args.gps_outage:
        p["gps"]["outage"] = (SETTLE + args.gps_outage[0], args.gps_outage[1])
    if args.ekf or s.get("hover_ekf", 0) > 0:
        p |= {"ekf": True, "armed_s": EKF_REST}
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
        if p.get("ekf") and args.gps:
            out |= filter_errors(log, t, position, math.radians(p["gps"]["heading"]))
    if not result["stable"]:
        return out | {"fate": "lost", "at": result["fail_s"]}
    if lifted is None:
        return out | {"fate": "no liftoff"}
    angles = np.degrees([e[2][:2] for e in log])
    peak = position[(t >= 0) & (t < TAKEOFF_S), 2].max()
    out["overshoot"] = float(peak - position[0, 2] - s["hover_climb"])
    settled = (t >= TAKEOFF_S) & (t < TAKEOFF_S + SETTLED_S)
    if settled.any():
        out["overshoot_settled"] = float(peak - np.median(position[settled, 2]))
    hold = t > SETTLE
    distance = np.hypot(position[hold, 0], position[hold, 1])
    if result.get("touched") is not None:
        landed = any(phase == "land" for _, phase in out.get("failsafe", []))
        return out | {"fate": "failsafe landed" if landed else "touched", "at": result["touched"], "landed_distance": float(np.hypot(*position[-1, :2]))}
    if distance.max() > args.flyaway:
        return out | {"fate": "flew off", "at": float(t[hold][np.argmax(distance > args.flyaway)])}
    height = position[hold, 2] - position[0, 2] - s["hover_climb"]
    if args.gps:
        # The hold frame's heading error at the end (degrees): its x axis points along the world's forward axis (the GPS heading, clockwise from north) turned left by the true yaw less the hover's gyro yaw. And when hover.c first pinned the heading (s after liftoff; NaN if never or if it never waited, as before the gate).
        heading, yaw, _, _ = log[-1][13]
        error = math.degrees(math.remainder(heading - math.radians(p["gps"]["heading"]) + (log[-1][2][2] - yaw), 2 * math.pi))
        flags = [bool(e[13][2]) for e in log]
        pinned = float(t[flags.index(True)]) if True in flags and not all(flags) else math.nan
        out |= {"heading_error": abs(error), "pinned": pinned}
        # The calibration pulses (hover_gps_calibrate, hover.c CALIBRATE_AFTER_S): how far the drone went from where they began until 2 s after their last tick (m; NaN without any).
        pulsing = np.flatnonzero([e[13][3] > 2.0 for e in log])
        if len(pulsing):
            span = slice(pulsing[0], min(pulsing[-1] + 200, len(log)))
            out["calibrate_distance"] = float(np.hypot(*(position[span, :2] - position[pulsing[0], :2]).T).max())
    held = {"fate": "held", "distance": float(np.median(distance)), "distance_max": float(distance.max()), "height": float(np.sqrt(np.mean(height ** 2))),
            "tilt": float(np.abs(angles[hold]).max()), "pitch_rms": float(np.std(angles[hold, 1])), "swing_band": float(np.sqrt(np.mean(band(angles[:, 1], *SWING)[hold] ** 2)))}
    if args.sticks or args.speed_sticks:
        # After each stick is let go (SETTLE + start + length after liftoff, as p["stick"] flies it): the pitch's peak to peak from 0.3 s to 3 s after (the hold braking, and any swing it leaves).
        releases = [(SETTLE + start, SETTLE + start + length) for start, _, _, length in (args.sticks or []) + (args.speed_sticks or [])]
        swings = [float(np.ptp(angles[(t > end + 0.3) & (t < end + 3.0), 1])) for _, end in releases if ((t > end + 0.3) & (t < end + 3.0)).any()]
        held["swing"] = max(swings) if swings else 0.0
        # Braking: the peak ground speed from the stick to STOP_WINDOW after release, how long after release the speed first falls under STOPPED, and how far the drone went from where it was let go by then (NaN if it never stops in the window).
        speed = np.hypot(*np.gradient(position[:, :2], t, axis=0).T)
        brakes = []
        for begin, end in releases:
            window = (t >= begin) & (t <= end + STOP_WINDOW)
            after = np.flatnonzero((t > end) & (t <= end + STOP_WINDOW) & (speed < STOPPED))
            released = np.flatnonzero(t >= end)[0]
            stop = after[0] if len(after) else None
            brakes.append({"peak": float(speed[window].max()), "stop_time": float(t[stop] - end) if stop is not None else math.nan,
                           "stop_distance": float(np.hypot(*(position[stop, :2] - position[released, :2]))) if stop is not None else math.nan})
        held |= {key: max(b[key] for b in brakes) for key in ("peak", "stop_time", "stop_distance")}
    return out | held


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=48, help="per wind")
    parser.add_argument("--winds", default="0,2,4,6", help="mean wind speeds (m/s)")
    parser.add_argument("--seconds", type=float, default=40.0, help="hold after settling")
    parser.add_argument("--climb", type=float, default=1.5, help="hold height above the takeoff point (m)")
    parser.add_argument("--gps", action="store_true")
    parser.add_argument("--heading-error", type=float, default=30.0, help="with --gps: hover_gps_heading off the truth by up to this (degrees, uniform)")
    parser.add_argument("--gps-noise", type=float, help="with --gps: the GPS velocity noise (m/s RMS; default GPS's 0.05)")
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
    parser.add_argument("--source", help="another copy of hover.c to fly (sw/sim/landing_test.py install), e.g. the previous commit's")
    parser.add_argument("--defines", default="", help="compile-time overrides for --source, space separated")
    parser.add_argument("--sticks", type=lambda text: [tuple(float(v) for v in item.split(",")) for item in text.split(";")], help="a pilot's stick sequence during the hold: start s after settling, pitch and roll degrees, seconds held; ';' between sticks (e.g. '5,10,0,2;12,-10,0,2'); adds each run's largest pitch peak to peak 0.3-3 s after a stick is let go, and the braking: peak ground speed, time and distance from release to under STOPPED")
    parser.add_argument("--speed-sticks", type=lambda text: [tuple(float(v) for v in item.split(",")) for item in text.split(";")], help="the same as speeds: start s after settling, forward and left m/s, seconds held (the gamepad's right stick on the GPS hold, which moves the hold point at up to hover_nudge_speed); reported as --sticks")
    parser.add_argument("--level-sd", type=float, default=0.0, help="degrees: each run's level-reference error drawn from a normal of this spread in roll and pitch (2026-11-09 flights: about 1.3)")
    parser.add_argument("--gps-outage", type=lambda text: tuple(float(v) for v in text.split(",")), help="start,seconds: the GPS gives no solution for this long, from start s after settling (sweep.run plant[\"gps\"][\"outage\"])")
    parser.add_argument("--ekf", action="store_true", help="run the Kalman filter (fw/drone/main/ekf.c) beside the hover in every run and print its errors against the truth; --set hover_ekf=1 (or 2, its height too) has the hold fly on it")
    parser.add_argument("--flyaway", type=float, default=FLYAWAY, help="m from takeoff that counts as flown off")
    parser.add_argument("--speed-draw", type=lambda text: tuple(float(v) for v in text.split(",")), default=(0.5, 3.0), help="range the forward speed moment is drawn in, times the airframe's (log-uniform; default 0.5,3 from before any flight measured it; models fitted to the outdoor flights warrant narrower)")
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text())
    flown = recording_settings(recording(args.settings)) | {"hover_camera": 0, "hover_center": 0, "hover_climb": args.climb, "hover_ceiling": args.climb}
    if args.gps:
        flown |= {"hover_gps_tau": 0.5, "hover_gps_place_tau": 2.0}
    if args.baro:
        flown |= {"hover_baro_tau": args.baro}
    flown |= parse_settings(args.set)
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
            if args.sticks or args.speed_sticks:
                line += f", swing after release {q('swing', 50):.1f}° (p90 {q('swing', 90):.1f})"
                stopped = [r for r in held if not math.isnan(r["stop_time"])]
                line += f", peak speed {q('peak', 50):.2f} m/s (p90 {q('peak', 90):.2f}), stopped under {STOPPED} m/s in {len(stopped)}/{len(held)}"
                if stopped:
                    s = lambda key, pct: np.percentile([r[key] for r in stopped], pct)
                    line += f" after {s('stop_time', 50):.1f} s (p90 {s('stop_time', 90):.1f}) and {s('stop_distance', 50):.1f} m (p90 {s('stop_distance', 90):.1f})"
        overshoot = [r["overshoot"] for r in runs if "overshoot" in r]
        if overshoot:
            line += f" | takeoff overshoot median {np.median(overshoot):.2f} m (p90 {np.percentile(overshoot, 90):.2f}, max {max(overshoot):.2f})"
            settled = [r["overshoot_settled"] for r in runs if "overshoot_settled" in r]
            line += f", over the settled height median {np.median(settled):.2f} m (p90 {np.percentile(settled, 90):.2f}, max {max(settled):.2f})"
        if args.gps and held:
            errors = [r["heading_error"] for r in held]
            pinned = [r["pinned"] for r in held if not math.isnan(r["pinned"])]
            line += f", end heading error median {np.median(errors):.0f}° (p90 {np.percentile(errors, 90):.0f})"
            if pinned:
                line += f", pinned in {len(pinned)}/{len(held)} at median {np.median(pinned):.1f} s (p90 {np.percentile(pinned, 90):.1f})"
            moved = [r["calibrate_distance"] for r in held if "calibrate_distance" in r]
            if moved:
                line += f", calibration pulses in {len(moved)} moved it {np.median(moved):.2f} m (p90 {np.percentile(moved, 90):.2f})"
        print(line, flush=True)
        filtered = [r["ekf"] for r in runs if "ekf" in r]
        if filtered:
            # The Kalman filter beside the hover's own estimates, both against the truth: medians over the runs (90th percentile).
            q = lambda key: f"{np.nanmedian([f[key] for f in filtered]):.2f} ({np.nanpercentile([f[key] for f in filtered], 90):.2f})"
            known = [f["aligned_at"] for f in filtered if not math.isnan(f["aligned_at"])]
            print(f"    Kalman filter: heading known in {len(known)}/{len(filtered)}" + (f" at {np.median(known):.1f} s median (p90 {np.percentile(known, 90):.1f}), then {q('heading')}° RMS off" if known else "")
                  + f"; velocity {q('velocity')} m/s RMS off, the hold's own {q('own_velocity')}; height {q('height')} m, own {q('own_height')}; tilt {q('tilt')}°, attitude filter {q('own_tilt')}; flown on it {100 * np.mean([f['flown'] for f in filtered]):.0f}% of the time", flush=True)
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
