"""
IMU-only hover drift with and without the rotor-drag velocity estimator (hover_drag): uv run -m sw.sim.drag [--seeds 2] [--lost]
Flies the FITTED plant from takeoff with the camera off on the settings last flown, through the error cases the estimator has to survive (accelerometer bias, level-reference error, drag coefficient mismatch, servo tilt model errors, wind), and prints the horizontal distance from the takeoff point at 10, 20 and 30 s after liftoff; with --lost it flies on the tag first and measures from where the tag was lost.
"""

import argparse
import math
from multiprocessing import Pool

from sw.sim.sweep import FITTED, run

# The settings flown on 2026-10-25T09-20-21 (the recordings are not in git), with the camera off.
SETTINGS = {
    "roll_p": 40, "roll_i": 20, "roll_damping": 10, "pitch_p": 20, "pitch_i": 20, "pitch_damping": 10, "yaw_damping": 10, "integral_limit": 20,
    "pitch_trim": -2.7, "left_trim": 0.9, "right_trim": -0.9, "pivot_height": 0.03, "com_x": 0, "roll_inertia": 0.011848, "pitch_inertia": 0.002303,
    "yaw_inertia": 0.010671, "reaction_torque_ratio": 0.015, "arm": 0.172, "roll_bias": -1.2, "pitch_bias": 5, "yaw_bias": 0.8,
    "servo_compensation": 0.5, "integrate_throttle": 580, "target_rate": 30, "velocity_damping": 0, "velocity_leak": 3, "yaw_i": 10,
    "hover_camera": 0, "hover_kp": 0.5, "hover_kd": 1.2, "hover_tilt": 4, "hover_kside": 0.5, "hover_kz": 1.5, "hover_kdz": 4, "hover_kiz": 2,
    "hover_mass": 0.86, "hover_climb": 0.3, "hover_climb_rate": 0.3, "hover_land_rate": 0.2, "hover_kyaw": 2, "hover_yaw_rate": 30,
    "hover_center": 1, "hover_distance": 2, "hover_approach": 0.25, "hover_ceiling": 0.8, "hover_cam_pitch": 3,
}
# The estimator as recommended for the drone (k from sw/fit_drag.py); in the simulation the servos have no zero error, so the tilt offset is 0 unless a case adds one.
DRAG = {"hover_drag": 0.5, "hover_drag_tilt": 0.0, "hover_drag_tau": 0.7}
# IMU only from takeoff: the attitude controller's own damping (off as flown, or flight_math's tilt dead reckoning) against the drag estimator.
CONFIGS = {
    "as flown (no velocity hold)": {"velocity_damping": 0},
    "tilt damping (velocity_damping 8)": {"velocity_damping": 8},
    "drag estimator": DRAG | {"velocity_damping": 0},
}
# The tag held, then lost: hover's tilt dead reckoning with the trim learned on the tag (as before) against the drag estimator with the offset learned on it.
LOST_AFTER = 12
LOST_CONFIGS = {
    "tilt dead reckoning (hover_drag 0)": {"velocity_damping": 0, "hover_camera": 1, "hover_ceiling": 1.5},
    "drag estimator": DRAG | {"velocity_damping": 0, "hover_camera": 1, "hover_ceiling": 1.5},
}
# The tag a metre up, so the camera hold flies high enough that its own height error at the loss does not put the drone on the floor.
LOST_PLANT = {"camera_lost": LOST_AFTER, "camera": {"tag": (2.3, 0.0, 1.0)}}
CASES = {
    "nominal": {},
    "accel bias x +0.02 g": {"accel_bias": (0.02, 0.0)},
    "accel bias x -0.02 g": {"accel_bias": (-0.02, 0.0)},
    "accel bias y +0.02 g": {"accel_bias": (0.0, 0.02)},
    "level error pitch +1°": {"level_error": (0.0, 1.0)},
    "level error pitch -1°": {"level_error": (0.0, -1.0)},
    "level error roll +1°": {"level_error": (1.0, 0.0)},
    "drag k 0.25 (set 0.5)": {"rotor_drag": 0.25},
    "drag k 0.75 (set 0.5)": {"rotor_drag": 0.75},
    "servo tilt offset +1°": {"tilt_offset": 1.0},
    "servo tilt offset -1°": {"tilt_offset": -1.0},
    "servo tilt gain 0.7": {"tilt_gain": 0.7},
    "wind gusts 0.3 m/s": {"wind": 0.3},
    "bias -0.02 g + level -1° + tilt +1°": {"accel_bias": (-0.02, 0.0), "level_error": (0.0, -1.0), "tilt_offset": 1.0},
}
# The IMU-only height hold wanders by centimetres per second in the simulation (the measured vertical specific force, projected with an attitude estimate a degree or so off while the servos tilt the thrust 5° forward, reads a few mg wrong) and lets the drone down within 10-20 s; this study is of the horizontal estimator, so the hover gets the true vertical specific force.
PLANT = FITTED | {"ideal_height": True}


def fly(job):
    """
    One flight; returns the horizontal distance (m) each whole second after liftoff from the takeoff point, or with the tag lost, from where it was lost.
    """
    config, case, seed, seconds, lost = job
    plant = PLANT | CASES[case] | (LOST_PLANT if lost else {})
    result = run(SETTINGS | (LOST_CONFIGS if lost else CONFIGS)[config], plant, seconds=seconds, seed=seed, hover=True)
    places = {t: (x, y) for t, x, y in result.get("places", []) if result.get("touched") is None or t < result["touched"]}
    origin = places.get(LOST_AFTER, (0.0, 0.0)) if lost else (0.0, 0.0)
    return config, case, seed, result["stable"], {t - (LOST_AFTER if lost else 0): math.hypot(x - origin[0], y - origin[1]) for t, (x, y) in places.items()}


def table(results, configs, cases, marks, title):
    print(f"\n{title}\n{'case':40s} {'estimator':36s} " + " ".join(f"{t:>12d} s" for t in marks) + "   (mean / worst over seeds still airborne, m; Nv: N seeds already on the floor, let down by the height hold)")
    for case in cases:
        for config in configs:
            runs = [r for r in results if r[0] == config and r[1] == case]
            cells = []
            for t in marks:
                values = [r[4][t] for r in runs if r[3] and t in r[4]]
                down = f" {len(runs) - len(values)}v" if len(values) < len(runs) else "   "
                cells.append(f"{sum(values) / len(values):5.2f}/{max(values):4.1f}{down}" if values else "all down")
            print(f"{case:40s} {config:36s} " + " ".join(f"{cell:>14s}" for cell in cells))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=32.0)
    parser.add_argument("--seeds", type=int, default=2)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--cases", nargs="*", help="only these cases (default all)")
    parser.add_argument("--lost", action="store_true", help=f"fly on the tag, lose it {LOST_AFTER} s after liftoff, and measure from there")
    args = parser.parse_args()
    cases = args.cases or list(CASES)
    configs = LOST_CONFIGS if args.lost else CONFIGS
    seconds = LOST_AFTER + args.seconds if args.lost else args.seconds
    jobs = [(config, case, seed, seconds, args.lost) for case in cases for config in configs for seed in range(args.seeds)]
    with Pool(args.workers) as pool:
        results = pool.map(fly, jobs, chunksize=1)
    title = f"Distance from where the tag was lost ({LOST_AFTER} s after liftoff)" if args.lost else "IMU only from takeoff: distance from the takeoff point"
    table(results, configs, cases, (10, 20, 30), title)


if __name__ == "__main__":
    main()
