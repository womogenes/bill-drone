"""
The firmware's roll flip (fw/drone/main/flip.c) in the full-attitude simulation: uv run -m sw.sim.flip_firmware [--n 256] [--height 1.5] [--bands -2.5] [--settings 2026-11-11T23-35] [--workers 16]

Each run flies drone 1 as of 2026-11-09 (sw/sim/toss.py plant, randomized as recover.randomization) in sw/sim/rigid.py on the firmware's own C: the attitude estimate (flight_math.c flight_attitude, on the gyro alone while the flip runs and FLIP_GRACE after, as flight_rt.c does), the attitude controller (flight_control) at a recording's gains, and flip.c, wired as flight_rt.c wires them. It hovers at --height on a fixed collective, flips at FLIP_AT, and the attitude controller takes it back level with the roll integrators as they were before the flip. The hover hold's height loop is not flown (the collective stays at the hover's), so the height a flip loses is not won back here.
Clean, as sw/sim/flip.py counts it: never touched the ground, within 10 degrees of upright and turning under 1 rad/s at the end, the whole turn made; plus the firmware's tip cut (tip_limit, 45 degrees) not passed from FLIP_GRACE after the hand-back.
"""

import argparse
import ctypes
import math
import os
import subprocess
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import recording, settings as recording_settings
from sw.sim import flight
from sw.sim import recover as C
from sw.sim import rigid as R

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/flip.c"
LIBRARY = Path(__file__).with_name("libflip.so")
FLIP_AT = 1.5
# After the hand-back the attitude estimate stays on the gyro and the tip cut stays off this long (s), as in flight_rt.c.
FLIP_GRACE = 1.0
TIP_LIMIT = math.radians(45)
LOST_WINDOW = 2.5


class Params(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("climb_thrust", "climb_time", "spin_rate", "k_spin", "spin_thrust", "handoff", "kp", "brake", "max_rate", "k_roll", "k_pitch", "kd_yaw", "flip_thrust", "right_angle", "servo_max", "kv", "arrest_time", "timeout")]


class State(ctypes.Structure):
    _fields_ = [("stage", ctypes.c_int), ("time", ctypes.c_float), ("turned", ctypes.c_float), ("thrust", ctypes.c_float), ("vertical", ctypes.c_float), ("arrested", ctypes.c_float), ("landing_roll", ctypes.c_float)]


class Authority(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("roll", "pitch", "yaw", "weight")]


class Output(ctypes.Structure):
    _fields_ = [("owned", ctypes.c_bool), ("collective_set", ctypes.c_bool), ("handed_back", ctypes.c_bool)] + [(name, ctypes.c_float) for name in ("left", "right", "left_tilt", "right_tilt", "collective")]


if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(SOURCE.stat().st_mtime, SOURCE.with_suffix(".h").stat().st_mtime):
    scratch = LIBRARY.with_suffix(".%d.tmp" % os.getpid())
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-iquote", str(SOURCE.parent), str(SOURCE), "-o", str(scratch), "-lm"], check=True)
    scratch.replace(LIBRARY)
library = ctypes.CDLL(str(LIBRARY))
library.flip_start.argtypes = [ctypes.POINTER(State), ctypes.c_float]
library.flip_step.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Params), ctypes.POINTER(Authority), ctypes.c_float * 3, ctypes.c_float, ctypes.c_float, ctypes.POINTER(Output)]
DEFAULTS = Params.in_dll(library, "flip_defaults")


def fly(job, trace=None):
    """One batch of flips: per run, clean and the measures; trace, a list, gets every tick's time, positions, true roll (body z's tilt toward y) and estimated roll, and flip stages."""
    settings, height, offset, seed, n, seconds = job
    from sw.sim import toss
    plant = toss.plant() | {"tilt_offset": offset, "accel_bias": (0.025 if offset else 0.0, 0.0, 0.0)}
    sim = R.Sim(n, plant, seed=seed, scale=C.randomization(n, seed))
    hover = sim.mass * R.G / 2
    obs = sim.reset(np.tile([0, 0, height], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)), motor=np.column_stack([np.full(n, hover), np.full(n, hover)]))
    # The firmware's newtons to throttle (its table less thrust_scale, which on the drone is calibrated to its motors): here calibrated to the plant's motors at hover, so the collective that hovers is the throttle that hovers this plant.
    bases = sim.throttles[:, -1].mean(axis=1)
    calibrations = [flight.thrust(b) / hover for b in bases]
    flown = settings | {"enabled": 1, "roll": 0, "pitch": 0, "yaw_rate": 0, "base_throttle": 0.0, "left_thrust_scale": 1.0, "right_thrust_scale": 1.0, "roll_bias": 0.0, "pitch_bias": 0.0, "yaw_bias": 0.0}
    trims = (math.radians(flown["left_trim"]), math.radians(flown["right_trim"]))
    controllers, estimates = [], []
    for _ in range(n):
        controller = flight.Controller()
        controller.configure(flown)
        controller.reset()
        controller.state[8] = 10.0  # airborne (s): the takeoff ramp is long done
        controllers.append(controller)
        estimates.append(flight.Attitude())
    model = flight.MODEL
    authority = Authority(roll=model["arm"] / settings["roll_inertia"], pitch=settings["pivot_height"] / settings["pitch_inertia"], yaw=model["arm"] / settings["yaw_inertia"], weight=settings["hover_mass"] * 9.81)
    flips = [State() for _ in range(n)]
    saved = [None] * n
    handed = np.full(n, np.inf)
    tipped = np.zeros(n, bool)
    lowest = np.full(n, height)
    out = Output()
    for k in range(int(seconds / R.DT)):
        now = k * R.DT
        throttle = np.zeros((n, 2))
        tilt = np.zeros((n, 2))
        for i in range(n):
            gyro = obs["gyro"][i]
            flipping = flips[i].stage != 0 or now - handed[i] < FLIP_GRACE
            accel = obs["accel"][i] / R.G
            roll, pitch = estimates[i].update(accel, gyro, R.DT, True, banked=flipping)
            # The measured specific force along the estimated up (g), as flight_rt.c's vertical.
            vertical = -accel[0] * math.sin(pitch) + math.cos(pitch) * (accel[1] * math.sin(roll) + accel[2] * math.cos(roll))
            if k == round(FLIP_AT / R.DT):
                library.flip_start(ctypes.byref(flips[i]), hover * 2)
                saved[i] = (controllers[i].state[0], controllers[i].state[1], controllers[i].state[16])
            library.flip_step(ctypes.byref(flips[i]), ctypes.byref(DEFAULTS), ctypes.byref(authority), (ctypes.c_float * 3)(*gyro), vertical, R.DT, ctypes.byref(out))
            if out.handed_back:
                # As flight_rt.c: the estimate's roll wrapped back into a half turn, and the integrators as they were before the flip.
                estimates[i].state[0] = math.atan2(math.sin(estimates[i].state[0]), math.cos(estimates[i].state[0]))
                controllers[i].state[0], controllers[i].state[1], controllers[i].state[16] = saved[i]
                handed[i] = now
            controllers[i].command[6 + flight.TUNING_LAYOUT.index("base_throttle")] = flight.throttle(out.collective / 2 * calibrations[i]) if out.collective_set else bases[i]
            left, right, left_angle, right_angle = controllers[i].update((estimates[i].state[0], estimates[i].state[1]), gyro, R.DT)
            if out.owned:
                throttle[i] = (flight.throttle(max(out.left, 0.0) * calibrations[i]), flight.throttle(max(out.right, 0.0) * calibrations[i]))
                tilt[i] = (out.left_tilt, out.right_tilt)
            else:
                throttle[i] = (left, right)
                tilt[i] = (model["left_servo_direction"] * (left_angle - trims[0]), model["right_servo_direction"] * (right_angle - trims[1]))
            if now - handed[i] >= FLIP_GRACE and not flipping and max(abs(estimates[i].state[0]), abs(estimates[i].state[1])) > TIP_LIMIT:
                tipped[i] = True
        obs = sim.step(throttle, tilt)
        if trace is not None:
            trace.append((now, sim.pos.copy(), sim.up().copy(), [e.state[0] for e in estimates], [f.stage for f in flips], throttle.copy()))
        # The height lost to the flip itself: until LOST_WINDOW after it starts (afterwards the fixed collective's own drift, which the hover hold would take out).
        if now <= FLIP_AT + LOST_WINDOW:
            lowest = np.minimum(lowest, sim.pos[:, 2])
    up = sim.up()[:, 2]
    still = np.linalg.norm(sim.omega, axis=1) < 1.0
    turned = np.array([f.turned for f in flips])
    clean = ~sim.touched & ~tipped & (up > math.cos(math.radians(10))) & still & (turned >= 2 * math.pi - DEFAULTS.handoff - 0.01)
    return [{"clean": bool(clean[i]), "touched": bool(sim.touched[i]), "tipped": bool(tipped[i]), "lost": float(height - lowest[i]), "up": float(up[i])} for i in range(n)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--height", type=float, default=1.5)
    parser.add_argument("--bands", type=float, nargs="*", default=[0.0, -2.5, -5.0], help="tilt offsets (degrees) to fly; nonzero ones get the accelerometer's 0.025 g too")
    parser.add_argument("--settings", default="2026-11-11T23-35")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()
    settings = recording_settings(recording(args.settings))
    batch = 16
    for offset in args.bands:
        jobs = [(settings, args.height, offset, 1000 + seed, batch, args.seconds) for seed in range(0, args.n, batch)]
        with Pool(args.workers) as pool:
            runs = [r for rows in pool.map(fly, jobs) for r in rows]
        lost = np.array([r["lost"] for r in runs])
        print(f"tilt offset {offset:+.1f} deg, from {args.height} m, {len(runs)} flips: clean {np.mean([r['clean'] for r in runs]) * 100:.1f}%, hit the ground {np.mean([r['touched'] for r in runs]) * 100:.1f}%, tip cut after the hand-back {np.mean([r['tipped'] for r in runs]) * 100:.1f}%, upright at the end {np.mean([r['up'] > math.cos(math.radians(10)) for r in runs]) * 100:.0f}%, height lost median {np.median(lost):.2f} m (90th {np.percentile(lost, 90):.2f})")


if __name__ == "__main__":
    main()
