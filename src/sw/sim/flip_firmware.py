"""
The firmware's roll flip (fw/drone/main/flip.c) as flight_rt.c flies it, from the GPS hold, in the full-attitude simulation: uv run -m sw.sim.flip_firmware [--n 256] [--bands 0 -2.5 -5] [--settings 2026-11-11T23-35] [--flip-at 25] [--workers 16] [--no-flip] [--airframe airframe_2026-11-12_flip.json] [--randomize wide|flip|none] [--batch 16]

Each run flies drone 1 as of 2026-11-09 (sw/sim/toss.py plant, randomized as recover.randomization; --airframe another, --randomize flip draws every term the flip leans on that nothing real tests over its whole range: uncertain) in sw/sim/rigid.py on the firmware's own C, ticked in flight_rt.c's order: the attitude estimate (flight_math.c flight_attitude), the flip (flip.c flip_tick), the tip cut (tip_limit, off where flip_tip_exempt says), the fly-away failsafe (guard.c), the hover hold (hover.c) on the GPS (10 Hz, as sweep.run's, sw/sim/outdoor.py GPS) and the barometer (sweep.BARO), the attitude controller (flight_control) and, while the flip owns the motors, flip_actuators. It rests, arms, takes off in the hold and climbs to hover_climb; from --flip-at s after liftoff it asks for the flip once a second as the dashboard's Flip button does, through flip_refusal with the checks flight_runtime_flip makes, until one starts. The IMU reads no more than --gyro-range and --accel-range (the firmware's ±2000 deg/s and ±2 g by default). Calm air: rigid.py has no wind.

Settings: the recording's (--settings), then FLIGHT (the GPS hold as the roll-step ladder was cleared with on 2026-11-12, the hold at 3 m, the flip on), then --set. The firmware's thrust scales and hover_mass are set to each drawn plant's, as the drone's are calibrated to its own motors and weight.

Clean, as sw/sim/flip.py counts it: never touched the ground after the flip began, the tip cut never fired, within 10 degrees of upright and turning under 1 rad/s at the end, the whole turn made. It also prints the height lost (from the flip's start to the lowest point in LOST_WINDOW), the lowest height above the ground, how long the hold took to come back within 0.3 m of its height, and how far the flip carried the drone over the ground. With --no-flip it flies the same plants without the request: the firmware's attitude loop at the recording's gains does not hold every drawn plant (recover.randomization's roll authority spans 0.8-5 times the fit), so the flip's results are also given among the plants that hold without one.
"""

import argparse
import ctypes
import math
import os
import random
import subprocess
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording, settings as recording_settings
from sw.sim import flight
from sw.sim import recover as C
from sw.sim import rigid as R
from sw.sim.hover import GAIN_KEYS, GPS_LATENCY, BARO_LATENCY, HOLD, Gains, Hover, State as HoverState
from sw.sim.guard import Guard
from sw.sim.outdoor import GPS
from sw.sim.sweep import BARO

MAIN = Path(__file__).resolve().parents[2] / "fw/drone/main"
SOURCES = [MAIN / "flip.c", MAIN / "flight_math.c"]
HEADERS = [MAIN / "flip.h", MAIN / "flight_math.h", MAIN / "hover.h", MAIN / "guard.h"]
LIBRARY = Path(__file__).with_name("libflip.so")
# The GPS hold as the roll-step ladder was cleared with on 2026-11-12 (sw/flight_steps.py ladder), held at 3 m (hover_climb and hover_ceiling as settings.def now allows), the flip allowed from 2.5 m.
FLIGHT = {"hover_camera": 0, "hover_center": 0, "hover_gps_tau": 0.5, "hover_gps_place_tau": 2.0, "hover_tilt": 10.0, "hover_kd": 1.5, "hover_thrust_direction": 0.8, "pitch_damping": 6.0,
          "hover_gps_calibrate": 5.0, "roll_target_rate": 360.0, "hover_climb": 3.0, "hover_ceiling": 3.5, "hover_ekf": 0, "flip_enable": 1, "flip_height": 2.5, "thrust_voltage": 0.0}
# The height lost to the flip: its lowest point within this long after it begins (s).
LOST_WINDOW = 4.0
# The belly's height below the center of mass (rigid.py HULL): the height above the ground is the position's less this.
BELLY = 0.035
# The motors' load through the flip (from its start until it is done, at most LOST_WINDOW): time at FULL or more of the throttle, and the current, modeled from each motor's thrust as momentum theory has power (I = CURRENT_FULL (T / THRUST_FULL)^1.5) and pinned at SunnySky's X2212 980KV table, about 31 A on a 10x4.7 at full throttle on 14.8 V (the firmware's table's 11.22 N at 1200 times the 1.28 trim). Not measured on this drone.
FULL = 0.95 * 1200
CURRENT_FULL, THRUST_FULL = 31.0, 11.22 * 1.28
# Keep each flip's trace in its row (flip_replay.py --closed sets it).
TRACE = False
# A fresh pack under load (V), for flip_refusal's battery check.
BATTERY = 16.0


class Params(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("climb_thrust", "climb_time", "spin_rate", "k_spin", "spin_thrust", "handoff", "kp", "brake", "max_rate", "k_roll", "k_pitch", "kd_yaw", "flip_thrust", "right_angle", "servo_max", "roll_over", "kv", "arrest_time", "timeout", "floor", "ceiling", "handoff_rate", "slew", "throttle_cap", "spin_lift")]


class Flip(ctypes.Structure):
    _fields_ = [("stage", ctypes.c_int), ("q", ctypes.c_float * 4)] + [(name, ctypes.c_float) for name in ("time", "turned", "thrust", "vertical", "arrested")]


class Authority(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("roll", "pitch", "yaw", "weight")]


class Output(ctypes.Structure):
    _fields_ = [("owned", ctypes.c_bool), ("collective_set", ctypes.c_bool), ("handed_back", ctypes.c_bool)] + [(name, ctypes.c_float) for name in ("left", "right", "left_tilt", "right_tilt", "collective", "roll", "pitch")]


class Check(ctypes.Structure):
    _fields_ = ([(name, ctypes.c_bool) for name in ("enabled", "armed", "done", "holding", "camera", "gps", "aligned", "located", "ekf", "landing", "failsafe", "pattern")]
                + [(name, ctypes.c_float) for name in ("airborne", "height", "climb_rate", "speed", "roll", "pitch")] + [("rate", ctypes.c_float * 3), ("min_height", ctypes.c_float), ("battery", ctypes.c_float)])


class Run(ctypes.Structure):
    _fields_ = [("flip", Flip), ("authority", Authority), ("done", ctypes.c_bool), ("since", ctypes.c_float), ("saved", ctypes.c_float * 18), ("height_integral", ctypes.c_float), ("params", ctypes.POINTER(Params)), ("throttle", ctypes.c_float * 2)]


if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(path.stat().st_mtime for path in SOURCES + HEADERS):
    scratch = LIBRARY.with_suffix(".%d.tmp" % os.getpid())
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-iquote", str(MAIN), *map(str, SOURCES), "-o", str(scratch), "-lm"], check=True)
    scratch.replace(LIBRARY)
library = ctypes.CDLL(str(LIBRARY))
FLOATS = ctypes.POINTER(ctypes.c_float)
library.flip_refusal.argtypes = [ctypes.POINTER(Check)]
library.flip_refusal.restype = ctypes.c_char_p
library.flip_reset.argtypes = [ctypes.POINTER(Run)]
library.flip_begin.argtypes = [ctypes.POINTER(Run), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
library.flip_abort.argtypes = [ctypes.POINTER(Run), FLOATS]
library.flip_abort.restype = ctypes.c_bool
for name in ("flip_active", "flip_gyro_only", "flip_tip_exempt"):
    getattr(library, name).argtypes = [ctypes.POINTER(Run)]
    getattr(library, name).restype = ctypes.c_bool
library.flip_phase.argtypes = [ctypes.POINTER(Run)]
library.flip_tick.argtypes = [ctypes.POINTER(Run), ctypes.c_float * 3, ctypes.c_float * 3, ctypes.c_float, ctypes.c_void_p, ctypes.c_void_p, FLOATS, ctypes.POINTER(Output)]
library.flip_limit.argtypes = [ctypes.POINTER(Run), ctypes.c_float * 2, ctypes.c_float]
library.flip_actuators.argtypes = [ctypes.POINTER(Output), ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float * 2, ctypes.c_float * 2]
DEFAULTS = Params.in_dll(library, "flip_defaults")
# Command fields flight.Controller's configure leaves alone: the base throttle and attitude targets (radians), the thrust scales, and the hover gains' mass (flip_begin's weight).
BASE, ROLL, PITCH, YAW = (6 + flight.TUNING_LAYOUT.index(key) for key in ("base_throttle", "roll", "pitch", "yaw_rate"))
SCALES = [6 + flight.TUNING_LAYOUT.index(key) for key in ("left_thrust_scale", "right_thrust_scale")]
HOVER_MASS = flight.PITCH_SPEED + 1 + GAIN_KEYS.index("mass")


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Sensors:
    """One drone's GPS and barometer as sweep.run makes them: GPS solutions at GPS["rate"] with white velocity noise and a wandering position error, arriving GPS["latency"] after their epoch (and a false descent at liftoff); barometer readings at BARO["rate"] with white noise and a slow drift, BARO["latency"] late. Each comes once, stamped as flight_rt.c stamps it."""

    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.gps, self.baro = [], []
        self.gps_capture = self.baro_capture = 0.0
        self.error = [0.0, 0.0, 0.0]
        self.drift = 0.0
        self.descent = self.rng.uniform(*GPS["liftoff_descent"])

    def step(self, now, position, velocity, lifted, ground):
        period = 1 / GPS["rate"]
        while self.gps_capture <= now:
            keep = math.exp(-period / GPS["position_tau"])
            sizes = [GPS["position_noise"]] * 2 + [2 * GPS["position_noise"]]
            self.error = [keep * e + s * math.sqrt(1 - keep * keep) * self.rng.gauss(0, 1) for e, s in zip(self.error, sizes)]
            climb = -self.descent if lifted is not None and now - lifted < GPS["liftoff_s"] else velocity[2]
            # North is the world's forward axis, east its right.
            speed = [velocity[0] + self.rng.gauss(0, GPS["velocity_noise"]), -velocity[1] + self.rng.gauss(0, GPS["velocity_noise"]), climb + self.rng.gauss(0, GPS["vertical_noise"])]
            place = [position[0] + self.error[0], -position[1] + self.error[1], position[2] + self.error[2]]
            self.gps.append((self.gps_capture + GPS["latency"], speed, place))
            self.gps_capture += period
        gps = None
        while self.gps and self.gps[0][0] <= now:
            arrived, speed, place = self.gps.pop(0)
            gps = (round((arrived - GPS_LATENCY + 1) * 1e6), speed, place)
        period = 1 / BARO["rate"]
        while self.baro_capture <= now:
            keep = math.exp(-period / BARO["drift_tau"])
            self.drift = keep * self.drift + BARO["drift"] * math.sqrt(1 - keep * keep) * self.rng.gauss(0, 1)
            self.baro.append((self.baro_capture + BARO["latency"], 100.0 + position[2] - ground + self.drift + self.rng.gauss(0, BARO["noise"])))
            self.baro_capture += period
        baro = None
        while self.baro and self.baro[0][0] <= now:
            arrived, altitude = self.baro.pop(0)
            baro = (round((arrived - BARO_LATENCY + 1) * 1e6), altitude)
        return gps, baro


def uncertain(seed, n):
    """
    One draw of every plant and sensor term the flip leans on that no real data pins down (models/NOTES.md "The first flip, replayed"), spanning what the flip's replay, the alternatives that fit it and physics allow rather than a spread about the fit: airframe overrides shared by a batch, per-run Sim scales, and per-run gyro scale factors.
    """
    rng = np.random.default_rng(seed + 31)
    u = lambda lo, hi: float(math.exp(rng.uniform(math.log(lo), math.log(hi))))
    overrides = {
        # The rotors' spin momentum: 0.006-0.05 fits the flip under one structure or another; a 10x4.5 prop's I times its speed gives about 0.02.
        "rotor_momentum": u(0.006, 0.04),
        # A cut motor's thrust: coasting (0.15 s fitted, about 0.2 from the prop's drag and inertia) or braked by the ESC (as fast as it rises).
        "motor_coast": u(0.05, 0.3),
        # Roll damping at the flip's rates: 0.04-0.3 across the fits (0.046 in hover).
        "inflow_damping": u(0.03, 0.3),
        # Yaw from roll: the hover's lump (-0.023) to the flip's (+0.002) and past it.
        "yaw_from_roll": float(rng.uniform(-0.025, 0.01)),
        "prop_torque": float(rng.uniform(-0.015, 0.015)),
        # The motors' rise: the hover fit's 0.066 s, unmeasured on the bench.
        "motor_tau": u(0.04, 0.1),
        # The table's top was never flown: full throttle on the ESCs' 30 A and a loaded pack may give 70-100% of it (11.2 N at 1200, times the trim).
        "motor_ceiling": float(rng.uniform(0.7, 1.0) * 11.22 * 1.28),
        # Whether the ESC stops a motor asked for throttle 1 (then it restarts from standstill, motor_start 0.16 s).
        "motor_idle": float(rng.choice([0.0, 30.0])),
    }
    overrides["right_motor_tau"] = overrides["motor_tau"]
    w = lambda lo, hi: np.exp(rng.uniform(math.log(lo), math.log(hi), n))
    # The differential thrust's share far from hover (the flip fits 0.42-0.57 if it is a plant term, the ladders and the PRBS hovers 1), the tilt's pitch authority, the inertias (estimated, not measured, after the pivots were raised), drag.
    scale = {"roll_thrust": w(0.45, 1.1), "pitch_tilt": w(0.7, 1.4), "inertia": w(0.8, 1.3), "drag": w(0.6, 1.6), "disturbance": w(0.5, 2.0)}
    # The MPU6050's sensitivity tolerance (±3%) and cross-axis error, as a scale per axis.
    gyro = rng.uniform(0.97, 1.03, (n, 3))
    return overrides, scale, gyro


def plant_of(airframe):
    """A rigid.Sim plant from a saved airframe (sw/sim/toss.py's rotor drag and no disturbance terms)."""
    from sw.sim import toss
    params = R.A.model(airframe)
    return {"airframe": params, "geometry": R.A.geometry_of(params), "disturbances": [], "rotor_drag": toss.ROTOR_DRAG}


def fly(job):
    """One batch of runs on the same drawn plants: per run, whether and when the flip started, why it was refused, and the measures."""
    settings, offset, seed, n, seconds, gyro_range, accel_range, flip_at, flipping, airframe, randomize = job[:11]
    # The maneuver's parameters: flip.c's flip_defaults with the job's changes (job[11], a dict, for tuning).
    params = Params.from_buffer_copy(DEFAULTS)
    for key, value in (job[11] if len(job) > 11 else {}).items():
        setattr(params, key, value)
    plant = plant_of(airframe) | {"tilt_offset": offset, "accel_bias": (0.025 if offset else 0.0, 0.0, 0.0)}
    gyro_scale = np.ones((n, 3))
    if randomize == "flip":
        overrides, scale, gyro_scale = uncertain(seed, n)
        plant["airframe"] = plant["airframe"] | overrides
    else:
        scale = C.randomization(n, seed) if randomize == "wide" else None
    sim = R.Sim(n, plant, seed=seed, scale=scale)
    obs = sim.reset(np.tile([0, 0, BELLY], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)))
    hover_n = sim.mass * R.G / 2
    # Each plant's motors against the firmware's thrust table: the drone's thrust scales are calibrated to its own motors (requested newtons are divided by them before the table lookup).
    sim.reset(np.tile([0, 0, 3.0], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)), motor=np.column_stack([np.full(n, hover_n), np.full(n, hover_n)]))
    scales = [hover_n / flight.thrust(b) for b in sim.throttles[:, -1].mean(axis=1)]
    obs = sim.reset(np.tile([0, 0, BELLY], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)))
    flown = settings | {"enabled": 1, "roll": 0, "pitch": 0, "yaw_rate": 0, "base_throttle": 0.0, "hover_mass": sim.mass}
    trim = math.radians(flown["pitch_trim"])
    trims = (math.radians(flown["left_trim"]), math.radians(flown["right_trim"]))
    directions = (flight.MODEL["left_servo_direction"], flight.MODEL["right_servo_direction"])
    tip_limit = math.radians(flown.get("tip_limit", 45.0))
    lanes = []
    for i in range(n):
        controller = flight.Controller()
        controller.configure(flown)
        controller.command[flight.YAW_I] = flown.get("yaw_i", 0.0)
        controller.command[HOVER_MASS] = sim.mass
        for k, scale in zip(SCALES, (scales[i], scales[i])):
            controller.command[k] = scale
        controller.reset()
        run = Run()
        library.flip_reset(ctypes.byref(run))
        run.params = ctypes.pointer(params)
        hover = Hover(flown)
        lanes.append({"controller": controller, "estimate": flight.Attitude(accel=obs["accel"][i] / R.G), "hover": hover, "guard": Guard(flown), "run": run, "sensors": Sensors(seed * 1000 + i),
                      "integral": ctypes.cast(ctypes.addressof(hover.state) + HoverState.integral.offset, FLOATS), "commanded": (0, 0), "dead": False, "tipped": False, "lifted": None,
                      "started": None, "refusals": [], "asked": None, "touched": False, "lowest": np.inf, "start_height": None, "start_place": None, "back": None, "phase": 0, "target": None})
    armed = round(1.0 / R.DT)
    out = Output()
    for k in range(int(seconds / R.DT)):
        now = k * R.DT
        now_us = round((now + 1) * 1e6)
        throttle = np.zeros((n, 2))
        tilt = np.zeros((n, 2))
        for i, lane in enumerate(lanes):
            if lane["dead"]:
                continue
            controller, estimate, hover, run = lane["controller"], lane["estimate"], lane["hover"], lane["run"]
            height = sim.pos[i, 2] - BELLY
            if lane["lifted"] is None and k > armed and height > 0.05:
                lane["lifted"] = now
            # The dashboard's Flip button, once a second from flip_at after liftoff until one starts: flight_runtime_flip's checks against the last tick's state.
            if flipping and lane["started"] is None and lane["lifted"] is not None and now - lane["lifted"] >= flip_at and (lane["asked"] is None or now - lane["asked"] >= 1.0):
                lane["asked"] = now
                s = hover.state
                pose = (wrap(estimate.state[0]), wrap(estimate.state[1]))
                check = Check(enabled=flown["flip_enable"] >= 1, armed=True, done=run.done, holding=bool(s.engaged), camera=flown.get("hover_camera", 1) != 0, gps=flown.get("hover_gps_tau", 0) > 0,
                              aligned=bool(s.aligned), located=bool(s.gps_us) and now_us - s.gps_us <= 500000, ekf=flown.get("hover_ekf", 0) > 0, landing=False, failsafe=lane["guard"].state.failsafe != 0,
                              pattern=False, airborne=controller.state[10], height=s.height, climb_rate=s.climb_rate, speed=math.hypot(s.gps_fix[0], s.gps_fix[1]), roll=pose[0], pitch=pose[1] - trim,
                              rate=(ctypes.c_float * 3)(*lane["gyro"]), min_height=flown["flip_height"], battery=BATTERY)
                refusal = library.flip_refusal(ctypes.byref(check))
                if refusal:
                    lane["refusals"].append(refusal.decode())
                else:
                    library.flip_begin(ctypes.byref(run), controller.command.buffer_info()[0], controller.state.buffer_info()[0], s.integral, s.force, estimate.state[0], estimate.state[1], s.climb_rate)
                    lane["started"], lane["start_height"], lane["start_place"], lane["target"] = now, height, sim.pos[i, :2].copy(), s.target
            gyro = np.clip(obs["gyro"][i] * gyro_scale[i], -gyro_range, gyro_range)
            accel = np.clip(obs["accel"][i] / R.G, -accel_range, accel_range)
            lane["gyro"] = gyro
            spinning = sum(lane["commanded"]) > 0
            estimate.update(accel, gyro, R.DT, spinning, banked=library.flip_gyro_only(ctypes.byref(run)))
            enabled = k >= armed
            if enabled:
                library.flip_tick(ctypes.byref(run), (ctypes.c_float * 3)(*gyro), (ctypes.c_float * 3)(*accel), R.DT, estimate.state.buffer_info()[0], controller.state.buffer_info()[0], lane["integral"], ctypes.byref(out))
            else:
                out = Output()
            roll, pitch = wrap(estimate.state[0]), wrap(estimate.state[1])
            if enabled and not library.flip_tip_exempt(ctypes.byref(run)) and max(abs(roll), abs(pitch)) > tip_limit:
                lane["dead"] = lane["tipped"] = True
                continue
            vertical = -accel[0] * math.sin(estimate.state[1]) + math.cos(estimate.state[1]) * (accel[1] * math.sin(estimate.state[0]) + accel[2] * math.cos(estimate.state[0]))
            gps, baro = lane["sensors"].step(now, sim.pos[i], sim.vel[i], lane["lifted"], BELLY)
            if not enabled:
                hover.rest(vertical, False, R.DT, baro)
                throttle[i] = (0, 0)
                continue
            s = hover.state
            flying = s.force if s.engaged else 0.0
            heading_rate = (gyro[1] * math.sin(roll) + gyro[2] * math.cos(roll)) / math.cos(pitch)
            left_force, right_force = flight.thrust(controller.state[2]), flight.thrust(controller.state[3])
            left_tilt, right_tilt = directions[0] * (controller.state[4] - trims[0]), directions[1] * (controller.state[5] - trims[1])
            up = left_force * math.cos(left_tilt) + right_force * math.cos(right_tilt)
            drag = (list(accel), math.atan2(left_force * math.sin(left_tilt) + right_force * math.sin(right_tilt), up) if up > 0 else 0.0)
            gains = Gains.from_buffer_copy(hover.gains)
            mode = lane["guard"].hover(s, gains, HOLD, now_us, R.DT)
            if lane["guard"].state.failsafe:
                library.flip_abort(ctypes.byref(run), lane["integral"])
            if library.flip_active(ctypes.byref(run)):
                mode = HOLD
            result = hover.update(mode, None, now_us, R.DT, roll, pitch, heading_rate, vertical, flying, drag, None, gps, baro, gains=gains)
            base = min(1200.0, flight.throttle(result.force / 2 / math.cos(trim))) if result.force > 0 else 0.0
            if out.collective_set:
                base = min(1200.0, flight.throttle(out.collective / 2 / math.cos(trim)))
            controller.command[BASE], controller.command[ROLL], controller.command[PITCH], controller.command[YAW] = base, result.roll, result.pitch, flown.get("yaw_rate", 0.0)
            left, right, left_angle, right_angle = controller.update((roll, pitch), gyro, R.DT)
            # As flight_rt.c: the attitude controller's throttles rounded, or the flip's own while it owns the motors, then flip_limit's slew and cap.
            throttles = (ctypes.c_float * 2)(round(min(max(left, 0.0), 1200.0)), round(min(max(right, 0.0), 1200.0)))
            if out.owned:
                servos = (ctypes.c_float * 2)()
                library.flip_actuators(ctypes.byref(out), controller.command.buffer_info()[0], scales[i], scales[i], 1200.0, 60.0, throttles, servos)
                left_angle, right_angle = math.radians(servos[0]), math.radians(servos[1])
            library.flip_limit(ctypes.byref(run), throttles, R.DT)
            left, right = round(throttles[0]), round(throttles[1])
            lane["commanded"] = (left, right)
            throttle[i] = (left, right)
            tilt[i] = (directions[0] * (left_angle - trims[0]), directions[1] * (right_angle - trims[1]))
            lane["phase"] = max(lane["phase"], library.flip_phase(ctypes.byref(run)))
            # The flip as the log has it (flip_replay.py and the plots compare it with the real one): from 0.2 s before it to 2 s after its start, the gyro (deg/s), throttles, servo angles (degrees), phase and height.
            if TRACE and lane["started"] is not None and now - lane["started"] <= 2.0:
                lane.setdefault("trace", []).append((now - lane["started"], *np.degrees(gyro), left, right, math.degrees(left_angle), math.degrees(right_angle), library.flip_phase(ctypes.byref(run)), height))
        obs = sim.step(throttle, tilt)
        for i, lane in enumerate(lanes):
            if lane["started"] is None:
                continue
            height = sim.pos[i, 2] - BELLY
            if now - lane["started"] <= LOST_WINDOW:
                if library.flip_active(ctypes.byref(lane["run"])):
                    current = CURRENT_FULL * (np.maximum(sim.state.motor[i], 0.0) / THRUST_FULL) ** 1.5
                    lane["full"] = lane.get("full", np.zeros(2)) + R.DT * (np.array(lane["commanded"]) >= FULL)
                    lane["peak_current"] = max(lane.get("peak_current", 0.0), float(current.max()))
                    lane["charge"] = lane.get("charge", np.zeros(2)) + current * R.DT
                    lane["active"] = lane.get("active", 0.0) + R.DT
                lane["lowest"] = min(lane["lowest"], height)
                lane["touched"] |= bool(sim.contact_now[i] > 0)
                lane["carried"] = float(np.linalg.norm(sim.pos[i, :2] - lane["start_place"]))
            if lane["back"] is None and not library.flip_active(ctypes.byref(lane["run"])) and abs(height - lane["start_height"]) < 0.3:
                lane["back"] = now - lane["started"]
    up = sim.up()[:, 2]
    still = np.linalg.norm(sim.omega, axis=1) < 1.0
    rows = []
    for i, lane in enumerate(lanes):
        upright = bool(up[i] > math.cos(math.radians(10)) and still[i] and not lane["tipped"])
        turned = lane["run"].flip.turned
        row = {"held": upright and sim.pos[i, 2] - BELLY > 0.5, "tipped": lane["tipped"], "started": lane["started"], "refusals": lane["refusals"], "phase": lane["phase"]}
        if lane["started"] is not None:
            row |= {"clean": upright and not lane["touched"] and turned >= 2 * math.pi - DEFAULTS.handoff - 0.01, "touched": lane["touched"], "lost": lane["start_height"] - lane["lowest"], "lowest": lane["lowest"],
                    "back": lane["back"], "carried": lane.get("carried", 0.0), "start_height": lane["start_height"], "end_height": float(sim.pos[i, 2] - BELLY), "trace": lane.get("trace"),
                    "full": float(lane.get("full", np.zeros(2)).max()), "peak_current": lane.get("peak_current", 0.0), "mean_current": float((lane.get("charge", np.zeros(2)) / max(lane.get("active", 0.0), R.DT)).max())}
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--bands", type=float, nargs="*", default=[0.0, -2.5, -5.0], help="tilt offsets (degrees) to fly; nonzero ones get the accelerometer's 0.025 g too")
    parser.add_argument("--settings", default="2026-11-11T23-35")
    parser.add_argument("--set", default="", help="settings on top of FLIGHT, name=value,...")
    parser.add_argument("--flip-at", type=float, default=25.0, help="the first flip request, s after liftoff")
    parser.add_argument("--seconds", type=float, default=50.0)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--gyro-range", type=float, default=2000.0, help="the gyro's full scale (deg/s; the firmware's ±2000)")
    parser.add_argument("--accel-range", type=float, default=2.0, help="the accelerometer's full scale (g)")
    parser.add_argument("--no-flip", action="store_true", help="the same plants without the request only")
    parser.add_argument("--airframe", default="airframe_2026-11-09.json", help="the plant's airframe in sw/sim/models (airframe_2026-11-12_flip.json: fitted to the first real flip)")
    parser.add_argument("--randomize", choices=("wide", "flip", "none"), default="wide", help="wide: recover.randomization about the airframe (as before); flip: every term the flip leans on that nothing real tests, drawn over its whole plausible range (uncertain); none: the airframe as saved")
    parser.add_argument("--batch", type=int, default=16, help="runs per job; with --randomize flip each job draws its own untested terms")
    args = parser.parse_args()
    settings = recording_settings(recording(args.settings)) | FLIGHT | parse_settings(args.set)
    batch = args.batch
    print(f"settings {args.settings} with {','.join(f'{k}={v:g}' for k, v in (FLIGHT | parse_settings(args.set)).items())}; gyro ±{args.gyro_range:g} deg/s, accelerometer ±{args.accel_range:g} g; flip asked from {args.flip_at:g} s after liftoff")
    for offset in args.bands:
        results = {}
        for flipping in (False,) if args.no_flip else (False, True):
            jobs = [(settings, offset, 1000 + seed, batch, args.seconds, math.radians(args.gyro_range), args.accel_range, args.flip_at, flipping, args.airframe, args.randomize) for seed in range(0, args.n, batch)]
            with Pool(args.workers) as pool:
                results[flipping] = [r for rows in pool.map(fly, jobs) for r in rows]
        base = results[False]
        print(f"  tilt offset {offset:+.1f} deg, {len(base)} plants: without a flip {sum(r['held'] for r in base)} hold (tip cut {sum(r['tipped'] for r in base)})")
        if args.no_flip:
            continue
        runs = results[True]
        started = [r for r in runs if r["started"] is not None]
        refused = {}
        for r in runs:
            for reason in r["refusals"]:
                refused[reason] = refused.get(reason, 0) + 1
        if not started:
            print(f"    no flip started; refusals {refused}")
            continue
        held = [f for b, f in zip(base, runs) if b["held"] and f["started"] is not None]
        col = lambda rows, key: np.array([r[key] for r in rows], float)
        lost, lowest, carried = col(started, "lost"), col(started, "lowest"), col(started, "carried")
        back = np.array([r["back"] if r["back"] is not None else np.inf for r in started])
        print(f"    flips started {len(started)} of {len(runs)} (at {np.median(col(started, 'started')):.1f} s median, height {np.median(col(started, 'start_height')):.2f} m); refusals before {refused}")
        print(f"    clean {np.mean([r['clean'] for r in started]) * 100:.1f}%, touched the ground {np.mean([r['touched'] for r in started]) * 100:.1f}%, tip cut {np.mean([r['tipped'] for r in started]) * 100:.1f}%, holding at the end {np.mean([r['held'] for r in started]) * 100:.1f}%; "
              f"among the {len(held)} plants that hold without a flip: clean {np.mean([r['clean'] for r in held]) * 100 if held else float('nan'):.1f}%, touched {np.mean([r['touched'] for r in held]) * 100 if held else float('nan'):.1f}%")
        print(f"    height lost median {np.median(lost):.2f} m (90th {np.percentile(lost, 90):.2f}, max {lost.max():.2f}); lowest above the ground median {np.median(lowest):.2f} m (10th {np.percentile(lowest, 10):.2f}, min {lowest.min():.2f}); "
              f"back within 0.3 m of the start height {np.median(back):.1f} s median (90th {np.percentile(back, 90):.1f}); carried over the ground {np.median(carried):.2f} m median (90th {np.percentile(carried, 90):.2f})")
        full, peak, mean = col(started, "full"), col(started, "peak_current"), col(started, "mean_current")
        print(f"    motors through the flip (the busier one): at {FULL:.0f} throttle or more {np.median(full):.2f} s median (90th {np.percentile(full, 90):.2f}, max {full.max():.2f}); modeled current peak {np.median(peak):.0f} A median (max {peak.max():.0f}), mean {np.median(mean):.1f} A median (max {mean.max():.1f})")


if __name__ == "__main__":
    main()
