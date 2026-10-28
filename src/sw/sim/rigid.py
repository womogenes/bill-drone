"""
A vectorized full-attitude simulation of the bicopter for recovery control and policy training: many drones at once (numpy, [env, ...] arrays), any orientation (quaternions), any rates, with the identified actuators and the calibrated disturbances.

Frames: world x forward, y left, z up; body x forward, y left, z up through the rotors' shafts at zero tilt. Quaternions are (w, x, y, z), body to world.
Actuators and torques are the identified airframe model itself (sw/sim/airframe.py step, the post-ESC-recalibration fit by default), so near hover this sim predicts exactly as well as the identification: motors with dead time and lag on the thrust table, servos with dead time, a second-order loop, slew, link and play, the rubber bands, the identified effectiveness, damping, attitude and speed terms and steady pushes. Its attitude terms are fed the roll and pitch of the body's tilt (clipped to ±0.6 rad, beyond which they were never measured). This sim adds what that model leaves out: rotation in any orientation (quaternions, with the full ω × Iω coupling), translation (the thrust vector, gravity, the calibrated rotor drag and quadratic air drag), ground contact, and sw/sim/disturbance.py terms, which come from the rotors' wash and the flight model's error, so both the force and the torque terms are scaled by the motors' total thrust over the weight (clipped to 0-1) and fade out on a landed drone. Motors cannot reverse, and servos stop at ±60°.
Ground: penalty contact (spring, damper, Coulomb friction) at points on the body's hull (the belly the drone rests on, the motor pods and the top), so a landing on any side is simulated. Impact is the largest contact force.
Sensors: a gyro with bias and noise and an accelerometer with noise, as specific force in the body frame.
Backends: the per-tick physics runs in C (sw/sim/rigid_step.c, compiled on first use) by default; backend="numpy" is the reference it is tested against (sw/sim/test_rigid.py). The C backend updates the state arrays in place.
"""

import ctypes
import json
import math
import os
import subprocess
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim.disturbance import CHANNELS
from sw.sim.flight import MODEL, THRUST_TABLE

G = 9.81
DT = 0.01
SUB = 4
MASS = 0.94
TABLE = np.array(THRUST_TABLE, float)
THROTTLE_MAX = 1200
SERVO_LIMIT = math.radians(60)
# Hull contact points in the body frame (m): the belly's four corners (the drone flies without its landing legs: sw/sim/sweep.py BODY_FEET), the motor pods at the arm tips, and the top of the deck.
HULL = np.array(
    [(x, y, -0.035) for x in (0.07, -0.07) for y in (0.1, -0.1)]
    + [(0.0, y, z) for y in (0.172, -0.172) for z in (0.03, -0.03)]
    + [(x, y, 0.06) for x in (0.055, -0.055) for y in (0.078, -0.078)]
)
GROUND_K, GROUND_C, FRICTION = 3000.0, 40.0, 0.6
# Below this slip speed (m/s) friction is viscous; much lower and its damping, through the small pitch inertia, reaches the substep's stability limit (rigid_step.c SLIP).
SLIP = 0.15
# Quadratic air drag (N per (m/s)²) for falls; the rotor drag at hover speeds is the calibrated linear term.
AIR_DRAG = 0.03
MODELS = Path(__file__).with_name("models")
SOURCE = Path(__file__).with_name("rigid_step.c")
LIBRARY = Path(__file__).with_name("rigid_step.so")
# The scalar parameters in rigid_step.c's Params order (airframe parameters, then the controller model's geometry, then this sim's constants, then HULL).
C_AIRFRAME = ("motor_delay", "right_motor_delay", "motor_tau", "right_motor_tau", "right_thrust_gain", "servo_delay", "right_servo_delay", "servo_wn", "right_servo_wn", "servo_zeta", "servo_slew", "link_wn", "link_zeta", "inertia_ratio", "left_play", "right_play", "left_band", "right_band", "band_stiffness", "right_gain", "roll_gyro", "roll_lift", "roll_damping", "roll_stiffness", "roll_velocity", "roll_from_yaw", "roll_prop_drag", "pitch_lift", "pitch_damping", "pitch_stiffness", "pitch_velocity", "yaw_tilt", "yaw_drag", "yaw_damping", "yaw_from_roll", "vx_pitch", "vx_tilt", "vx_drag", "vy_roll", "vy_drag")
C_MODEL = ("arm", "pivot_height", "roll_inertia", "pitch_inertia", "yaw_inertia", "reaction_torque_ratio")
loaded = None


def library():
    """rigid_step.c through ctypes, compiled next to it when missing or older than the source."""
    global loaded
    if loaded is None:
        if not LIBRARY.exists() or LIBRARY.stat().st_mtime < SOURCE.stat().st_mtime:
            # Built under a temporary name and moved into place, so parallel workers never load a half-written library.
            partial = LIBRARY.with_name(f"{LIBRARY.name}.{os.getpid()}")
            subprocess.run(["cc", "-O2", "-ffp-contract=off", "-fPIC", "-shared", str(SOURCE), "-o", str(partial), "-lm"], check=True)
            os.replace(partial, LIBRARY)
        loaded = ctypes.CDLL(str(LIBRARY))
        loaded.rigid_param_count.restype = ctypes.c_size_t
        loaded.rigid_step.argtypes = [ctypes.c_size_t, ctypes.c_size_t] + [ctypes.c_void_p] * 2 + [ctypes.c_size_t] + [ctypes.c_void_p] * 27
        loaded.rigid_step.restype = None
        assert loaded.rigid_param_count() == len(C_AIRFRAME) + len(C_MODEL) + 7 + HULL.size
    return loaded


def load_plant(airframe="airframe_2026-10-28.json", disturbances="disturbances_2026-10-28_calibrated.json"):
    """The default plant: the identified airframe, its calibrated disturbances and drag (sw/sim/sweep.py IDENTIFIED_ESC)."""
    return {"airframe": json.loads((MODELS / airframe).read_text()), "disturbances": json.loads((MODELS / disturbances).read_text()) if disturbances else [], "rotor_drag": (0.58, 0.34), "mass": MASS}


def thrust(throttle):
    return np.interp(throttle, TABLE[:, 0], TABLE[:, 1])


def throttle_for(force):
    """The throttle for a thrust (N) on the table's rising part."""
    peak = int(np.argmax(TABLE[:, 1]))
    return np.interp(force, TABLE[: peak + 1, 1], TABLE[: peak + 1, 0])


def quat_multiply(a, b):
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], -1)


def rotation(q):
    """Body-to-world rotation matrices [env, 3, 3]."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def quat_conjugate(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def rotation_vector(q):
    """The rotation q as axis times angle (rad), the short way round."""
    q = q * np.where(q[..., :1] < 0, -1.0, 1.0)
    sine = np.linalg.norm(q[..., 1:], axis=-1, keepdims=True)
    return q[..., 1:] * (2 * np.arctan2(sine, q[..., :1]) / np.maximum(sine, 1e-12))


def integrate_quat(q, omega, dt):
    """q advanced by body rates omega over dt (exact for constant omega)."""
    angle = np.linalg.norm(omega, axis=-1, keepdims=True) * dt
    axis = omega / np.maximum(np.linalg.norm(omega, axis=-1, keepdims=True), 1e-9)
    step = np.concatenate([np.cos(angle / 2), axis * np.sin(angle / 2)], -1)
    q = quat_multiply(q, step)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


class Disturbances:
    """sw/sim/disturbance.py's terms for every env at once: values [env, channel]."""

    def __init__(self, terms, n, rng, strength=None):
        self.terms, self.n, self.rng, self.time = [dict(t) for t in terms or ()], n, rng, 0.0
        self.strength = np.ones(n) if strength is None else np.asarray(strength, float)
        for t in self.terms:
            t["state"] = rng.normal(0, t["sd"], n) if t["kind"] == "drift" else np.zeros(n)

    def step(self, dt):
        self.time += dt
        out = np.zeros((self.n, len(CHANNELS)))
        for t in self.terms:
            kind = t["kind"]
            if kind == "constant":
                value = t["value"] * np.ones(self.n)
            elif kind == "drift":
                keep = math.exp(-dt / t["tau"])
                t["state"] = keep * t["state"] + t["sd"] * math.sqrt(1 - keep * keep) * self.rng.normal(0, 1, self.n)
                value = t["state"]
            elif kind == "white":
                value = t["sd"] * self.rng.normal(0, 1, self.n)
            else:
                value = t["amplitude"] * math.sin(2 * math.pi * self.time / t["period"] + t.get("phase", 0.0)) * np.ones(self.n)
            out[:, CHANNELS.index(t["channel"])] += value
        return out * self.strength[:, None]


def random_quat(rng, n):
    u = rng.random((n, 3))
    return np.stack([np.sqrt(1 - u[:, 0]) * np.sin(2 * np.pi * u[:, 1]), np.sqrt(1 - u[:, 0]) * np.cos(2 * np.pi * u[:, 1]), np.sqrt(u[:, 0]) * np.sin(2 * np.pi * u[:, 2]), np.sqrt(u[:, 0]) * np.cos(2 * np.pi * u[:, 2])], -1)


class Sim:
    """
    n drones stepped together. Controls each tick: throttle [env, 2] (0-1200, left, right) and physical tilt commands [env, 2] (radians, positive forward). Optional per-env parameter scales randomize the plant (scale dict of [env] arrays: "pitch_tilt", "roll_thrust", "inertia", "drag", "disturbance"; the airframe's dead times are shared by all envs). backend is "c" or "numpy" (the same physics).
    """

    def __init__(self, n, plant=None, seed=0, scale=None, backend="c"):
        assert backend in ("c", "numpy")
        self.backend = backend
        plant = plant or load_plant()
        self.n, self.rng = n, np.random.default_rng(seed)
        a = plant["airframe"]
        self.a = a
        self.mass = plant.get("mass", MASS)
        scale = scale or {}
        ones = np.ones(n)
        s = lambda key: np.asarray(scale.get(key, ones), float)
        m = MODEL
        self.inertia = np.stack([m["roll_inertia"] * ones, m["pitch_inertia"] * ones, m["yaw_inertia"] * ones], -1) * s("inertia")[:, None]
        self.p = A.complete(a) | {"pitch_tilt": a["pitch_tilt"] * s("pitch_tilt"), "roll_thrust": a["roll_thrust"] * s("roll_thrust")}
        self.p["right_thrust_gain"] = a["right_thrust_gain"]
        self.push = np.tile([a.get("roll_push", 0.0), a.get("pitch_push", 0.0), a.get("yaw_push", 0.0)], (n, 1))
        self.inertia_scale = s("inertia")
        # For controllers that allocate with the identified effectiveness.
        self.k_roll = self.p["roll_thrust"] * m["arm"] / m["roll_inertia"]
        self.k_pitch = self.p["pitch_tilt"] * m["pivot_height"] / m["pitch_inertia"]
        self.k_yaw = a["yaw_tilt"] * m["arm"] / m["yaw_inertia"] * ones
        self.thrust_gain = np.stack([ones, a["right_thrust_gain"] * ones], -1)
        self.rotor_drag = np.array(plant.get("rotor_drag", (0.58, 0.34))) * s("drag")[:, None]
        self.disturbances = Disturbances(plant.get("disturbances"), n, self.rng, s("disturbance"))
        self.history = 40  # control ticks of command history (0.4 s) for the dead times
        # The gyro's bias left after the firmware's at-rest calibration (rad/s): about 0.03°/s measured at rest after a 2026-10-28 flight; four times that, for drift in flight.
        self.gyro_bias = self.rng.normal(0, 0.002, (n, 3))

    def reset(self, position, velocity, quat, omega, motor=None, tilt=None):
        """motor (N) and tilt (radians, physical) start the actuators steady there (default off and level)."""
        n = self.n
        self.pos, self.vel = np.array(position, float), np.array(velocity, float)
        self.quat, self.omega = np.array(quat, float), np.array(omega, float)
        self.state = A.State(n)
        self.state.motor[:] = 0.0 if motor is None else np.array(motor, float)
        tilt_deg = np.zeros((n, 2)) if tilt is None else np.degrees(np.array(tilt, float))
        self.state.horn[:] = self.state.tilt[:] = tilt_deg
        self.throttles = np.zeros((n, self.history, 2)) if motor is None else np.repeat(throttle_for(self.state.motor / self.thrust_gain)[:, None], self.history, 1)
        self.commands = np.repeat(tilt_deg[:, None], self.history, 1)
        self.time = 0.0
        self.contact = np.zeros(n)  # largest contact force so far (N)
        self.touched = np.zeros(n, bool)
        self.accel = np.tile([0.0, 0.0, G], (n, 1))  # specific force, body frame (m/s²)
        self.contact_now = np.zeros(n)
        self.turned = self.omega.copy()
        if self.backend == "c":
            self.bind()
        return self.observe()

    def bind(self):
        """The C backend's arguments: the parameters and pointers to every (contiguous, float64) array it reads or updates in place."""
        p, st = self.p, self.state
        for name in ("rate", "att", "vel", "motor", "horn", "hvel", "tilt", "tvel"):
            setattr(st, name, np.ascontiguousarray(getattr(st, name), float))
        for name in ("throttles", "commands", "pos", "vel", "quat", "omega", "accel", "contact", "contact_now", "pitch_tilt", "roll_thrust", "inertia", "inertia_scale", "rotor_drag", "push"):
            if name in ("pitch_tilt", "roll_thrust"):
                p[name] = np.ascontiguousarray(np.broadcast_to(p[name], (self.n,)), float)
            else:
                setattr(self, name, np.ascontiguousarray(getattr(self, name), float))
        self.touched = np.ascontiguousarray(self.touched, bool)
        scalars = [p[key] for key in C_AIRFRAME] + [MODEL[key] for key in C_MODEL] + [self.mass, THROTTLE_MAX, SERVO_LIMIT, GROUND_K, GROUND_C, FRICTION, AIR_DRAG]
        self.c_params = np.concatenate([np.array(scalars, float), HULL.ravel()])
        self.c_table = np.ascontiguousarray(TABLE)
        self.c_disturbance = np.zeros((self.n, 6))
        self.c_throttle, self.c_tilt = np.zeros((self.n, 2)), np.zeros((self.n, 2))
        address = lambda array: array.ctypes.data
        self.c_args = (
            [self.n, self.history, address(self.c_params), address(self.c_table), len(self.c_table)]
            + [address(a) for a in (p["pitch_tilt"], p["roll_thrust"], self.inertia, self.inertia_scale, self.rotor_drag, self.push, self.c_throttle, self.c_tilt, self.c_disturbance)]
            + [address(a) for a in (self.throttles, self.commands, self.pos, self.vel, self.quat, self.omega, st.rate, st.att, st.vel, st.motor, st.horn, st.hvel, st.tilt, st.tvel, self.accel, self.contact, self.contact_now, self.touched)]
        )
        self.c_bound = (self.pos, self.vel, self.quat, self.omega, self.state.motor, self.state.tilt, self.touched)

    def delayed(self, buffer, delay):
        x = (self.history - 1) - delay / DT
        low = np.clip(np.floor(x).astype(int), 0, self.history - 1)
        high = np.clip(low + 1, 0, self.history - 1)
        f = np.clip(x - low, 0, 1)
        lane = np.arange(self.n)[:, None]
        side = np.array([[0, 1]])
        return buffer[lane, low, side] * (1 - f) + buffer[lane, high, side] * f

    def step(self, throttle, tilt_command):
        """Advance one control tick (DT) with these commands; returns the observation."""
        before = self.quat.copy()
        if self.backend == "c":
            self.step_c(throttle, tilt_command)
        else:
            self.step_numpy(throttle, tilt_command)
        self.turned = rotation_vector(quat_multiply(quat_conjugate(before), self.quat)) / DT
        return self.observe()

    def step_numpy(self, throttle, tilt_command):
        self.throttles = np.roll(self.throttles, -1, 1)
        self.commands = np.roll(self.commands, -1, 1)
        self.throttles[:, -1] = np.clip(throttle, 0, THROTTLE_MAX)
        self.commands[:, -1] = np.degrees(np.clip(tilt_command, -SERVO_LIMIT, SERVO_LIMIT))
        sampled = self.disturbances.step(DT)
        force_disturbance, torque_disturbance = sampled[:, :3], sampled[:, 3:]
        # The identified airframe's actuators and angular accelerations for this tick, from the body's rates, tilt and speed now.
        R0 = rotation(self.quat)
        up = R0[:, 2, :]  # world up in the body frame
        st = self.state
        st.rate = self.omega.copy()
        st.att = np.clip(np.column_stack([np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))]), -0.6, 0.6)
        st.vel = np.einsum("nji,nj->ni", R0, self.vel)[:, :2]
        # The airframe's in-flight terms (attitude and speed moments, pushes) come with the rotors' thrust: off on the ground, and fading as the motors spin down, so a drone that has cut its motors is not pushed over.
        airborne = (self.contact_now == 0) * np.clip((self.state.motor[:, 0] + self.state.motor[:, 1]) / (self.mass * G), 0, 1)
        self.state, alpha_model = A.step(self.p, st, self.throttles, self.commands, np.full(self.n, self.history - 1), np.ones(self.n), self.push, airborne)
        alpha_model = alpha_model / self.inertia_scale[:, None]
        wash = np.clip((self.state.motor[:, 0] + self.state.motor[:, 1]) / (self.mass * G), 0, 1)
        force_disturbance, torque_disturbance = force_disturbance * wash[:, None], torque_disturbance * wash[:, None]
        tilt = self.state.tilt * np.array([1.0, self.p["right_gain"]]) * math.pi / 180
        left, right = self.state.motor[:, 0], self.state.motor[:, 1]
        sl, sr, cl, cr = np.sin(tilt[:, 0]), np.sin(tilt[:, 1]), np.cos(tilt[:, 0]), np.cos(tilt[:, 1])
        h = DT / SUB
        peak = np.zeros(self.n)
        for _ in range(SUB):
            alpha = alpha_model + torque_disturbance
            I = self.inertia
            gyroscopic = -np.cross(self.omega, I * self.omega) / I
            R = rotation(self.quat)
            # Ground contact at the hull points.
            points = np.einsum("nij,pj->npi", R, HULL) + self.pos[:, None]
            point_vel = self.vel[:, None] + np.einsum("nij,npj->npi", R, np.cross(self.omega[:, None], HULL[None]))
            depth = -points[..., 2]
            touching = depth > 0
            normal = np.where(touching, np.maximum(0.0, GROUND_K * depth - GROUND_C * point_vel[..., 2]), 0.0)
            slip = np.linalg.norm(point_vel[..., :2], axis=-1)
            friction = -FRICTION * normal[..., None] * point_vel[..., :2] / np.maximum(slip, SLIP)[..., None]
            contact_world = np.concatenate([friction, normal[..., None]], -1)
            contact_force = contact_world.sum(1)
            contact_torque_world = np.cross(points - self.pos[:, None], contact_world).sum(1)
            contact_alpha = np.einsum("nji,nj->ni", R, contact_torque_world) / I
            peak = np.maximum(peak, np.linalg.norm(contact_world, axis=-1).sum(1))
            self.touched |= touching.any(1)
            # Thrust, drag and gravity.
            thrust_body = np.stack([left * sl + right * sr, np.zeros(self.n), left * cl + right * cr], -1)
            v_body = np.einsum("nji,nj->ni", R, self.vel)
            drag_body = -np.concatenate([self.rotor_drag * v_body[:, :2] * self.mass, np.zeros((self.n, 1))], -1)
            force_world = np.einsum("nij,nj->ni", R, thrust_body + drag_body) - AIR_DRAG * np.linalg.norm(self.vel, axis=-1, keepdims=True) * self.vel + contact_force
            accel_world = force_world / self.mass + force_disturbance - np.array([0, 0, G])
            self.accel = np.einsum("nji,nj->ni", R, accel_world + np.array([0, 0, G]))
            self.vel = self.vel + accel_world * h
            self.pos = self.pos + self.vel * h
            self.omega = self.omega + (alpha + gyroscopic + contact_alpha) * h
            self.quat = integrate_quat(self.quat, self.omega, h)
        self.contact = np.maximum(self.contact, peak)
        self.contact_now = peak
        self.time += DT

    def step_c(self, throttle, tilt_command):
        """step through rigid_step.c, the disturbances still sampled here (the same random stream as the numpy backend)."""
        assert all(a is b for a, b in zip(self.c_bound, (self.pos, self.vel, self.quat, self.omega, self.state.motor, self.state.tilt, self.touched))), "the C backend's arrays were replaced; update them in place"
        self.c_throttle[:] = throttle
        self.c_tilt[:] = tilt_command
        self.c_disturbance[:] = self.disturbances.step(DT)
        library().rigid_step(*self.c_args)
        self.time += DT

    @property
    def motor(self):
        """Each motor's thrust now (N)."""
        return self.state.motor

    @property
    def tilt(self):
        """Each assembly's physical tilt now (radians, positive forward)."""
        return self.state.tilt * np.array([1.0, self.p["right_gain"]]) * math.pi / 180

    def observe(self):
        """What the drone senses: gyro (rad/s, with bias and noise; the tick's whole rotation over DT, as the firmware's estimator integrating the gyro at 1 kHz sees it, rather than the rate at the tick's end, which over a fast spin-up would cost degrees a tick) and accelerometer (m/s², specific force in the body frame, with noise)."""
        return {"gyro": self.turned + self.gyro_bias + self.rng.normal(0, 0.02, (self.n, 3)), "accel": self.accel + self.rng.normal(0, 0.3, (self.n, 3))}

    def up(self):
        """The body's z axis in the world frame [env, 3]: (0, 0, 1) is upright."""
        return rotation(self.quat)[:, :, 2]
