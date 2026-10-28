"""
A vectorized full-attitude simulation of the bicopter for recovery control and policy training: many drones at once (numpy, [env, ...] arrays), any orientation (quaternions), any rates, with the identified actuators and the calibrated disturbances.

Frames: world x forward, y left, z up; body x forward, y left, z up through the rotors' shafts at zero tilt. Quaternions are (w, x, y, z), body to world.
Actuators (sw/sim/airframe.py parameters, the post-ESC-recalibration fit by default): each motor's thrust follows its throttle through a dead time and a first-order lag on the firmware's thrust table; each servo follows its tilt command through a dead time, a second-order loop and a slew limit, with the assembly at the horn. Motors cannot reverse, and servos stop at ±60°.
Body: the thrust torques at the controller model's geometry times the identified effectiveness, the identified rate damping and steady pushes, the full rigid-body (ω × Iω) coupling, and sw/sim/disturbance.py terms. Translation: the thrust vector, gravity, the calibrated rotor drag in the body plane and quadratic air drag.
Ground: penalty contact (spring, damper, Coulomb friction) at points on the body's hull (the belly the drone rests on, the motor pods and the top), so a landing on any side is simulated. Impact is the largest contact force.
Sensors: a gyro with bias and noise and an accelerometer with noise, as specific force in the body frame.
"""

import json
import math
from pathlib import Path

import numpy as np

from sw.sim.disturbance import CHANNELS
from sw.sim.flight import MODEL, THRUST_TABLE

G = 9.81
DT = 0.01
SUB = 4
MASS = 0.94
TABLE = np.array(THRUST_TABLE, float)
THROTTLE_MAX = 1200
SERVO_LIMIT = math.radians(60)
# Hull contact points in the body frame (m): the belly's four corners (sw/sim/sweep.py FEET), the motor pods at the arm tips, and the top of the deck.
HULL = np.array(
    [(x, y, -0.087 - 0.0172) for x in (0.055, -0.055) for y in (0.078, -0.078)]
    + [(0.0, y, z) for y in (0.172, -0.172) for z in (0.03, -0.03)]
    + [(x, y, 0.06) for x in (0.055, -0.055) for y in (0.078, -0.078)]
)
GROUND_K, GROUND_C, FRICTION = 3000.0, 40.0, 0.6
# Quadratic air drag (N per (m/s)²) for falls; the rotor drag at hover speeds is the calibrated linear term.
AIR_DRAG = 0.03
MODELS = Path(__file__).with_name("models")


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
    n drones stepped together. Controls each tick: throttle [env, 2] (0-1200, left, right) and physical tilt commands [env, 2] (radians, positive forward). Optional per-env parameter scales randomize the plant (scale dict of [env] arrays: "pitch_tilt", "roll_thrust", "thrust", "motor_delay", "servo_delay", "inertia", "drag", "disturbance").
    """

    def __init__(self, n, plant=None, seed=0, scale=None):
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
        self.k_roll = a["roll_thrust"] * m["arm"] / m["roll_inertia"] * s("roll_thrust")
        self.k_pitch = a["pitch_tilt"] * m["pivot_height"] / m["pitch_inertia"] * s("pitch_tilt")
        self.k_yaw = a["yaw_tilt"] * m["arm"] / m["yaw_inertia"] * ones
        self.k_yaw_drag = a["yaw_drag"] / m["yaw_inertia"] * ones
        self.damping = np.array([a["roll_damping"], a["pitch_damping"], a["yaw_damping"]])
        self.push = np.array([a.get("roll_push", 0.0), a.get("pitch_push", 0.0), a.get("yaw_push", 0.0)])
        self.thrust_gain = np.stack([ones, a["right_thrust_gain"] * ones], -1) * s("thrust")[:, None]
        self.motor_delay = np.stack([a["motor_delay"] * ones, a["right_motor_delay"] * ones], -1) * s("motor_delay")[:, None]
        self.motor_tau = np.array([a["motor_tau"], a["right_motor_tau"]])
        self.servo_delay = np.stack([a["servo_delay"] * ones, a["right_servo_delay"] * ones], -1) * s("servo_delay")[:, None]
        self.servo_wn = np.array([a["servo_wn"], a["right_servo_wn"]])
        self.servo_zeta, self.servo_slew = a["servo_zeta"], math.radians(a["servo_slew"])
        self.rotor_drag = np.array(plant.get("rotor_drag", (0.58, 0.34))) * s("drag")[:, None]
        self.disturbances = Disturbances(plant.get("disturbances"), n, self.rng, s("disturbance"))
        self.history = 40  # control ticks of command history (0.4 s) for the dead times
        self.gyro_bias = self.rng.normal(0, 0.01, (n, 3))

    def reset(self, position, velocity, quat, omega, motor=None, tilt=None):
        n = self.n
        self.pos, self.vel = np.array(position, float), np.array(velocity, float)
        self.quat, self.omega = np.array(quat, float), np.array(omega, float)
        self.motor = np.zeros((n, 2)) if motor is None else np.array(motor, float)
        self.tilt = np.zeros((n, 2)) if tilt is None else np.array(tilt, float)
        self.tilt_rate = np.zeros((n, 2))
        self.throttles = np.zeros((n, self.history, 2)) if motor is None else np.repeat(throttle_for(self.motor / self.thrust_gain)[:, None], self.history, 1)
        self.commands = np.repeat(self.tilt[:, None], self.history, 1)
        self.time = 0.0
        self.contact = np.zeros(n)  # largest contact force so far (N)
        self.touched = np.zeros(n, bool)
        self.accel = np.tile([0.0, 0.0, G], (n, 1))  # specific force, body frame (m/s²)
        return self.observe()

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
        self.throttles = np.roll(self.throttles, -1, 1)
        self.commands = np.roll(self.commands, -1, 1)
        self.throttles[:, -1] = np.clip(throttle, 0, THROTTLE_MAX)
        self.commands[:, -1] = np.clip(tilt_command, -SERVO_LIMIT, SERVO_LIMIT)
        target = thrust(self.delayed(self.throttles, self.motor_delay)) * self.thrust_gain
        goal = self.delayed(self.commands, self.servo_delay)
        sampled = self.disturbances.step(DT)
        force_disturbance, torque_disturbance = sampled[:, :3], sampled[:, 3:]
        h = DT / SUB
        peak = np.zeros(self.n)
        for _ in range(SUB):
            self.motor = target + (self.motor - target) * np.exp(-h / self.motor_tau)
            accel_servo = self.servo_wn ** 2 * (goal - self.tilt) - 2 * self.servo_zeta * self.servo_wn * self.tilt_rate
            self.tilt_rate = np.clip(self.tilt_rate + accel_servo * h, -self.servo_slew, self.servo_slew)
            self.tilt = np.clip(self.tilt + self.tilt_rate * h, -SERVO_LIMIT, SERVO_LIMIT)
            left, right = self.motor[:, 0], self.motor[:, 1]
            sl, sr, cl, cr = np.sin(self.tilt[:, 0]), np.sin(self.tilt[:, 1]), np.cos(self.tilt[:, 0]), np.cos(self.tilt[:, 1])
            alpha = np.stack([
                self.k_roll * (left * cl - right * cr),
                self.k_pitch * (left * sl + right * sr),
                self.k_yaw * (right * sr - left * sl) + self.k_yaw_drag * (right * cr - left * cl),
            ], -1) + self.damping * self.omega + self.push + torque_disturbance
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
            friction = -FRICTION * normal[..., None] * point_vel[..., :2] / np.maximum(slip, 0.05)[..., None]
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
            accel_world = force_world / self.mass + force_disturbance * (self.pos[:, 2:3] > 0.02) - np.array([0, 0, G])
            self.accel = np.einsum("nji,nj->ni", R, accel_world + np.array([0, 0, G]))
            self.vel = self.vel + accel_world * h
            self.pos = self.pos + self.vel * h
            self.omega = self.omega + (alpha + gyroscopic + contact_alpha) * h
            self.quat = integrate_quat(self.quat, self.omega, h)
        self.contact = np.maximum(self.contact, peak)
        self.time += DT
        return self.observe()

    def observe(self):
        """What the drone senses: gyro (rad/s, with bias and noise) and accelerometer (m/s², specific force in the body frame, with noise)."""
        return {"gyro": self.omega + self.gyro_bias + self.rng.normal(0, 0.02, (self.n, 3)), "accel": self.accel + self.rng.normal(0, 0.3, (self.n, 3))}

    def up(self):
        """The body's z axis in the world frame [env, 3]: (0, 0, 1) is upright."""
        return rotation(self.quat)[:, :, 2]
