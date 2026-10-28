"""
Recovery from a fall in any orientation, in the full-attitude simulation (sw/sim/rigid.py): uv run -m sw.sim.recover [--params params.json] [--n 512] [--seed 0]

The controller sees only what the drone senses: it integrates the gyro into a quaternion and the accelerometer, turned to the world, into a velocity, both from the estimate it had when control was lost (the truth plus a little error); no camera and no height. Once upright, while the specific force is close to 1 g, the accelerometer's direction pulls the estimated up back over level_tau s (as the firmware's attitude filter does), so gyro bias cannot tilt the velocity estimate away.
  right    while the thrust axis points more than right_angle from up: aim it up with little collective (inverted, thrust would only push down), rolling with differential thrust and pitching with the motors' common tilt
  arrest   aim the thrust to stop the vertical and horizontal velocity, then descend at descent m/s
  landed   a touchdown (specific force touch_g beyond what the commanded thrust explains, or the descent stalled with thrust low) stops the motors
Each tick turns the attitude error into a wanted rotation rate toward the target, no faster than the drone can brake from (sqrt(2 brake angle), at most max_rate, and kp per rad near the target), then into wanted angular accelerations (k_rate per axis) and allocates them to the two thrusts and two tilts with the identified effectiveness (sw/sim/airframe.py), keeping roll authority when a thrust would saturate. The motors take about 0.2 s to follow (dead time and lag), so the tilts that turn thrust into pitch and yaw are worked out from the thrust a motor model says the motors have now, not from what was just asked.
A fall counts as a safe landing when the drone first touches with its thrust axis within 30° of up, below 1.5 m/s down and under 60 N of contact force, and ends upright.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sw.sim import rigid as R

DEFAULTS = {
    "kp": 6.0, "brake": 15.0, "max_rate": 5.0, "k_roll": 8.0, "k_pitch": 12.0, "kd_yaw": 4.0,
    "kv": 3.0, "kh": 1.0, "descent": 0.5, "flip_thrust": 0.35, "right_angle": 45.0, "tilt_max": 15.0,
    "touch_g": 1.0, "engage": 0.15, "servo_max": 45.0, "level_tau": 1.0,
}
RIGHT, ARREST, LANDED = 0, 1, 2


class Controller:
    def __init__(self, sim, params):
        self.sim, self.p = sim, DEFAULTS | (params or {})
        n = sim.n
        self.phase = np.zeros(n, int)
        self.low_time = np.zeros(n)

    def reset(self, quat_estimate, velocity_estimate):
        self.q = np.array(quat_estimate, float)
        self.v = np.array(velocity_estimate, float)
        self.collective = np.zeros(self.sim.n)
        a = self.sim.a
        self.delay = round((a["motor_delay"] + a["right_motor_delay"]) / 2 / R.DT)
        self.tau = (a["motor_tau"] + a["right_motor_tau"]) / 2
        self.asked = np.zeros((self.sim.n, self.delay + 1, 2))
        self.thrust = np.zeros((self.sim.n, 2))
        self.phase[:] = RIGHT
        self.low_time[:] = 0

    def __call__(self, obs, time):
        sim, p, n = self.sim, self.p, self.sim.n
        # Tilt correction from gravity: rotate the estimate so the measured specific force points along estimated up, slowly, when it is near 1 g and the drone is upright.
        accel = obs["accel"]
        size = np.linalg.norm(accel, axis=1)
        trust = (np.abs(size - R.G) < 0.15 * R.G) & (self.phase == ARREST)
        measured_up = accel / np.maximum(size, 1e-6)[:, None]
        estimated_up = R.rotation(self.q)[:, 2, :]
        correction = np.cross(measured_up, estimated_up) * (R.DT / p["level_tau"])
        rates = obs["gyro"] + np.where(trust[:, None], correction / R.DT, 0.0)
        self.q = R.integrate_quat(self.q, rates, R.DT)
        Rq = R.rotation(self.q)
        self.v += (np.einsum("nij,nj->ni", Rq, obs["accel"]) - np.array([0, 0, R.G])) * R.DT
        up_body = Rq[:, 2, :]  # world up in the body frame
        upright = up_body[:, 2]
        # Wanted thrust direction (world): straight up while righting; afterwards to stop the fall and drift.
        a_xy = np.clip(-p["kh"] * self.v[:, :2], -R.G * math.tan(math.radians(p["tilt_max"])), R.G * math.tan(math.radians(p["tilt_max"])))
        a_z = -p["kv"] * (self.v[:, 2] + p["descent"])
        arresting = self.phase == ARREST
        wanted = np.where(arresting[:, None], np.column_stack([a_xy, R.G + a_z]), np.array([0, 0, 1.0]))
        wanted /= np.linalg.norm(wanted, axis=1, keepdims=True)
        d_body = np.einsum("nji,nj->ni", Rq, wanted)
        axis = np.cross(np.array([0, 0, 1.0]), d_body)
        sin_angle = np.linalg.norm(axis, axis=1)
        angle = np.arctan2(sin_angle, d_body[:, 2])
        # Exactly inverted: any axis will do; roll is the stronger one.
        axis = np.where(sin_angle[:, None] > 1e-6, axis / np.maximum(sin_angle, 1e-9)[:, None], np.array([1.0, 0, 0]))
        omega = obs["gyro"]
        speed = np.minimum.reduce([np.full(n, p["max_rate"]), np.sqrt(2 * p["brake"] * angle), p["kp"] * angle])
        wanted_rate = axis * speed[:, None]
        alpha = np.column_stack([p["k_roll"] * (wanted_rate[:, 0] - omega[:, 0]), p["k_pitch"] * (wanted_rate[:, 1] - omega[:, 1]), -p["kd_yaw"] * omega[:, 2]])
        weight = sim.mass * R.G
        collective = np.where(arresting, sim.mass * (R.G + a_z) / np.maximum(upright, 0.5), p["flip_thrust"] * weight)
        collective = np.clip(collective, 0.15 * weight, 2.2 * weight)
        self.phase = np.where((self.phase == RIGHT) & (upright > math.cos(math.radians(p["right_angle"]))), ARREST, self.phase)
        # Allocation: roll by differential thrust, pitch by common tilt, yaw by differential tilt.
        diff = alpha[:, 0] / sim.k_roll
        tmax = R.thrust(R.THROTTLE_MAX)
        total = np.clip(collective, np.abs(diff), 2 * tmax - np.abs(diff))
        left, right = (total + diff) / 2, (total - diff) / 2
        # The thrust the motors have now, by the motor model.
        self.thrust += (self.asked[:, 0] - self.thrust) * (1 - math.exp(-R.DT / self.tau))
        have = np.maximum(self.thrust.sum(1), 0.5)
        limit = math.sin(math.radians(p["servo_max"]))
        common = np.arcsin(np.clip(alpha[:, 1] / (sim.k_pitch * have), -limit, limit))
        split = np.clip(alpha[:, 2] / (sim.k_yaw * have * np.cos(common)), -0.3, 0.3)
        tilt = np.column_stack([common - split, common + split])
        throttle = np.column_stack([R.throttle_for(left / sim.thrust_gain[:, 0]), R.throttle_for(right / sim.thrust_gain[:, 1])])
        # Touchdown: a hit, or a descent that has stalled with the thrust low.
        hit = np.linalg.norm(obs["accel"], axis=1) - self.thrust.sum(1) / sim.mass > p["touch_g"] * R.G
        self.low_time = np.where(arresting & (np.abs(self.v[:, 2]) < 0.1) & (collective < 0.9 * weight), self.low_time + R.DT, 0)
        self.phase = np.where(arresting & (time > p["engage"] + 0.5) & (hit | (self.low_time > 0.3)), LANDED, self.phase)
        self.collective = collective
        off = (self.phase == LANDED) | (time < p["engage"])
        self.asked = np.roll(self.asked, -1, 1)
        self.asked[:, -1] = np.where(off[:, None], 0.0, np.column_stack([left, right]))
        throttle[off] = 0
        tilt[off] = 0
        return throttle, tilt


def scenarios(n, seed, height=(2.0, 5.0)):
    rng = np.random.default_rng(seed)
    position = np.column_stack([np.zeros(n), np.zeros(n), rng.uniform(*height, n)])
    velocity = rng.normal(0, 1.0, (n, 3))
    quat = R.random_quat(rng, n)
    omega = rng.normal(0, 3.0, (n, 3))
    return position, velocity, quat, omega


def randomization(n, seed, spread=1.0):
    """Per-env plant variations: what the identification cannot pin down, for robustness."""
    rng = np.random.default_rng(seed + 99)
    u = lambda lo, hi: np.exp(rng.uniform(math.log(1 + (lo - 1) * spread), math.log(1 + (hi - 1) * spread), n))
    return {"pitch_tilt": u(0.7, 1.5), "roll_thrust": u(0.8, 1.25), "thrust": u(0.9, 1.1), "motor_delay": u(0.8, 1.4), "servo_delay": u(0.8, 1.4), "inertia": u(0.85, 1.2), "drag": u(0.6, 1.6), "disturbance": u(0.5, 2.0)}


def run(params, n=256, seed=0, seconds=8.0, randomize=True, record=False):
    sim = R.Sim(n, seed=seed, scale=randomization(n, seed) if randomize else None)
    position, velocity, quat, omega = scenarios(n, seed)
    obs = sim.reset(position, velocity, quat, omega)
    control = Controller(sim, params)
    rng = np.random.default_rng(seed + 7)
    control.reset(R.quat_multiply(quat, np.column_stack([np.ones(n), rng.normal(0, 0.02, (n, 3))])), velocity + rng.normal(0, 0.2, (n, 3)))
    first_tilt = np.full(n, np.nan)
    first_speed = np.full(n, np.nan)
    trace = []
    for k in range(int(seconds / R.DT)):
        throttle, tilt = control(obs, sim.time)
        before = sim.touched.copy()
        vz = sim.vel[:, 2].copy()
        obs = sim.step(throttle, tilt)
        new = sim.touched & ~before
        up = sim.up()
        first_tilt[new] = np.degrees(np.arccos(np.clip(up[new, 2], -1, 1)))
        first_speed[new] = -vz[new]
        if record:
            trace.append((sim.time, sim.pos.copy(), sim.quat.copy(), sim.motor.copy(), sim.tilt.copy(), control.phase.copy()))
        if (control.phase == LANDED).all() and sim.touched.all():
            break
    final_up = sim.up()[:, 2]
    safe = (first_tilt < 30) & (first_speed < 1.5) & (sim.contact < 60) & (final_up > 0.8)
    result = {"safe": safe, "first_tilt": first_tilt, "first_speed": first_speed, "contact": sim.contact, "final_up": final_up, "landed": control.phase == LANDED}
    return (result, trace) if record else result


def summary(result):
    safe = result["safe"]
    touched = np.isfinite(result["first_tilt"])
    return (f"safe {safe.mean() * 100:.1f}% of {len(safe)} | touched {touched.mean() * 100:.0f}%, tilt at touch median {np.nanmedian(result['first_tilt']):.0f}° (90th {np.nanpercentile(result['first_tilt'], 90):.0f}°), "
            f"speed median {np.nanmedian(result['first_speed']):.2f} m/s (90th {np.nanpercentile(result['first_speed'], 90):.2f}), contact force median {np.median(result['contact']):.0f} N, ends upright {(result['final_up'] > 0.8).mean() * 100:.0f}%")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path)
    parser.add_argument("--n", type=int, default=512)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--nominal", action="store_true", help="no plant randomization")
    args = parser.parse_args()
    params = json.loads(args.params.read_text()) if args.params else {}
    print(summary(run(params, args.n, args.seed, randomize=not args.nominal)))


if __name__ == "__main__":
    main()
