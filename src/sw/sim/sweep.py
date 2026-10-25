"""
Gain sweeps: uv run -m sw.sim.sweep. Runs the drone's C attitude estimator and controller (via sw.sim.flight) against a lean 6-DOF rigid-body plant with actuator delay, lag, servo backlash/slew, IMU noise, drag, landing-leg ground contact and model mismatch. Free flight starts at hover in the air; a plant with "liftoff" starts on the ground at the dashboard's T collective instead, level or resting at "ground_pitch" degrees on its "feet" (contact points relative to the CoM, default FEET).
"""

import itertools
import math
import random
from collections import deque

from sw.sim.flight import MODEL, Attitude, Controller, throttle, thrust

MASS = 0.940  # kg, the final assembly with the printed deck and IMU bracket (Bill's estimate, 2026-10-24)
WEIGHT = MASS * 9.81
DT = 0.01
SUB = 10
TAKEOFF_THROTTLE = 720  # dashboard T
CLIMB = 0.5  # seconds at T after liftoff before collective drops to hover
# Landing-leg contact points relative to the CoM, body frame (m): CAD legs touch at x ±55 mm, y ±78 mm, 87 mm below the tube; the CoM is 17.2 mm above it.
FEET = [(x, y, -0.087 - 0.0172) for x in (0.055, -0.055) for y in (0.078, -0.078)]
GROUND_K = 2000.0  # N/m per foot, about 1 mm of static sag
GROUND_C = 30.0  # N·s/m per foot, near critical
FRICTION = 0.5
SLIP = 0.05  # m/s; friction is viscous below this so the integration stays stable
DRAG = 0.03  # N/(m/s)², rough bluff-body estimate
ACCEL_NOISE = 0.0004 * math.sqrt(44)  # g RMS: MPU6050 noise density × DLPF bandwidth


def rotation(roll, pitch, yaw):
    """
    Body-to-world matrix (Z-Y-X). Positive roll lifts the left side; positive pitch is nose-down.
    """
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def apply(matrix, vector, transpose=False):
    if transpose:
        return [sum(matrix[j][i] * vector[j] for j in range(3)) for i in range(3)]
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def run(gains, plant, seconds=10.0, seed=0, log=None):
    """
    Simulate one flight. A log list receives (time, position, roll/pitch/yaw, estimated roll/pitch, servo tilts, motor thrusts) every control step.
    """
    rng = random.Random(seed)
    accel_rng = random.Random(seed + 1000)
    settings = dict(MODEL) | {"roll": 0, "pitch": 0, "yaw_rate": 0, "pitch_trim": 0, "left_trim": 0, "right_trim": 0,
                               "roll_bias": 0, "pitch_bias": 0, "yaw_bias": 0, "left_thrust_scale": 1, "right_thrust_scale": 1,
                               "reaction_torque_ratio": 0.015, "servo_compensation": 0.75, "integrate_throttle": 580, "target_rate": 360, "velocity_damping": 8, "velocity_leak": 3} | gains
    k = plant["thrust_k"]
    hover = round(throttle(WEIGHT / 2 / k))
    liftoff = plant.get("liftoff", False)
    settings["base_throttle"] = TAKEOFF_THROTTLE if liftoff else hover
    controller = Controller()
    controller.configure(settings | {"enabled": 1})
    controller.reset()
    # Constant pitch angular acceleration not in the controller's model (servo zero or CoM error), rad/s², positive nose-down.
    disturbance = plant.get("disturbance", 0.0)
    lifted = None
    inertia = [MODEL["roll_inertia"] * plant["roll_inertia"], MODEL["pitch_inertia"] * plant["pitch_inertia"], MODEL["yaw_inertia"] * plant["yaw_inertia"]]
    height = MODEL["pivot_height"] * plant["height"]
    arm = MODEL["arm"]
    feet = plant.get("feet", FEET)
    angles = [0.0, math.radians(plant.get("ground_pitch", 0.0)), 0.0] if liftoff else [math.radians(plant.get("roll0", 3)), math.radians(plant.get("pitch0", -3)), 0.0]
    # World position and velocity of the CoM; on the ground the lowest feet start at their static sag. Free flight starts high enough that sinking without altitude hold never reaches the floor.
    lowest = min(apply(rotation(*angles), foot)[2] for foot in feet)
    position = [0.0, 0.0, -lowest - WEIGHT / len(feet) / GROUND_K if liftoff else 50.0]
    velocity = [0.0, 0.0, 0.0]
    rate = [0.0, 0.0, 0.0]
    motor = [0.0, 0.0] if liftoff else [WEIGHT / 2, WEIGHT / 2]
    servo = [0.0, 0.0]  # physical tilt, rad, positive forward
    output = [0.0, 0.0]  # servo horn position before backlash
    motor_queue = deque([(0 if liftoff else hover,) * 2] * max(1, round(plant["motor_delay"] / DT)))
    servo_queue = deque([(0.0, 0.0)] * max(1, round(plant["servo_delay"] / DT)))
    slack = math.radians(plant["backlash"])
    slew = math.radians(plant["servo_slew"]) * DT / SUB
    scales = [k * (1 - plant["imbalance"] / 2), k * (1 + plant["imbalance"] / 2)]
    worst, peak_height, late, trace = 0.0, 0.0, [], []
    # The estimator starts from the true gravity direction, like the drone after its stationary calibration.
    specific = apply(rotation(*angles), [0.0, 0.0, 1.0], transpose=True)
    estimator = Attitude(accel=specific)
    gyro_filter = [0.0, 0.0, 0.0]
    accel_filter = list(specific)
    # The servo electronics ignore a command until it differs from the last accepted one by more than the deadband.
    accepted = [0.0, 0.0]
    deadband = math.radians(plant["deadband"])
    # Seconds for collective to rise linearly to T on the ground (0 = step).
    ramp = plant.get("ramp", 0.0)
    for step in range(int(seconds / DT)):
        if liftoff and lifted is None and step * DT < ramp:
            settings["base_throttle"] = round(TAKEOFF_THROTTLE * (step + 1) * DT / ramp)
            controller.configure(settings | {"enabled": 1})
        if lifted is not None and step * DT - lifted >= CLIMB and settings["base_throttle"] != hover:
            settings["base_throttle"] = hover
            controller.configure(settings | {"enabled": 1})
        noise = [rng.gauss(0, math.radians(plant["gyro_noise"])) for _ in range(3)]
        gyro_filter = [g + 0.7 * (r + n - g) for g, r, n in zip(gyro_filter, rate, noise)]
        accel_filter = [a + 0.7 * (s + accel_rng.gauss(0, ACCEL_NOISE) - a) for a, s in zip(accel_filter, specific)]
        attitude = estimator.update(accel_filter, gyro_filter, DT, powered=True)
        left, right, left_angle, right_angle = controller.update(attitude, gyro_filter, DT)
        motor_queue.append((left, right))
        servo_queue.append((left_angle * MODEL["left_servo_direction"], right_angle * MODEL["right_servo_direction"]))
        target_throttle = motor_queue.popleft()
        for side, angle in enumerate(servo_queue.popleft()):
            if abs(angle - accepted[side]) > deadband:
                accepted[side] = angle
        target_servo = accepted
        for _ in range(SUB):
            h = DT / SUB
            for side in range(2):
                force = thrust(target_throttle[side]) * scales[side]
                motor[side] += (force - motor[side]) * (1 - math.exp(-h / plant["motor_tau"]))
                horn = output[side] + max(-slew, min(slew, (target_servo[side] - output[side]) * (1 - math.exp(-h / plant["servo_tau"]))))
                output[side] = horn
                if horn - servo[side] > slack:
                    servo[side] = horn - slack
                elif servo[side] - horn > slack:
                    servo[side] = horn + slack
            vertical = [motor[s] * math.cos(servo[s]) for s in range(2)]
            forward = [motor[s] * math.sin(servo[s]) for s in range(2)]
            torque = [
                arm * (vertical[0] - vertical[1]),
                height * (forward[0] + forward[1]) + plant["com_x"] * (vertical[0] + vertical[1]) + disturbance * inertia[1],
                arm * (forward[1] - forward[0]) + 0.015 * (vertical[1] - vertical[0]),
            ]
            matrix = rotation(*angles)
            speed = math.sqrt(sum(v * v for v in velocity))
            # Every force except gravity, in the world frame: thrust, drag, then each foot below the floor.
            push = [f - DRAG * speed * v for f, v in zip(apply(matrix, [sum(forward), 0.0, sum(vertical)]), velocity)]
            spin = apply(matrix, rate)
            touching = False
            for foot in feet:
                lever = apply(matrix, foot)
                depth = position[2] + lever[2]
                if depth >= 0:
                    continue
                touching = True
                contact = [v + w for v, w in zip(velocity, cross(spin, lever))]
                normal = max(0.0, -GROUND_K * depth - GROUND_C * contact[2])
                grip = FRICTION * normal / max(SLIP, math.hypot(contact[0], contact[1]))
                reaction = [-grip * contact[0], -grip * contact[1], normal]
                push = [p + r for p, r in zip(push, reaction)]
                torque = [t + c for t, c in zip(torque, cross(foot, apply(matrix, reaction, transpose=True)))]
            if lifted is None and liftoff and not touching:
                lifted = step * DT
            velocity = [v + (p / MASS - (9.81 if axis == 2 else 0.0)) * h for axis, (v, p) in enumerate(zip(velocity, push))]
            position = [x + v * h for x, v in zip(position, velocity)]
            specific = [value / WEIGHT for value in apply(matrix, push, transpose=True)]
            rate = [r + t / i * h for r, t, i in zip(rate, torque, inertia)]
            p, q, r = rate
            roll, pitch = angles[0], angles[1]
            angles = [
                roll + (p + math.tan(pitch) * (q * math.sin(roll) + r * math.cos(roll))) * h,
                pitch + (q * math.cos(roll) - r * math.sin(roll)) * h,
                angles[2] + (q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch) * h,
            ]
        if log is not None:
            log.append((step * DT, list(position), list(angles), list(attitude), list(servo), list(motor)))
        roll, pitch = angles[0], angles[1]
        angle = max(abs(roll), abs(pitch))
        worst = max(worst, angle)
        if lifted is not None:
            trace.append((step * DT - lifted, math.degrees(pitch)))
            peak_height = max(peak_height, position[2] + lowest)
        if angle > math.radians(45):
            return {"stable": False, "fail_s": step * DT, "worst": 45.0, "axis": "roll" if abs(roll) > abs(pitch) else "pitch", "yaw_rate": math.degrees(rate[2])}
        if step * DT >= seconds / 2:
            late.append((roll, pitch, rate[2]))
    rms = [math.degrees(math.sqrt(sum(v[i] ** 2 for v in late) / len(late))) for i in range(3)]
    result = {"stable": True, "worst": math.degrees(worst), "roll_rms": rms[0], "pitch_rms": rms[1], "yaw_rms": rms[2], "drift": math.hypot(position[0], position[1])}
    if liftoff:
        # Peak pitch in the first second off the ground, when pitch last left ±5° (the settling time), and the highest foot clearance.
        result["liftoff_s"] = lifted
        result["peak_pitch"] = max((abs(p) for t, p in trace if t <= 1.0), default=0.0)
        result["settle_s"] = max((t for t, p in trace if abs(p) > 5), default=0.0)
        result["peak_height"] = peak_height
    return result


# Plant as measured (2026-10-21): pivot 20 mm above the CoM, pitch inertia 0.00161 kg m^2, thrust 1.09x the table at ~16 V.
NOMINAL = {"thrust_k": 1.09, "height": 0.0234 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 0.00161 / MODEL["pitch_inertia"], "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
           "servo_delay": 0.04, "servo_tau": 0.06, "backlash": 0.5, "deadband": 0.45, "servo_slew": 600, "imbalance": 0.06, "com_x": 0.002, "gyro_noise": 2.0}
# Plausible mismatch: pivot height ±40 %, inertia ×0.7–1.5, servo delay 40–70 ms (2026-10-21 bench: motion starts 50–60 ms after the command), motor delay 20–40 ms.
PLANTS = [NOMINAL | {"height": h, "roll_inertia": i, "pitch_inertia": i, "yaw_inertia": i, "servo_delay": sd, "motor_delay": md}
          for h, i, sd, md in itertools.product((0.6, 1.0, 1.4), (0.7, 1.5), (0.04, 0.07), (0.02, 0.04))]


def score(gains, plants=PLANTS, seeds=(0,)):
    cases = [(plant, seed) for plant in plants for seed in seeds]
    results = [run(gains, plant, seed=seed) for plant, seed in cases]
    stable = [r for r in results if r["stable"]]
    return {
        "stable": f"{len(stable)}/{len(results)}",
        "worst_rms": max((max(r["roll_rms"], r["pitch_rms"]) for r in stable), default=float("nan")),
        "median_rms": sorted(max(r["roll_rms"], r["pitch_rms"]) for r in stable)[len(stable) // 2] if stable else float("nan"),
        "fails": [(p["height"], p["roll_inertia"], p["servo_delay"], p["motor_delay"], seed) for (p, seed), r in zip(cases, results) if not r["stable"]],
    }


if __name__ == "__main__":
    import time
    flown = {"roll_p": 40, "roll_i": 50, "roll_damping": 6, "pitch_p": 25, "pitch_i": 10, "pitch_damping": 5, "yaw_damping": 10, "integral_limit": 8}
    t = time.time()
    print("nominal, flown gains:", run(flown, NOMINAL), f"{time.time() - t:.2f} s/run")
