"""
Gain sweeps: uv run -m sw.sim.sweep. Runs the drone's C controller (via sw.sim.flight) against a lean rigid-body attitude plant with actuator delay, lag, servo backlash/slew, gyro noise and model mismatch. Free flight at hover, no ground.
"""

import itertools
import math
import random
from collections import deque

from sw.sim.flight import MODEL, Controller, throttle, thrust

WEIGHT = 0.900 * 9.81
DT = 0.01
SUB = 5


def run(gains, plant, seconds=10.0, seed=0):
    rng = random.Random(seed)
    settings = dict(MODEL) | {"roll": 0, "pitch": 0, "yaw_rate": 0, "pitch_trim": 0, "left_trim": 0, "right_trim": 0,
                               "roll_bias": 0, "pitch_bias": 0, "yaw_bias": 0, "left_thrust_scale": 1, "right_thrust_scale": 1,
                               "reaction_torque_ratio": 0.015, "servo_compensation": 0.75, "integrate_throttle": 580} | gains
    k = plant["thrust_k"]
    settings["base_throttle"] = round(throttle(WEIGHT / 2 / k))
    controller = Controller()
    controller.configure(settings | {"enabled": 1})
    controller.reset()
    inertia = [MODEL["roll_inertia"] * plant["roll_inertia"], MODEL["pitch_inertia"] * plant["pitch_inertia"], MODEL["yaw_inertia"] * plant["yaw_inertia"]]
    height = MODEL["pivot_height"] * plant["height"]
    arm = MODEL["arm"]
    roll, pitch = math.radians(plant.get("roll0", 3)), math.radians(plant.get("pitch0", -3))
    rate = [0.0, 0.0, 0.0]
    motor = [WEIGHT / 2, WEIGHT / 2]
    servo = [0.0, 0.0]  # physical tilt, rad, positive forward
    output = [0.0, 0.0]  # servo horn position before backlash
    motor_queue = deque([(settings["base_throttle"],) * 2] * max(1, round(plant["motor_delay"] / DT)))
    servo_queue = deque([(0.0, 0.0)] * max(1, round(plant["servo_delay"] / DT)))
    slack = math.radians(plant["backlash"])
    slew = math.radians(plant["servo_slew"]) * DT / SUB
    scales = [k * (1 - plant["imbalance"] / 2), k * (1 + plant["imbalance"] / 2)]
    worst, late = 0.0, []
    gyro_filter = [0.0, 0.0, 0.0]
    # The servo electronics ignore a command until it differs from the last accepted one by more than the deadband.
    accepted = [0.0, 0.0]
    deadband = math.radians(plant["deadband"])
    for step in range(int(seconds / DT)):
        noise = [rng.gauss(0, math.radians(plant["gyro_noise"])) for _ in range(3)]
        gyro_filter = [g + 0.7 * (r + n - g) for g, r, n in zip(gyro_filter, rate, noise)]
        left, right, left_angle, right_angle = controller.update((roll, pitch), gyro_filter, DT)
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
                height * (forward[0] + forward[1]) + plant["com_x"] * (vertical[0] + vertical[1]),
                arm * (forward[1] - forward[0]) + 0.015 * (vertical[1] - vertical[0]),
            ]
            rate = [r + t / i * h for r, t, i in zip(rate, torque, inertia)]
            p, q, r = rate
            roll += (p + math.tan(pitch) * (q * math.sin(roll) + r * math.cos(roll))) * h
            pitch += (q * math.cos(roll) - r * math.sin(roll)) * h
        angle = max(abs(roll), abs(pitch))
        worst = max(worst, angle)
        if angle > math.radians(45):
            return {"stable": False, "fail_s": step * DT, "worst": 45.0, "axis": "roll" if abs(roll) > abs(pitch) else "pitch", "yaw_rate": math.degrees(rate[2])}
        if step * DT >= seconds / 2:
            late.append((roll, pitch, rate[2]))
    rms = [math.degrees(math.sqrt(sum(v[i] ** 2 for v in late) / len(late))) for i in range(3)]
    return {"stable": True, "worst": math.degrees(worst), "roll_rms": rms[0], "pitch_rms": rms[1], "yaw_rms": rms[2]}


NOMINAL = {"thrust_k": 0.93, "height": 1.0, "roll_inertia": 1.0, "pitch_inertia": 1.0, "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
               "servo_delay": 0.04, "servo_tau": 0.06, "backlash": 0.5, "deadband": 0.45, "servo_slew": 600, "imbalance": 0.06, "com_x": 0.002, "gyro_noise": 2.0}
# Plausible mismatch: pivot height ±40 %, inertia ×0.7–1.5, servo delay 40–70 ms (2026-10-21 bench: motion starts 50–60 ms after the command), motor delay 20–40 ms.
PLANTS = [NOMINAL | {"height": h, "roll_inertia": i, "pitch_inertia": i, "yaw_inertia": i, "servo_delay": sd, "motor_delay": md}
          for h, i, sd, md in itertools.product((0.6, 1.0, 1.4), (0.7, 1.5), (0.04, 0.07), (0.02, 0.04))]


def score(gains, plants=PLANTS, seeds=(0,)):
    results = [run(gains, plant, seed=seed) for plant in plants for seed in seeds]
    stable = [r for r in results if r["stable"]]
    return {
        "stable": f"{len(stable)}/{len(results)}",
        "worst_rms": max((max(r["roll_rms"], r["pitch_rms"]) for r in stable), default=float("nan")),
        "median_rms": sorted(max(r["roll_rms"], r["pitch_rms"]) for r in stable)[len(stable) // 2] if stable else float("nan"),
        "fails": [(p["height"], p["roll_inertia"], p["servo_delay"], p["motor_delay"]) for p, r in zip(plants, results) if not r["stable"]],
    }


if __name__ == "__main__":
    import time
    flown = {"roll_p": 40, "roll_i": 50, "roll_damping": 6, "pitch_p": 25, "pitch_i": 10, "pitch_damping": 5, "yaw_damping": 10, "integral_limit": 8}
    t = time.time()
    print("nominal, flown gains:", run(flown, NOMINAL), f"{time.time() - t:.2f} s/run")
