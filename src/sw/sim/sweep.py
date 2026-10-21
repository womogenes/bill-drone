"""
Gain sweeps: uv run -m sw.sim.sweep. Runs the drone's C controller (via sw.sim.flight) against a lean rigid-body attitude plant with actuator delay, lag, servo backlash/slew, gyro noise and model mismatch. Free flight at hover, no ground; a plant with "liftoff" starts level on the ground at the dashboard's T collective instead.
"""

import itertools
import math
import random
from collections import deque

from sw.sim.flight import MODEL, Controller, throttle, thrust

WEIGHT = 0.881 * 9.81  # with the bench deck
DT = 0.01
SUB = 5
TAKEOFF_THROTTLE = 720  # dashboard T
CLIMB = 0.5  # seconds at T after liftoff before collective drops to hover


def run(gains, plant, seconds=10.0, seed=0):
    rng = random.Random(seed)
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
    grounded, lifted = liftoff, None
    inertia = [MODEL["roll_inertia"] * plant["roll_inertia"], MODEL["pitch_inertia"] * plant["pitch_inertia"], MODEL["yaw_inertia"] * plant["yaw_inertia"]]
    height = MODEL["pivot_height"] * plant["height"]
    arm = MODEL["arm"]
    roll, pitch = (0.0, 0.0) if liftoff else (math.radians(plant.get("roll0", 3)), math.radians(plant.get("pitch0", -3)))
    rate = [0.0, 0.0, 0.0]
    motor = [0.0, 0.0] if liftoff else [WEIGHT / 2, WEIGHT / 2]
    servo = [0.0, 0.0]  # physical tilt, rad, positive forward
    output = [0.0, 0.0]  # servo horn position before backlash
    motor_queue = deque([(0 if liftoff else hover,) * 2] * max(1, round(plant["motor_delay"] / DT)))
    servo_queue = deque([(0.0, 0.0)] * max(1, round(plant["servo_delay"] / DT)))
    slack = math.radians(plant["backlash"])
    slew = math.radians(plant["servo_slew"]) * DT / SUB
    scales = [k * (1 - plant["imbalance"] / 2), k * (1 + plant["imbalance"] / 2)]
    worst, late, trace = 0.0, [], []
    gyro_filter = [0.0, 0.0, 0.0]
    # The servo electronics ignore a command until it differs from the last accepted one by more than the deadband.
    accepted = [0.0, 0.0]
    deadband = math.radians(plant["deadband"])
    for step in range(int(seconds / DT)):
        if lifted is not None and step * DT - lifted >= CLIMB and settings["base_throttle"] != hover:
            settings["base_throttle"] = hover
            controller.configure(settings | {"enabled": 1})
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
            if grounded:
                # Level feet hold the attitude until the motors carry the weight.
                if sum(vertical) < WEIGHT:
                    continue
                grounded, lifted = False, step * DT
            torque[1] += disturbance * inertia[1]
            rate = [r + t / i * h for r, t, i in zip(rate, torque, inertia)]
            p, q, r = rate
            roll += (p + math.tan(pitch) * (q * math.sin(roll) + r * math.cos(roll))) * h
            pitch += (q * math.cos(roll) - r * math.sin(roll)) * h
        angle = max(abs(roll), abs(pitch))
        worst = max(worst, angle)
        if lifted is not None:
            trace.append((step * DT - lifted, math.degrees(pitch)))
        if angle > math.radians(45):
            return {"stable": False, "fail_s": step * DT, "worst": 45.0, "axis": "roll" if abs(roll) > abs(pitch) else "pitch", "yaw_rate": math.degrees(rate[2])}
        if step * DT >= seconds / 2:
            late.append((roll, pitch, rate[2]))
    rms = [math.degrees(math.sqrt(sum(v[i] ** 2 for v in late) / len(late))) for i in range(3)]
    result = {"stable": True, "worst": math.degrees(worst), "roll_rms": rms[0], "pitch_rms": rms[1], "yaw_rms": rms[2]}
    if liftoff:
        # Peak pitch in the first second off the ground, and when pitch last left ±5° (the settling time).
        result["liftoff_s"] = lifted
        result["peak_pitch"] = max(abs(p) for t, p in trace if t <= 1.0)
        result["settle_s"] = max((t for t, p in trace if abs(p) > 5), default=0.0)
    return result


# Plant as measured (2026-10-21): pivot 20 mm above the CoM, pitch inertia 0.00161 kg m^2, thrust 1.09x the table at ~16 V.
NOMINAL = {"thrust_k": 1.09, "height": 0.020 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 0.00161 / MODEL["pitch_inertia"], "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
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
