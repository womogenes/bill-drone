"""
Gain sweeps: uv run -m sw.sim.sweep. Runs the drone's C attitude estimator and controller (via sw.sim.flight) against a lean 6-DOF rigid-body plant with actuator delay, lag, servo backlash/slew, IMU noise, drag, landing-leg ground contact and model mismatch. Free flight starts at hover in the air; a plant with "liftoff" starts on the ground at the dashboard's T collective instead, level or resting at "ground_pitch" degrees on its "feet" (contact points relative to the CoM, default FEET).
"""

import itertools
import math
import random
from collections import deque

from sw.sim.flight import MODEL, MODEL_KEYS, MODEL_OFFSET, Attitude, Controller, throttle, thrust

MASS = 0.940  # kg, the final assembly with the printed deck and IMU bracket (Bill's estimate, 2026-10-24)
WEIGHT = MASS * 9.81
DT = 0.01
# Physics substeps per control step: the foot contacts' friction is stiff, and at 10 a gripping contact chatters (the sampled gyro and accelerometer read tens of deg/s and 2-3 g at rest).
SUB = 40
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
# The ESP32-CAM tag sighting (fw/esp32cam/README.md): the tag centre's world position (m; forward, left, up from the takeoff point), the field of view from the camera axis (degrees; half across, above, below: the central 1536x1152 window, fx 493 px and cy 108 at 320x240), frame rate, capture-to-use latency (s), and position noise as a fraction of range.
CAMERA = {"tag": (2.0, 0.0, 0.5), "fov": (18.0, 12.4, 15.0), "fps": 29.4, "latency": 0.06, "noise": 0.01}
# The full 2048x1536 array (before a780870): fx 369.5 px, cy 111, at about 22 fps.
FULL_ARRAY = {"fov": (23.4, 16.7, 19.2), "fps": 22.0}
# The drone has no landing legs: it rests on the bottom of its body (contact points relative to the CoM, m).
BODY_FEET = [(x, y, -0.035) for x in (0.07, -0.07) for y in (0.1, -0.1)]


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


def sighting(camera, position, angles, cam_pitch, rng):
    """
    The tag centre in the camera frame (m; right, down, forward), or None when it is outside the field of view.
    """
    offset = [t - p for t, p in zip(camera["tag"], position)]
    x, y, z = apply(rotation(*angles), offset, transpose=True)
    ca, sa = math.cos(cam_pitch), math.sin(cam_pitch)
    right, down, ahead = -y, x * sa - z * ca, x * ca + z * sa
    across, above, below = camera["fov"]
    if ahead <= 0 or abs(math.atan2(right, ahead)) > math.radians(across) or not -math.radians(above) <= math.atan2(down, ahead) <= math.radians(below):
        return None
    spread = camera["noise"] * math.sqrt(x * x + y * y + z * z)
    return [right + rng.gauss(0, spread), down + rng.gauss(0, spread), ahead + rng.gauss(0, spread)]


def run(gains, plant, seconds=10.0, seed=0, log=None, hover=False):
    """
    Simulate one flight. A log list receives (time, position, roll/pitch/yaw, estimated roll/pitch, servo tilts, motor thrusts, pitch target, tag seen) every control step.
    With hover, the drone arms on the ground and the firmware's hover loop (fw/drone/main/hover.c, gains from the hover_* settings) flies it from takeoff on the plant's camera (CAMERA, overridden by plant["camera"]), glued to the attitude controller as flight_rt.c does.
    """
    rng = random.Random(seed)
    accel_rng = random.Random(seed + 1000)
    camera_rng = random.Random(seed + 2000)
    gust_rng = random.Random(seed + 3000)
    settings = dict(MODEL) | {"roll": 0, "pitch": 0, "yaw_rate": 0, "pitch_trim": 0, "left_trim": 0, "right_trim": 0,
                               "roll_bias": 0, "pitch_bias": 0, "yaw_bias": 0, "left_thrust_scale": 1, "right_thrust_scale": 1,
                               "reaction_torque_ratio": 0.015, "servo_compensation": 0.75, "integrate_throttle": 580, "target_rate": 360, "velocity_damping": 8, "velocity_leak": 3} | gains
    mass = plant.get("mass", MASS)
    weight = mass * 9.81
    k = plant["thrust_k"]
    hover_throttle = round(throttle(weight / 2 / k))
    liftoff = plant.get("liftoff", False) or hover
    settings["base_throttle"] = TAKEOFF_THROTTLE if liftoff else hover_throttle
    controller = Controller()
    controller.configure(settings | {"enabled": 1})
    controller.reset()
    # Runtime-only yaw-rate integral gain, after thrust_voltage in the C Command.
    controller.command[MODEL_OFFSET + len(MODEL_KEYS) + 1] = settings.get("yaw_i", 0.0)
    # Constant pitch angular acceleration not in the controller's model (servo zero or CoM error), rad/s², positive nose-down.
    disturbance = plant.get("disturbance", 0.0)
    # Pitch angular acceleration per rad/s of pitch rate, per radian of pitch beyond pitch_trim and per m/s of forward airspeed (rad/s² each; positive nose-down, so a negative velocity_moment is the rotors' flapping back in forward flight), and a random one (rad/s² RMS, Ornstein-Uhlenbeck with time constant gust_tau).
    rate_moment, angle_moment, velocity_moment = plant.get("rate_moment", 0.0), plant.get("angle_moment", 0.0), plant.get("velocity_moment", 0.0)
    gust_size, gust_tau, gust = plant.get("gust", 0.0), plant.get("gust_tau", 0.5), 0.0
    trim = math.radians(settings["pitch_trim"])
    # Each servo's zero is its trim: the controller adds the trim to reach the physical angle.
    servo_zero = [math.radians(settings["left_trim"]), math.radians(settings["right_trim"])]
    lifted = None
    inertia = [MODEL["roll_inertia"] * plant["roll_inertia"], MODEL["pitch_inertia"] * plant["pitch_inertia"], MODEL["yaw_inertia"] * plant["yaw_inertia"]]
    height = MODEL["pivot_height"] * plant["height"]
    arm = MODEL["arm"]
    feet = plant.get("feet", BODY_FEET if hover else FEET)
    angles = [0.0, math.radians(plant.get("ground_pitch", 0.0)), 0.0] if liftoff else [math.radians(plant.get("roll0", 3)), math.radians(plant.get("pitch0", -3)), 0.0]
    # World position and velocity of the CoM; on the ground the lowest feet start at their static sag. Free flight starts high enough that sinking without altitude hold never reaches the floor.
    lowest = min(apply(rotation(*angles), foot)[2] for foot in feet)
    position = [0.0, 0.0, -lowest - weight / len(feet) / GROUND_K if liftoff else 50.0]
    velocity = [0.0, 0.0, 0.0]
    rate = [0.0, 0.0, 0.0]
    motor = [0.0, 0.0] if liftoff else [weight / 2, weight / 2]
    servo = [0.0, 0.0]  # physical tilt, rad, positive forward
    output = [0.0, 0.0]  # servo horn position before backlash
    motor_queue = deque([(0 if liftoff else hover_throttle,) * 2] * max(1, round(plant["motor_delay"] / DT)))
    servo_queue = deque([(0.0, 0.0)] * max(1, round(plant["servo_delay"] / DT)))
    # Half of each servo's play (rad): the horn moves this far either way before the motor mount follows.
    slack = [math.radians(plant.get("left_backlash", plant["backlash"])), math.radians(plant.get("right_backlash", plant["backlash"]))]
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
    # Like the drone, a liftoff starts armed at zero collective for "armed_s" (at least one step), so the takeoff setpoint and the upright-takeoff flag start from the resting attitude. A hover first rests long enough to learn the resting specific force.
    armed = max(round(0.5 / DT) if hover else 1, round(plant.get("armed_s", 0.0) / DT)) if liftoff else 0
    if armed:
        settings["base_throttle"] = 0
        controller.configure(settings | {"enabled": 1})
    if hover:
        from sw.sim.hover import HOLD, Hover, Pose
        outer = Hover(settings)
        camera = CAMERA | plant.get("camera", {})
        # Frames waiting out their latency (ready time, capture time, sighting or None), the newest delivered one, and the next capture time.
        frames, newest, capture, sequence = deque(), None, 0.0, 0
        seen, ranges = [], []
    for step in range(int(seconds / DT)):
        now = step * DT
        if armed and step == armed and not hover:
            settings["base_throttle"] = TAKEOFF_THROTTLE
            controller.configure(settings | {"enabled": 1})
        if liftoff and not hover and lifted is None and now < ramp:
            settings["base_throttle"] = round(TAKEOFF_THROTTLE * (step + 1) * DT / ramp)
            controller.configure(settings | {"enabled": 1})
        if not hover and lifted is not None and now - lifted >= CLIMB and settings["base_throttle"] != hover_throttle:
            settings["base_throttle"] = hover_throttle
            controller.configure(settings | {"enabled": 1})
        noise = [rng.gauss(0, math.radians(plant["gyro_noise"])) for _ in range(3)]
        gyro_filter = [g + 0.7 * (r + n - g) for g, r, n in zip(gyro_filter, rate, noise)]
        accel_filter = [a + 0.7 * (s + accel_rng.gauss(0, ACCEL_NOISE) - a) for a, s in zip(accel_filter, specific)]
        attitude = estimator.update(accel_filter, gyro_filter, DT, powered=True)
        visible = None
        if hover:
            roll_estimate, pitch_estimate = attitude
            vertical = -accel_filter[0] * math.sin(pitch_estimate) + math.cos(pitch_estimate) * (accel_filter[1] * math.sin(roll_estimate) + accel_filter[2] * math.cos(roll_estimate))
            while capture <= now:
                frames.append((capture + camera["latency"], capture, sighting(camera, position, angles, math.radians(settings.get("hover_cam_pitch", 0.0)), camera_rng)))
                capture += 1 / camera["fps"]
            while frames and frames[0][0] <= now:
                newest = frames.popleft()
                sequence += 1
            # Like camera_candidates: the newest delivered frame's tag, if it saw one and was captured at most 150 ms ago.
            pose = None
            if newest and newest[2] and now - newest[1] <= 0.15:
                pose = Pose(sequence=sequence, capture_us=round((newest[1] + 1) * 1e6), tag=tuple(newest[2]))
            visible = pose is not None
            if step < armed:
                outer.rest(vertical, False, DT)
            else:
                flying = outer.state.force if outer.state.engaged else 0.0
                heading_rate = (gyro_filter[1] * math.sin(roll_estimate) + gyro_filter[2] * math.cos(roll_estimate)) / math.cos(pitch_estimate)
                out = outer.update(HOLD, pose, round((now + 1) * 1e6), DT, roll_estimate, pitch_estimate, heading_rate, vertical, flying)
                settings["base_throttle"] = min(1100.0, throttle(out.force / 2 / math.cos(trim))) if out.force > 0 else 0
                settings["roll"], settings["pitch"], settings["yaw_rate"] = math.degrees(out.roll), math.degrees(out.pitch), math.degrees(out.yaw_rate)
                controller.configure(settings | {"enabled": 1})
                if out.tracking:
                    controller.state[13], controller.state[14] = out.velocity[0], out.velocity[1]
        left, right, left_angle, right_angle = controller.update(attitude, gyro_filter, DT)
        motor_queue.append((left, right))
        servo_queue.append(((left_angle - servo_zero[0]) * MODEL["left_servo_direction"], (right_angle - servo_zero[1]) * MODEL["right_servo_direction"]))
        target_throttle = motor_queue.popleft()
        for side, angle in enumerate(servo_queue.popleft()):
            if abs(angle - accepted[side]) > deadband:
                accepted[side] = angle
        target_servo = accepted
        if gust_size:
            keep = math.exp(-DT / gust_tau)
            gust = keep * gust + gust_size * math.sqrt(1 - keep * keep) * gust_rng.gauss(0, 1)
        for _ in range(SUB):
            h = DT / SUB
            for side in range(2):
                force = thrust(target_throttle[side]) * scales[side]
                motor[side] += (force - motor[side]) * (1 - math.exp(-h / plant["motor_tau"]))
                horn = output[side] + max(-slew, min(slew, (target_servo[side] - output[side]) * (1 - math.exp(-h / plant["servo_tau"]))))
                output[side] = horn
                if horn - servo[side] > slack[side]:
                    servo[side] = horn - slack[side]
                elif servo[side] - horn > slack[side]:
                    servo[side] = horn + slack[side]
            vertical = [motor[s] * math.cos(servo[s]) for s in range(2)]
            forward = [motor[s] * math.sin(servo[s]) for s in range(2)]
            # The extra pitch terms act only in flight: on the ground the feet carry them.
            extra = disturbance + gust + (rate_moment * rate[1] + angle_moment * (angles[1] - trim) + velocity_moment * apply(rotation(*angles), velocity, transpose=True)[0] if lifted is not None or not liftoff else 0.0)
            torque = [
                arm * (vertical[0] - vertical[1]),
                height * (forward[0] + forward[1]) + plant["com_x"] * (vertical[0] + vertical[1]) + extra * inertia[1],
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
                lifted = now
            velocity = [v + (p / mass - (9.81 if axis == 2 else 0.0)) * h for axis, (v, p) in enumerate(zip(velocity, push))]
            position = [x + v * h for x, v in zip(position, velocity)]
            specific = [value / weight for value in apply(matrix, push, transpose=True)]
            rate = [r + t / i * h for r, t, i in zip(rate, torque, inertia)]
            p, q, r = rate
            roll, pitch = angles[0], angles[1]
            angles = [
                roll + (p + math.tan(pitch) * (q * math.sin(roll) + r * math.cos(roll))) * h,
                pitch + (q * math.cos(roll) - r * math.sin(roll)) * h,
                angles[2] + (q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch) * h,
            ]
        if log is not None:
            log.append((now, list(position), list(angles), list(attitude), list(servo), list(motor), math.radians(settings["pitch"]), visible))
        roll, pitch = angles[0], angles[1]
        angle = max(abs(roll), abs(pitch - trim))
        worst = max(worst, angle)
        if lifted is not None:
            trace.append((now - lifted, math.degrees(pitch - trim)))
            peak_height = max(peak_height, position[2] + lowest)
            if hover and now - lifted >= 1.0:
                seen.append(visible)
                ranges.append(math.hypot(camera["tag"][0] - position[0], camera["tag"][1] - position[1]))
        if angle > math.radians(45) or (hover and lifted is not None and position[2] + lowest < -0.01):
            return {"stable": False, "fail_s": now, "worst": 45.0, "axis": "roll" if abs(roll) > abs(pitch - trim) else "pitch", "yaw_rate": math.degrees(rate[2])}
        if now >= seconds / 2:
            late.append((roll, pitch - trim, rate[2]))
    rms = [math.degrees(math.sqrt(sum(v[i] ** 2 for v in late) / len(late))) for i in range(3)]
    result = {"stable": True, "worst": math.degrees(worst), "roll_rms": rms[0], "pitch_rms": rms[1], "yaw_rms": rms[2], "drift": math.hypot(position[0], position[1])}
    if liftoff:
        # Peak pitch in the first second off the ground, when pitch last left ±5° (the settling time), and the highest foot clearance.
        result["liftoff_s"] = lifted
        result["peak_pitch"] = max((abs(p) for t, p in trace if t <= 1.0), default=0.0)
        result["settle_s"] = max((t for t, p in trace if abs(p) > 5), default=0.0)
        result["peak_height"] = peak_height
    if hover:
        # From a second after liftoff: the fraction of control ticks with the tag in view, the worst pitch beyond trim, and the range's spread and mean error from hover_distance.
        late_pitch = [abs(p) for t, p in trace if t >= 1.0]
        result["seen"] = sum(bool(v) for v in seen) / max(1, len(seen))
        result["pitch_peak"] = max(late_pitch, default=0.0)
        mean = sum(ranges) / max(1, len(ranges))
        result["range_rms"] = math.sqrt(sum((x - mean) ** 2 for x in ranges) / max(1, len(ranges)))
        result["range_error"] = mean - settings.get("hover_distance", 0.0)
    return result


# Plant as measured (2026-10-21): pivot 20 mm above the CoM, pitch inertia 0.00161 kg m^2, thrust 1.09x the table at ~16 V.
NOMINAL = {"thrust_k": 1.09, "height": 0.0234 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 0.00161 / MODEL["pitch_inertia"], "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
           "servo_delay": 0.04, "servo_tau": 0.06, "backlash": 0.5, "deadband": 0.45, "servo_slew": 600, "imbalance": 0.06, "com_x": 0.002, "gyro_noise": 2.0}
# Fitted to the 2026-10-25 hover flights (08-52 to 09-15) by closed-loop multi-step pitch prediction from replayed controller states: effectiveness 1.42x the controller's model (pivot_height 0.03, pitch_inertia 0.0023), 70 ms dead time + 10 ms lag at 340°/s, a -10.6 rad/s² tail-heavy push, a weak restoring angle and forward-speed moment, and an unexplained slow pitch disturbance of about 5 rad/s² RMS. Backlash and deadband were not identifiable (fits went to zero), so play here is the bench's "small, left sloppier". It reproduces the 09-01 pitch_damping 14 oscillation (0.38-0.40 s period vs 0.41 s logged) and damping 10 staying clear of it. Fly it with run(..., hover=True) and a recording's settings.
FITTED = {"thrust_k": 1.03, "mass": 0.86, "height": 1.42 * 0.03 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 1.0, "yaw_inertia": 1.0,
          "motor_delay": 0.02, "motor_tau": 0.05, "servo_delay": 0.07, "servo_tau": 0.01, "backlash": 0.3, "left_backlash": 0.5, "right_backlash": 0.3,
          "deadband": 0.1, "servo_slew": 340, "imbalance": 0.03, "com_x": 0.0, "disturbance": -10.6, "angle_moment": -3.9, "velocity_moment": -4.0,
          "gust": 5.0, "gust_tau": 0.7, "gyro_noise": 2.0, "camera": {"tag": (2.3, 0.0, 0.3)}}
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
