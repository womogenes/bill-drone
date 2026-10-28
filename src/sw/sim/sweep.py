"""
Gain sweeps: uv run -m sw.sim.sweep. Runs the drone's C attitude estimator and controller (via sw.sim.flight) against a lean 6-DOF rigid-body plant with actuator delay, lag, servo backlash/slew, IMU noise, drag, landing-leg ground contact and model mismatch. Free flight starts at hover in the air; a plant with "liftoff" starts on the ground at the dashboard's T collective instead, level or resting at "ground_pitch" degrees on its "feet" (contact points relative to the CoM, default FEET).
"""

import itertools
import json
import math
import random
from collections import deque
from pathlib import Path

from sw.sim.flight import MODEL, MODEL_KEYS, MODEL_OFFSET, Attitude, Controller, throttle, thrust

MASS = 0.940  # kg, the final assembly with the printed deck and IMU bracket (Bill's estimate, 2026-10-24)
WEIGHT = MASS * 9.81
DT = 0.01
# Physics substeps per control step: the foot contacts' friction is stiff, and at 10 a gripping contact chatters (the sampled gyro and accelerometer read tens of deg/s and 2-3 g at rest). Clear of the floor the rigid body and actuators need far fewer.
SUB = 40
AIR_SUB = 4
# Height of the lowest foot above the floor (m) above which a control step uses AIR_SUB.
CLEAR = 0.02
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


def level_turn(vector, roll, pitch):
    """
    A body vector turned by the level reference (roll about forward, then pitch about left), as flight_rt.c drag_inputs turns the IMU's specific force by pose_zero.
    """
    x, y, z = vector
    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    return [x * math.cos(pitch) + z * math.sin(pitch), y, -x * math.sin(pitch) + z * math.cos(pitch)]


def sighting(camera, position, angles, cam_pitch, rng, cam_roll=0.0):
    """
    The tag centre in the camera frame (m; right, down, forward) as a camera rolled by cam_roll about its axis reports it (hover.c level turns it back), or None when it is outside the field of view.
    """
    offset = [t - p for t, p in zip(camera["tag"], position)]
    x, y, z = apply(rotation(*angles), offset, transpose=True)
    ca, sa = math.cos(cam_pitch), math.sin(cam_pitch)
    right, down, ahead = -y, x * sa - z * ca, x * ca + z * sa
    across, above, below = camera["fov"]
    if ahead <= 0 or abs(math.atan2(right, ahead)) > math.radians(across) or not -math.radians(above) <= math.atan2(down, ahead) <= math.radians(below):
        return None
    spread = camera["noise"] * math.sqrt(x * x + y * y + z * z)
    right, down, ahead = right + rng.gauss(0, spread), down + rng.gauss(0, spread), ahead + rng.gauss(0, spread)
    cc, sc = math.cos(cam_roll), math.sin(cam_roll)
    return [cc * right + sc * down, -sc * right + cc * down, ahead]


def run(gains, plant, seconds=10.0, seed=0, log=None, hover=False):
    """
    Simulate one flight. A log list receives (time, position, roll/pitch/yaw, estimated roll/pitch, servo tilts, motor thrusts, pitch target, tag seen) every control step.
    With hover, the drone arms on the ground and the firmware's hover loop (fw/drone/main/hover.c, gains from the hover_* settings) flies it from takeoff on the plant's camera (CAMERA, overridden by plant["camera"]; none with hover_camera 0, and none from plant["camera_lost"] seconds after liftoff), glued to the attitude controller as flight_rt.c does.
    With plant["airframe"] (parameters fitted by sw/sim/identify_airframe.py), sw/sim/airframe.py supplies the motors, servos and angular accelerations instead of the plant's own actuator and torque model; the rest (translation, ground contact, sensors, camera) stays.
    IMU errors for the hover's estimators: plant["accel_bias"] (g; forward, left) on the accelerometer, "vibration" (g RMS, white; one value or forward, left, up), and "level_error" (degrees; roll, pitch) in the level reference the drone flies and turns its specific force by. Rotor drag: "rotor_drag" (1/s; one value, or forward and left) pulls the body's horizontal velocity relative to the air, "wind" (m/s RMS per horizontal axis, time constant "wind_tau"); "tilt_offset" (degrees) and "tilt_gain" make the servos' physical forward tilt differ from their commands.
    """
    rng = random.Random(seed)
    accel_rng = random.Random(seed + 1000)
    camera_rng = random.Random(seed + 2000)
    gust_rng = random.Random(seed + 3000)
    wind_rng = random.Random(seed + 4000)
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
    # Roll angular acceleration not in the model (rad/s², positive raising the left side): a constant one (thrust imbalance the roll bias cancels) and a random one like the pitch gust.
    roll_disturbance, roll_gust_size, roll_gust = plant.get("roll_disturbance", 0.0), plant.get("roll_gust", 0.0), 0.0
    roll_rng = random.Random(seed + 5000)
    trim = math.radians(settings["pitch_trim"])
    rotor_drag, wind_size, wind_tau, wind = plant.get("rotor_drag", 0.0), plant.get("wind", 0.0), plant.get("wind_tau", 2.0), [0.0, 0.0, 0.0]
    accel_bias = list(plant.get("accel_bias", (0.0, 0.0))) + [0.0]
    vibration = plant.get("vibration", 0.0)
    vibration = list(vibration) if isinstance(vibration, (list, tuple)) else [vibration] * 3
    level_error = [math.radians(value) for value in plant.get("level_error", (0.0, 0.0))]
    tilt_offset, tilt_gain = math.radians(plant.get("tilt_offset", 0.0)), plant.get("tilt_gain", 1.0)
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
    commanded = (0.0, 0.0, 0.0, 0.0)
    airframe = None
    if plant.get("airframe"):
        from sw.sim.airframe import Airframe
        airframe = Airframe(plant["airframe"])
        airframe.reset(0 if liftoff else hover_throttle)
        motor = list(airframe.motor)
    places, touched = [], None
    if hover:
        from sw.sim.hover import HOLD, Hover, Pose
        use_camera = settings.get("hover_camera", 1)
        if not use_camera:
            settings["hover_center"] = 0
        outer = Hover(settings)
        camera = CAMERA | plant.get("camera", {})
        # Frames waiting out their latency (ready time, capture time, sighting or None), the newest delivered one, and the next capture time.
        frames, newest, capture, sequence, missing_frames = deque(), None, 0.0, 0, 0
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
        accel_filter = [a + 0.7 * (s + b + accel_rng.gauss(0, ACCEL_NOISE) + (accel_rng.gauss(0, v) if v else 0.0) - a) for a, s, b, v in zip(accel_filter, specific, accel_bias, vibration)]
        attitude = estimator.update(accel_filter, gyro_filter, DT, powered=True)
        # The drone flies relative to its level reference: an error in it offsets the attitude both loops see.
        attitude = (attitude[0] - level_error[0], attitude[1] - level_error[1])
        visible = None
        if hover:
            roll_estimate, pitch_estimate = attitude
            vertical = -accel_filter[0] * math.sin(pitch_estimate) + math.cos(pitch_estimate) * (accel_filter[1] * math.sin(roll_estimate) + accel_filter[2] * math.cos(roll_estimate))
            if plant.get("ideal_height"):
                # A study of the horizontal loop alone: the true vertical specific force, so the IMU-only height hold cannot wander off (the measured one, projected with the estimated attitude, drifts by centimetres per second).
                vertical = apply(rotation(*angles), specific)[2]
            while capture <= now:
                # Detector misses come in bursts (blur, glare): a frame starts one with probability camera["miss"], and each burst lasts camera["burst"] frames on average.
                missing = missing_frames > 0 or camera_rng.random() < camera.get("miss", 0.0)
                missing_frames = missing_frames - 1 if missing_frames > 0 else (round(camera_rng.expovariate(1 / camera.get("burst", 1))) if missing else 0)
                frames.append((capture + camera["latency"], capture, None if missing else sighting(camera, position, angles, math.radians(settings.get("hover_cam_pitch", 0.0)), camera_rng, math.radians(settings.get("hover_cam_roll", 0.0)))))
                capture += 1 / camera["fps"]
            while frames and frames[0][0] <= now:
                newest = frames.popleft()
                sequence += 1
            # Like camera_candidates: the newest delivered frame's tag, if it saw one and was captured at most 150 ms ago.
            pose = None
            lost = plant.get("camera_lost") is not None and lifted is not None and now - lifted >= plant["camera_lost"]
            if use_camera and not lost and newest and newest[2] and now - newest[1] <= 0.15:
                pose = Pose(sequence=sequence, capture_us=round((newest[1] + 1) * 1e6), tag=tuple(newest[2]))
            visible = pose is not None
            if step < armed:
                outer.rest(vertical, False, DT)
            else:
                flying = outer.state.force if outer.state.engaged else 0.0
                heading_rate = (gyro_filter[1] * math.sin(roll_estimate) + gyro_filter[2] * math.cos(roll_estimate)) / math.cos(pitch_estimate)
                # As flight_rt.c drag_inputs: the specific force in the level reference's axes, and the thrust tilt the controller last commanded.
                left_force, right_force = thrust(commanded[0]), thrust(commanded[1])
                left_tilt = MODEL["left_servo_direction"] * (commanded[2] - servo_zero[0])
                right_tilt = MODEL["right_servo_direction"] * (commanded[3] - servo_zero[1])
                up = left_force * math.cos(left_tilt) + right_force * math.cos(right_tilt)
                tilt = math.atan2(left_force * math.sin(left_tilt) + right_force * math.sin(right_tilt), up) if up > 0 else 0.0
                drag = (level_turn(accel_filter, *level_error), tilt)
                out = outer.update(HOLD, pose, round((now + 1) * 1e6), DT, roll_estimate, pitch_estimate, heading_rate, vertical, flying, drag)
                settings["base_throttle"] = min(1100.0, throttle(out.force / 2 / math.cos(trim))) if out.force > 0 else 0
                settings["roll"], settings["pitch"], settings["yaw_rate"] = math.degrees(out.roll), math.degrees(out.pitch), math.degrees(out.yaw_rate)
                controller.configure(settings | {"enabled": 1})
                if out.tracking:
                    controller.state[13], controller.state[14] = out.velocity[0], out.velocity[1]
        left, right, left_angle, right_angle = controller.update(attitude, gyro_filter, DT)
        commanded = (left, right, left_angle, right_angle)
        motor_queue.append((left, right))
        servo_queue.append(((left_angle - servo_zero[0]) * MODEL["left_servo_direction"] * tilt_gain + tilt_offset, (right_angle - servo_zero[1]) * MODEL["right_servo_direction"] * tilt_gain + tilt_offset))
        target_throttle = motor_queue.popleft()
        for side, angle in enumerate(servo_queue.popleft()):
            if abs(angle - accepted[side]) > deadband:
                accepted[side] = angle
        target_servo = accepted
        if gust_size:
            keep = math.exp(-DT / gust_tau)
            gust = keep * gust + gust_size * math.sqrt(1 - keep * keep) * gust_rng.gauss(0, 1)
        if roll_gust_size:
            keep = math.exp(-DT / gust_tau)
            roll_gust = keep * roll_gust + roll_gust_size * math.sqrt(1 - keep * keep) * roll_rng.gauss(0, 1)
        if wind_size:
            keep = math.exp(-DT / wind_tau)
            wind = [keep * w + wind_size * math.sqrt(1 - keep * keep) * wind_rng.gauss(0, 1) for w in wind[:2]] + [0.0]
        if airframe:
            body_velocity = apply(rotation(*angles), velocity, transpose=True)
            tilt_commands = [math.degrees(left_angle - servo_zero[0]) * MODEL["left_servo_direction"], math.degrees(right_angle - servo_zero[1]) * MODEL["right_servo_direction"]]
            # The near-ground pitch term goes by the lowest foot's height (as the logged height above the engage point) or the time since liftoff, as it was fitted.
            ground_by = plant["airframe"].get("ground_by")
            near = None if not ground_by else max(0.0, position[2] + min(apply(rotation(*angles), foot)[2] for foot in feet)) if ground_by == "height" else (now - lifted if lifted is not None else 0.0) if liftoff else 10.0
            accel, motor, servo = airframe.step((left, right), tilt_commands, 1.0, rate, angles[:2], body_velocity[:2], airborne=lifted is not None or not liftoff, near=near)
            motor, servo = list(motor), list(servo)
        clearance = position[2] + min(apply(rotation(*angles), foot)[2] for foot in feet)
        substeps = AIR_SUB if clearance > CLEAR + math.hypot(*velocity) * DT else SUB
        h = DT / substeps
        slew = math.radians(plant["servo_slew"]) * h
        for _ in range(substeps):
            for side in range(2):
                if airframe:
                    break
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
                arm * (vertical[0] - vertical[1]) + ((roll_disturbance + roll_gust) * inertia[0] if lifted is not None or not liftoff else 0.0),
                height * (forward[0] + forward[1]) + plant["com_x"] * (vertical[0] + vertical[1]) + extra * inertia[1],
                arm * (forward[1] - forward[0]) + 0.015 * (vertical[1] - vertical[0]),
            ]
            if airframe:
                # The airframe's accelerations already hold the thrust torques; the random and constant extras still add.
                airborne = lifted is not None or not liftoff
                torque = [accel[0] * inertia[0] + ((roll_disturbance + roll_gust) * inertia[0] if airborne else 0.0), accel[1] * inertia[1] + (gust + disturbance if airborne else 0.0) * inertia[1], accel[2] * inertia[2]]
            matrix = rotation(*angles)
            speed = math.sqrt(sum(v * v for v in velocity))
            # Every force except gravity, in the world frame: thrust, drag, then each foot below the floor.
            push = [f - DRAG * speed * v for f, v in zip(apply(matrix, [sum(forward), 0.0, sum(vertical)]), velocity)]
            if rotor_drag:
                # Blade flapping and induced drag: a force against the airspeed in the rotor plane, proportional to it.
                air = apply(matrix, [v - w for v, w in zip(velocity, wind)], transpose=True)
                drag_x, drag_y = rotor_drag if isinstance(rotor_drag, (list, tuple)) else (rotor_drag, rotor_drag)
                push = [p + d for p, d in zip(push, apply(matrix, [-mass * drag_x * air[0], -mass * drag_y * air[1], 0.0]))]
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
            if hover and round((now - lifted) / DT) % round(1 / DT) == 0:
                places.append((round(now - lifted), position[0], position[1]))
            if hover and touched is None and now - lifted >= 2.0 and position[2] + lowest < 0.01:
                touched = now - lifted
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
        # Horizontal place (m; forward, left from the takeoff point) each whole second after liftoff.
        result["places"] = places
        # Seconds after liftoff the feet first came back within 1 cm of the floor (from 2 s on), or None: the height hold let it down.
        result["touched"] = touched
    return result


# Plant as measured (2026-10-21): pivot 20 mm above the CoM, pitch inertia 0.00161 kg m^2, thrust 1.09x the table at ~16 V.
NOMINAL = {"thrust_k": 1.09, "height": 0.0234 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 0.00161 / MODEL["pitch_inertia"], "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
           "servo_delay": 0.04, "servo_tau": 0.06, "backlash": 0.5, "deadband": 0.45, "servo_slew": 600, "imbalance": 0.06, "com_x": 0.002, "gyro_noise": 2.0}
# Rotor drag (0.5 1/s) and accelerometer vibration are from the same flights (sw/fit_drag.py): logged hover vibration is 0.15 g broadband horizontally, but averaged over 0.5-1.5 s (what the drag model sees) it is as low as white noise of about 0.1 g before the 0.7 smoothing here; 0.05 g vertically.
# Fitted to the 2026-10-25 hover flights (08-52 to 09-15) by closed-loop multi-step pitch prediction from replayed controller states: effectiveness 1.42x the controller's model (pivot_height 0.03, pitch_inertia 0.0023), 70 ms dead time + 10 ms lag at 340°/s, a -10.6 rad/s² tail-heavy push, a weak restoring angle and forward-speed moment, and an unexplained slow pitch disturbance of about 5 rad/s² RMS. Backlash and deadband were not identifiable (fits went to zero), so play here is the bench's "small, left sloppier". It reproduces the 09-01 pitch_damping 14 oscillation (0.38-0.40 s period vs 0.41 s logged) and damping 10 staying clear of it. Fly it with run(..., hover=True) and a recording's settings.
FITTED = {"thrust_k": 1.03, "mass": 0.86, "height": 1.42 * 0.03 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 1.0, "yaw_inertia": 1.0,
          "motor_delay": 0.02, "motor_tau": 0.05, "servo_delay": 0.07, "servo_tau": 0.01, "backlash": 0.3, "left_backlash": 0.5, "right_backlash": 0.3,
          "deadband": 0.1, "servo_slew": 340, "imbalance": 0.03, "com_x": 0.0, "disturbance": -10.6, "angle_moment": -3.9, "velocity_moment": -4.0,
          "gust": 5.0, "gust_tau": 0.7, "gyro_noise": 2.0, "camera": {"tag": (2.3, 0.0, 0.3)}, "rotor_drag": 0.5, "vibration": (0.1, 0.1, 0.05)}
# Identified from the 34 hover flights of 2026-10-27 (both rubber bands on) by closed-loop multi-step prediction with the firmware controller in the loop (sw/sim/identify_airframe.py, parameters in models/airframe_2026-10-27.json). Held out by flight it predicts roll to 1.3° and pitch to 3.7° a second ahead (yaw rate R² 0.55) from the logged references alone. Rotor drag is from the camera velocity over 0.25-1 s against thrust through the logged attitude and modelled tilts (gain within 10 % of 1). The random disturbance is what of the one-step residual lies below 2 Hz (the rest, mostly 10-50 Hz in pitch, does not move the attitude): 0.89 rad/s² RMS in roll and 0.49 in pitch at 0.5 s.
AIRFRAME = json.loads((Path(__file__).with_name("models") / "airframe_2026-10-27.json").read_text())
IDENTIFIED = FITTED | {"airframe": AIRFRAME, "mass": 0.94, "thrust_k": 1.0, "height": 1.0, "disturbance": 0.0, "angle_moment": 0.0, "velocity_moment": 0.0, "gust": 0.49, "gust_tau": 0.5, "roll_gust": 0.89,
                       "camera": {"tag": (2.0, 0.1, 0.45), "miss": 0.006, "burst": 6}, "rotor_drag": (0.6, 0.27)}
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
