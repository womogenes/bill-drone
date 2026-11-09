"""
The host flight simulation: run() flies the drone's C attitude estimator and controller (via sw.sim.flight), and with hover its camera-hover loop (via sw.sim.hover), against a lean 6-DOF rigid-body plant with actuator delay, lag, servo backlash/slew, IMU noise, drag and landing-leg ground contact; uv run -m sw.sim.sweep flies NOMINAL once with the flown gains. Free flight starts at hover in the air; a plant with "liftoff" starts on the ground at the dashboard's T collective instead, level or resting at "ground_pitch" degrees on its "feet" (contact points relative to the CoM, default FEET).
"""

import json
import math
import random
from collections import deque
from pathlib import Path

from sw.sim.disturbance import Disturbances
from sw.sim.flight import MODEL, PITCH_SPEED, YAW_I, Attitude, Controller, throttle, thrust

MASS = 0.940  # kg, the final assembly with the printed deck and IMU bracket (Bill's estimate, 2026-10-24)
DT = 0.01
# The hold-point nudge's limit each way (m; fw/drone/main/flight_rt.c NUDGE_MAX).
NUDGE_MAX = 1.5
# Physics substeps per control step: the foot contacts' friction is stiff, and at 10 a gripping contact chatters (the sampled gyro and accelerometer read tens of deg/s and 2-3 g at rest). Clear of the floor the rigid body and actuators need far fewer.
SUB = 40
AIR_SUB = 4
# Height of the lowest foot above the floor (m) above which a control step uses AIR_SUB.
CLEAR = 0.02
TAKEOFF_THROTTLE = 690  # dashboard T
# The hover loop's collective cap (fw/drone/main/flight_rt.c HOVER_THROTTLE).
HOVER_THROTTLE = 1200.0
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
# Rate, delay and noise as flown on 2026-10-28 (06:55-07:01 hovers): 21-22 sightings a second, 50 ms median age at use (90th percentile 70), range noise 0.5% of range about a 9-frame mean.
# noise is the measured 0.5% range noise; lateral_noise from the camera hovers' sideways spectrum above 2 Hz, about 100x below the range noise in power.
CAMERA = {"tag": (2.0, 0.0, 0.5), "fov": (18.0, 12.4, 15.0), "fps": 22.0, "latency": 0.045, "noise": 0.005, "lateral_noise": 0.0005}
# The BMP388 as flight_rt.c reads it (run's plant["baro"]): 50 Hz, about 75 ms behind, its noise in moving air, a slow drift (weather, the chip warming), the liftoff's kick and no steady wash, as measured on drone 1's 2026-11-09 outdoor flights against the accelerometer (sw/sim/gps_truth.py, sw/sim/baro_floor.py): 0.06-0.09 m of scatter above 1 Hz in flight, the floor moving -0.46 to +0.60 m between takeoff and landing, -0.13 to +0.14 m with the motors spinning on the floor, and in the first 0.4 s after liftoff 0.3-2.9 m off either way (0-0.2 s: median -0.42 m, 10-90% -1.42 to +0.79). The kick is drawn per run as a gaussian of "kick" m RMS decaying over "kick_tau" s from liftoff. Near the floor in a landing ("wash", fading to none at "wash_height" m) the sign and size are unresolved (+0.25 to +0.45 m against the floor read after the cut, uncertain), so landing_test draws it.
BARO = {"rate": 50.0, "latency": 0.075, "noise": 0.08, "drift": 0.35, "drift_tau": 60.0, "wash": 0.0, "wash_height": 0.3, "kick": 0.8, "kick_tau": 0.2}
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
    """
    The matrix (or its transpose) times the vector. Each component is a sum() of its three products, which rounds as Python's compensated float sum does, not as chained additions would.
    """
    x, y, z = vector
    if transpose:
        return [sum((matrix[0][i] * x, matrix[1][i] * y, matrix[2][i] * z)) for i in range(3)]
    return [sum((row[0] * x, row[1] * y, row[2] * z)) for row in matrix]


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
    # Range noise (along the line of sight) is the tag size's pixel error; across the image the tag centre is far more precise ("lateral_noise", default the same).
    distance = math.sqrt(x * x + y * y + z * z)
    spread, lateral = camera["noise"] * distance, camera.get("lateral_noise", camera["noise"]) * distance
    right, down, ahead = right + rng.gauss(0, lateral), down + rng.gauss(0, lateral), ahead + rng.gauss(0, spread)
    cc, sc = math.cos(cam_roll), math.sin(cam_roll)
    return [cc * right + sc * down, -sc * right + cc * down, ahead]


def run(gains, plant, seconds=10.0, seed=0, log=None, hover=False):
    """
    Simulate one flight. A log list receives (time, position, roll/pitch/yaw, estimated roll/pitch, servo tilts, motor thrusts, pitch target, tag seen, with hover the hover's own tag-relative position estimate, arc and range setpoint as the drone logs them, the roll target, the test pattern's part of it, and the IMU as the drone reads it: filtered gyro in rad/s and specific force in g, body forward, left, up) every control step.
    With hover and plant["land_at"], the hover loop is switched to its landing (HOVER_LAND, or HOVER_LAND_FAST with plant["land_fast"]) that many seconds after liftoff, as the dashboard's L does; the result then also holds when it came down and how.
    With hover, the drone arms on the ground and the firmware's hover loop (fw/drone/main/hover.c, gains from the hover_* settings) flies it from takeoff on the plant's camera (CAMERA, overridden by plant["camera"]; none with hover_camera 0, and none from plant["camera_lost"] seconds after liftoff), glued to the attitude controller as flight_rt.c does.
    With plant["airframe"] (parameters fitted by sw/sim/identify_airframe.py, at the geometry they name), sw/sim/airframe.py supplies the motors, servos and angular accelerations and the body's inertia instead of the plant's own actuator and torque model; the rest (translation, ground contact, sensors, camera) stays.
    With hover and plant["excite"] (a test pattern as a recording's "pattern" event, sw/excite.py: {"after", "repeat", "segments"}), its roll and pitch steps add to the hover's attitude targets and its forward and left speeds move the hold point (on top of plant["nudge"], within NUDGE_MAX) from "after" seconds past collective first reaching integrate_throttle, "repeat" times (0 = until the end), as flight_rt.c excite does; its other channels are ignored.
    GPS (fw/drone/main/flight_rt.c gps_input): plant["gps"] = {"rate" (Hz), "latency" (s, measurement to arrival; the firmware assumes 0.06), "velocity_noise" and "vertical_noise" (m/s RMS, white), "position_noise" (m RMS) wandering over "position_tau" (s), "liftoff_descent" (a range of m/s, drawn per run: the climb rate it reads instead of the truth for "liftoff_s" s from liftoff), "heading": where the world's forward axis (the drone's nose at takeoff) points, degrees clockwise from north} gives the hover each fix once; settings hover_gps_tau (and hover_gps_heading, the drone's belief about that heading) turn it on.
    Barometer (fw/drone/main/flight_rt.c baro_input): plant["baro"] = {"rate" (Hz), "latency" (s, measurement to the hover; the firmware assumes 0.075), "noise" (m RMS, white per reading), "drift" (m RMS wandering over "drift_tau" s: weather and the chip's warming), "wash" (m the rotors' wash and ground effect add while the motors are commanded, fading to none at "wash_height" m above the start), "kick" (m RMS, drawn per run: the liftoff's transient, decaying over "kick_tau" s)} gives the hover each reading once, and at rest too (its ground); setting hover_baro_tau turns it on.
    Optical flow (fw/drone/main/flight_rt.c flow_input): plant["flow"] = {"depth": the scene's true distance along the camera (m), "noise_px": per-frame shift noise (full-resolution px), "fps", "latency" (s), "max_px": the largest shift per frame and axis the camera's block search finds (full-resolution px; fw/esp32cam/src/flow.h reach, about 14; none by default)} gives the hover the camera's flow, the mean image motion over each frame interval with the mean measured gyro over it taken out as the firmware does; a frame whose shift exceeds max_px on either axis gives none; settings hover_flow_depth and hover_flow_tau turn it on.
    IMU errors for the hover's estimators: plant["accel_bias"] (g; forward, left) on the accelerometer, "flight_accel_bias" (g, along body up) on it only while the motors are commanded (vibration rectification and the like, which the resting calibration of the vertical specific force cannot learn), "vibration" (g RMS, white, while the motors are commanded; one value or forward, left, up), and "level_error" (degrees; roll, pitch) in the level reference the drone flies and turns its specific force by. Rotor drag: "rotor_drag" (1/s; one value, or forward and left) pulls the body's horizontal velocity relative to the air. The air: "wind_mean" (m/s; world forward, left) blowing steadily, plus turbulence of "wind" (m/s RMS per horizontal axis) and "wind_vertical" (m/s RMS up) with time constants "wind_tau" and "wind_vertical_tau" (s, the latter defaulting to the former; outdoors about the turbulence length scale over the mean wind: near the ground about 15 m horizontally and the height vertically, MIL-F-8785C); the airframe's speed terms (its forward and sideways speed moments and drag) and the bluff-body drag go by the airspeed, and "vertical_drag" (1/s) pulls the vertical velocity toward the air's (the rotors' thrust change with their inflow: about g / (2 v_induced), 0.7/s for these rotors at hover; 0 by default, as the indoor calibrations had it); "tilt_offset" (degrees) and "tilt_gain" make the servos' physical forward tilt differ from their commands. "disturbances" is a list of force and torque terms acting once airborne, any mix of constant, drifting, white, periodic and ramping (sw/sim/disturbance.py); with "sensed_disturbances" the accelerometer feels their forces.
    """
    rng = random.Random(seed)
    accel_rng = random.Random(seed + 1000)
    camera_rng = random.Random(seed + 2000)
    gust_rng = random.Random(seed + 3000)
    wind_rng = random.Random(seed + 4000)
    settings = MODEL | {"roll": 0, "pitch": 0, "yaw_rate": 0, "pitch_trim": 0, "left_trim": 0, "right_trim": 0,
                               "roll_bias": 0, "pitch_bias": 0, "yaw_bias": 0, "left_thrust_scale": 1, "right_thrust_scale": 1,
                               "servo_compensation": 0.75, "integrate_throttle": 580, "target_rate": 360, "velocity_damping": 8, "velocity_leak": 3} | gains
    mass = plant.get("mass", MASS)
    weight = mass * 9.81
    k = plant["thrust_k"]
    hover_throttle = round(throttle(weight / 2 / k))
    # A hover takes off from the ground unless plant["liftoff"] is False: then it is engaged in the air, as pressing H in a manual flight does.
    liftoff = plant.get("liftoff", hover)
    settings["base_throttle"] = TAKEOFF_THROTTLE if liftoff else hover_throttle
    controller = Controller()

    def configure():
        controller.configure(settings | {"enabled": 1})

    configure()
    controller.reset()
    controller.command[YAW_I] = settings.get("yaw_i", 0.0)
    # Constant pitch angular acceleration not in the controller's model (servo zero or CoM error), rad/s², positive nose-down.
    disturbance = plant.get("disturbance", 0.0)
    # Pitch angular acceleration per rad/s of pitch rate, per radian of pitch beyond pitch_trim and per m/s of forward airspeed (rad/s² each; positive nose-down, so a negative velocity_moment is the rotors' flapping back in forward flight), and a random one (rad/s² RMS, Ornstein-Uhlenbeck with time constant gust_tau).
    rate_moment, angle_moment, velocity_moment = plant.get("rate_moment", 0.0), plant.get("angle_moment", 0.0), plant.get("velocity_moment", 0.0)
    gust_size, gust_keep, gust = plant.get("gust", 0.0), math.exp(-DT / plant.get("gust_tau", 0.5)), 0.0
    # Roll angular acceleration not in the model (rad/s², positive raising the left side): a constant one (thrust imbalance the roll bias cancels) and a random one like the pitch gust.
    roll_disturbance, roll_gust_size, roll_gust = plant.get("roll_disturbance", 0.0), plant.get("roll_gust", 0.0), 0.0
    roll_rng = random.Random(seed + 5000)
    trim = math.radians(settings["pitch_trim"])
    rotor_drag, wind_size, wind_keep = plant.get("rotor_drag", 0.0), plant.get("wind", 0.0), math.exp(-DT / plant.get("wind_tau", 2.0))
    wind_mean = list(plant.get("wind_mean", (0.0, 0.0))) + [0.0]
    wind_vertical, vertical_drag = plant.get("wind_vertical", 0.0), plant.get("vertical_drag", 0.0)
    ground_effect = plant.get("ground_effect")
    vertical_keep = math.exp(-DT / plant.get("wind_vertical_tau", plant.get("wind_tau", 2.0)))
    turbulence = [0.0, 0.0, 0.0]
    wind = list(wind_mean)
    drag_x, drag_y = rotor_drag if isinstance(rotor_drag, (list, tuple)) else (rotor_drag, rotor_drag)
    disturbances = Disturbances(plant.get("disturbances"), seed)
    # plant["sensed_disturbances"]: the disturbances' forces are real forces (wind, the room's flow), which the accelerometer feels; otherwise they move the drone unfelt, as the earlier calibrations fitted them.
    sensed = plant.get("sensed_disturbances", False)
    accel_bias = list(plant.get("accel_bias", (0.0, 0.0))) + [0.0]
    flight_accel_bias = plant.get("flight_accel_bias", 0.0)
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
    matrix = rotation(*angles)
    lowest = min(apply(matrix, foot)[2] for foot in feet)
    position = [0.0, 0.0, -lowest - weight / len(feet) / GROUND_K if liftoff else 50.0]
    start_height = position[2]
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
    specific = apply(matrix, [0.0, 0.0, 1.0], transpose=True)
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
        configure()
    commanded = (0.0, 0.0, 0.0, 0.0)
    airframe = None
    if plant.get("airframe"):
        from sw.sim.airframe import Airframe
        airframe = Airframe(plant["airframe"])
        ground_by = plant["airframe"].get("ground_by")
        # The airframe's accelerations come at its geometry; the body takes the other torques (contact, disturbances) with the same inertia.
        inertia = [airframe.geometry[key] for key in ("roll_inertia", "pitch_inertia", "yaw_inertia")]
        airframe.reset(0 if liftoff else hover_throttle)
        motor = list(airframe.motor)
        # Models with tilt_push carry the share of the tilted thrust's horizontal force that moves the drone (the rest its slipstream cancels on the airframe); older ones move it with all of it.
        tilt_share = airframe.p["tilt_push"] if "tilt_push" in plant["airframe"] else 1.0
    else:
        tilt_share = 1.0
    ideal_height, nudge, camera_lost, land_at, land_fast = plant.get("ideal_height"), plant.get("nudge"), plant.get("camera_lost"), plant.get("land_at"), plant.get("land_fast")
    gyro_noise, slew_rate = math.radians(plant["gyro_noise"]), math.radians(plant["servo_slew"])
    # Narrowband gyro vibration (plant["gyro_vibration"]: RMS deg/s per axis at "hz" with "bandwidth" Hz, as the 100 Hz control samples see it): a resonant AR(2) driven by white noise and scaled to that RMS. The rotors shake the pitch gyro without moving the attitude; through the D term it jitters the servos.
    shake = plant.get("gyro_vibration")
    if shake:
        radius, angle = math.exp(-math.pi * shake["bandwidth"] * DT), 2 * math.pi * shake["hz"] * DT
        a1, a2 = 2 * radius * math.cos(angle), -radius * radius
        unit = math.sqrt((1 - a2) / ((1 + a2) * ((1 - a2) ** 2 - a1 * a1)))
        shake_size = [math.radians(value) / unit for value in shake["rms"]]
    shaking = [[0.0, 0.0] for _ in range(3)]
    shake_rng = random.Random(seed + 6000)
    places, touched = [], None
    # Landing (plant["land_at"]): when the hover's touchdown flag set and collective reached zero (s after liftoff), the descent speed, the horizontal speed and the largest tilt from level in the 1 s before and after the feet came back.
    landing = {"landed_s": None, "off_s": None, "contact_speed": None, "contact_drift": None, "contact_tilt": 0.0}
    if hover:
        from sw.sim.hover import HOLD, LAND, LAND_FAST, Hover, Pose
        use_camera = settings.get("hover_camera", 1)
        if not use_camera:
            settings["hover_center"] = 0
        outer = Hover(settings)
        camera = CAMERA | plant.get("camera", {})
        cam_pitch, cam_roll = math.radians(settings.get("hover_cam_pitch", 0.0)), math.radians(settings.get("hover_cam_roll", 0.0))
        # Frames waiting out their latency (ready time, capture time, sighting or None), the newest delivered one, and the next capture time.
        frames, newest, capture, sequence, missing_frames = deque(), None, 0.0, 0, 0
        flow_plant, flows, flow_capture, flow_rng = plant.get("flow"), deque(), 0.0, random.Random(seed + 4000)
        # Each tick's true rotation and velocity in camera axes and measured rotation, for the means over a frame interval.
        flow_ticks = deque(maxlen=16)
        gps_plant, gps_fixes, gps_capture, gps_rng = plant.get("gps"), deque(), 0.0, random.Random(seed + 6000)
        gps_error = [0.0, 0.0, 0.0]
        baro_plant, baro_readings, baro_capture, baro_rng, baro_drift = plant.get("baro"), deque(), 0.0, random.Random(seed + 8000), 0.0
        baro_kick = baro_rng.gauss(0, baro_plant.get("kick", 0.0)) if baro_plant else 0.0
        # The GPS's false descent at liftoff (plant["gps"]["liftoff_descent"]: a range in m/s, drawn per run, read for "liftoff_s" s from liftoff instead of the climb).
        gps_descent = random.Random(seed + 6500).uniform(*gps_plant["liftoff_descent"]) if gps_plant and gps_plant.get("liftoff_descent") else 0.0
        seen, ranges = [], []
    # The test pattern's roll, pitch and hold-point speeds per control tick, the step collective first reached integrate_throttle, and how far the pattern has moved the hold point (m; toward the tag, left).
    excite = plant.get("excite") if hover else None
    pattern = [tuple(segment.get(key, 0.0) for key in ("roll", "pitch", "forward", "left")) for segment in excite["segments"] for _ in range(segment["ticks"])] if excite else []
    integrating, excited, excite_hold = None, 0.0, [0.0, 0.0]
    for step in range(int(seconds / DT)):
        now = step * DT
        if armed and step == armed and not hover:
            settings["base_throttle"] = TAKEOFF_THROTTLE
            configure()
        if liftoff and not hover and lifted is None and now < ramp:
            settings["base_throttle"] = round(TAKEOFF_THROTTLE * (step + 1) * DT / ramp)
            configure()
        if not hover and lifted is not None and now - lifted >= CLIMB and settings["base_throttle"] != hover_throttle:
            settings["base_throttle"] = hover_throttle
            configure()
        noise = [rng.gauss(0, gyro_noise) for _ in range(3)]
        if shake:
            for axis in range(3):
                shaking[axis] = [a1 * shaking[axis][0] + a2 * shaking[axis][1] + shake_size[axis] * shake_rng.gauss(0, 1), shaking[axis][0]]
                noise[axis] += shaking[axis][0]
        gyro_filter = [g + 0.7 * (r + n - g) for g, r, n in zip(gyro_filter, rate, noise)]
        # The rotors' vibration and the in-flight bias only while the motors are commanded: at rest the drone learns its resting specific force from a quiet accelerometer.
        spinning = commanded[0] + commanded[1] > 0
        bias = accel_bias[:2] + [flight_accel_bias if spinning else 0.0]
        accel_filter = [a + 0.7 * (s + b + accel_rng.gauss(0, ACCEL_NOISE) + (accel_rng.gauss(0, v) if v and spinning else 0.0) - a) for a, s, b, v in zip(accel_filter, specific, bias, vibration)]
        attitude = estimator.update(accel_filter, gyro_filter, DT, powered=True)
        # The drone flies relative to its level reference: an error in it offsets the attitude both loops see.
        attitude = (attitude[0] - level_error[0], attitude[1] - level_error[1])
        # The body-to-world matrix until the physics substeps move the body.
        matrix = rotation(*angles)
        visible = None
        if hover:
            roll_estimate, pitch_estimate = attitude
            vertical = -accel_filter[0] * math.sin(pitch_estimate) + math.cos(pitch_estimate) * (accel_filter[1] * math.sin(roll_estimate) + accel_filter[2] * math.cos(roll_estimate))
            if ideal_height:
                # A study of the horizontal loop alone: the true vertical specific force, so the IMU-only height hold cannot wander off (the measured one, projected with the estimated attitude, drifts by centimetres per second).
                vertical = apply(matrix, specific)[2]
            while capture <= now:
                # Detector misses come in bursts (blur, glare): a frame starts one with probability camera["miss"], and each burst lasts camera["burst"] frames on average.
                missing = missing_frames > 0 or camera_rng.random() < camera.get("miss", 0.0)
                missing_frames = missing_frames - 1 if missing_frames > 0 else (round(camera_rng.expovariate(1 / camera.get("burst", 1))) if missing else 0)
                frames.append((capture + camera["latency"], capture, None if missing else sighting(camera, position, angles, cam_pitch, camera_rng, cam_roll)))
                capture += 1 / camera["fps"]
            while frames and frames[0][0] <= now:
                newest = frames.popleft()
                sequence += 1
            flow = None
            if flow_plant:
                period = 1 / flow_plant.get("fps", 43.0)
                flow_ticks.append((now, outer.body_to_camera(rate), outer.body_to_camera(apply(matrix, velocity, transpose=True)), outer.body_to_camera(gyro_filter)))
                while flow_capture <= now:
                    # The true image motion of a scene at depth over the frame interval: the body's rotation sweeps the centre (-W_down, +W_right) and its velocity across the camera (-v / depth); the shift gets pixel noise; the firmware takes the mean measured rotation over the interval back out (flight_rt.c flow_input).
                    ticks = [tick for tick in flow_ticks if flow_capture - period < tick[0] <= flow_capture] or [flow_ticks[-1]]
                    w, v, measured = [[sum(tick[part][axis] for tick in ticks) / len(ticks) for axis in range(3)] for part in (1, 2, 3)]
                    depth = flow_plant["depth"]
                    shift = [(-w[1] - v[0] / depth) * 370.0 * period + flow_rng.gauss(0, flow_plant.get("noise_px", 0.3)),
                             (w[0] - v[1] / depth) * 370.0 * period + flow_rng.gauss(0, flow_plant.get("noise_px", 0.3))]
                    if not flow_plant.get("max_px") or max(abs(shift[0]), abs(shift[1])) <= flow_plant["max_px"]:
                        # Stamped at the interval's middle, as flow_input stamps it.
                        flows.append((flow_capture + flow_plant.get("latency", 0.045), flow_capture - period / 2,
                                      (shift[0] / 370.0 / period + measured[1], shift[1] / 370.0 / period - measured[0])))
                    flow_capture += period
                while flows and flows[0][0] <= now:
                    _, captured, translational = flows.popleft()
                    flow = (round((captured + 1) * 1e6), translational)
            gps = None
            if gps_plant:
                period = 1 / gps_plant.get("rate", 10.0)
                while gps_capture <= now:
                    # The true velocity and position in north, east, up (the world's forward axis points gps_plant["heading"] degrees clockwise from north; left is 90 degrees counterclockwise of it), with white velocity noise and a slowly wandering position error (Ornstein-Uhlenbeck, position_noise m RMS over position_tau s; height twice that).
                    psi = math.radians(gps_plant.get("heading", 0.0))
                    keep = math.exp(-period / gps_plant.get("position_tau", 30.0))
                    sizes = [gps_plant.get("position_noise", 0.7)] * 2 + [2 * gps_plant.get("position_noise", 0.7)]
                    gps_error = [keep * e + s * math.sqrt(1 - keep * keep) * gps_rng.gauss(0, 1) for e, s in zip(gps_error, sizes)]
                    north = lambda x, y: x * math.cos(psi) + y * math.sin(psi)
                    east = lambda x, y: x * math.sin(psi) - y * math.cos(psi)
                    noise = gps_plant.get("velocity_noise", 0.05)
                    climb = -gps_descent if lifted is not None and now - lifted < gps_plant.get("liftoff_s", 0.4) else velocity[2]
                    speed = [north(*velocity[:2]) + gps_rng.gauss(0, noise), east(*velocity[:2]) + gps_rng.gauss(0, noise), climb + gps_rng.gauss(0, gps_plant.get("vertical_noise", 0.1))]
                    place = [north(*position[:2]) + gps_error[0], east(*position[:2]) + gps_error[1], position[2] + gps_error[2]]
                    gps_fixes.append((gps_capture + gps_plant.get("latency", 0.06), speed, place))
                    gps_capture += period
                while gps_fixes and gps_fixes[0][0] <= now:
                    arrived, speed, place = gps_fixes.popleft()
                    # Stamped as flight_rt.c gps_input does: its arrival less the latency it assumes.
                    gps = (round((arrived - 0.06 + 1) * 1e6), speed, place)
            baro = None
            if baro_plant:
                period = 1 / baro_plant.get("rate", 50.0)
                while baro_capture <= now:
                    # The pressure altitude (an arbitrary 100 m above sea level at the start) with white noise, a slowly wandering error, and the rotors' wash near the floor.
                    keep = math.exp(-period / baro_plant.get("drift_tau", 60.0))
                    baro_drift = keep * baro_drift + baro_plant.get("drift", 0.3) * math.sqrt(1 - keep * keep) * baro_rng.gauss(0, 1)
                    above = position[2] - start_height
                    wash = baro_plant.get("wash", 0.0) * max(0.0, 1 - above / baro_plant.get("wash_height", 0.5)) if commanded[0] + commanded[1] > 0 else 0.0
                    kick = baro_kick * math.exp(-(baro_capture - lifted) / baro_plant.get("kick_tau", 0.2)) if lifted is not None and baro_capture >= lifted else 0.0
                    baro_readings.append((baro_capture + baro_plant.get("latency", 0.075), 100.0 + above + baro_drift + wash + kick + baro_rng.gauss(0, baro_plant.get("noise", 0.15))))
                    baro_capture += period
                while baro_readings and baro_readings[0][0] <= now:
                    arrived, altitude = baro_readings.popleft()
                    # Stamped as flight_rt.c baro_input does: its arrival less the latency it assumes.
                    baro = (round((arrived - 0.075 + 1) * 1e6), altitude)
            # Like camera_candidates: the newest delivered frame's tag, if it saw one and was captured at most 150 ms ago.
            pose = None
            lost = camera_lost is not None and lifted is not None and now - lifted >= camera_lost
            if use_camera and not lost and newest and newest[2] and now - newest[1] <= 0.15:
                pose = Pose(sequence=sequence, capture_us=round((newest[1] + 1) * 1e6), tag=tuple(newest[2]))
            visible = pose is not None
            if step < armed:
                outer.rest(vertical, False, DT, baro)
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
                if nudge:
                    # (start s after liftoff, toward-tag and left speeds in m/s, seconds): the hold point moving as the dashboard's held arrows move it.
                    start, forward_speed, left_speed, length = nudge
                    # From liftoff, or from the start for a hover engaged in the air.
                    since = now - lifted if lifted is not None else (now if not liftoff else None)
                    moving = 0.0 if since is None else min(max(since - start, 0.0), length)
                    outer.gains.nudge_forward, outer.gains.nudge_left = forward_speed * moving, left_speed * moving
                    # plant["nudge_wrap"] (m): wrapped at that limit, as the dashboard does without the tag.
                    if plant.get("nudge_wrap"):
                        limit = plant["nudge_wrap"]
                        outer.gains.nudge_forward, outer.gains.nudge_left = [(value + limit) % (2 * limit) - limit for value in (outer.gains.nudge_forward, outer.gains.nudge_left)]
                if plant.get("stick"):
                    # (start s after liftoff, pitch and roll in degrees, seconds), or a list of them for a pilot's sequence: the laptop's tilt held without the tag, as the dashboard's arrows and the gamepad's right stick send it.
                    sticks = plant["stick"] if isinstance(plant["stick"][0], (list, tuple)) else [plant["stick"]]
                    outer.gains.stick_pitch, outer.gains.stick_roll = 0.0, 0.0
                    for start, stick_pitch, stick_roll, length in sticks:
                        if lifted is not None and start <= now - lifted < start + length:
                            outer.gains.stick_pitch, outer.gains.stick_roll = math.radians(stick_pitch), math.radians(stick_roll)
                if pattern:
                    # As flight_rt.c: on top of the laptop's nudge, a tick late.
                    outer.gains.nudge_forward = (outer.gains.nudge_forward if nudge else 0.0) + excite_hold[0]
                    outer.gains.nudge_left = (outer.gains.nudge_left if nudge else 0.0) + excite_hold[1]
                mode = HOLD if land_at is None or lifted is None or now - lifted < land_at else LAND_FAST if land_fast else LAND
                out = outer.update(mode, pose, round((now + 1) * 1e6), DT, roll_estimate, pitch_estimate, heading_rate, vertical, flying, drag, flow, gps, baro)
                settings["base_throttle"] = min(HOVER_THROTTLE, throttle(out.force / 2 / math.cos(trim))) if out.force > 0 else 0
                settings["roll"], settings["pitch"], settings["yaw_rate"] = math.degrees(out.roll), math.degrees(out.pitch), math.degrees(out.yaw_rate)
                excited = 0.0
                if pattern:
                    if integrating is None and settings["base_throttle"] >= settings["integrate_throttle"]:
                        integrating = step
                    tick = -1 if integrating is None else step - integrating - round(excite["after"] / DT)
                    if 0 <= tick and (not excite.get("repeat", 1) or tick < len(pattern) * excite["repeat"]):
                        excited = pattern[tick % len(pattern)][0]
                        settings["roll"] += excited
                        settings["pitch"] += pattern[tick % len(pattern)][1]
                        excite_hold = [min(max(held + speed * DT, -NUDGE_MAX), NUDGE_MAX) for held, speed in zip(excite_hold, pattern[tick % len(pattern)][2:])]
                configure()
                # As flight_rt.c: the speed moment compensation only while the hold flies on the tag, whose velocity it takes.
                controller.command[PITCH_SPEED] = settings.get("pitch_speed", 0.0) if out.tracking else 0.0
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
            gust = gust_keep * gust + gust_size * math.sqrt(1 - gust_keep * gust_keep) * gust_rng.gauss(0, 1)
        sampled = disturbances.step(DT)
        extra_force, extra_torque = sampled[:3], sampled[3:]
        if roll_gust_size:
            roll_gust = gust_keep * roll_gust + roll_gust_size * math.sqrt(1 - gust_keep * gust_keep) * roll_rng.gauss(0, 1)
        if wind_size or wind_vertical:
            turbulence = [keep * w + size * math.sqrt(1 - keep * keep) * wind_rng.gauss(0, 1) for w, size, keep in zip(turbulence, (wind_size, wind_size, wind_vertical), (wind_keep, wind_keep, vertical_keep))]
            wind = [m + t for m, t in zip(wind_mean, turbulence)]
        clearance = position[2] + min(apply(matrix, foot)[2] for foot in feet)
        if airframe:
            body_velocity = apply(matrix, [v - w for v, w in zip(velocity, wind)], transpose=True)
            tilt_commands = [math.degrees(left_angle - servo_zero[0]) * MODEL["left_servo_direction"], math.degrees(right_angle - servo_zero[1]) * MODEL["right_servo_direction"]]
            # The near-ground pitch term goes by the lowest foot's height (as the logged height above the engage point) or the time since liftoff, as it was fitted.
            near = None if not ground_by else max(0.0, clearance) if ground_by == "height" else (now - lifted if lifted is not None else 0.0) if liftoff else 10.0
            accel, motor, servo = airframe.step((left, right), tilt_commands, 1.0, rate, angles[:2], body_velocity[:2], airborne=lifted is not None or not liftoff, near=near)
            motor, servo = list(motor), list(servo)
        substeps = AIR_SUB if clearance > CLEAR + math.hypot(*velocity) * DT else SUB
        # Ground effect (plant["ground_effect"] = {"radius": rotor radius, "rotor": the rotors' height above the body's lowest point, m}): Cheeseman and Bennett's 1 / (1 - (R / 4z)^2) at the rotors' height z, at most 1.5.
        cushion = 1.0
        if ground_effect:
            z = max(0.01, clearance + ground_effect["rotor"])
            cushion = 1 / (1 - min(1 / 3, (ground_effect["radius"] / (4 * z)) ** 2))
        h = DT / substeps
        slew = slew_rate * h
        for _ in range(substeps):
            matrix = rotation(*angles)
            # The extra torques act only in flight: on the ground the feet carry them.
            airborne = lifted is not None or not liftoff
            if not airframe:
                for side in range(2):
                    force = thrust(target_throttle[side]) * scales[side]
                    motor[side] += (force - motor[side]) * (1 - math.exp(-h / plant["motor_tau"]))
                    horn = output[side] + max(-slew, min(slew, (target_servo[side] - output[side]) * (1 - math.exp(-h / plant["servo_tau"]))))
                    output[side] = horn
                    if horn - servo[side] > slack[side]:
                        servo[side] = horn - slack[side]
                    elif servo[side] - horn > slack[side]:
                        servo[side] = horn + slack[side]
            vertical = [motor[s] * math.cos(servo[s]) * cushion for s in range(2)]
            forward = [motor[s] * math.sin(servo[s]) * tilt_share * cushion for s in range(2)]
            if airframe:
                # The airframe's accelerations already hold the thrust torques; the random and constant extras still add.
                torque = [accel[0] * inertia[0] + ((roll_disturbance + roll_gust + extra_torque[0]) * inertia[0] if airborne else 0.0), accel[1] * inertia[1] + (gust + disturbance + extra_torque[1] if airborne else 0.0) * inertia[1], accel[2] * inertia[2] + (extra_torque[2] * inertia[2] if airborne else 0.0)]
            else:
                extra = disturbance + gust + (extra_torque[1] if airborne else 0.0) + (rate_moment * rate[1] + angle_moment * (angles[1] - trim) + velocity_moment * apply(matrix, velocity, transpose=True)[0] if airborne else 0.0)
                torque = [
                    arm * (vertical[0] - vertical[1]) + ((roll_disturbance + roll_gust + extra_torque[0]) * inertia[0] if airborne else 0.0),
                    height * (forward[0] + forward[1]) + plant["com_x"] * (vertical[0] + vertical[1]) + extra * inertia[1],
                    arm * (forward[1] - forward[0]) + 0.015 * (vertical[1] - vertical[0]),
                ]
            relative = [v - w for v, w in zip(velocity, wind)] if lifted is not None or not liftoff else velocity
            speed = math.sqrt(sum(v * v for v in relative))
            # Every force except gravity, in the world frame: thrust, drag, then each foot below the floor.
            push = [f - DRAG * speed * v for f, v in zip(apply(matrix, [sum(forward), 0.0, sum(vertical)]), relative)]
            if vertical_drag and (lifted is not None or not liftoff):
                push[2] -= mass * vertical_drag * relative[2]
            if rotor_drag:
                # Blade flapping and induced drag: a force against the airspeed in the rotor plane, proportional to it.
                air = apply(matrix, relative, transpose=True)
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
            felt = [mass * f if lifted is not None and sensed else 0.0 for f in extra_force]
            velocity = [v + (p / mass - (9.81 if axis == 2 else 0.0) + (extra_force[axis] if lifted is not None else 0.0)) * h for axis, (v, p) in enumerate(zip(velocity, push))]
            position = [x + v * h for x, v in zip(position, velocity)]
            specific = [value / weight for value in apply(matrix, [a + b for a, b in zip(push, felt)], transpose=True)]
            rate = [r + t / i * h for r, t, i in zip(rate, torque, inertia)]
            p, q, r = rate
            roll, pitch = angles[0], angles[1]
            angles = [
                roll + (p + math.tan(pitch) * (q * math.sin(roll) + r * math.cos(roll))) * h,
                pitch + (q * math.cos(roll) - r * math.sin(roll)) * h,
                angles[2] + (q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch) * h,
            ]
        if log is not None:
            log.append((now, list(position), list(angles), list(attitude), list(servo), list(motor), math.radians(settings["pitch"]), visible, (list(outer.state.position), outer.state.arc, outer.state.setpoint) if hover else None, math.radians(settings["roll"]), math.radians(excited), list(gyro_filter), list(accel_filter)))
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
                landing["contact_speed"] = -velocity[2]
                landing["contact_drift"] = math.hypot(velocity[0], velocity[1])
            if hover and land_at is not None and now - lifted >= land_at:
                if landing["landed_s"] is None and outer.state.landed:
                    landing["landed_s"] = now - lifted
                if landing["off_s"] is None and settings["base_throttle"] == 0:
                    landing["off_s"] = now - lifted
                if touched is not None and abs(now - lifted - touched) <= 1.0:
                    landing["contact_tilt"] = max(landing["contact_tilt"], math.degrees(max(abs(roll), abs(pitch - trim))))
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
        if land_at is not None:
            result |= landing
    return result


def model(name):
    """A saved model file from sw/sim/models (an airframe's parameters or a list of disturbances)."""
    return json.loads((Path(__file__).with_name("models") / name).read_text())


# Plant as measured (2026-10-21): pivot 20 mm above the CoM, pitch inertia 0.00161 kg m^2, thrust 1.09x the table at ~16 V.
NOMINAL = {"thrust_k": 1.09, "height": 0.0234 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 0.00161 / MODEL["pitch_inertia"], "yaw_inertia": 1.0, "motor_delay": 0.02, "motor_tau": 0.05,
           "servo_delay": 0.04, "servo_tau": 0.06, "backlash": 0.5, "deadband": 0.45, "servo_slew": 600, "imbalance": 0.06, "com_x": 0.002, "gyro_noise": 2.0}
# Rotor drag (0.5 1/s) and accelerometer vibration are from the same flights (sw/fit_drag.py): logged hover vibration is 0.15 g broadband horizontally, but averaged over 0.5-1.5 s (what the drag model sees) it is as low as white noise of about 0.1 g before the 0.7 smoothing here; 0.05 g vertically.
# Fitted to the 2026-10-25 hover flights (08-52 to 09-15) by closed-loop multi-step pitch prediction from replayed controller states: effectiveness 1.42x the controller's model (pivot_height 0.03, pitch_inertia 0.0023), 70 ms dead time + 10 ms lag at 340°/s, a -10.6 rad/s² tail-heavy push, a weak restoring angle and forward-speed moment, and an unexplained slow pitch disturbance of about 5 rad/s² RMS. Backlash and deadband were not identifiable (fits went to zero), so play here is the bench's "small, left sloppier". It reproduces the 09-01 pitch_damping 14 oscillation (0.38-0.40 s period vs 0.41 s logged) and damping 10 staying clear of it. Fly it with run(..., hover=True) and a recording's settings.
FITTED = {"thrust_k": 1.03, "mass": 0.86, "height": 1.42 * 0.03 / MODEL["pivot_height"], "roll_inertia": 1.0, "pitch_inertia": 1.0, "yaw_inertia": 1.0,
          "motor_delay": 0.02, "motor_tau": 0.05, "servo_delay": 0.07, "servo_tau": 0.01, "backlash": 0.3, "left_backlash": 0.5, "right_backlash": 0.3,
          "deadband": 0.1, "servo_slew": 340, "imbalance": 0.03, "com_x": 0.0, "disturbance": -10.6, "angle_moment": -3.9, "velocity_moment": -4.0,
          "gust": 5.0, "gust_tau": 0.7, "gyro_noise": 2.0, "camera": {"tag": (2.3, 0.0, 0.3)}, "rotor_drag": 0.5, "vibration": (0.1, 0.1, 0.05)}
# Identified from the 34 hover flights of 2026-10-27 (both rubber bands on, the ESCs' earlier range: fly it with flight.PRE_RECALIBRATION_TABLE in mind) by closed-loop multi-step prediction with the firmware controller in the loop (sw/sim/identify_airframe.py, parameters in models/airframe_2026-10-27.json, refit 2026-10-28 in the geometry-explicit form). Held out by flight it predicts roll to 1.3° and pitch to 3.7° a second ahead (yaw rate R² 0.55) from the logged references alone. Rotor drag is from the camera velocity over 0.25-1 s against thrust through the logged attitude and modelled tilts (gain within 10 % of 1). The random disturbance is what of the one-step residual lies below 2 Hz (the rest, mostly 10-50 Hz in pitch, does not move the attitude): 0.89 rad/s² RMS in roll and 0.49 in pitch at 0.5 s.
AIRFRAME = model("airframe_2026-10-27.json")
# Refit 2026-10-28 on the four hovers after both ESCs were recalibrated (06:54-07:01 UTC, 170 s, the rescaled thrust table), at models/geometry_2026-10-28.json, and calibrated to the camera hovers' spectra (sw/sim/calibrate_sim.py); roll authority is poorly pinned by hover (see models/NOTES.md).
AIRFRAME_ESC = model("airframe_2026-10-28.json")
IDENTIFIED = FITTED | {"airframe": AIRFRAME, "mass": 0.94, "thrust_k": 1.0, "height": 1.0, "disturbance": 0.0, "angle_moment": 0.0, "velocity_moment": 0.0, "gust": 0.49, "gust_tau": 0.5, "roll_gust": 0.89,
                       "camera": {"tag": (2.0, 0.1, 0.45), "miss": 0.006, "burst": 6}, "rotor_drag": (0.6, 0.27)}
# The ESC-recalibrated drone as flown on 2026-10-28 (see AIRFRAME_ESC); IDENTIFIED_ESC below is the current one.
# Its disturbances are sw/sim/fit_disturbances.py's terms (models/disturbances_2026-10-28.json: a steady -1.09 m/s² push away from the tag, drifting forces and torques), sized so the sim's closed-loop hover statistics match the real ones; the horizontal drag is half the airframe fit's (which was fitted to the drone's own velocity estimate). Flying each hover's settings it matches range error RMS 0.21 against 0.20 m, its fading time 1.8 against 2.0 s, sideways 0.18 against 0.17 m fading in 1.9 against 1.8 s, pitch rate 22.6 against 23.4 deg/s and roll rate 9.0 against 9.7.
# Its pitch gyro carries the rotors' vibration as the camera hovers logged it: 10 deg/s RMS in a 17-25 Hz band (roll and yaw under 1 deg/s).
IDENTIFIED_ESC_1028 = IDENTIFIED | {"airframe": AIRFRAME_ESC, "rotor_drag": (0.58, 0.34), "gust": 0.0, "roll_gust": 0.0, "gyro_vibration": {"rms": (0.0, 10.0, 0.0), "hz": 21.0, "bandwidth": 8.0},
                                    "disturbances": model("disturbances_2026-10-28_calibrated.json")}
# IDENTIFIED_ESC_1029, the drone as flown on the night of 2026-10-29 (05:53-06:18 UTC, after a CoM change; see models/NOTES.md): the roll terms refit on those hovers and the ±3° PRBS flight 06-17-44 (models/airframe_2026-10-29.json; pitch, servos and slipstream as on 2026-10-28), the 2026-10-28 disturbances with smaller drifts (roll ×0.35, pitch ×0.5, sideways ×0.7) and sideways rotor drag 0.2/s instead of 0.34, which together match tonight's camera hovers' statistics and the PRBS flight's growing 0.4 Hz sideways swing. Tonight's right motor may have been slipping (two thrust losses after 06:25), so its roll authority may be degraded. It gets hover_kd backwards (at 2.0 it sways where the real drone was calm): do not tune hover_kd on it.
IDENTIFIED_ESC_1029 = IDENTIFIED_ESC_1028 | {"airframe": model("airframe_2026-10-29.json"), "rotor_drag": (0.58, 0.2), "disturbances": model("disturbances_2026-10-29_calibrated.json")}
# Rooms: what of the plant belongs to where the drone flies, not to the airframe: the drifting disturbances (sw/sim/disturbance.py, calibrated to that room's camera hovers), the rotor drag as calibrated there (it stands partly for the room's flow: on 2026-10-29 the camera measures almost no drag) and the camera's tag. The hover height comes with each flight's settings and the steady push with each flight (the push observer learns it). The 2026-10-28 hovers (06:54-07:01, about 0.8 m up) and the 2026-10-29 ones (about 0.45 m) were flown in different rooms.
ROOMS = {"2026-10-28": {"disturbances": IDENTIFIED_ESC_1028["disturbances"], "rotor_drag": (0.58, 0.34), "camera": IDENTIFIED["camera"]},
         "2026-10-29": {"disturbances": IDENTIFIED_ESC_1029["disturbances"], "rotor_drag": (0.58, 0.2), "camera": IDENTIFIED["camera"]}}


def in_room(airframe, room):
    """The hover-sim plant for an airframe (parameters as saved by identify_airframe) flown in one of ROOMS."""
    return IDENTIFIED_ESC_1028 | {"airframe": airframe} | ROOMS[room]


# The current plant, 2026-10-29b (see models/NOTES.md): the airframe refit on every 9-inch flight after the CoM change (05:53-07:43, runaways cut, the ESC swap at 06:45 followed) with the roll actuator pinned by the PRBS flights (table authority, about 11 ms dead time and 50 ms lag) and no roll attitude spring, an ESC start-up time from standstill and the servo tilt's measured forward push; the sideways speed's roll moment (loose in the fit) at the 2026-10-29 fit's value, and the 2026-10-29 disturbances and rotor drag (recalibrating them to the hovers' statistics traded that moment against the roll drift and broke the PRBS and hover_kd checks). It flies hover_kd the right way round (calm at 2.0, the 0.4 Hz swing at 0.81 that the ±3° PRBS grows). Its liftoff kick peaks too late at pitch_bias 3.9 (0.4-0.6 s against 0.2 s).
IDENTIFIED_ESC = in_room(model("airframe_2026-10-29b.json"), "2026-10-29")
# The 10-inch drone as the camera measured it on 2026-10-31 (the tag hover 20:57 UTC): almost no forward rotor drag (0.04/s, its drag model's forward reading uncorrelated with the real speed) and about 0.1/s sideways, the room's drifts as real forces the accelerometer feels, and no steady room push (each flight's push depends on where the CoM sits; give one with a constant or ramping pitch term). Fly tag-free hovers and manual flights on this one: the earlier rooms' 0.58/s forward drag let the drag model see a speed the real drone cannot.
# Its airframe carries no steady push of its own (the zero-fit's were the 10-29 flights' CoM): a centred CoM, as the redesign intends; scenarios add an offset (a constant pitch term) or a slide (a ramp). Even so a tag-free hover on it runs away forward (9.5 m in 8 s from takeoff, the servos holding 8 degrees forward): the forward speed's nose-up moment (drag_z_x, the rotors flapping back, fitted on the 10-29 hovers at their small speeds) grows with a speed the drag model cannot see, and the forward tilt that holds it adds speed; without that moment it drifts 2 m. The real tag-free hops were stopped within seconds of drifting, so nothing yet says which is right at higher speed.
# Until 2026-11-08 its airframe was the 10-inch zero-fit (airframe_2026-10-29_10in_zerofit.json): that and the 2026-11-02 fit flew drone 1's hop settings calm (pitch 18-30 degrees peak to peak, decaying) where the real hops swung at 0.6-0.9 Hz to 20-48 degrees, and did not shake at pitch_damping 8 (5-8 against the real 67 deg/s in 2.2-3.5 Hz).
# Now drone 1 as flown on 2026-11-02 and 2026-11-08 (models/airframe_2026-11-08.json, models/NOTES.md 2026-11-08): the pitch path calibrated to the hops' closed-loop pitch spectra (sw/sim/calibrate_pitch.py: a fast, overdamped servo with a heavier reflected inertia and a little compliance, the forward speed's moment, the held tilt's lagged share), the rest refit on every 10-inch flight. Its mass is the model's geometry's (1.0 kg, the camera and GPS on): the airframe's thrust trim carries that weight, so a lighter body would hover at less thrust and less tilt authority than identified. Check pitch gain and filter changes with sw/sim/pitch_check.py on it before flying.
# From 2026-11-08 night (models/airframe_2026-11-08b.json, models/NOTES.md "Fewer parameters, held out by session"): the 10-inch zero-fit with the pitch inertia doubled (geometry_2026-11-08b.json, 1.05 kg) and only the forward speed's moment fitted, which predicts the held-out outdoor and indoor 2026-11-08 flights best and swings at the 2026-11-02 hovers' damping at their hardware's 1.5x. The calibrated airframe_2026-11-08.json predicted held-out flights worse than no fit; it stays as the check for the 2.6 Hz shake (pitch_check --airframe models/airframe_2026-11-08.json), which only it reproduces.
CURRENT_AIRFRAME = model("airframe_2026-11-08b.json")
CURRENT = in_room(CURRENT_AIRFRAME | {f"{axis}_push": 0.0 for axis in ("roll", "pitch", "yaw")}, "2026-10-29") | {
    "rotor_drag": (0.04, 0.1), "sensed_disturbances": True, "mass": json.loads((Path(__file__).with_name("models") / CURRENT_AIRFRAME["geometry"]).read_text())["mass"],
    "disturbances": [d for d in ROOMS["2026-10-29"]["disturbances"] if d["kind"] != "constant"]}


if __name__ == "__main__":
    import time
    flown = {"roll_p": 40, "roll_i": 50, "roll_damping": 6, "pitch_p": 25, "pitch_i": 10, "pitch_damping": 5, "yaw_damping": 10, "integral_limit": 8}
    t = time.time()
    print("nominal, flown gains:", run(flown, NOMINAL), f"{time.time() - t:.2f} s/run")
