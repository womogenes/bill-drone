"""
The drone's attitude estimator and flight controller for the host simulation, through ctypes on the firmware's own C math (fw/drone/main/flight_math.c), compiled here on first use. Angles are radians internally; dashboard targets and trims are degrees.
"""

import ctypes
import math
import re
import struct
import subprocess
from array import array
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/flight_math.c"
MANY = Path(__file__).with_name("flight_many.c")
LIBRARY = Path(__file__).with_name("flight_math.so")
# flight_math.h embeds hover.h HoverGains in Command, so a change to either moves the fields after it.
if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(path.stat().st_mtime for path in (SOURCE, SOURCE.with_suffix(".h"), SOURCE.with_name("hover.h"), SOURCE.with_name("indi.c"), SOURCE.with_name("indi.h"), MANY)):
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-I", str(SOURCE.parent), str(MANY), "-o", str(LIBRARY), "-lm"], check=True)
library = ctypes.CDLL(str(LIBRARY))
library.flight_attitude.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
library.flight_attitude.restype = None
library.flight_attitude_start.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
library.flight_attitude_start.restype = None
library.flight_control.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
library.flight_control.restype = None
count = ctypes.c_size_t.in_dll(library, "flight_thrust_count").value
THRUST_BUFFER = array("f", (ctypes.c_float * (2 * count)).in_dll(library, "flight_thrust"))
# [throttle, thrust_N] rows, as the drone uses them.
THRUST_TABLE = [[int(THRUST_BUFFER[2 * index]), THRUST_BUFFER[2 * index + 1]] for index in range(count)]
PEAK = max(range(len(THRUST_TABLE)), key=lambda index: THRUST_TABLE[index][1])
# Both ESCs were recalibrated to 1000-2200 us at this time (UTC, as recordings are named); the firmware's table was rescaled for it (commit a44c672). Recordings from before were flown with PRE_RECALIBRATION_TABLE, the earlier ESC range's table, and replay only with it.
RECALIBRATED = "2026-10-28T06-40"
PRE_RECALIBRATION_TABLE = [[0, 0.0], [200, 0.0], [250, 0.758689175], [300, 1.140325966], [350, 1.544788528], [400, 1.955279404], [450, 2.353557228], [500, 2.775707658], [550, 3.261558474], [600, 3.673818136], [650, 4.264941843],
                           [700, 4.898708952], [750, 5.566624474], [800, 6.313341249], [850, 7.172694844], [900, 8.00416899], [950, 8.815765343], [1000, 9.61023936], [1050, 10.438741463], [1100, 11.216827129], [1150, 11.052149476], [1200, 10.992227314]]


def thrust_table(recording):
    """The thrust table a recording (its path or name, starting with its UTC time) was flown with: [throttle, thrust_N] rows."""
    return PRE_RECALIBRATION_TABLE if Path(recording).name[:16] < RECALIBRATED else THRUST_TABLE


def interpolate(value, points, input_column=0):
    """Interpolate between table rows; clamp to the endpoint values."""
    output_column = 1 - input_column
    if value <= points[0][input_column]:
        return points[0][output_column]

    for before, after in zip(points, points[1:]):
        if value <= after[input_column]:
            fraction = (value - before[input_column]) / (after[input_column] - before[input_column])
            return before[output_column] + fraction * (after[output_column] - before[output_column])

    return points[-1][output_column]


def thrust(throttle):
    """Convert an ESC command to newtons."""
    return interpolate(throttle, THRUST_TABLE)


def throttle(force):
    """Convert newtons to an ESC command using the table's rising part, capped at its peak."""
    return interpolate(force, THRUST_TABLE[:PEAK + 1], input_column=1)


# The IMU's full scales as the firmware sets them (runtime.h GYRO_COUNTS_PER_DEG_S and ACCEL_COUNTS_PER_G over 16-bit samples): deg/s and g.
RUNTIME = SOURCE.with_name("runtime.h").read_text()
GYRO_RANGE = 32768 / float(re.search(r"#define GYRO_COUNTS_PER_DEG_S ([0-9.]+)f", RUNTIME).group(1))
ACCEL_RANGE = 32768 / float(re.search(r"#define ACCEL_COUNTS_PER_G ([0-9.]+)f", RUNTIME).group(1))

# The airframe's one model (flight_math.h Model; settings.def roll_authority): angular acceleration (rad/s²) per newton of differential thrust, per newton per radian of common tilt, per newton per radian of differential tilt, then the servos' range and directions.
MODEL = {
    "roll_authority": 8.0,
    "pitch_authority": 8.68,
    "yaw_authority": 11.35,
    "servo_min": math.radians(-60),
    "servo_max": math.radians(60),
    # Positive left tilts forward; positive right tilts backward.
    "left_servo_direction": 1.0,
    "right_servo_direction": -1.0,
}
# This order is the C buffer layout; it must not depend on dictionary iteration.
MODEL_KEYS = ("roll_authority", "pitch_authority", "yaw_authority", "servo_min", "servo_max", "left_servo_direction", "right_servo_direction")
AUTHORITY_KEYS = MODEL_KEYS[:3]


def authorities(settings):
    """The roll, pitch and yaw authority settings give (sw/flight_log.py migrated reads older recordings' as the firmware does), the defaults where they have none."""
    return tuple(settings.get(key, MODEL[key]) for key in AUTHORITY_KEYS)


# Equivalent to retaining 0.99 of gyro integration every 10 ms; with the motors spinning, settings.def attitude_tau (default 50 s, the 0.9998 it always was).
GRAVITY_TAU = -0.01 / math.log(0.99)
POWERED_GRAVITY_TAU = 50.0
ANGULAR_KEYS = ("roll", "pitch", "yaw_rate", "pitch_trim", "left_trim", "right_trim", "servo_compensation", "target_rate")

GAIN_KEYS = ("roll_p", "roll_i", "roll_damping", "pitch_p", "pitch_i", "pitch_damping", "yaw_damping", "integral_limit", "pitch_trim", "left_trim", "right_trim")
COMMAND_KEYS = ("enabled", "roll", "pitch", "yaw_rate", "base_throttle") + GAIN_KEYS
BIAS_KEYS = ("roll_bias", "pitch_bias", "yaw_bias")
THRUST_SCALE_KEYS = ("left_thrust_scale", "right_thrust_scale")
FEEDFORWARD_KEYS = BIAS_KEYS + THRUST_SCALE_KEYS
# C Command tuning layout after the six motion floats; the model block follows.
TUNING_LAYOUT = COMMAND_KEYS + FEEDFORWARD_KEYS + ("servo_compensation", "integrate_throttle", "target_rate")
MODEL_OFFSET = 6 + len(TUNING_LAYOUT)
# The runtime-only Command fields after the model block: the battery voltage thrust compensation is relative to, and the yaw-rate integral gain.
THRUST_VOLTAGE = MODEL_OFFSET + len(MODEL_KEYS)
YAW_I = THRUST_VOLTAGE + 1
# The speed moment compensation (rad/s² per m/s, 0 = off; flight_math.c), runtime-only after yaw_i.
PITCH_SPEED = YAW_I + 1
# The pitch damping's rate low-pass time constant (s, 0 = raw rate; flight_math.c), the last runtime-only field: after pitch_speed come the hover gains (HOVER_GAINS of them: sw/sim/hover.py GAIN_KEYS, which checks the count; not imported here, since importing hover.py compiles hover.c), hover_camera and auto_trim.
HOVER_GAINS = 47
DAMPING_TAU = PITCH_SPEED + 1 + HOVER_GAINS + 2
# The roll setpoint's own slew rate (rad/s, 0 = target_rate; flight_math.c), after pitch_damping_tau.
ROLL_TARGET_RATE = DAMPING_TAU + 1
# INDI's share per axis (roll, pitch, yaw; 0 = off), then the unmodeled acceleration on the controller's model (rad/s²), which the harness sets each tick as flight_rt.c does (indi.h).
INDI = ctypes.c_size_t.in_dll(library, "flight_command_indi").value
# The attitude quaternion (w, x, y, z; body to level) the controller's error takes, which the harness sets each tick as flight_rt.c does (Controller.update q).
ATTITUDE = ctypes.c_size_t.in_dll(library, "flight_command_attitude").value
COMMAND_SIZE = ctypes.c_size_t.in_dll(library, "flight_command_size").value


def body_quaternion(q, roll, pitch):
    """flight_rt.c relative_attitude: the estimate's quaternion turned by a level reference's roll and pitch (radians), q ⊗ zero⁻¹."""
    z = from_euler(roll, pitch)
    return (q[0] * z[0] + q[1] * z[1] + q[2] * z[2] + q[3] * z[3], -q[0] * z[1] + q[1] * z[0] - q[2] * z[3] + q[3] * z[2],
            -q[0] * z[2] + q[1] * z[3] + q[2] * z[0] - q[3] * z[1], -q[0] * z[3] - q[1] * z[2] + q[2] * z[1] + q[3] * z[0])


def from_euler(roll, pitch):
    """flight_math.c quaternion_from_euler: roll and pitch (radians; about forward, then left) as a quaternion (w, x, y, z), the heading zero."""
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (cr * cp, sr * cp, cr * sp, -sr * sp)


def euler(q):
    """flight_math.c quaternion_euler: roll and pitch (radians) of a quaternion (w, x, y, z)."""
    return (math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)), math.asin(max(-1.0, min(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))))


class Attitude:
    # flight_math.h Attitude: roll, pitch and the quaternion q (w, x, y, z) they are read from, started at rest from the accelerometer (flight_attitude_start).
    def __init__(self, accel=(0.0, 0.0, 1.0)):
        self.state = array("f", [0.0] * 6)
        specific = array("f", accel)
        library.flight_attitude_start(self.state.buffer_info()[0], specific.buffer_info()[0])

    @property
    def q(self):
        return tuple(self.state[2:6])

    def update(self, accel, gyro, dt, powered, gyro_only=False, powered_tau=POWERED_GRAVITY_TAU):
        # gyro_only: the gyro alone (through a burst or a recovery and their grace), an infinite time constant.
        motion = array("f", list(accel) + list(gyro) + [dt, math.inf if gyro_only else powered_tau if powered else GRAVITY_TAU])
        library.flight_attitude(self.state.buffer_info()[0], motion.buffer_info()[0])
        return self.state[0], self.state[1]


class Controller:
    def __init__(self, model=MODEL):
        # Two integrals, the four actuator outputs, each servo's compensation direction and extreme, the takeoff setpoint state, the upright-takeoff flag, the yaw integrator and the damping's filtered pitch rate.
        self.state = array("f", [0.0] * 18)
        # Motion (6 floats), tuning (TUNING_LAYOUT), then model parameters.
        # Runtime-only Command fields after the model (thrust_voltage at THRUST_VOLTAGE, yaw_i at YAW_I, hover, the radio settings and the guards' GuardGains) start at zero, off unless the caller sets them, up to the whole Command (COMMAND_SIZE).
        self.command = array("f", [0.0] * MODEL_OFFSET + [model[key] for key in MODEL_KEYS] + [0.0] * (COMMAND_SIZE - MODEL_OFFSET - len(MODEL_KEYS)))
        for key in THRUST_SCALE_KEYS:
            self.command[6 + TUNING_LAYOUT.index(key)] = 1.0

    def reset(self):
        """
        Start the roll integrator from the clamped roll bias and the pitch integrator from zero (the pitch bias fades in after liftoff), like the native flight_reset.
        """
        limit = self.command[6 + TUNING_LAYOUT.index("integral_limit")]
        roll = max(-limit, min(limit, self.command[6 + TUNING_LAYOUT.index("roll_bias")]))
        struct.pack_into("<2f", self.state, 0, roll, 0.0)
        # No upright takeoff: free flight starts in the air, and a liftoff arms first, where the native flight_reset sets the flag from the resting attitude.
        self.state[15] = 0.0
        # The damping's filtered pitch rate starts at the current rate, as flight_reset does.
        self.state[17] = self.command[3]

    def configure(self, settings):
        # Setpoints and the enable flag are native-owned on the drone; missing ones are zero.
        values = [
            math.radians(settings.get(key, 0.0)) if key in ANGULAR_KEYS else settings.get(key, 0.0)
            for key in TUNING_LAYOUT
        ]
        struct.pack_into("<%df" % len(TUNING_LAYOUT), self.command, 24, *values)
        self.command[MODEL_OFFSET:MODEL_OFFSET + 3] = array("f", authorities(settings))
        self.command[DAMPING_TAU] = settings.get("pitch_damping_tau", 0.0)
        self.command[ROLL_TARGET_RATE] = math.radians(settings.get("roll_target_rate", 0.0))

    def update(self, attitude, gyro, dt, q):
        """attitude: roll and pitch (radians); q: the attitude quaternion they are read from (w, x, y, z; body to level)."""
        struct.pack_into("<6f", self.command, 0, *attitude, *gyro, dt)
        self.command[ATTITUDE:ATTITUDE + 4] = array("f", q)
        library.flight_control(self.state.buffer_info()[0], self.command.buffer_info()[0], THRUST_BUFFER.buffer_info()[0], len(THRUST_BUFFER) // 2)
        return int(self.state[2]), int(self.state[3]), self.state[4], self.state[5]


# indi.h's Indi (INDI_SERVO_DELAY 6) on the Model's head (its authorities, servo range and directions); flight_rt.c's INDI_FADE_S and INDI_HEIGHT.
INDI_FADE_S, INDI_HEIGHT = 0.5, 0.3


class IndiModel(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in MODEL_KEYS]


class Indi(ctypes.Structure):
    _fields_ = [("sent_tilt", ctypes.c_float * 2 * 6), ("sent_thrust", ctypes.c_float * 2 * 6), ("next", ctypes.c_int), ("tilt", ctypes.c_float * 2), ("tilt_rate", ctypes.c_float * 2), ("thrust", ctypes.c_float * 2),
                ("filter", ctypes.c_float * 2 * 6), ("gyro", ctypes.c_float * 3), ("model", ctypes.c_float * 3), ("sent", ctypes.c_bool), ("started", ctypes.c_bool),
                ("unmodeled", ctypes.c_float * 3), ("beyond", ctypes.c_float)]


library.indi_sent.argtypes = [ctypes.POINTER(Indi), ctypes.c_float * 2, ctypes.c_float * 2]
library.indi_update.argtypes = [ctypes.POINTER(Indi), ctypes.c_float * 3, ctypes.POINTER(IndiModel), ctypes.c_float]
library.indi_predict.argtypes = [ctypes.POINTER(Indi), ctypes.POINTER(IndiModel), ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]


class Incremental:
    """
    INDI as flight_rt.c runs it (incremental, fly): settings' indi (roll and yaw) and indi_pitch are the shares of the correction taken (0 = off); each tick update() before the controller (and the burst and the recovery, which take the same unmodeled acceleration on the same model), apply() to the controller's command, and sent() with the actuators as sent update() also runs the servos' dead-time predictor (indi.h indi_predict; settings' pitch_predict its share), whose change in pitch and pitch rate apply() hands the controller and the harness the burst's and the recovery's authority (flip.h FlipAuthority predicted).
    """

    def __init__(self, settings, model=MODEL):
        self.shares = (settings.get("indi", 0.0), settings.get("indi_pitch", 0.0), settings.get("indi", 0.0))

        self.integrate = settings.get("integrate_throttle", 0.0)
        # The airframe's model as the settings give it (flight_math.h Model), which the controller, the burst and the recovery all fly on: authority() hands the harness the roll, pitch and yaw.
        self.model = IndiModel(*(dict(zip(AUTHORITY_KEYS, authorities(settings))) | {key: model[key] for key in MODEL_KEYS[3:]}).values())
        self.state = Indi()
        self.fade = 0.0
        # The servos' dead-time predictor's share (settings.def pitch_predict) and, each tick, its pitch and pitch rate change (flight_rt.c incremental).
        self.predict = settings.get("pitch_predict", 0.0)
        self.predicted = (0.0, 0.0)
        # Whether anything takes INDI's estimate (on the drone it runs whenever armed; off, it changes nothing).
        self.on = max(self.shares) > 0 or self.predict > 0

    def update(self, gyro, holding, height, base, owned=False, dt=0.01):
        """gyro (rad/s); whether the hold flies (engaged in a hover mode and not landed), its height above where it engaged (m), last tick's collective throttle, whether the burst or a recovery flies the motors."""
        flying = self.on and (owned or (holding and height >= INDI_HEIGHT and base >= self.integrate))
        self.fade = min(max(self.fade + (dt if flying else -dt) / INDI_FADE_S, 0.0), 1.0)
        library.indi_update(ctypes.byref(self.state), (ctypes.c_float * 3)(*gyro), ctypes.byref(self.model), dt)
        rate, angle = ctypes.c_float(), ctypes.c_float()
        library.indi_predict(ctypes.byref(self.state), ctypes.byref(self.model), gyro[1], dt, ctypes.byref(rate), ctypes.byref(angle))
        self.predicted = (angle.value, rate.value)

    def prediction(self):
        """The predictor's change in pitch and pitch rate at the share taken now (flight_rt.c incremental: pitch_predict faded in with INDI)."""
        return [self.fade * self.predict * x for x in self.predicted]

    def share(self):
        """The share taken per axis now (roll, pitch, yaw): the settings faded in (flight_rt.c incremental)."""
        return [self.fade * share for share in self.shares]

    def unmodeled(self):
        return list(self.state.unmodeled)

    def authority(self):
        """The model's roll, pitch and yaw authority (flight_rt.c maneuver)."""
        return (self.model.roll_authority, self.model.pitch_authority, self.model.yaw_authority)

    def apply(self, controller):
        controller.command[INDI - 1:INDI + 8] = array("f", [self.predict] + self.share() + self.unmodeled() + self.prediction())

    def sent(self, tilt, thrust):
        """Each assembly's physical forward tilt (radians) and each motor's thrust (N) as sent."""
        library.indi_sent(ctypes.byref(self.state), (ctypes.c_float * 2)(*tilt), (ctypes.c_float * 2)(*thrust))
