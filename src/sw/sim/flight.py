"""
The drone's attitude estimator and flight controller for the host simulation, through ctypes on the firmware's own C math (fw/drone/main/flight_math.c), compiled here on first use. Angles are radians internally; dashboard targets and trims are degrees.
"""

import ctypes
import math
import struct
import subprocess
from array import array
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/flight_math.c"
MANY = Path(__file__).with_name("flight_many.c")
LIBRARY = Path(__file__).with_name("flight_math.so")
if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(path.stat().st_mtime for path in (SOURCE, SOURCE.with_suffix(".h"), MANY)):
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-I", str(SOURCE.parent), str(MANY), "-o", str(LIBRARY), "-lm"], check=True)
library = ctypes.CDLL(str(LIBRARY))
library.flight_attitude.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
library.flight_attitude.restype = None
library.flight_control.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
library.flight_control.restype = None
library.flight_control_many.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t]
library.flight_control_many.restype = None
count = ctypes.c_size_t.in_dll(library, "flight_thrust_count").value
THRUST_BUFFER = array("f", (ctypes.c_float * (2 * count)).in_dll(library, "flight_thrust"))
# [throttle, thrust_N] rows, as the drone uses them; thrust declines above 1100.
THRUST_TABLE = [[int(THRUST_BUFFER[2 * index]), THRUST_BUFFER[2 * index + 1]] for index in range(count)]
PEAK = max(range(len(THRUST_TABLE)), key=lambda index: THRUST_TABLE[index][1])


def attitude_step(state, accel, gyro, dt, tau):
    motion = array("f", list(accel) + list(gyro) + [dt, tau])
    library.flight_attitude(state.buffer_info()[0], motion.buffer_info()[0])
    return state[0], state[1]


def control_step(state, command, table, attitude, gyro, dt):
    struct.pack_into("<6f", command, 0, *attitude, *gyro, dt)
    library.flight_control(state.buffer_info()[0], command.buffer_info()[0], table.buffer_info()[0], len(table) // 2)
    return int(state[2]), int(state[3]), state[4], state[5]


def control_many(states, commands):
    """
    Step every lane's controller in place: states and commands are C-contiguous float32 arrays [lane, ...] laid out as Controller.state and Controller.command, with each command's motion filled in.
    """
    assert states.dtype == commands.dtype == "float32" and states.flags.c_contiguous and commands.flags.c_contiguous
    library.flight_control_many(states.ctypes.data, states.shape[1], commands.ctypes.data, commands.shape[1], THRUST_BUFFER.buffer_info()[0], len(THRUST_BUFFER) // 2, len(states))


def interpolate(value, points, input_column=0):
    """
    Interpolate between table rows; clamp to the endpoint values.
    """
    output_column = 1 - input_column
    if value <= points[0][input_column]:
        return points[0][output_column]

    for before, after in zip(points, points[1:]):
        if value <= after[input_column]:
            fraction = (value - before[input_column]) / (after[input_column] - before[input_column])
            return before[output_column] + fraction * (after[output_column] - before[output_column])

    return points[-1][output_column]


def thrust(throttle):
    """
    Convert an ESC command to newtons, including the measured decline above 1100.
    """
    return interpolate(throttle, THRUST_TABLE)


def throttle(force):
    """
    Convert newtons to an ESC command using the rising curve, capped at 1100.
    """
    return interpolate(force, THRUST_TABLE[:PEAK + 1], input_column=1)


class GyroCalibration:
    def __init__(self):
        self.bias = [0.0, 0.0, 0.0]
        self.remaining = 0

    def start(self, samples=200):
        self.samples = samples
        self.total = [0.0, 0.0, 0.0]
        self.remaining = samples

    def update(self, gyro):
        if self.remaining:
            for axis in range(3):
                self.total[axis] += gyro[axis]
            self.remaining -= 1
            count = self.samples - self.remaining
            self.bias = [value / count for value in self.total]

        return [value - bias for value, bias in zip(gyro, self.bias)]


DEFAULT_TUNING = {
    "roll_p": 20.0,
    "roll_i": 3.0,
    "roll_damping": 6.0,
    "pitch_p": 20.0,
    "pitch_i": 3.0,
    "pitch_damping": 10.0,
    "yaw_damping": 4.0,
    "integral_limit": 3.0,
    "pitch_trim": 0.0,
    "left_trim": 0.9,
    "right_trim": -0.9,
    # Integrator starting values in rad/s², for a known steady disturbance.
    "roll_bias": 0.0,
    "pitch_bias": 0.0,
    # Constant yaw torque term in rad/s².
    "yaw_bias": 0.0,
    # Per-motor thrust multipliers; a weak motor below 1.0 gets more throttle for the same force.
    "left_thrust_scale": 1.0,
    "right_thrust_scale": 1.0,
    # Degrees added to each servo command in the direction it last moved (servo deadband and play).
    "servo_compensation": 0.75,
    # Collective throttle below which the integrators hold their biases.
    "integrate_throttle": 580,
    # Degrees per second the roll/pitch setpoint may move; liftoff ramps from the ground attitude.
    "target_rate": 360,
    # Degrees of target tilt per m/s of dead-reckoned velocity (0 = off), and its leak time in seconds.
    "velocity_damping": 8,
    "velocity_leak": 3,
}


# Geometry and inertia of the current 900 g drone, in metres and kg·m².
MODEL = {
    "arm": 0.172,
    "pivot_height": 0.0253692458911615,
    "com_x": 0.0,  # Fitted to 2026-10-20 liftoff data; CAD gave -0.00202.
    "roll_inertia": 0.0118483264434811,
    "pitch_inertia": 0.0023030647846826,
    "yaw_inertia": 0.0106709253540914,
    "reaction_torque_ratio": 0.015,
    "servo_min": math.radians(-60),
    "servo_max": math.radians(60),
    # Positive left tilts forward; positive right tilts backward.
    "left_servo_direction": 1.0,
    "right_servo_direction": -1.0,
}
# This order is the C buffer layout; it must not depend on dictionary iteration.
MODEL_KEYS = (
    "arm", "pivot_height", "com_x",
    "roll_inertia", "pitch_inertia", "yaw_inertia", "reaction_torque_ratio",
    "servo_min", "servo_max", "left_servo_direction", "right_servo_direction",
)
# Model parameters tunable from the dashboard, in SI units.
MODEL_TUNING_KEYS = (
    "pivot_height", "com_x", "roll_inertia", "pitch_inertia", "yaw_inertia",
    "reaction_torque_ratio", "arm",
)
DEFAULT_TUNING.update({key: MODEL[key] for key in MODEL_TUNING_KEYS})

# Equivalent to retaining 0.99 / 0.9998 of gyro integration every 10 ms.
GRAVITY_TAU = -0.01 / math.log(0.99)
POWERED_GRAVITY_TAU = -0.01 / math.log(0.9998)
ANGULAR_KEYS = ("roll", "pitch", "yaw_rate", "pitch_trim", "left_trim", "right_trim", "servo_compensation", "target_rate", "velocity_damping")

GAIN_KEYS = (
    "roll_p", "roll_i", "roll_damping", "pitch_p", "pitch_i", "pitch_damping",
    "yaw_damping", "integral_limit", "pitch_trim", "left_trim", "right_trim",
)
COMMAND_KEYS = ("enabled", "roll", "pitch", "yaw_rate", "base_throttle") + GAIN_KEYS
BIAS_KEYS = ("roll_bias", "pitch_bias", "yaw_bias")
THRUST_SCALE_KEYS = ("left_thrust_scale", "right_thrust_scale")
FEEDFORWARD_KEYS = BIAS_KEYS + THRUST_SCALE_KEYS
# C Command tuning layout after the six motion floats; the model block follows.
TUNING_LAYOUT = COMMAND_KEYS + FEEDFORWARD_KEYS + ("servo_compensation", "integrate_throttle", "target_rate", "velocity_damping", "velocity_leak")
MODEL_OFFSET = 6 + len(TUNING_LAYOUT)
# Dashboard field IDs and TEL1 order. Append only; this is independent of TUNING_LAYOUT.
CONTROL_KEYS = COMMAND_KEYS + MODEL_TUNING_KEYS + FEEDFORWARD_KEYS


class Attitude:
    def __init__(
        self,
        accel=(0.0, 0.0, 1.0),
        gravity_tau=GRAVITY_TAU,
        powered_tau=POWERED_GRAVITY_TAU,
    ):
        self.gravity_tau = gravity_tau
        self.powered_tau = powered_tau
        self.state = array(
            "f",
            [
                math.atan2(accel[1], accel[2]),
                math.atan2(-accel[0], math.sqrt(accel[1] ** 2 + accel[2] ** 2)),
            ],
        )

    def update(self, accel, gyro, dt, powered):
        tau = self.powered_tau if powered else self.gravity_tau
        return attitude_step(self.state, accel, gyro, dt, tau)


class Controller:
    def __init__(self, model=MODEL):
        # Two integrals, the four actuator outputs, each servo's compensation direction and extreme, the takeoff setpoint state, then the upright-takeoff flag.
        self.state = array("f", [0.0] * 17)
        # Motion (6 floats), tuning (TUNING_LAYOUT), then model parameters.
        # Runtime-only Command fields after the model (thrust_voltage, yaw_i, hover) stay zero: off in the simulation.
        self.command = array("f", [0.0] * MODEL_OFFSET + [model[key] for key in MODEL_KEYS] + [0.0] * 64)
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

    def configure(self, settings):
        # Setpoints and the enable flag are native-owned on the drone; missing ones are zero.
        values = [
            math.radians(settings.get(key, 0.0)) if key in ANGULAR_KEYS else settings.get(key, 0.0)
            for key in TUNING_LAYOUT
        ]
        struct.pack_into("<%df" % len(TUNING_LAYOUT), self.command, 24, *values)
        for key in MODEL_TUNING_KEYS:
            self.command[MODEL_OFFSET + MODEL_KEYS.index(key)] = settings[key]

    def update(self, attitude, gyro, dt):
        return control_step(self.state, self.command, THRUST_BUFFER, attitude, gyro, dt)
