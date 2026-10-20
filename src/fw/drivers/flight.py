"""
Shared native attitude estimator and flight controller. Angles are radians internally;
dashboard targets and trims are degrees. Build with `make -C fw/native host` for simulation.
"""

import math
import struct
import sys
from array import array
from .calibration import THRUST_TABLE

if sys.implementation.name == "micropython":
    from .flight_native import attitude_step, control_step
else:
    import ctypes
    from pathlib import Path

    library = ctypes.CDLL(str(Path(__file__).parents[1] / "native/flight_native.so"))
    library.flight_attitude.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    library.flight_attitude.restype = None
    library.flight_control.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_size_t,
    ]
    library.flight_control.restype = None

    def attitude_step(state, accel, gyro, dt, tau):
        motion = array("f", list(accel) + list(gyro) + [dt, tau])
        library.flight_attitude(state.buffer_info()[0], motion.buffer_info()[0])
        return state[0], state[1]

    def control_step(state, command, table, attitude, gyro, dt):
        struct.pack_into("<6f", command, 0, *attitude, *gyro, dt)
        library.flight_control(
            state.buffer_info()[0],
            command.buffer_info()[0],
            table.buffer_info()[0],
            len(table) // 2,
        )
        return int(state[2]), int(state[3]), state[4], state[5]


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
ANGULAR_KEYS = ("roll", "pitch", "yaw_rate", "pitch_trim", "left_trim", "right_trim")

GAIN_KEYS = (
    "roll_p", "roll_i", "roll_damping", "pitch_p", "pitch_i", "pitch_damping",
    "yaw_damping", "integral_limit", "pitch_trim", "left_trim", "right_trim",
)
COMMAND_KEYS = ("enabled", "roll", "pitch", "yaw_rate", "base_throttle") + GAIN_KEYS
BIAS_KEYS = ("roll_bias", "pitch_bias", "yaw_bias")
THRUST_SCALE_KEYS = ("left_thrust_scale", "right_thrust_scale")
FEEDFORWARD_KEYS = BIAS_KEYS + THRUST_SCALE_KEYS
# C Command tuning layout after the six motion floats; the model block follows.
TUNING_LAYOUT = COMMAND_KEYS + FEEDFORWARD_KEYS
MODEL_OFFSET = 6 + len(TUNING_LAYOUT)
# Dashboard field IDs and TEL1 order. Append only; this is independent of TUNING_LAYOUT.
CONTROL_KEYS = COMMAND_KEYS + MODEL_TUNING_KEYS + FEEDFORWARD_KEYS
THRUST_BUFFER = array("f", [value for pair in THRUST_TABLE for value in pair])


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
        # Two integrals, then the four actuator outputs.
        self.state = array("f", [0.0] * 6)
        # Motion (6 floats), tuning (TUNING_LAYOUT), then model parameters.
        self.command = array("f", [0.0] * MODEL_OFFSET + [model[key] for key in MODEL_KEYS])
        for key in THRUST_SCALE_KEYS:
            self.command[6 + TUNING_LAYOUT.index(key)] = 1.0

    def reset(self):
        """
        Start the integrators from the clamped biases, like the native flight_reset.
        """
        limit = self.command[6 + TUNING_LAYOUT.index("integral_limit")]
        bias = [
            max(-limit, min(limit, self.command[6 + TUNING_LAYOUT.index(key)]))
            for key in ("roll_bias", "pitch_bias")
        ]
        struct.pack_into("<2f", self.state, 0, *bias)

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
