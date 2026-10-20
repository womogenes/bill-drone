"""
Start the MPU6050 and the native control task on core 1. The native settings store holds the calibration and servo offsets.
"""

import math
import time
import flight_rt
from drivers import actuators
from drivers.flight import GRAVITY_TAU, POWERED_GRAVITY_TAU, Controller, THRUST_BUFFER
from drivers.log_format import SAMPLE_HZ
from drivers.calibration import SENSOR_TO_BODY

TIP_LIMIT = 45
# MPU6050 DLPF_CFG: 3 is 42 Hz gyro bandwidth with 4.8 ms delay, 2 is 98 Hz with 2.8 ms but aliases vibration above the 50 Hz Nyquist rate.
DLPF = 3


def init():
    """
    Start the native sensor task using the board wiring and calibrated PWM channels.
    """
    config = {
        "sda": 19,
        "scl": 22,
        "address": 0x68,
        "sample_hz": SAMPLE_HZ,
        "accel_scale": tuple(sign / 16384 for sign in SENSOR_TO_BODY),
        "gyro_scale": tuple(sign * math.pi / (180 * 131) for sign in SENSOR_TO_BODY),
        "temperature_scale": 1 / 340,
        "temperature_offset": 36.53,
        "gravity_tau": GRAVITY_TAU,
        "powered_tau": POWERED_GRAVITY_TAU,
        "tip_limit": math.radians(TIP_LIMIT),
        "heartbeat_ms": 500,
        # Tolerate one missed control tick; a second consecutive miss stops the motors.
        "max_sample_ms": 2500 // SAMPLE_HZ,
        "dlpf": DLPF,
        "servo_limit": actuators.SERVO_LIMIT,
        "throttle_max": actuators.THROTTLE_MAX,
        "pwm": actuators.native_pwm(),
    }
    flight_rt.start(config)
    time.sleep_ms(100)
    # An initial I2C timeout can leave the configured sensor idle in the driver.
    if not flight_rt.snapshot()["online"]:
        flight_rt.start(config)
        time.sleep_ms(100)
    # The thrust table and fixed model parameters; the native store overrides every tunable value.
    flight_rt.configure(Controller().command, THRUST_BUFFER)


def stop():
    flight_rt.shutdown()
