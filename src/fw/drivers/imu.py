"""
MPU6050 telemetry and calibration; native sampling/control runs on core 1.
"""

import asyncio
import errno
import json
import os
import math
import time
import flight_rt
from drivers import actuators, control
from drivers.flight import GRAVITY_TAU, POWERED_GRAVITY_TAU
from drivers.log_format import SAMPLE_HZ
from drivers.calibration import SENSOR_TO_BODY

CALIBRATION_SAMPLES = 2 * SAMPLE_HZ
# MPU6050 DLPF_CFG: 3 is 42 Hz gyro bandwidth with 4.8 ms delay, 2 is 98 Hz with 2.8 ms but aliases vibration above the 50 Hz Nyquist rate.
DLPF = 3
CALIBRATION_FILE = "/imu-calibration.json"
save_pending = False
calibration_count = 0
last_read_ms = 0


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
        "tip_limit": math.radians(control.TIP_LIMIT),
        "heartbeat_ms": 500,
        # Tolerate one missed control tick; a second consecutive miss stops the motors.
        "max_sample_ms": 2500 // SAMPLE_HZ,
        "dlpf": DLPF,
        "servo_limit": actuators.SERVO_LIMIT,
        "throttle_max": actuators.THROTTLE_MAX,
        "pwm": actuators.native_pwm(),
        "servo_offsets": [actuators.SERVO_OFFSETS[side] for side in actuators.SIDES],
    }
    flight_rt.start(config)
    time.sleep_ms(100)
    # An initial I2C timeout can leave the configured sensor idle in the driver.
    if not flight_rt.snapshot()["online"]:
        flight_rt.start(config)
        time.sleep_ms(100)
    load_calibration()
    actuators.running = True
    control.apply_native()


def calibrate():
    """
    Stop outputs, estimate gyro bias, and set the stationary pose as zero.
    """
    global calibration_count, save_pending
    control.stop("IMU calibration")
    control.configure({"pitch_trim": 0.0})
    flight_rt.calibrate(CALIBRATION_SAMPLES)
    calibration_count = CALIBRATION_SAMPLES
    save_pending = True


def load_calibration():
    """
    Restore only calibration recorded for the current sensor coordinate frame.
    """
    try:
        with open(CALIBRATION_FILE) as file:
            saved = json.load(file)
    except OSError as error:
        if error.errno != errno.ENOENT:
            raise
        return
    if saved["version"] != 1 or saved["sensor_to_body"] != list(SENSOR_TO_BODY):
        return
    flight_rt.restore_calibration(saved["gyro_bias_rad_s"], saved["pose_zero_deg"])


async def save_calibration():
    """
    Commit completed calibration on the networking core before reporting ready.
    """
    global save_pending
    while True:
        await asyncio.sleep_ms(50)
        if not save_pending:
            continue
        state = flight_rt.snapshot()
        if state["calibrating"]:
            continue
        if not state["calibrated"]:
            save_pending = False
            continue
        if state["enabled"] or any(state["throttle"]):
            continue
        saved = {
            "version": 1,
            "sensor_to_body": list(SENSOR_TO_BODY),
            "gyro_bias_rad_s": list(state["gyro_bias"]),
            "pose_zero_deg": list(state["pose_zero"]),
        }
        with open(CALIBRATION_FILE + ".tmp", "w") as file:
            json.dump(saved, file)
        os.rename(CALIBRATION_FILE + ".tmp", CALIBRATION_FILE)
        os.sync()
        save_pending = False


def stop():
    flight_rt.shutdown()
    actuators.running = False


def read(snapshot=None):
    """
    Copy the latest native sample without waiting for another conversion.
    """
    global calibration_count, last_read_ms
    state = snapshot if snapshot is not None else flight_rt.snapshot()
    busy = state["calibrating"] or save_pending
    calibration_count = int(busy)
    last_read_ms = time.ticks_add(time.ticks_ms(), -state["age_ms"])
    if not state["online"] or state["age_ms"] > 100:
        return None

    return {
        "calibrating": busy,
        "calibrated": state["calibrated"] and not save_pending,
        "accel": list(state["accel"]),
        "gyro": list(state["gyro"]),
        "roll": state["roll"],
        "pitch": state["pitch"],
        "temperature": state["temperature"],
    }
