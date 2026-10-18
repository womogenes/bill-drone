"""
Apply dashboard settings, run the shared flight controller, and write motor outputs.
"""

import asyncio
import errno
import json
import math
import os
import time
from drivers import actuators
from drivers.flight import Controller, DEFAULT_TUNING

TILT_LIMIT = 15
RATE_LIMIT = 45
COLLECTIVE_LIMIT = actuators.THROTTLE_MAX
TIP_LIMIT = 45

# Confirmed: X forward, Y left, Z up; motor pivots above center of mass.
MAPPING_CONFIRMED = True
settings = {
    "enabled": False,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw_rate": 0.0,
    "base_throttle": 0.0,
}
settings.update(DEFAULT_TUNING)
GAIN_KEYS = tuple(DEFAULT_TUNING)
GAINS_FILE = "/flight-tuning.json"
save_requested_ms = None
status = {"reason": "", "mapping_confirmed": MAPPING_CONFIRMED, "save_pending": False}
controller = Controller()
last_update_us = None


def load_gains():
    """
    Load saved gains and trims. Start disabled with zero throttle.
    """
    global save_requested_ms

    try:
        with open(GAINS_FILE) as file:
            gains = json.load(file)
    except OSError as error:
        if error.errno != errno.ENOENT:
            raise
        save_requested_ms = time.ticks_ms()
        status["save_pending"] = True
        return

    configure({key: gains[key] for key in GAIN_KEYS if key in gains})
    save_requested_ms = None
    status["save_pending"] = False


async def save_gains():
    """
    Save tuning after edits have paused for a second and motors and control are stopped.
    """
    global save_requested_ms

    while True:
        await asyncio.sleep_ms(250)
        if save_requested_ms is None or time.ticks_diff(time.ticks_ms(), save_requested_ms) < 1000:
            continue
        if settings["enabled"] or any(value["throttle"] for value in actuators.read().values()):
            continue

        with open(GAINS_FILE + ".tmp", "w") as file:
            json.dump({key: settings[key] for key in GAIN_KEYS}, file)
        os.rename(GAINS_FILE + ".tmp", GAINS_FILE)
        os.sync()

        save_requested_ms = None
        status["save_pending"] = False


def reset():
    """
    Clear accumulated control error and the previous sample time.
    """
    global last_update_us
    controller.reset()
    last_update_us = None


def stop(reason=None):
    """
    Disable stabilization, clear controller history, and command zero throttle.
    """
    settings["enabled"] = False
    settings["base_throttle"] = 0.0
    if reason is not None:
        status["reason"] = reason

    reset()
    actuators.stop()


def configure(values):
    """
    Apply dashboard setpoints, gains, and trims. Require zero throttle when enabling.
    """
    global save_requested_ms

    updated = dict(settings)

    for key, value in values.items():
        if key not in settings:
            raise ValueError("Unknown control setting")

        updated[key] = value

    if not all(math.isfinite(value) for value in updated.values()):
        raise ValueError("Control settings must be finite")

    if (
        abs(updated["roll"]) > TILT_LIMIT
        or abs(updated["pitch"]) > TILT_LIMIT
        or abs(updated["yaw_rate"]) > RATE_LIMIT
        or not 0 <= updated["base_throttle"] <= COLLECTIVE_LIMIT
        or any(updated[key] < 0 for key in GAIN_KEYS if not key.endswith("_trim"))
        or any(abs(updated[key]) > 15 for key in GAIN_KEYS if key.endswith("_trim"))
    ):
        raise ValueError("Control setting out of range")

    if updated["enabled"] and not settings["enabled"]:
        if not MAPPING_CONFIRMED:
            status["reason"] = "IMU mounting needs confirmation"
            return

        if updated["base_throttle"] != 0:
            status["reason"] = "Set throttle to zero before enabling"
            return

        reset()

    if any(updated[key] != settings[key] for key in GAIN_KEYS):
        reset()
        save_requested_ms = time.ticks_ms()
        status["save_pending"] = True

    settings.update(updated)
    status["reason"] = ""

    if not settings["enabled"]:
        stop()


def update(reading, now):
    """
    Control roll/pitch attitude and yaw rate in the body frame.
    """
    global last_update_us

    if not settings["enabled"]:
        return

    if abs(reading["roll"]) > TIP_LIMIT or abs(reading["pitch"]) > TIP_LIMIT:
        stop("Tilt limit exceeded")
        return

    dt = time.ticks_diff(now, last_update_us) / 1_000_000 if last_update_us is not None else 0.01
    last_update_us = now
    if not 0 < dt <= 0.05:
        stop("Control sample delayed")
        return

    base = settings["base_throttle"]
    if base == 0:
        reset()
        actuators.stop()
        actuators.set_servo("left", 0)
        actuators.set_servo("right", 0)
        return

    attitude = [math.radians(reading["roll"]), math.radians(reading["pitch"])]
    gyro = [math.radians(value) for value in reading["gyro"]]
    left, right, left_angle, right_angle = controller.update(attitude, gyro, dt, settings)
    actuators.set_throttle("left", left)
    actuators.set_throttle("right", right)
    actuators.set_servo("left", math.degrees(left_angle))
    actuators.set_servo("right", math.degrees(right_angle))
