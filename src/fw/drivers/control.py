"""
Cascaded attitude/rate control, updated by fresh IMU samples.
"""

import asyncio
import errno
import json
import math
import os
import time
from drivers import actuators

TILT_LIMIT = 15
RATE_LIMIT = 45
SERVO_LIMIT = actuators.SERVO_LIMIT
PITCH_CORRECTION_LIMIT = 15
YAW_CORRECTION_LIMIT = 7.5
THROTTLE_LIMIT = actuators.THROTTLE_MAX
COLLECTIVE_LIMIT = THROTTLE_LIMIT
TIP_LIMIT = 45

# Confirmed: X forward, Y left, Z up; motor pivots above center of mass.
MAPPING_CONFIRMED = True
SERVO_HYSTERESIS = 0.3
RATE_FILTER_SECONDS = 0.02

settings = {
    "enabled": False,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw_rate": 0.0,
    "base_throttle": 0.0,
    "attitude_kp": 2.0,
    "roll_kp": 0.5,
    "roll_ki": 0.0,
    "roll_kd": 0.0,
    "pitch_kp": 0.15,
    "pitch_ki": 0.0,
    "pitch_kd": 0.0,
    "yaw_kp": 0.15,
    "yaw_ki": 0.05,
    "yaw_kd": 0.0,
}
GAIN_KEYS = tuple(key for key in settings if key.endswith(("_kp", "_ki", "_kd")))
GAINS_FILE = "/control-gains.json"
save_requested_ms = None
status = {"reason": "", "mapping_confirmed": MAPPING_CONFIRMED, "save_pending": False}
integrals = [0.0, 0.0, 0.0]
derivatives = [0.0, 0.0, 0.0]
previous_rates = None
filtered_rates = None
last_update_us = None


def load_gains():
    """
    Restore tuning only; throttle and enable state always start at zero.
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
    Coalesce edits and write flash only while motors and control are stopped.
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


def clamp(value, limit):
    """
    Limit a signed command.
    """
    return max(-limit, min(limit, value))


def reset():
    """
    Reset rate-controller history.
    """
    global previous_rates, filtered_rates, last_update_us

    integrals[:] = [0.0, 0.0, 0.0]
    derivatives[:] = [0.0, 0.0, 0.0]
    previous_rates = None
    filtered_rates = None
    last_update_us = None


def stop(reason=None):
    """
    Disable stabilization and remove motor thrust.
    """
    settings["enabled"] = False
    settings["base_throttle"] = 0.0
    if reason is not None:
        status["reason"] = reason

    reset()
    actuators.stop()


def configure(values):
    """
    Apply dashboard setpoints and gains; enabling always starts at zero thrust.
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
        or any(value < 0 for key, value in updated.items() if key.endswith(("_kp", "_ki", "_kd")))
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


def rate_output(index, axis, target, rate, dt, limit):
    """
    Rate PID with filtered measurement derivative and saturation anti-windup.
    """
    error = target - rate

    if previous_rates is not None:
        alpha = dt / (0.03 + dt)
        derivatives[index] += alpha * ((rate - previous_rates[index]) / dt - derivatives[index])

    proportional = settings[axis + "_kp"] * error
    derivative = -settings[axis + "_kd"] * derivatives[index]
    candidate = integrals[index] + settings[axis + "_ki"] * error * dt
    output = proportional + candidate + derivative

    if abs(output) <= limit or output * error < 0:
        integrals[index] = candidate

    return clamp(proportional + integrals[index] + derivative, limit)


def update(reading, now):
    """
    Control roll/pitch attitude and yaw rate in the body frame.
    """
    global last_update_us, previous_rates, filtered_rates

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

    targets = [
        clamp(settings["attitude_kp"] * (settings["roll"] - reading["roll"]), RATE_LIMIT),
        clamp(settings["attitude_kp"] * (settings["pitch"] - reading["pitch"]), RATE_LIMIT),
        settings["yaw_rate"],
    ]

    raw_rates = reading["gyro"]
    if filtered_rates is None:
        filtered_rates = list(raw_rates)
    else:
        alpha = dt / (RATE_FILTER_SECONDS + dt)
        filtered_rates = [
            old + alpha * (new - old) for old, new in zip(filtered_rates, raw_rates)
        ]

    rates = filtered_rates
    # Cancel body pitch before adding the rate controller's corrective tilt.
    pitch_compensation = -reading["pitch"]
    pitch_headroom = max(0, SERVO_LIMIT - abs(pitch_compensation) - YAW_CORRECTION_LIMIT)
    limits = [
        min(base, THROTTLE_LIMIT - base),
        min(PITCH_CORRECTION_LIMIT, pitch_headroom),
        YAW_CORRECTION_LIMIT,
    ]
    roll, pitch, yaw = [
        rate_output(index, axis, targets[index], rates[index], dt, limits[index])
        for index, axis in enumerate(("roll", "pitch", "yaw"))
    ]
    previous_rates = rates
    pitch += pitch_compensation

    actuators.set_throttle("left", round(base + roll))
    actuators.set_throttle("right", round(base - roll))

    # Confirmed commands: left negative/right positive gives forward thrust;
    # both negative gives counterclockwise yaw.
    for side, angle in (("left", -pitch - yaw), ("right", pitch - yaw)):
        if abs(angle - actuators.read()[side]["servo_angle"]) >= SERVO_HYSTERESIS:
            actuators.set_servo(side, angle)
