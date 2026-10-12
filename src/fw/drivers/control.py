"""
Roll PID, updated once per fresh IMU sample.
"""

import math
import time
from drivers import actuators

settings = {
    "enabled": False,
    "target": 0.0,
    "kp": 0.0,
    "ki": 0.0,
    "kd": 0.0,
    "base_throttle": 0.0,
}
integral = 0.0
last_roll = None
last_update_us = None


def reset():
    """
    Clear accumulated error and derivative history.
    """
    global integral, last_roll, last_update_us
    integral = 0.0
    last_roll = None
    last_update_us = None


def calculate(roll, now, limit):
    """
    Return bounded PID output; differentiate measurement to avoid setpoint kick.
    """
    global integral, last_roll, last_update_us
    error = (settings["target"] - roll + 180) % 360 - 180
    dt = (
        time.ticks_diff(now, last_update_us) / 1_000_000
        if last_update_us is not None
        else 0
    )
    rate = ((roll - last_roll + 180) % 360 - 180) / dt if dt > 0 else 0
    last_roll, last_update_us = roll, now
    proportional = settings["kp"] * error
    derivative = -settings["kd"] * rate
    candidate = integral + settings["ki"] * error * dt
    output = proportional + candidate + derivative

    # Integrate only while unsaturated, or when the error unwinds saturation.
    if (
        -limit <= output <= limit
        or (output > limit and error < 0)
        or (output < -limit and error > 0)
    ):
        integral = candidate

    return max(-limit, min(limit, proportional + integral + derivative))


def stop():
    """
    Disable control and stop both ESCs.
    """
    settings["enabled"] = False
    reset()
    actuators.stop()


def configure(values):
    """
    Apply target, gains, base throttle, or enabled state from the dashboard.
    """
    updated = dict(settings)
    updated.update(values)
    if not all(
        math.isfinite(updated[key])
        for key in ("target", "kp", "ki", "kd", "base_throttle")
    ):
        raise ValueError("Control settings must be finite")
    if (
        not -60 <= updated["target"] <= 60
        or not 0 <= updated["base_throttle"] <= actuators.THROTTLE_MAX
    ):
        raise ValueError("Control target or throttle out of range")
    if min(updated["kp"], updated["ki"], updated["kd"]) < 0:
        raise ValueError("PID gains must be nonnegative")
    if any(key != "target" for key in values):
        reset()
    settings.update(updated)
    if not settings["enabled"]:
        stop()


def update(roll, now):
    """
    Positive roll correction raises left throttle and lowers right throttle.
    """
    if not settings["enabled"]:
        return
    base = settings["base_throttle"]
    correction = calculate(roll, now, min(base, actuators.THROTTLE_MAX - base))
    actuators.set_throttle("left", round(base + correction))
    actuators.set_throttle("right", round(base - correction))
