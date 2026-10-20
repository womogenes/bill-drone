"""
Apply dashboard settings, run the shared flight controller, and write motor outputs.
"""

import asyncio
import errno
import json
import math
import os
import time
import flight_rt
from .log_format import STOP_REASONS
from drivers import actuators, recorder
from drivers.flight import (
    Controller,
    BIAS_KEYS,
    DEFAULT_TUNING,
    FEEDFORWARD_KEYS,
    GAIN_KEYS,
    MODEL_TUNING_KEYS,
    THRUST_BUFFER,
    THRUST_SCALE_KEYS,
)

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
GAINS_FILE = "/flight-tuning.json"
save_requested_ms = None
status = {"reason": "", "mapping_confirmed": MAPPING_CONFIRMED, "save_pending": False}
TUNING_KEYS = GAIN_KEYS + MODEL_TUNING_KEYS + FEEDFORWARD_KEYS
controller = Controller()
last_native_reason = 0


def load_gains():
    """
    Load saved gains, trims, and model parameters. Start disabled with zero throttle.
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

    actuators.SERVO_OFFSETS.update(gains.get("servo_offsets", {}))
    for side in actuators.SERVO_OFFSETS:
        actuators.set_servo(side, 0)

    configure({key: gains[key] for key in TUNING_KEYS if key in gains})
    save_requested_ms = None
    status["save_pending"] = False


async def save_gains():
    """
    Save tuning after edits have paused for a second and motors and control are stopped.
    """
    global save_requested_ms

    while True:
        await asyncio.sleep_ms(250)
        if (
            save_requested_ms is None
            or time.ticks_diff(time.ticks_ms(), save_requested_ms) < 1000
        ):
            continue
        if settings["enabled"] or any(
            value["throttle"] for value in actuators.read().values()
        ):
            continue

        with open(GAINS_FILE + ".tmp", "w") as file:
            saved = {key: settings[key] for key in TUNING_KEYS}
            saved["servo_offsets"] = actuators.SERVO_OFFSETS
            json.dump(saved, file)
        os.rename(GAINS_FILE + ".tmp", GAINS_FILE)
        os.sync()

        save_requested_ms = None
        status["save_pending"] = False


def zero_servo(side):
    """
    Save a servo's current position as zero while motors and control are stopped.
    """
    global save_requested_ms
    if settings["enabled"] or any(
        value["throttle"] for value in actuators.read().values()
    ):
        return

    actuators.zero_servo(side)
    save_requested_ms = time.ticks_ms()
    status["save_pending"] = True


def apply_native():
    """
    Copy configured parameters into native-owned memory.
    """
    flight_rt.configure(controller.command, THRUST_BUFFER)


def sync(snapshot=None):
    """
    Reflect native failsafe shutdowns in dashboard state.
    """
    global last_native_reason
    state = snapshot if snapshot is not None else flight_rt.snapshot()
    if not state["enabled"] and settings["enabled"]:
        settings["enabled"] = False
        settings["base_throttle"] = 0.0
    if state["reason"] != last_native_reason:
        last_native_reason = state["reason"]
        status["reason"] = STOP_REASONS[state["reason"]]
        if state["reason"]:
            recorder.stop(status["reason"])


def reset():
    controller.reset()
    if actuators.running:
        flight_rt.reset()


def stop(reason=None):
    """
    Stop outputs on the native core and close the recording.
    """
    global last_native_reason
    settings["enabled"] = False
    settings["base_throttle"] = 0.0
    if reason is not None:
        status["reason"] = reason
    reset()
    if actuators.running:
        last_native_reason = 1
        flight_rt.stop(1)
    else:
        actuators.stop()
    recorder.stop(reason)


def configure(values):
    """
    Apply dashboard setpoints, gains, trims, and model parameters. Require zero throttle when enabling.
    """
    global save_requested_ms

    if actuators.running:
        sync()
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
        or any(
            updated[key] <= 0
            for key in (
                "arm",
                "pivot_height",
                "roll_inertia",
                "pitch_inertia",
                "yaw_inertia",
            )
        )
        or abs(updated["com_x"]) > 0.02
        or abs(updated["reaction_torque_ratio"]) > 0.1
        or any(abs(updated[key]) > 10 for key in BIAS_KEYS)
        or any(not 0.5 <= updated[key] <= 1.5 for key in THRUST_SCALE_KEYS)
    ):
        raise ValueError("Control setting out of range")

    enabling = updated["enabled"] and not settings["enabled"]
    if enabling:
        if not flight_rt.snapshot()["calibrated"]:
            status["reason"] = "Calibrate IMU before enabling"
            return
        if not MAPPING_CONFIRMED:
            status["reason"] = "IMU mounting needs confirmation"
            return

        if updated["base_throttle"] != 0:
            status["reason"] = "Set throttle to zero before enabling"
            return

        if not recorder.status()["ready"]:
            status["reason"] = "Flight recorder is not ready"
            return

        reset()

    retuned = any(updated[key] != settings[key] for key in TUNING_KEYS)
    if retuned:
        recorder.settings_changed(updated)
        save_requested_ms = time.ticks_ms()
        status["save_pending"] = True

    controller.configure(updated)
    if actuators.running:
        apply_native()
    if retuned:
        # After applying, so the integrators restart from the new biases.
        reset()
    settings.update(updated)
    if enabling:
        recorder.start(updated, actuators.SERVO_OFFSETS)
    status["reason"] = ""

    if not settings["enabled"]:
        stop()
    elif settings["base_throttle"] == 0 and not actuators.running:
        # PWM keeps these outputs between samples; apply them when zero is selected.
        reset()
        actuators.stop()
        actuators.set_servo("left", 0)
        actuators.set_servo("right", 0)
