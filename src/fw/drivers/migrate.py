"""
One-time import of JSON settings into the native store. After NVS restores the migration flag on the next boot, remove the old copies so a lost store cannot revive stale calibration.
"""

import errno
import json
import os
import settings
from drivers.calibration import SENSOR_TO_BODY

TUNING_FILE = "/flight-tuning.json"
CALIBRATION_FILE = "/imu-calibration.json"


def load(path):
    try:
        with open(path) as file:
            return json.load(file)
    except OSError as error:
        if error.errno != errno.ENOENT:
            raise
        return {}


def run():
    if settings.migrated():
        for path in (TUNING_FILE, CALIBRATION_FILE):
            try:
                os.remove(path)
            except OSError as error:
                if error.errno != errno.ENOENT:
                    raise
        return
    tuning = load(TUNING_FILE)
    values = {key: value for key, value in tuning.items() if key != "servo_offsets"}
    for side, offset in tuning.get("servo_offsets", {}).items():
        values[f"servo_offset_{side}"] = offset
    calibration = load(CALIBRATION_FILE)
    # As before: a calibration counts only in the sensor frame it was taken in.
    if calibration.get("version") == 1 and calibration["sensor_to_body"] == list(SENSOR_TO_BODY):
        for axis, bias in zip("xyz", calibration["gyro_bias_rad_s"]):
            values[f"gyro_bias_{axis}"] = bias
        values["pose_zero_roll"], values["pose_zero_pitch"] = calibration["pose_zero_deg"]
        values["calibrated"] = 1
        values["calibration_frame"] = sum(1 << axis for axis, sign in enumerate(SENSOR_TO_BODY) if sign < 0)
    settings.migrate(values)
    print(f"Imported {len(values)} settings into the native store")
