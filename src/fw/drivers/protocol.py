"""
What the application adds to the native link: its block of the status message (native/runtime/runtime.h STATUS_APP) and the recording metadata, both decoded by controller/bridge.py.
"""

import json
import struct
import settings
from drivers import actuators, imu
from drivers.calibration import SENSOR_TO_BODY, THRUST_TABLE
from drivers.log_format import BATCH_SIZE, RECORD_SIZE, SAMPLE_FORMAT, SAMPLE_HZ

# Force, force age (ms), force sequence, ADC raw, battery volts, settings version, flags (1 force valid).
APP_FORMAT = "<iIIIfHBx"
block = bytearray(struct.calcsize(APP_FORMAT))


def app_status(force, force_age_ms, force_sequence, adc_raw, voltage):
    struct.pack_into(APP_FORMAT, block, 0, force or 0, force_age_ms, force_sequence, adc_raw, voltage,
                     settings.version() & 0xFFFF, int(force is not None))
    return block


def metadata():
    """
    The recording's description; the native recorder resends it after each start and whenever it changes.
    """
    return json.dumps({
        "type": "metadata",
        "version": 1,
        "sample_format": SAMPLE_FORMAT,
        "record_size": RECORD_SIZE,
        "target_hz": SAMPLE_HZ,
        "batch_ms": 1000 * BATCH_SIZE // SAMPLE_HZ,
        "imu_dlpf": imu.DLPF,
        "pwm_hz": actuators.PWM_HZ,
        "control_core": 1,
        "network_core": 0,
        "sensor_to_body": SENSOR_TO_BODY,
        "servo_offsets": {side: settings.get(f"servo_offset_{side}") for side in actuators.SIDES},
        "settings": json.loads(settings.json()),
        "thrust_table": THRUST_TABLE,
    }).encode()
