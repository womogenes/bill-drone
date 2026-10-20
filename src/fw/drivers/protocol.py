"""
Dashboard wire format. All numbers are little-endian; see README.md.
"""

import struct
import time
from .flight import CONTROL_KEYS

TELEMETRY_FORMAT = "<4sIIi5I41f"


def decode_command(packet):
    code = packet[0]
    if code == ord("Z"):
        return {"command": "zero_servo", "side": ("left", "right")[packet[1]]}

    if code == ord("P"):
        return {
            "command": "control",
            "settings": {
                CONTROL_KEYS[packet[offset]]: struct.unpack_from(
                    "<f", packet, offset + 1
                )[0]
                for offset in range(1, len(packet), 5)
            },
        }

    if code in (ord("S"), ord("T")):
        return {
            "command": "servo" if code == ord("S") else "throttle",
            "side": ("left", "right")[packet[1]],
            "value": struct.unpack_from("<f", packet, 2)[0],
        }

    return {
        "command": {ord("H"): "heartbeat", ord("X"): "stop", ord("C"): "calibrate_imu"}[
            code
        ]
    }


def encode_reading(reading):
    imu = reading["imu"]
    logging = reading["logging"]
    control = reading["control"]
    left, right = reading["actuators"]["left"], reading["actuators"]["right"]
    flags = (
        int(imu is not None)
        | (int(imu is not None and imu["calibrating"]) << 1)
        | (int(logging["connected"]) << 2)
        | (int(logging["active"]) << 3)
        | (int(logging["ready"]) << 4)
        | (int(bool(control["enabled"])) << 5)
        | (int(reading["force"] is not None) << 6)
        | (int(reading["control_status"]["mapping_confirmed"]) << 7)
        | (int(imu is not None and imu["calibrated"]) << 8)
    )
    axes = (
        imu["accel"] + imu["gyro"] + [imu["roll"], imu["pitch"], imu["temperature"]]
        if imu
        else [0.0] * 9
    )

    return struct.pack(
        TELEMETRY_FORMAT,
        b"TEL1",
        time.ticks_ms(),
        flags,
        reading["force"] or 0,
        reading["force_age_ms"] or 0,
        reading["force_sequence"],
        reading["adc_raw"],
        logging["samples"],
        logging["dropped"],
        reading["adc_voltage"],
        left["servo_angle"],
        left["throttle"],
        right["servo_angle"],
        right["throttle"],
        *axes,
        *(control[key] for key in CONTROL_KEYS[1:]),
    )
