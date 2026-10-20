"""
Binary flight-log layout from the drone (drone/main/flight_rt.c Record and radio.c), shared by the laptop service and sw/flight_log.py.
"""

import struct

SAMPLE_FORMAT = "<II7h2h2H2h3hHHHH"
HEADER_FORMAT = "<4sIHH"
RECORD_SIZE = struct.calcsize(SAMPLE_FORMAT)  # 48 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 12 bytes
MAX_BATCH = 8  # Records per FLG1 batch at most (radio.c LOG_MAX).

STOP_REASONS = (
    "", "Stopped", "Controller heartbeat lost", "IMU read failed",
    "Tilt limit exceeded", "Control sample delayed", "IMU calibration",
)
