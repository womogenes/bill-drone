"""
Binary flight-log layout from the drone (drone/main/flight_rt.c Record and radio.c), shared by the laptop service and sw/flight_log.py.
"""

import struct

# The 48-byte record of older logs, then the hover loop's state (flight_rt.c Record): tag position (mm), sighting age (ms, 65535 none), velocity (mm/s), height (mm), climb rate (mm/s), height and range targets (mm), arc and yaw (centidegrees), trim (mm/s²) and drag-model velocity (mm/s).
BASE_FORMAT = "<II7h2h2H2h3hHHHH"
SAMPLE_FORMAT = BASE_FORMAT + "3hH12h"
HEADER_FORMAT = "<4sIHH"
BASE_SIZE = struct.calcsize(BASE_FORMAT)  # 48 bytes
RECORD_SIZE = struct.calcsize(SAMPLE_FORMAT)  # 80 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 12 bytes
# Each FLG1 header's record size picks the layout, so older recordings still decode.
FORMATS = {BASE_SIZE: BASE_FORMAT, RECORD_SIZE: SAMPLE_FORMAT}
MAX_BATCH = 8  # Records per FLG1 batch at most (radio.c LOG_MAX: 5 since the 80-byte record, 8 before).

STOP_REASONS = (
    "", "Stopped", "Controller heartbeat lost", "IMU read failed",
    "Tilt limit exceeded", "Control sample delayed", "IMU calibration", "Landed", "Landed after link loss",
)
