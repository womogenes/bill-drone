"""
Binary flight-log layout from the drone (drone/main/flight_rt.c Record and radio.c) and the test pattern's channels, shared by the laptop service, sw/flight_log.py and sw/excite.py.
"""

import struct

# The 48-byte record of older logs, then the hover loop's state (flight_rt.c Record): tag position (mm), sighting age (ms, 65535 none), velocity (mm/s), height (mm), climb rate (mm/s), height and range targets (mm), arc and yaw (centidegrees), trim (mm/s²) and drag-model velocity (mm/s); then the barometer's pressure altitude in the standard atmosphere (mm, BARO_NONE without a reading in the last 100 ms); then the guards (drone/main/guard.h): bits 0-1 the fly-away failsafe's phase (FAILSAFE_PHASES), bit 2 set off by distance, bits 3-5 why the test pattern stopped (PATTERN_STOPS).
BASE_FORMAT = "<II7h2h2H2h3hHHHH"
HOVER_FORMAT = BASE_FORMAT + "3hH12h"
BARO_RECORD_FORMAT = HOVER_FORMAT + "i"
SAMPLE_FORMAT = BARO_RECORD_FORMAT + "H"
HEADER_FORMAT = "<4sIHH"
BASE_SIZE = struct.calcsize(BASE_FORMAT)  # 48 bytes
HOVER_SIZE = struct.calcsize(HOVER_FORMAT)  # 80 bytes
BARO_RECORD_SIZE = struct.calcsize(BARO_RECORD_FORMAT)  # 84 bytes
RECORD_SIZE = struct.calcsize(SAMPLE_FORMAT)  # 86 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 12 bytes
BARO_NONE = -(2**31)
# Each FLG1 header's record size picks the layout, so older recordings still decode.
FORMATS = {BASE_SIZE: BASE_FORMAT, HOVER_SIZE: HOVER_FORMAT, BARO_RECORD_SIZE: BARO_RECORD_FORMAT, RECORD_SIZE: SAMPLE_FORMAT}
MAX_BATCH = 8  # Records per FLG1 batch at most (radio.c LOG_MAX: 5 since the 80-byte record, 8 before).

STOP_REASONS = (
    "", "Stopped", "Controller heartbeat lost", "IMU read failed",
    "Tilt limit exceeded", "Control sample delayed", "IMU calibration", "Landed", "Landed after link loss", "Landed by the fly-away failsafe",
)
# The guards' states (drone/main/guard.h FAILSAFE_* and PATTERN_*), as the record's guard field and the status flags' bits 8-13 hold them.
FAILSAFE_PHASES = (None, "brake", "hold", "land")
PATTERN_STOPS = (None, "distance", "tag lost", "tilt", "stick", None, None, None)


def guard_state(bits):
    """The guards from their bits: the failsafe's phase (None when off), whether distance set it off, and why the test pattern stopped (None when it did not)."""
    return {"failsafe": FAILSAFE_PHASES[bits & 3], "failsafe_far": bool(bits & 4), "pattern_stop": PATTERN_STOPS[bits >> 3 & 7]}

# The test pattern's channels in the drone's order (drone/main/flight_rt.c excite), and their units: roll and pitch targets and servo tilts in degrees (servos positive forward), the yaw-rate target in degrees/s, collective and motors in throttle, the hover hold point's speed toward the tag and to the drone's left in m/s.
PATTERN_CHANNELS = ("roll", "pitch", "yaw", "base", "left_servo", "right_servo", "left_motor", "right_motor", "forward", "left")
