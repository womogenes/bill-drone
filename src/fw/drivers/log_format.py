"""
Binary flight-log layout, shared by the ESP recorder and laptop decoder.
"""

import struct

SAMPLE_FORMAT = "<II7h2h2H2h3hHHHH"
HEADER_FORMAT = "<4sIHH"
RECORD_SIZE = struct.calcsize(SAMPLE_FORMAT)  # 48 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 12 bytes
# Native control and log rate; must divide the 100 Hz FreeRTOS tick.
SAMPLE_HZ = 100
# Send a batch every 40 ms: 25 messages/s of 204 bytes (one direct-link frame each) at 100 Hz.
BATCH_SIZE = max(1, SAMPLE_HZ // 25)
MAX_BATCH = 8  # 396 bytes: small enough to send without large allocations; the 200-record native ring covers 2 s of stalls

STOP_REASONS = (
    "", "Stopped", "Controller heartbeat lost", "IMU read failed",
    "Tilt limit exceeded", "Control sample delayed", "IMU calibration",
)
