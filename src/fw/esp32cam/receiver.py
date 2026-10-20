"""
Receive camera poses over SPI on the MicroPython flight controller.
Call poll from asyncio; read latest from control code.
"""

import struct
from time import ticks_ms, ticks_diff

PACKET_SIZE = 80
FORMAT = "<4sIIIHH14fHH"


def crc16(data):
    """
    CRC-16/CCITT-FALSE, shared with packet.h.
    """
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for bit in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


class TagPose:
    def __init__(self, spi, cs, ready, tag_id=0, max_age_ms=250):
        self.spi, self.cs, self.ready = spi, cs, ready
        self.cs(1)
        self.tag_id = tag_id
        self.max_age_ms = max_age_ms
        self.buffer = bytearray(PACKET_SIZE)
        self.pose = None
        self.received_ms = ticks_ms()
        self.polled_ms = self.received_ms
        self.synced = False
        self.last_sequence = None

    def poll(self):
        """
        Clock one ready packet. Never wait for camera processing.
        A startup packet or a packet queued during a polling stall is discarded.
        """
        now = ticks_ms()
        poll_gap = ticks_diff(now, self.polled_ms)
        self.polled_ms = now
        if not self.ready():
            self.synced = True
            return
        self.cs(0)
        try:
            self.spi.readinto(self.buffer)
        finally:
            self.cs(1)
        self.pose = None
        if not self.synced or poll_gap > self.max_age_ms:
            self.synced = True
            return
        values = struct.unpack(FORMAT, self.buffer)
        if values[0] != b"TAG1" or values[-1] != crc16(memoryview(self.buffer)[:-2]):
            return
        if values[1] == self.last_sequence:
            return  # An ISR delayed during OTA must never refresh an old pose.
        self.last_sequence = values[1]
        # READY can have risen any time since the previous poll. Include that
        # entire interval, plus a rounded-up 1 ms for the 4 MHz transfer.
        age_ms = (values[3] + 999) // 1000 + poll_gap + 1
        if values[4] == self.tag_id and values[5] == 1 and age_ms <= self.max_age_ms:
            self.pose = {
                "sequence": values[1],
                "capture_us": values[2],
                "age_ms": age_ms,
                "position_m": values[6:9],
                "rotation": values[9:18],
                "margin": values[18],
                "reprojection_px": values[19],
            }
            self.received_ms = now

    @property
    def latest(self):
        """
        Return a fresh pose, or None. Also expires if the polling task stalls.
        """
        now = ticks_ms()
        if self.pose is not None:
            self.pose["age_ms"] += ticks_diff(now, self.received_ms)
            self.received_ms = now
            if self.pose["age_ms"] > self.max_age_ms:
                self.pose = None
        return self.pose
