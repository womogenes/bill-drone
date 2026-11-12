# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5", "aiohttp>=3.11,<4"]
# ///
"""
The bridge takes one packet of each kind the controller's USB carries, passes over one it cannot take, and outlives its controller's device: uv run src/fw/controller/test_bridge.py

2026-11-10 a merge dropped the return after receive()'s GPS-report branch, and each report was decoded as a status message from then on; this fails on that.
"""

import asyncio
import os
import struct
import tempfile
from pathlib import Path

import bridge
import log_format
from wire import LINK_CHANNEL, pack, serial_encode

SESSION = 7


class Device:
    """A stand-in for the controller's serial device: read() gives what was queued, or fails like an unplugged one."""

    def __init__(self):
        self.port = "test"
        self.data = b""
        self.unplugged = False
        self.reader, self.writer = os.pipe()

    @property
    def in_waiting(self):
        return len(self.data)

    def fileno(self):
        return self.reader

    def read(self, size):
        if self.unplugged:
            raise OSError("device gone")
        data, self.data = self.data, b""
        return data

    def close(self):
        os.close(self.reader)
        os.close(self.writer)


def frame(channel, payload, sequence):
    return bytes(serial_encode(pack(channel, SESSION, sequence, 1, payload)))


async def main():
    with tempfile.TemporaryDirectory() as directory:
        service = bridge.Service(Path(directory))
        assert service.state() == "controller_missing"
        device = Device()
        service.attach(device)
        assert service.state() == "drone_silent"

        status = bytes(bridge.STATUS_SIZE + bridge.CAMERA_SIZE + bridge.GPS_SIZE + struct.calcsize(bridge.BARO_FORMAT))
        device.data = frame(0, status, 1)
        service.read()
        assert service.state() == "online" and service.reading["time"] == 0

        # A recovery rides in the status reason's top bits (drone/main/runtime.h Status): leveling after the pilot's level-and-hold, with no stop reason.
        version, reading = bridge.decode_status(status[:14] + bytes([2 << 6 | 3 << 4]) + status[15:], None)
        assert reading["control_status"]["recovery"] == "leveling" and reading["control_status"]["recovery_cause"] == "level" and reading["control_status"]["reason"] == "", reading["control_status"]
        version, reading = bridge.decode_status(status[:14] + bytes([10]) + status[15:], None)
        assert reading["control_status"]["recovery"] is None and reading["control_status"]["reason"] == "Recovery failed", reading["control_status"]

        # A GPS report with no satellites is shorter than a status message, and one with many is longer: neither is the reading.
        reading = service.reading
        for satellites in (0, 20):
            report = struct.pack(bridge.GPS_REPORT_FORMAT, b"GPD1", *[0] * (len(struct.unpack(bridge.GPS_REPORT_FORMAT, bytes(bridge.GPS_REPORT_SIZE))) - 1))
            device.data = frame(0, report + bytes(5 * satellites), 2 + satellites)
            service.read()
            assert service.reading is reading and service.gps_report is not None

        link = struct.pack(bridge.LINK_FORMAT, b"RLY1", *[0] * (len(struct.unpack(bridge.LINK_FORMAT, bytes(struct.calcsize(bridge.LINK_FORMAT)))) - 1))
        # A status message too short to decode, then a link report in the same read: the first is counted, the second taken.
        device.data = frame(0, b"bad", 30) + frame(LINK_CHANNEL, link, 31)
        service.read()
        assert service.errors == 1 and service.link["controller"] is not None, (service.errors, service.link)

        device.unplugged = True
        service.read()
        assert service.port is None and service.state() == "controller_missing"
        for task in (service.settings_task, service.firmware_task):
            if task is not None:
                task.cancel()

    # The pattern limits the bridge and the simulator check are the drone's own: until 2026-11-12 the drone capped roll at 10 degrees while the ladder was simmed at 35.
    source = (Path(__file__).parents[1] / "drone/main/flight_rt.c").read_text()
    firmware = source.split("excite_limit[EXCITE_CHANNELS] = {")[1].split("}")[0]
    assert tuple(float(value) for value in firmware.split(",")) == log_format.PATTERN_LIMITS, firmware
    # The recovery's names in the drone's order (drone/main/recover.h), and its failed righting as the stop reason the firmware gives it.
    header = (Path(__file__).parents[1] / "drone/main/recover.h").read_text()
    stages = header.split("enum { RECOVER_IDLE, ")[1].split("}")[0].split(", ")
    causes = header.split("enum { RECOVER_NONE, ")[1].split("}")[0].split(", ")
    assert len(stages) == len([s for s in log_format.RECOVERY_STAGES if s]) and len(causes) == len([c for c in log_format.RECOVERY_CAUSES if c]), (stages, causes)
    assert log_format.STOP_REASONS[int(header.split("#define RECOVER_FAILED_REASON ")[1].split()[0])] == "Recovery failed"
    print("ok")


if __name__ == "__main__":
    asyncio.run(main())
