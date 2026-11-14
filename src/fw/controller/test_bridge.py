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
from wire import LINK_CHANNEL, PAYLOAD_SIZE, SerialDecoder, pack, serial_encode, unpack

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


def log_message(first, counts):
    """A log message of FLG1 batches laid end to end (drone/main/radio.c LOG_BATCHES), records numbered from first."""
    message = b""
    for count in counts:
        message += struct.pack(log_format.HEADER_FORMAT, b"FLG1", SESSION, count, log_format.RECORD_SIZE)
        for sequence in range(first, first + count):
            message += struct.pack("<II", sequence * 10000, sequence) + bytes(log_format.RECORD_SIZE - 8)
        first += count
    return message


def fragments(channel, message, sequence):
    parts = [message[start:start + PAYLOAD_SIZE] for start in range(0, len(message), PAYLOAD_SIZE)]
    return [bytes(serial_encode(pack(channel, SESSION, sequence, 1, part, index, len(parts)))) for index, part in enumerate(parts)]


def sent(service):
    """The packets the service queued for the controller since the last call, unpacked."""
    packets = [unpack(SerialDecoder().feed(packet)[0]) for _, packet, _ in service.outgoing]
    service.outgoing.clear()
    return [packet for packet in packets if packet[0] == 5]


async def log_repair(service, device, directory):
    """
    A two-batch log message (a newer drone catching up) with its second fragment lost: the next fragment's arrival and the last's ask for a repair with the fragments held, the lost one completes it, it is written and acknowledged once, a resent copy is acknowledged again without a second write, and a later message after samples the drone dropped is a gap. The recording reads back in order and closes with its counts.
    """
    writer = asyncio.create_task(service.record())
    service.outgoing.clear()
    service.last_status = bridge.time.monotonic()
    device.data = frame(2, b"LGS1" + struct.pack("<II5fB", SESSION, 0, 0, 0, 0, 0, 0, 1), 99)
    service.read()
    await service.record_queue.join()
    sent(service)
    first = fragments(2, log_message(0, [4, 4]), 100)
    assert len(first) == 4
    for index in (0, 2, 3):
        device.data = first[index]
        service.read()
    # Fragment 2 shows 1 lost; the last fragment says again what is held.
    reports = [(p[0], p[2], p[6]) for p in sent(service)]
    assert reports == [(5, 100, bytes([2, 0b101])), (5, 100, bytes([2, 0b1101]))], reports
    device.data = first[1]
    service.read()
    await service.record_queue.join()
    acks = [(p[0], p[2], p[6]) for p in sent(service)]
    assert acks == [(5, 100, bytes([2]))], (acks, service.recorder.error, service.recorder.counts)
    device.data = first[0]
    service.read()
    assert [(p[0], p[2], p[6]) for p in sent(service)] == [(5, 100, bytes([2]))]
    # Samples 8-11 were dropped on the drone; an older drone's single-batch message follows.
    for packet in fragments(2, log_message(12, [4]), 101):
        device.data = packet
        service.read()
    await service.record_queue.join()
    counts = service.recorder.status()["log"]
    assert counts == {"messages": 2, "batches": 3, "samples": 12, "gaps": 1, "missing": 4, "repair_requests": 2, "repaired": 1, "duplicates": 1, "test_dropped": 0}, counts
    service.recorder.close("test")
    data = (directory / service.recorder.file).read_bytes()
    sequences = [struct.unpack_from("<I", data, offset + 4 + record * size)[0] for offset, _, count, size in log_format.batches(data) for record in range(count)]
    assert sequences == [*range(8), *range(12, 16)], sequences
    events = [bridge.json.loads(line) for line in (directory / service.recorder.file).with_suffix(".jsonl").read_text().splitlines()]
    assert [event["type"] for event in events] == ["start", "gap", "log_stats", "closed"], events
    assert events[2]["repaired"] == 1 and events[1]["missing"] == 4
    writer.cancel()
    await asyncio.gather(writer, return_exceptions=True)


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

        await log_repair(service, device, Path(directory))

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
    # The pattern's stop reasons in the drone's order (drone/main/guard.h): a new one there needs its name here.
    guard = (Path(__file__).parents[1] / "drone/main/guard.h").read_text()
    stops = guard.split("enum { PATTERN_PLAYING, ")[1].split("}")[0].split(", ")
    assert len(stops) == len([s for s in log_format.PATTERN_STOPS if s]), stops
    print("ok")


if __name__ == "__main__":
    asyncio.run(main())
