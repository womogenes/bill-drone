# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5", "aiohttp>=3.11,<4"]
# ///
"""
The bridge takes one packet of each kind the controller's USB carries, passes over one it cannot take, and outlives its controller's device; on simulated USB serial ports (pty pairs) it finds the controller by its packets among receivers, a console and a silent board, never opens a port another process holds, writes nothing while it looks, and finds the controller again after a re-plug onto another device or a silence: uv run src/fw/controller/test_bridge.py

2026-11-10 a merge dropped the return after receive()'s GPS-report branch, and each report was decoded as a status message from then on; this fails on that.
"""

import asyncio
import json
import os
import struct
import subprocess
import sys
import tempfile
import time
import tty
from pathlib import Path

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

import bridge
import find_port
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


def log_message(first, counts, size=log_format.CAMERA_RECORD_SIZE):
    """A log message of FLG1 batches laid end to end (drone/main/radio.c LOG_BATCHES), records numbered from first; 112-byte records by default, whose fragment layout log_repair counts on (the bridge takes every size log_format.FORMATS knows)."""
    message = b""
    for count in counts:
        message += struct.pack(log_format.HEADER_FORMAT, b"FLG1", SESSION, count, size)
        for sequence in range(first, first + count):
            message += struct.pack("<II", sequence * 10000, sequence) + bytes(size - 8)
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
    # Samples 8-11 were dropped on the drone; a message of today's 120-byte records follows (a 3-record batch and a 1-record remainder).
    for packet in fragments(2, log_message(12, [3, 1], log_format.RECORD_SIZE), 101):
        device.data = packet
        service.read()
    await service.record_queue.join()
    counts = service.recorder.status()["log"]
    assert counts == {"messages": 2, "batches": 4, "samples": 12, "gaps": 1, "missing": 4, "repair_requests": 2, "repaired": 1, "duplicates": 1, "test_dropped": 0}, counts
    service.recorder.close("test")
    data = (directory / service.recorder.file).read_bytes()
    sequences = [struct.unpack_from("<I", data, offset + 4 + record * size)[0] for offset, _, count, size in log_format.batches(data) for record in range(count)]
    assert sequences == [*range(8), *range(12, 16)], sequences
    events = [bridge.json.loads(line) for line in (directory / service.recorder.file).with_suffix(".jsonl").read_text().splitlines()]
    assert [event["type"] for event in events] == ["start", "gap", "log_stats", "closed"], events
    assert events[2]["repaired"] == 1 and events[1]["missing"] == 4
    writer.cancel()
    await asyncio.gather(writer, return_exceptions=True)


class Board:
    """A simulated USB serial board: a pty whose slave is the device the bridge sees and whose master is the board, sending what it is given every 50 ms while on."""

    def __init__(self, sends=b""):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        tty.setraw(slave)
        os.close(slave)
        os.set_blocking(self.master, False)
        self.sends = sends
        self.on = bool(sends)
        self.task = asyncio.create_task(self.run())

    async def run(self):
        while True:
            if self.on and self.master is not None:
                try:
                    os.write(self.master, self.sends)
                except (BlockingIOError, OSError):
                    pass
            await asyncio.sleep(0.05)

    def written(self):
        """What the bridge wrote to the board."""
        try:
            return os.read(self.master, 65536)
        except (BlockingIOError, OSError):
            return b""

    def unplug(self):
        os.close(self.master)
        self.master = None


def hold(path, lock):
    """Another process holding the device: open and flocked (an upload, the dashboard's brief lock) or only open and read (the dashboard's FPV receiver reader)."""
    code = f"""import fcntl, os, sys, time
fd = os.open({path!r}, os.O_RDWR | os.O_NOCTTY)
if {lock}:
    fcntl.flock(fd, fcntl.LOCK_EX)
print("held", flush=True)
while True:
    try:
        os.read(fd, 4096)
    except OSError:
        time.sleep(0.05)"""
    process = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
    assert process.stdout.readline().strip() == "held"
    return process


async def until(condition, seconds=10):
    deadline = time.monotonic() + seconds
    while not condition():
        assert time.monotonic() < deadline, "timed out"
        await asyncio.sleep(0.02)


async def status(service):
    return bridge.json.loads((await service.status(None)).text)


async def usb_search(directory):
    """
    The controller found among other boards by its packets, wherever it is plugged in. Boards: a silent one, an FPV receiver held open by the dashboard, a board flocked by another process, a flight controller's console that nobody holds, and two sockets the controller is plugged into in turn. Before the controller is plugged in, a search looks at every free board and passes over the held ones unopened; the controller is found on the first socket, then on the second after it is unplugged from the first, and again after a silence. The bridge writes nothing to any board while looking, and never opens a held one.
    """
    controller = bytes(serial_encode(pack(LINK_CHANNEL, SESSION, 1, 0, struct.pack(bridge.LINK_FORMAT, b"RLY1", *[0] * (len(struct.unpack(bridge.LINK_FORMAT, bytes(struct.calcsize(bridge.LINK_FORMAT)))) - 1)))))
    silent, receiver, uploading, console, first, second = boards = [Board(), Board(b"FPVR"), Board(b"FPVR"), Board(b"I (1234) flight: armed 0\r\n"), Board(), Board()]
    holders = [hold(receiver.path, False), hold(uploading.path, True)]
    assert find_port.holder(receiver.path) == "open" and find_port.holder(uploading.path) == "locked" and find_port.holder(silent.path) is None
    saved = bridge.LISTEN_S, bridge.SCAN_S, bridge.SILENT_S, bridge.RETRY_S, bridge.listen
    bridge.LISTEN_S, bridge.SCAN_S, bridge.SILENT_S, bridge.RETRY_S = 0.3, 0.05, 0.6, 0.5
    opened = []

    def listen(path, *arguments):
        opened.append(path)
        return saved[4](path, *arguments)

    bridge.listen = listen
    service = bridge.Service(directory)
    paths = [board.path for board in boards]
    tasks = [asyncio.create_task(task) for task in (service.record(), service.monitor(), bridge.search(service, "", 921600, lambda: [path for path in paths if os.path.exists(path)]))]
    try:
        # No controller yet: every free board is listened to, the held ones are passed over unopened.
        await until(lambda: service.usb["searches"] >= 2)
        assert service.port is None and service.usb["skipped"] == {receiver.path: "open", uploading.path: "locked"}, service.usb
        assert {silent.path, console.path, first.path, second.path} <= set(opened), opened
        first.sends, first.on = controller, True
        await until(lambda: service.port is not None)
        assert service.port.port == first.path and service.state() == "drone_silent"
        report = await status(service)
        assert report["port"] == first.path and report["rescans"] == 0, report
        # The dashboard skips the controller's port from the moment it is found.
        seen = subprocess.run([sys.executable, "-c", f"import sys; sys.path.insert(0, {str(Path(__file__).parent)!r}); import find_port; print(find_port.holder({first.path!r}))"], capture_output=True, text=True, check=True).stdout.strip()
        assert seen == "locked", seen
        # Unplugged and plugged into another socket: another device name.
        first.unplug()
        second.sends, second.on = controller, True
        await until(lambda: service.port is not None and service.port.port == second.path)
        report = await status(service)
        assert report["port"] == second.path and report["rescans"] == 1 and report["usb"]["last_loss"] == "[Errno 5] Input/output error", report
        # A controller that stops sending is lost after SILENT_S and found again on the same device once it sends.
        second.on = False
        await until(lambda: service.port is None)
        assert service.usb["last_loss"] == f"No packets for {bridge.SILENT_S:g} s"
        second.on = True
        await until(lambda: service.port is not None)
        report = await status(service)
        assert report["port"] == second.path and report["rescans"] == 2 and report["usb"]["silent_s"] < 1, report
        assert receiver.path not in opened and uploading.path not in opened, opened
        for board in (silent, console, receiver, uploading):
            assert board.written() == b"", board.path
    finally:
        bridge.LISTEN_S, bridge.SCAN_S, bridge.SILENT_S, bridge.RETRY_S, bridge.listen = saved
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        service.detach("test")
        for process in holders:
            process.kill()
            process.wait()
        for board in boards:
            board.task.cancel()
            if board.master is not None:
                os.close(board.master)

    # DTR and RTS are released in one request, never DTR alone while RTS is still asserted (that state holds an ESP32's EN low).
    calls = []
    ioctl = find_port.fcntl.ioctl
    find_port.fcntl.ioctl = lambda fd, request, argument: calls.append((request, struct.unpack("I", argument)[0]))
    try:
        port = find_port.QuietSerial()
        port.dtr = port.rts = False
        port.fd = -1
        port._update_dtr_state()
        port._update_rts_state()
    finally:
        find_port.fcntl.ioctl = ioctl
    both = find_port.termios.TIOCM_DTR | find_port.termios.TIOCM_RTS
    assert calls == [(find_port.termios.TIOCMBIC, both)] * 2, calls

    # The controller's last physical port comes first, then the device --port named, then the rest by name.
    with tempfile.TemporaryDirectory() as links:
        link = os.path.join(links, "pci-0000:c1:00.3-usb-0:1:1.0-port0")
        os.symlink("/dev/null", link)
        assert bridge.search_order(["/dev/a", "/dev/null", "/dev/z", "/dev/zero"], [link], "/dev/zero") == ["/dev/null", "/dev/zero", "/dev/a", "/dev/z"]
        assert bridge.search_order(["/dev/b", "/dev/a"], [], "") == ["/dev/a", "/dev/b"]


async def recordings(directory):
    """
    GET /recordings: a recording of 120 records (1.2 s) with a batch resent is listed and a 3-record one is not; its columns come back once per record in sequence order with null for NaN, past a line cut off mid-write; an unknown name or column is refused.
    """
    name, short = "2026-11-15T07-02-48-960Z_7", "2026-11-15T07-03-00-000Z_7"
    (directory / f"{name}.bin").write_bytes(log_message(0, [3] * 40, log_format.RECORD_SIZE) + log_message(5, [3], log_format.RECORD_SIZE))
    (directory / f"{short}.bin").write_bytes(log_message(0, [3], log_format.RECORD_SIZE))
    start = json.dumps({"type": "start", "session": SESSION, "gyro_bias_rad_s": [0, 0, 0], "sensor_to_body": [1, -1, -1]})
    for stem in (name, short):
        (directory / f"{stem}.jsonl").write_text(start + '\n{"type": "gps", "lat')
    app = web.Application()
    app.add_routes([web.get("/recordings", bridge.Service(directory).recordings)])
    async with TestClient(TestServer(app)) as client:
        listed = await (await client.get("/recordings")).json()
        assert [(entry["name"], entry["started"]) for entry in listed] == [(name, "2026-11-15T07:02:48.960Z")], listed
        response = await client.get("/recordings", params={"name": name, "columns": "sequence,esp_time_us,tag_qw"})
        body = await response.json()
        assert body["columns"]["sequence"] == list(range(120)) and body["columns"]["esp_time_us"] == [sequence * 10000 for sequence in range(120)], body["columns"]
        assert body["columns"]["tag_qw"] == [None] * 120 and not body["truncated"] and [event["type"] for event in body["events"]] == ["start"], body
        for query in ({"name": "../" + name}, {"name": name, "columns": "nope"}):
            response = await client.get("/recordings", params=query)
            assert response.status == 404, (query, await response.text())


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

    with tempfile.TemporaryDirectory() as directory:
        await usb_search(Path(directory))

    with tempfile.TemporaryDirectory() as directory:
        await recordings(Path(directory))

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
