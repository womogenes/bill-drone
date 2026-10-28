# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5", "aiohttp>=3.11,<4"]
# ///
"""
The laptop's drone service. It owns the controller's USB port and serves, on 127.0.0.1 only:

- /ws: the dashboard. The drone's status as JSON readings; the browser streams targets (including the manual/hover/land mode) every 50 ms and sends stop, arm, calibrate, zero and tune requests.
- /update: raw id-tagged requests for tools/ota_direct.py.
- GET /status, GET /settings, POST /settings {"settings": {...}}, GET /schema: status, tuning and the settings' names, ranges and defaults for local tools.
- POST /pattern {"after": s, "repeat": n, "segments": [{"ticks": n, "<channel>": value, ...}, ...]}: load the identification test pattern (PATTERN_CHANNELS; drone/main/flight_rt.c excite), or clear it with no segments. It lasts until the drone restarts, and every recording from then on starts with it as a "pattern" event. sw/excite.py builds and sends them.

It records every armed session into src/sw/recordings/<time>_<session>.bin (FLG1 batches) and .jsonl (start, metadata, gap, error, stop and closed events; sw/flight_log.py decodes both).
"""

import argparse
import asyncio
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import struct
import sys
import time

from aiohttp import WSMsgType, web
import serial

from log_format import HEADER_FORMAT, HEADER_SIZE, RECORD_SIZE, SAMPLE_FORMAT, STOP_REASONS
from wire import PAYLOAD_SIZE, SerialDecoder, pack, serial_encode, unpack

RECORDINGS = Path(__file__).resolve().parents[2] / "sw/recordings"

# drone/main/runtime.h Status, then its camera link fields (absent from firmware before the camera link).
STATUS_FORMAT = "<4sIIHB3f3f3f2f2H4f2IiIIHHf"
CAMERA_FORMAT = "<15I2B"
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)
CAMERA_KEYS = (
    "packets", "frames", "poses", "flows", "corrupt", "repeats", "stale",
    "unsynced", "skipped_bytes", "overflows", "line_errors", "sync_edges", "rx_edges",
    "sequence", "age_ms", "valid", "trusted",
)
REQUESTS = {"arm": b"A", "calibrate": b"G", "zero": b"Z", "tune": b"T"}
# Schema entries per D reply (drone/main/settings.h SCHEMA_PAGE).
SCHEMA_PAGE = 12


def decode_status(payload, settings):
    """
    The dashboard's reading (sw/dashboard/lib/telemetry.ts Reading) from a status message.
    """
    (magic, time_ms, target_sequence, flags, reason, *values) = struct.unpack_from(STATUS_FORMAT, payload)
    camera = None
    if len(payload) == STATUS_SIZE + struct.calcsize(CAMERA_FORMAT):
        camera = dict(zip(CAMERA_KEYS, struct.unpack_from(CAMERA_FORMAT, payload, STATUS_SIZE)))
        # No frame yet: the age is meaningless.
        if camera["age_ms"] == 0xFFFFFFFF:
            camera["age_ms"] = None
    accel, gyro = values[0:3], values[3:6]
    roll, pitch, temperature, left_servo, right_servo, left_throttle, right_throttle = values[6:13]
    target_roll, target_pitch, target_yaw, base = values[13:17]
    samples, dropped, force, force_age, force_sequence, adc_raw, version, voltage = values[17:]
    return version, {
        "time": time_ms,
        "force": force if flags & 128 else None,
        "force_age_ms": force_age if flags & 128 else None,
        "force_sequence": force_sequence,
        "adc_raw": adc_raw,
        "adc_voltage": voltage,
        "actuators": {
            "left": {"servo_angle": left_servo, "throttle": left_throttle},
            "right": {"servo_angle": right_servo, "throttle": right_throttle},
        },
        "imu": {
            # Calibrating includes a finished calibration the drone has not saved yet.
            "calibrated": bool(flags & 4) and not flags & 2,
            "calibrating": bool(flags & 2),
            "accel": list(accel),
            "gyro": list(gyro),
            "roll": roll,
            "pitch": pitch,
            "temperature": temperature,
        } if flags & 1 else None,
        "control": {
            **(settings or {}),
            "enabled": bool(flags & 8),
            "roll": target_roll,
            "pitch": target_pitch,
            "yaw_rate": target_yaw,
            "base_throttle": base,
        },
        "control_status": {
            "reason": STOP_REASONS[reason],
            "throttle_latched": bool(flags & 64),
            "target_sequence": target_sequence,
        },
        "logging": {
            "active": bool(flags & 16),
            "ready": bool(flags & 32),
            "samples": samples,
            "dropped": dropped,
        },
        "camera": camera,
    }


def now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


# The test pattern's channels in the drone's order, and their units: roll and pitch targets and servo tilts in degrees (servos positive forward), the yaw-rate target in degrees/s, collective and motors in throttle.
PATTERN_CHANNELS = ("roll", "pitch", "yaw", "base", "left_servo", "right_servo", "left_motor", "right_motor")


def encode_pattern(pattern):
    """The P request body for a pattern (drone/main/flight_rt.c flight_runtime_pattern), and the pattern as recorded."""
    segments = pattern.get("segments", [])
    unknown = {key for segment in segments for key in segment} - {"ticks", *PATTERN_CHANNELS}
    if unknown:
        raise ValueError(f"Unknown pattern channels {sorted(unknown)}")
    used = [i for i, name in enumerate(PATTERN_CHANNELS) if any(segment.get(name) for segment in segments)]
    mask = sum(1 << i for i in used)
    body = struct.pack("<fBBH", float(pattern.get("after", 2.0)), int(pattern.get("repeat", 0)), mask, len(segments))
    for segment in segments:
        body += struct.pack("<H", int(segment["ticks"])) + b"".join(struct.pack("<h", round(segment.get(PATTERN_CHANNELS[i], 0) * 100)) for i in used)
    recorded = {"after": float(pattern.get("after", 2.0)), "repeat": int(pattern.get("repeat", 0)),
                "segments": [{"ticks": int(segment["ticks"]), **{PATTERN_CHANNELS[i]: round(segment.get(PATTERN_CHANNELS[i], 0) * 100) / 100 for i in used}} for segment in segments]}
    return body, recorded


class Recorder:
    """
    One file pair per armed session, kept open through link hiccups until the drone's stop record arrives.
    """

    def __init__(self, directory):
        self.directory = directory
        self.metadata = None
        self.session = None
        self.data = self.events = None
        self.file = None
        self.bytes = 0
        self.next_sequence = None
        self.last_message = 0
        self.stalled = False
        self.error = None
        # The test pattern last loaded through this service, written at the start of every recording.
        self.pattern = None

    def status(self):
        return {"file": self.file, "bytes": self.bytes, "writing": self.data is not None, "error": self.error}

    def event(self, message):
        self.events.write(json.dumps({**message, "received_at": now()}) + "\n")

    def receive(self, channel, payload):
        self.last_message = time.monotonic()
        if self.stalled:
            self.stalled = False
            self.error = None
            if self.data is not None:
                self.event({"type": "resumed"})
        if channel == 3:
            self.metadata = json.loads(payload)
            if self.data is not None:
                self.event(self.metadata)
            return
        magic = payload[:4]
        if magic == b"LGS1":
            session, started = struct.unpack_from("<II", payload, 4)
            bias, pose = struct.unpack_from("<3f", payload, 12), struct.unpack_from("<2f", payload, 24)
            self.begin(session, {"started_ticks_us": started, "gyro_bias_rad_s": list(bias),
                                "pose_zero_deg": list(pose), "calibrated": bool(payload[32])})
            self.next_sequence = 0
        elif magic == b"FLG1":
            _, session, count, size = struct.unpack_from(HEADER_FORMAT, payload)
            if session != self.session or self.data is None:
                # This service started during the session and missed its start record.
                self.begin(session, {"resumed": True})
            first = struct.unpack_from("<I", payload, HEADER_SIZE + 4)[0]
            if self.next_sequence is not None and first != self.next_sequence:
                self.event({"type": "gap", "after": self.next_sequence - 1, "missing": first - self.next_sequence})
            self.next_sequence = first + count
            self.data.write(payload)
            self.bytes += len(payload)
        elif magic == b"LGE1":
            session, reason, samples, dropped, elapsed = struct.unpack_from("<IBIII", payload, 4)
            if session == self.session and self.data is not None:
                self.event({"type": "stop", "session": session, "reason": STOP_REASONS[reason] or "Disarmed",
                            "time_us": elapsed, "samples": samples, "dropped": dropped})
                self.close(None)
        else:
            raise ValueError(f"Unknown flight-log message {magic!r}")

    def begin(self, session, start):
        self.close("New session")
        self.directory.mkdir(parents=True, exist_ok=True)
        stem = self.directory / f"{now().replace(':', '-').replace('.', '-')}_{session}"
        self.data, self.events = open(f"{stem}.bin", "xb"), open(f"{stem}.jsonl", "x")
        self.file, self.bytes, self.session, self.next_sequence, self.error = f"{stem.name}.bin", 0, session, None, None
        # The first line describes the whole recording.
        self.event({**(self.metadata or {}), "type": "start", "session": session, **start})
        if self.pattern:
            self.event({"type": "pattern", **self.pattern})

    def check(self, recording):
        """
        Note a stalled stream in the recording; the file stays open for the rest of the session.
        """
        if self.data is None:
            return
        if recording and not self.stalled and time.monotonic() - self.last_message > 1:
            self.stalled = True
            self.error = "Flight log stalled"
            self.event({"type": "error", "error": "Flight log stalled for 1 s"})
        self.data.flush()
        self.events.flush()
        os.fsync(self.data.fileno())
        os.fsync(self.events.fileno())

    def close(self, reason):
        if self.data is None:
            return
        if reason is not None:
            self.event({"type": "closed", "reason": reason})
        for file in (self.data, self.events):
            file.flush()
            os.fsync(file.fileno())
            file.close()
        self.data = self.events = None


def encode_targets(message):
    """
    A 'T' target packet (drone/main/flight_rt.c flight_runtime_targets) from the browser's current controls: base throttle, roll, pitch, yaw rate, two servo angles, two throttles, the flight mode and the hover hold-point nudge (m; toward the tag, left, up).

    Mode 0 is manual, 1 hover (the drone holds position, height and heading from the camera), 2 land and 3 land fast. In modes 1-3 the drone computes its own base throttle, roll, pitch and yaw rate and ignores the streamed ones. Clients that send no mode get manual.
    """
    values = [message[key] for key in ("base_throttle", "roll", "pitch", "yaw_rate")]
    nudge = (list(message.get("nudge", ())) + [0, 0, 0])[:3]
    return b"T" + struct.pack("<12f", *values, *message["servo"], *message["throttle"], message.get("mode", 0), *nudge)


class ClientSender:
    """
    One sender per socket. Readings replace unsent readings; request replies retain their order.
    """

    def __init__(self, ws):
        self.ws = ws
        self.reading = None
        self.replies = deque()
        self.wake = asyncio.Event()
        self.overflow = False
        self.task = asyncio.create_task(self.run())

    def push(self, data, binary=False):
        if self.ws.closed or self.task.done():
            return
        if not binary and data.get("type") == "reading":
            self.reading = data
        elif len(self.replies) < 16:
            self.replies.append((data, binary))
        else:
            self.overflow = True
        self.wake.set()

    async def run(self):
        try:
            while True:
                await self.wake.wait()
                self.wake.clear()
                while self.replies or self.reading is not None:
                    if self.overflow:
                        return
                    if self.replies:
                        data, binary = self.replies.popleft()
                    else:
                        data, binary = self.reading, False
                        self.reading = None
                    send = self.ws.send_bytes(data) if binary else self.ws.send_json(data)
                    await asyncio.wait_for(send, 0.25)
        except (ConnectionError, TimeoutError):
            pass
        finally:
            try:
                await asyncio.wait_for(self.ws.close(), 0.25)
            except (ConnectionError, TimeoutError):
                pass

    async def close(self):
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)


class Service:
    def __init__(self, port, recordings):
        self.port = port
        self.decoder = SerialDecoder()
        self.outgoing = deque()
        self.target = None
        self.urgent = None
        self.wake = asyncio.Event()
        self.failed = asyncio.Event()
        self.session = 0
        self.link_generation = 0
        self.token = 0
        self.last_status = 0
        self.reading = None
        self.settings = None
        self.settings_version = None
        self.sequence = int(time.time() * 1000) & 0xFFFFFFFF
        self.fragments = {}
        self.waiting = {}
        self.dashboard = None
        self.updater = None
        self.recorder = Recorder(recordings)
        self.record_queue = asyncio.Queue(maxsize=64)
        self.record_pending = set()
        self.senders = {}
        self.transactions = {}
        self.arm_tasks = set()
        self.arm_requests = set()
        self.settings_task = None
        self.settings_retry = 0
        self.delivered = {}
        self.stopping = None
        self.stop_reply = None

    def fresh(self):
        return self.session != 0 and time.monotonic() - self.last_status < 0.25

    def next_sequence(self):
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF or 1
        return self.sequence

    def send(self, channel, payload, sequence=None, index=0, count=1, request_id=None):
        if not self.fresh():
            return False
        packet = serial_encode(pack(channel, self.session, sequence or self.next_sequence(), self.token, payload,
                                    index, count))
        queued = (time.monotonic(), packet, request_id)
        if channel == 6 and payload[4:5] == b"X":
            self.urgent = queued
        elif channel == 1:
            # Targets are never queued behind each other: the newest replaces one still waiting.
            self.target = queued
        elif len(self.outgoing) >= 64:
            # The writer drops stale packets, so a full queue means USB is stuck; every channel is retried.
            return False
        else:
            self.outgoing.append(queued)
        self.wake.set()
        return True

    async def send_message(self, channel, data, request_id=None):
        # Pace fragments to the radio rate; the relay keeps only 8 queued frames.
        count = (len(data) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        if not 1 <= count <= 32:
            raise ValueError("Message exceeds 32 fragments")
        sequence = self.next_sequence()
        for index in range(count):
            if not self.send(channel, data[index * PAYLOAD_SIZE:(index + 1) * PAYLOAD_SIZE], sequence, index, count,
                             request_id):
                return
            await asyncio.sleep(0.004)

    async def request(self, op, body=b"", timeout=2):
        """
        Send an id-tagged request until the drone answers; it replays its reply to a repeated id.
        """
        identifier = os.urandom(4)
        generation = self.link_generation
        session = self.session
        reply = asyncio.get_running_loop().create_future()
        self.waiting[identifier] = reply
        if op == b"A":
            self.arm_requests.add(identifier)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                if generation != self.link_generation or session != self.session:
                    raise RuntimeError("Drone link changed; request cancelled")
                await self.send_message(6, identifier + op + body, identifier)
                try:
                    data = await asyncio.wait_for(asyncio.shield(reply), 0.2)
                except TimeoutError:
                    continue
                if data[0] != 0:
                    raise RuntimeError(data[1:].decode(errors="replace"))
                return data[1:]
            raise TimeoutError(f"No reply to {op.decode()}")
        finally:
            self.waiting.pop(identifier, None)
            self.arm_requests.discard(identifier)
            self.outgoing = deque(item for item in self.outgoing if item[2] != identifier)
            if reply.done() and not reply.cancelled():
                reply.exception()

    def stop(self):
        """
        Stop preempts everything; it repeats every 50 ms until the drone confirms it.
        """
        for task in tuple(self.arm_tasks):
            task.cancel()
        for identifier in self.arm_requests:
            reply = self.waiting[identifier]
            if not reply.done():
                reply.set_exception(RuntimeError("Stopped"))
        self.outgoing = deque(item for item in self.outgoing if item[2] not in self.arm_requests)
        if self.stopping is None or self.stopping.done():
            self.stopping = asyncio.create_task(self.repeat_stop())
        return self.stopping

    async def repeat_stop(self):
        identifier = os.urandom(4)
        reply = asyncio.get_running_loop().create_future()
        self.waiting[identifier] = reply
        self.stop_reply = reply
        try:
            while not reply.done():
                self.send(6, identifier + b"X")
                await asyncio.wait([reply], timeout=0.05)
        finally:
            self.waiting.pop(identifier, None)
            self.stop_reply = None

    async def refresh_settings(self, version):
        try:
            self.settings = json.loads(await self.request(b"S"))
            self.settings_version = version
        except (RuntimeError, TimeoutError) as error:
            self.settings_retry = time.monotonic() + 1
            print(f"Reading settings failed: {error}", flush=True)

    def fail(self, reason):
        if not self.failed.is_set():
            print(reason, flush=True)
            self.failed.set()

    def push_hover(self, payload):
        """
        The newest flight record's hover state, for the dashboard's tag chart: {"type": "hover", ...} in the hover loop's yaw-turned level frame (m; forward, left, up), with the tag relative to the drone (None without a sighting), the range and arc targets as nudged (None before they are set), and the mode and tracking flags.
        """
        _, _, count, size = struct.unpack_from(HEADER_FORMAT, payload)
        if size != RECORD_SIZE or not count:
            return
        fields = struct.unpack_from(SAMPLE_FORMAT, payload, HEADER_SIZE + (count - 1) * size)
        flags = fields[20]
        x, y, z, age = fields[22:26]
        targeted = flags >> 7 & 1
        self.push(self.dashboard, {
            "type": "hover", "elapsed_us": fields[0], "mode": flags >> 1 & 3, "tracking": bool(flags >> 6 & 1),
            "tag": [x / 1000, y / 1000, z / 1000] if age != 65535 else None, "tag_age_ms": age if age != 65535 else None,
            "velocity": [fields[26] / 1000, fields[27] / 1000], "height": fields[28] / 1000, "height_target": fields[30] / 1000,
            "range_target": fields[31] / 1000 if targeted else None, "arc_deg": fields[32] / 100 if targeted else None,
        })

    def push(self, ws, data, binary=False):
        sender = self.senders.get(ws)
        if sender is not None:
            sender.push(data, binary)

    def queue_record(self, operation, *arguments):
        try:
            self.record_queue.put_nowait((operation, arguments))
            return True
        except asyncio.QueueFull:
            self.fail("Recording queue full; stopping service")
            return False

    async def record(self):
        """
        Serialize disk operations in a worker thread so disk latency cannot stall controls.
        """
        while True:
            operation, arguments = await self.record_queue.get()
            try:
                if operation == "receive":
                    session, channel, sequence, payload = arguments
                    await asyncio.to_thread(self.recorder.receive, channel, payload)
                    self.record_pending.discard((session, channel, sequence))
                    if session == self.session:
                        self.delivered[channel] = sequence
                        self.send(5, bytes([channel]), sequence)
                else:
                    await asyncio.to_thread(getattr(self.recorder, operation), *arguments)
            except (OSError, ValueError, struct.error) as error:
                self.recorder.error = str(error)
                self.fail(f"Recording failed: {error}")
            finally:
                self.record_queue.task_done()

    def disconnect(self, reason):
        """
        Close the local clients after a lost link or a drone restart; they reconnect when the drone is back.
        """
        print(reason, flush=True)
        self.link_generation += 1
        self.outgoing.clear()
        self.target = None
        self.urgent = None
        self.fragments.clear()
        for reply in self.waiting.values():
            # Stop is intentionally retried after reconnect; ordinary requests are bound to this link.
            if not reply.done() and reply is not self.stop_reply:
                reply.set_exception(RuntimeError(reason))
        for client in (self.dashboard, self.updater):
            if client is not None:
                asyncio.create_task(client.close(code=1012, message=reason.encode()))

    def read(self):
        try:
            data = self.port.read(max(1, self.port.in_waiting))
        except (OSError, serial.SerialException) as error:
            self.fail(str(error))
            return
        for packet in self.decoder.feed(data):
            self.receive(packet)

    def receive(self, packet):
        channel, session, sequence, token, index, count, payload = unpack(packet)
        if channel == 0:
            if self.session and session != self.session:
                self.disconnect("Drone restarted")
                self.delivered.clear()
                self.queue_record("close", "Drone restarted")
                self.settings_version = None
            self.session, self.token, self.last_status = session, token, time.monotonic()
            version, self.reading = decode_status(payload, self.settings)
            self.reading["recorder"] = self.recorder.status()
            if (version != self.settings_version
                    and time.monotonic() >= self.settings_retry
                    and (self.settings_task is None or self.settings_task.done())):
                self.settings_task = asyncio.create_task(self.refresh_settings(version))
            self.push(self.dashboard, {"type": "reading", **self.reading})
            return
        if session != self.session or channel not in (2, 3, 6):
            return
        if channel != 6 and self.delivered.get(channel) == sequence:
            # The acknowledgement was lost; the recorder already has this message.
            self.send(5, bytes([channel]), sequence)
            return
        if count > 1:
            entry = self.fragments.get(channel)
            if entry is None or entry[0] != sequence or len(entry[1]) != count:
                entry = (sequence, [None] * count)
                self.fragments[channel] = entry
            entry[1][index] = payload
            if None in entry[1]:
                return
            payload = b"".join(entry[1])
            del self.fragments[channel]
        if channel == 6:
            reply = self.waiting.get(payload[:4])
            if reply is not None:
                if not reply.done():
                    reply.set_result(payload[4:])
            else:
                self.push(self.updater, payload, binary=True)
        else:
            if channel == 2 and payload[:4] == b"FLG1":
                self.push_hover(payload)
            # Acknowledge after writing; the recorder flushes to disk once per second.
            key = (session, channel, sequence)
            if key not in self.record_pending and self.queue_record("receive", session, channel, sequence, payload):
                self.record_pending.add(key)

    async def write(self):
        while True:
            await self.wake.wait()
            self.wake.clear()
            while self.urgent is not None or self.target is not None or self.outgoing:
                if self.urgent is not None:
                    created, packet, request_id = self.urgent
                    self.urgent = None
                elif self.target is not None:
                    created, packet, request_id = self.target
                    self.target = None
                else:
                    created, packet, request_id = self.outgoing.popleft()
                # A late packet is dropped, not fatal: targets are superseded, requests and stops repeat, and the drone resends unacknowledged records.
                started = time.monotonic()
                if started - created > 0.1:
                    continue
                offset = 0
                while offset < len(packet):
                    try:
                        offset += self.port.write(packet[offset:])
                    except (OSError, serial.SerialException) as error:
                        self.fail(str(error))
                        return
                    if offset < len(packet):
                        if time.monotonic() - started > 2:
                            self.fail("USB write stalled for 2 s")
                            return
                        await asyncio.sleep(0.001)

    async def monitor(self):
        ticks = 0
        while True:
            await asyncio.sleep(0.05)
            if self.session and time.monotonic() - self.last_status > 0.5 and (self.dashboard or self.updater):
                self.disconnect("Direct radio link timed out; waiting for drone")
                self.last_status = 0
            ticks += 1
            if ticks % 20 == 0:
                # The recorder subscription; the drone arms only while it is fresh.
                self.send(4, b"\x01")
                self.queue_record("check", bool(self.reading and self.reading["logging"]["active"]))

    async def connect(self, request, slot):
        if getattr(self, slot) is not None or not self.fresh() or self.failed.is_set():
            raise web.HTTPServiceUnavailable(text="Drone link unavailable or endpoint in use")
        ws = web.WebSocketResponse(max_msg_size=8192, compress=False)
        setattr(self, slot, ws)
        try:
            await ws.prepare(request)
        except BaseException:
            setattr(self, slot, None)
            raise
        self.senders[ws] = ClientSender(ws)
        self.transactions[ws] = set()
        return ws

    async def dashboard_socket(self, request):
        ws = await self.connect(request, "dashboard")
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                self.dashboard_message(ws, json.loads(message.data))
        finally:
            self.dashboard = None
            self.target = None
            pending = self.transactions.pop(ws)
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            # A closed dashboard can no longer stop the drone.
            self.stop()
            await self.senders.pop(ws).close()
        return ws

    def dashboard_message(self, ws, message):
        kind = message["type"]
        if kind == "targets":
            self.send(1, encode_targets(message))
        elif kind == "stop":
            stop = self.stop()
            self.start_transaction(ws, self.confirm_stop(ws, stop))
        else:
            self.start_transaction(ws, self.transaction(ws, kind, message), arm=kind == "arm")

    def start_transaction(self, ws, operation, arm=False):
        pending = self.transactions[ws]
        task = asyncio.create_task(operation)
        pending.add(task)
        task.add_done_callback(pending.discard)
        if arm:
            self.arm_tasks.add(task)
            task.add_done_callback(self.arm_tasks.discard)

    async def confirm_stop(self, ws, stop):
        await asyncio.shield(stop)
        self.push(ws, {"type": "result", "request": "stop", "error": None})

    async def transaction(self, ws, kind, message):
        error = None
        try:
            body = b""
            if kind == "zero":
                body = bytes([("left", "right").index(message["side"])])
            elif kind == "tune":
                body = json.dumps(message["settings"]).encode()
            data = await self.request(REQUESTS[kind], body)
            if kind == "tune":
                self.settings = json.loads(data)
        except (RuntimeError, TimeoutError) as failure:
            error = str(failure)
        result = {"type": "result", "request": kind, "error": error}
        if kind == "zero":
            result["side"] = message.get("side")
        self.push(ws, result)

    async def update_socket(self, request):
        ws = await self.connect(request, "updater")
        try:
            async for message in ws:
                if message.type != WSMsgType.BINARY or not 0 < len(message.data) <= 32 * PAYLOAD_SIZE:
                    break
                await self.send_message(6, message.data)
        finally:
            self.updater = None
            self.transactions.pop(ws)
            await self.senders.pop(ws).close()
        return ws

    async def status(self, request):
        return web.json_response({"online": self.fresh(), "reading": self.reading})

    async def get_settings(self, request):
        return web.json_response({"settings": self.settings})

    async def post_settings(self, request):
        body = await request.json()
        try:
            self.settings = json.loads(await self.request(b"T", json.dumps(body["settings"]).encode()))
        except RuntimeError as error:
            return web.json_response({"error": str(error)}, status=400)
        except TimeoutError as error:
            return web.json_response({"error": str(error)}, status=504)
        return web.json_response({"settings": self.settings})

    async def post_pattern(self, request):
        try:
            body, recorded = encode_pattern(await request.json())
            summary = json.loads(await self.request(b"P", body))
        except (ValueError, KeyError, TypeError, struct.error, RuntimeError) as error:
            return web.json_response({"error": str(error)}, status=400)
        except TimeoutError as error:
            return web.json_response({"error": str(error)}, status=504)
        self.recorder.pattern = recorded if recorded["segments"] else None
        return web.json_response(summary)

    async def schema(self, request):
        """
        Every setting as {name, min, max, default, tunable}, collected page by page (drone/main/settings.c).
        """
        entries = []
        try:
            while True:
                page = json.loads(await self.request(b"D", bytes([len(entries) // SCHEMA_PAGE])))
                entries += page["settings"]
                if len(entries) >= page["total"]:
                    break
        except TimeoutError as error:
            return web.json_response({"error": str(error)}, status=504)
        except RuntimeError as error:
            return web.json_response({"error": str(error)}, status=502)
        keys = ("name", "min", "max", "default", "tunable")
        return web.json_response({"settings": [dict(zip(keys, entry)) for entry in entries]})


async def main(args):
    port = serial.Serial()
    port.port, port.baudrate = args.port, args.baud
    port.timeout = port.write_timeout = 0
    port.exclusive = True
    port.dtr = port.rts = False
    port.open()
    service = Service(port, args.recordings)
    loop = asyncio.get_running_loop()
    loop.add_reader(port.fileno(), service.read)
    app = web.Application()
    app.add_routes([
        web.get("/ws", service.dashboard_socket),
        web.get("/update", service.update_socket),
        web.get("/status", service.status),
        web.get("/settings", service.get_settings),
        web.post("/settings", service.post_settings),
        web.post("/pattern", service.post_pattern),
        web.get("/schema", service.schema),
    ])
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", args.listen).start()
    record_task = asyncio.create_task(service.record())
    tasks = [asyncio.create_task(service.write()), asyncio.create_task(service.monitor())]
    print(f"USB {args.port} → http://127.0.0.1:{args.listen}; waiting for drone", flush=True)
    try:
        await service.failed.wait()
    finally:
        service.failed.set()
        await asyncio.gather(*(sender.close() for sender in tuple(service.senders.values())))
        loop.remove_reader(port.fileno())
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await runner.cleanup()
        await service.record_queue.put(("close", ("Service stopped",)))
        await service.record_queue.join()
        record_task.cancel()
        await asyncio.gather(record_task, return_exceptions=True)
        port.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Controller USB serial device")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--listen", type=int, default=8765)
    parser.add_argument("--recordings", type=Path, default=RECORDINGS)
    try:
        asyncio.run(main(parser.parse_args()))
    except KeyboardInterrupt:
        pass
    else:
        sys.exit("Service failed")
