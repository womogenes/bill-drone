# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5", "websockets>=15,<17"]
# ///
"""
Expose the USB controller as the dashboard's /ws and /log endpoints and the /update channel.
"""

import argparse
import asyncio
from collections import deque
from dataclasses import dataclass, field
import time
import struct

import serial
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from wire import PAYLOAD_SIZE, SerialDecoder, pack, serial_encode, unpack

ENDPOINTS = {1: "/ws", 2: "/log", 3: "/log", 6: "/update"}


def setpoint_fields(message):
    """
    Return a P packet's {field id: 4-byte value}, or None when it must keep its place: malformed or carrying the enable field (id 0).
    """
    if message[:1] != b"P" or (len(message) - 1) % 5:
        return None
    fields = {message[offset]: message[offset + 1:offset + 5] for offset in range(1, len(message), 5)}
    return None if 0 in fields else fields


def supersedes(newer, older):
    """
    A newer setpoint replaces an older queued one: any non-enable P replaces a P, and S/T replace the same kind and side.
    """
    if newer[:1] in (b"S", b"T"):
        return len(newer) == len(older) == 6 and newer[:2] == older[:2]
    return setpoint_fields(newer) is not None and setpoint_fields(older) is not None


@dataclass
class Bridge:
    port: object
    clients: dict = field(default_factory=dict)
    queues: dict = field(default_factory=dict)
    sequence: dict = field(default_factory=dict)
    fragments: dict = field(default_factory=dict)
    delivered: dict = field(default_factory=dict)
    pending: set = field(default_factory=set)
    outgoing: deque = field(default_factory=deque)
    decoder: object = field(default_factory=SerialDecoder)
    wake: object = field(default_factory=asyncio.Event)
    failed: object = field(default_factory=asyncio.Event)
    session: int = 0
    token: int = 0
    last_beacon: float = 0
    recovering: bool = False
    stopping: bool = False
    unacked: tuple | None = None
    queued: deque = field(default_factory=deque)

    def fresh(self):
        return not self.recovering and self.session != 0 and time.monotonic() - self.last_beacon < 0.25

    def send(self, channel, payload, sequence=None):
        if not self.fresh():
            return
        if sequence is None:
            sequence = (self.sequence.get(channel, int(time.time() * 1000)) + 1) & 0xFFFFFFFF
            self.sequence[channel] = sequence
        packet = serial_encode(pack(channel, self.session, sequence, self.token, payload))
        self.write_packet(packet)
        return sequence, packet

    async def send_message(self, channel, data):
        # Pace fragments to the radio rate; the relay keeps only 8 queued frames.
        count = (len(data) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        sequence = (self.sequence.get(channel, int(time.time() * 1000)) + 1) & 0xFFFFFFFF
        self.sequence[channel] = sequence
        for index in range(count):
            if not self.fresh():
                return
            fragment = data[index * PAYLOAD_SIZE:(index + 1) * PAYLOAD_SIZE]
            self.write_packet(serial_encode(pack(channel, self.session, sequence, self.token, fragment, index, count)))
            await asyncio.sleep(0.004)

    def write_packet(self, packet):
        if len(self.outgoing) >= 32:
            self.fail("USB output stalled")
            return
        self.outgoing.append((time.monotonic(), packet))
        self.wake.set()

    def transmit(self, message):
        # One command awaits its acknowledgement; retries reuse its exact envelope.
        self.unacked = None
        sent = self.send(1, message)
        if sent is None:
            if message != b"X":
                self.timeout("Direct link unavailable for command")
            return
        self.unacked = (time.monotonic(), *sent, message)
        asyncio.get_running_loop().call_later(0.2, self.expire, sent[0])

    def expire(self, sequence):
        if self.unacked is None or self.unacked[1] != sequence:
            return
        message = self.unacked[3]
        self.unacked = None
        # An unacknowledged stop is reissued only by a later stop or browser heartbeat.
        if message == b"X":
            self.disconnect("Stop acknowledgement timed out")
        else:
            self.timeout("Command acknowledgement timed out")

    def timeout(self, reason):
        self.disconnect(reason)
        self.stop()

    def disconnect(self, reason):
        print(reason, flush=True)
        ws = self.clients.get("/ws")
        if ws is not None:
            asyncio.create_task(ws.close(1011, reason))

    def advance(self):
        if self.unacked is None and self.queued:
            created, message = self.queued.popleft()
            if time.monotonic() - created > 0.2:
                self.timeout("Queued command expired")
            else:
                self.transmit(message)

    def enqueue(self, message):
        # Coalesce only within the trailing run of setpoints: enable, calibrate, zero and stop keep their order.
        # The merged command moves to the newest position; every P also stops manual throttle while disabled,
        # so moving S/T/P later past other setpoints leaves the same final actuator state as sending them all.
        created = time.monotonic()
        for index in range(len(self.queued) - 1, -1, -1):
            older_created, older = self.queued[index]
            if older[:1] not in (b"S", b"T") and setpoint_fields(older) is None:
                break
            if supersedes(message, older):
                del self.queued[index]
                created = older_created
                if message[:1] == b"P":
                    fields = setpoint_fields(older) | setpoint_fields(message)
                    message = b"P" + b"".join(bytes([key]) + value for key, value in fields.items())
                break
        self.queued.append((created, message))
        self.advance()

    def stop(self):
        self.queued.clear()
        if not self.stopping or self.unacked is None:
            self.stopping = True
            self.outgoing.clear()
            self.transmit(b"X")

    def heartbeat(self):
        # A received browser heartbeat retries the pending command, never extends its authority.
        if self.unacked is not None:
            if time.monotonic() - self.unacked[0] < 0.2:
                self.write_packet(self.unacked[2])
        elif self.stopping:
            self.transmit(b"X")
        else:
            self.send(1, b"H")

    def command(self, message):
        disabling = message == b"X"
        if message[:1] == b"P" and (len(message) - 1) % 5 == 0:
            disabling = any(
                message[offset] == 0 and struct.unpack_from("<f", message, offset + 1)[0] == 0
                for offset in range(1, len(message), 5)
            )
        if disabling:
            self.stop()
        elif message == b"H":
            self.heartbeat()
        elif not self.stopping:
            self.enqueue(message)

    def fail(self, reason):
        if self.failed.is_set():
            return
        print(reason, flush=True)
        self.outgoing.clear()
        self.failed.set()
        for ws in list(self.clients.values()):
            asyncio.create_task(ws.close(1011, reason))

    def reconnect(self, reason):
        if self.recovering:
            return
        self.recovering = True
        self.outgoing.clear()
        self.queued.clear()
        self.unacked = None
        self.stopping = True
        self.fragments.clear()
        self.pending.clear()
        self.delivered.clear()
        print(reason, flush=True)
        clients = list(self.clients.values())
        self.clients.clear()
        self.queues.clear()
        asyncio.create_task(self.close_clients(clients, reason))

    async def close_clients(self, clients, reason):
        # A client that fails to close must not leave the bridge rejecting everyone.
        try:
            await asyncio.gather(*(ws.close(1012, reason) for ws in clients), return_exceptions=True)
        finally:
            self.recovering = False

    def read(self):
        try:
            data = self.port.read(max(1, self.port.in_waiting))
            for packet in self.decoder.feed(data):
                self.receive(packet)
        except (OSError, serial.SerialException) as error:
            self.fail(str(error))

    def receive(self, packet):
        channel, session, sequence, token, index, count, payload = unpack(packet)
        if channel == 0:
            if self.session and session != self.session:
                self.reconnect("Drone restarted; waiting for dashboard reconnect")
            self.session, self.token, self.last_beacon = session, token, time.monotonic()
            return
        if self.recovering or session != self.session:
            return
        if channel == 5:
            if payload == b"\x01" and self.unacked is not None and sequence == self.unacked[1]:
                if self.unacked[3] == b"X":
                    self.stopping = False
                self.unacked = None
                self.advance()
            return
        if channel not in ENDPOINTS:
            return
        endpoint = ENDPOINTS[channel]
        queue = self.queues.get(endpoint)
        if queue is None:
            return
        key = (channel, sequence)
        logged = endpoint == "/log"
        if logged and self.delivered.get(channel) == sequence:
            self.send(5, bytes([channel]), sequence)
            return
        if key in self.pending:
            return
        if not 1 <= count <= 32 or index >= count:
            return
        if count > 1:
            entry = self.fragments.get(channel)
            if entry is None or entry[0] != sequence:
                entry = (sequence, [None] * count)
                self.fragments[channel] = entry
            chunks = entry[1]
            if len(chunks) != count:
                return
            chunks[index] = payload
            if any(chunk is None for chunk in chunks):
                return
            payload = b"".join(chunks)
            del self.fragments[channel]
        try:
            message = payload.decode("utf-8") if channel == 3 else payload
            queue.put_nowait((channel, sequence, message))
            if logged:
                self.pending.add(key)
        except (asyncio.QueueFull, UnicodeDecodeError):
            self.fail("Dashboard cannot keep up with the USB stream")

    async def write(self):
        while True:
            await self.wake.wait()
            while self.outgoing:
                created, packet = self.outgoing.popleft()
                if time.monotonic() - created > 0.1:
                    self.fail("USB command expired")
                    return
                offset = 0
                while offset < len(packet):
                    try:
                        offset += self.port.write(packet[offset:])
                    except (OSError, serial.SerialException) as error:
                        self.fail(str(error))
                        return
                    if time.monotonic() - created > 0.1:
                        self.fail("USB write timed out")
                        return
                    if offset < len(packet):
                        await asyncio.sleep(0.001)
            self.wake.clear()

    async def monitor(self):
        while True:
            await asyncio.sleep(0.05)
            if self.clients and time.monotonic() - self.last_beacon > 0.5:
                self.reconnect("Direct radio link timed out; waiting for drone")

    async def stream(self, endpoint, ws, queue):
        while True:
            channel, sequence, message = await queue.get()
            await ws.send(message)
            if endpoint == "/log":
                self.pending.discard((channel, sequence))
                self.delivered[channel] = sequence
                self.send(5, bytes([channel]), sequence)

    async def websocket(self, ws):
        endpoint = ws.request.path
        if endpoint not in ("/ws", "/log", "/update"):
            await ws.close(1008, "Use /ws, /log or /update")
            return
        if endpoint in self.clients or not self.fresh() or self.failed.is_set():
            await ws.close(1013, "Direct link unavailable or endpoint in use")
            return
        queue = asyncio.Queue(maxsize=32)
        self.clients[endpoint], self.queues[endpoint] = ws, queue
        sender = asyncio.create_task(self.stream(endpoint, ws, queue))
        if endpoint == "/log":
            self.send(4, b"\x01")
        try:
            async for message in ws:
                if endpoint == "/log":
                    if message in ("H", b"H"):
                        self.send(4, b"H")
                elif endpoint == "/update":
                    if not isinstance(message, bytes) or not 0 < len(message) <= 32 * PAYLOAD_SIZE:
                        await ws.close(1008, "Expected binary update request")
                    else:
                        await self.send_message(6, message)
                elif isinstance(message, bytes) and 0 < len(message) <= 232:
                    self.command(message)
                else:
                    await ws.close(1008, "Expected binary control packet")
        except ConnectionClosed:
            pass
        finally:
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            if self.clients.get(endpoint) is ws:
                self.clients.pop(endpoint)
                self.queues.pop(endpoint)
                if endpoint == "/ws":
                    self.stop()
                elif endpoint == "/log":
                    self.send(4, b"\x00")
                    self.pending.clear()
                    self.fragments.clear()
                    self.delivered.clear()


async def main(args):
    port = serial.Serial()
    port.port, port.baudrate = args.port, args.baud
    port.timeout = port.write_timeout = 0
    port.dtr = port.rts = False
    port.open()
    bridge = Bridge(port)
    loop = asyncio.get_running_loop()
    loop.add_reader(port.fileno(), bridge.read)
    tasks = [asyncio.create_task(bridge.write()), asyncio.create_task(bridge.monitor())]
    try:
        async with serve(bridge.websocket, "127.0.0.1", args.listen, max_size=8192,
                         max_queue=16, compression=None, ping_interval=None):
            print(f"USB {args.port} → ws://127.0.0.1:{args.listen}; waiting for drone", flush=True)
            await bridge.failed.wait()
    finally:
        loop.remove_reader(port.fileno())
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        port.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Controller USB serial device")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--listen", type=int, default=8765)
    try:
        asyncio.run(main(parser.parse_args()))
    except KeyboardInterrupt:
        pass
