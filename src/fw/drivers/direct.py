"""
Direct ESP-NOW commands and recordings on a fixed radio channel.
"""

import asyncio
import espnow
import os
import time
from .direct_wire import pack, unpack, PAYLOAD_SIZE


class Link:
    def __init__(self, peer):
        self.peer = peer
        self.radio = espnow.ESPNow()
        self.radio.config(rxbuf=4096)
        self.radio.active(True)
        self.radio.add_peer(peer)
        self.session = int.from_bytes(os.urandom(4), "little") or 1
        self.sequence = 0
        self.received = [-1] * 6
        self.commands = []
        self.ready = asyncio.Event()
        self.command_ms = None
        self.command_sequence = None
        self.acked = -1
        self.pending_ack = None
        self.log_ms = None
        self.log_requested = False
        self.ack = None
        self.ack_ready = asyncio.Event()
        self.fragments = None
        self.requests = []
        self.request_ready = asyncio.Event()

    def next_sequence(self):
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return self.sequence

    def send(self, channel, payload, sequence=None, index=0, count=1):
        sent, responses, failures, received, dropped = self.radio.stats()
        # A few frames may wait in ESP-NOW; a full queue blocks send() for two seconds.
        if sent - responses >= 4:
            return False
        packet = pack(channel, self.session, self.next_sequence() if sequence is None else sequence,
                      time.ticks_ms(), payload, index, count)
        try:
            self.radio.send(self.peer, packet, False)
            return True
        except OSError:
            return False

    def send_ack(self):
        # A dropped acknowledgement disconnects the controller; retry it from the pump.
        if self.pending_ack is not None and self.send(5, b"\x01", self.pending_ack):
            self.pending_ack = None

    async def receive(self):
        # The command loop asks again only after applying the previous command.
        if self.command_sequence is not None:
            self.acked = self.command_sequence
            self.command_sequence = None
            self.pending_ack = self.acked
            self.send_ack()
        while True:
            while self.commands:
                token, sequence, command = self.commands.pop(0)
                if time.ticks_diff(time.ticks_ms(), token) <= 200:
                    if command != b"H":
                        self.command_sequence = sequence
                    return command
            if self.command_ms is None or time.ticks_diff(time.ticks_ms(), self.command_ms) > 500:
                raise OSError("Direct controller disconnected")
            self.ready.clear()
            await asyncio.wait_for_ms(self.ready.wait(), 500)

    async def pump(self):
        beacon_ms = time.ticks_ms()
        while True:
            self.send_ack()
            for iteration in range(16):
                try:
                    peer, packet = self.radio.irecv(0)
                except ValueError:
                    # MicroPython has already reset the desynchronized receive buffer.
                    break
                if peer is None:
                    break
                if peer != self.peer:
                    continue
                try:
                    channel, session, sequence, token, index, count, payload = unpack(packet)
                except ValueError:
                    continue
                age = time.ticks_diff(time.ticks_ms(), token)
                if session != self.session or not 0 <= age <= 200:
                    continue
                if channel == 6:
                    self.collect(sequence, index, count, payload)
                    continue
                if index or count != 1:
                    continue
                if channel == 5:
                    if len(payload) == 1 and self.ack == (payload[0], sequence):
                        self.ack_ready.set()
                    continue
                if channel not in (1, 4):
                    continue
                if channel == 1 and sequence == self.acked:
                    self.pending_ack = sequence
                    continue
                previous = self.received[channel]
                if previous >= 0 and not 0 < ((sequence - previous) & 0xFFFFFFFF) < 0x80000000:
                    continue
                self.received[channel] = sequence
                if channel == 1:
                    self.command_ms = time.ticks_ms()
                    if len(self.commands) == 8:
                        self.commands.clear()
                        self.commands.append((token, sequence, b"X"))
                    else:
                        self.commands.append((token, sequence, bytes(payload)))
                    self.ready.set()
                elif payload == b"\x00":
                    self.log_requested = False
                elif payload in (b"\x01", b"H"):
                    self.log_ms = time.ticks_ms()
                    if payload == b"\x01":
                        self.log_requested = True

            if time.ticks_diff(time.ticks_ms(), beacon_ms) >= 50:
                self.send(0, b"")
                beacon_ms = time.ticks_ms()
            await asyncio.sleep_ms(2)


    def collect(self, sequence, index, count, payload):
        if self.fragments is None or self.fragments[0] != sequence or len(self.fragments[1]) != count:
            self.fragments = (sequence, [None] * count)
        parts = self.fragments[1]
        parts[index] = bytes(payload)
        if None not in parts:
            self.fragments = None
            self.requests.append(b"".join(parts))
            self.request_ready.set()

    async def next_request(self):
        while not self.requests:
            self.request_ready.clear()
            await self.request_ready.wait()
        return self.requests.pop(0)

    async def send_message(self, channel, data):
        count = (len(data) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        if not 1 <= count <= 32:
            raise ValueError("Direct message exceeds 7424 bytes")
        sequence = self.next_sequence()
        started = time.ticks_ms()
        for index in range(count):
            while not self.send(channel, data[index * PAYLOAD_SIZE:(index + 1) * PAYLOAD_SIZE], sequence, index, count):
                if time.ticks_diff(time.ticks_ms(), started) >= 2000:
                    raise OSError("Direct radio send timed out")
                await asyncio.sleep_ms(1)


class Commands:
    def __init__(self, link):
        self.link = link

    async def receive(self):
        return await self.link.receive()

    async def send(self, packet):
        self.link.send(1, packet)


class Log:
    def __init__(self, link):
        self.link = link

    async def close(self):
        self.link.log_requested = False

    async def send(self, packet):
        channel = 3 if isinstance(packet, str) else 2
        data = packet.encode() if isinstance(packet, str) else packet
        count = (len(data) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        if not 1 <= count <= 32:
            raise ValueError("Direct log message exceeds 7424 bytes")
        sequence = self.link.next_sequence()
        self.link.ack = (channel, sequence)
        self.link.ack_ready.clear()
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < 2000:
            if not self.link.log_requested:
                raise OSError("Direct recorder disconnected")
            for index in range(count):
                fragment = data[index * PAYLOAD_SIZE:(index + 1) * PAYLOAD_SIZE]
                while not self.link.send(channel, fragment, sequence, index, count):
                    if time.ticks_diff(time.ticks_ms(), started) >= 2000:
                        raise OSError("Direct radio send timed out")
                    await asyncio.sleep_ms(1)
                await asyncio.sleep_ms(1)
            try:
                await asyncio.wait_for_ms(self.link.ack_ready.wait(), 80)
                return
            except asyncio.TimeoutError:
                pass
        raise OSError("Direct recorder acknowledgement timed out")


async def command_sessions(link, serve):
    while True:
        await link.ready.wait()
        try:
            await serve(Commands(link))
        except (OSError, asyncio.TimeoutError, ValueError, KeyError, IndexError):
            pass
        link.commands.clear()
        link.ready.clear()
        link.command_ms = None
        link.command_sequence = None


async def log_sessions(link, stream):
    while True:
        if not link.log_requested:
            await asyncio.sleep_ms(50)
            continue
        finished = []

        async def record():
            try:
                await stream(Log(link))
            finally:
                finished.append(True)

        task = asyncio.create_task(record())
        try:
            while link.log_requested and time.ticks_diff(time.ticks_ms(), link.log_ms) < 3000:
                if finished:
                    await task
                    break
                await asyncio.sleep_ms(50)
        except OSError:
            pass
        finally:
            link.log_requested = False
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def update_sessions(link):
    from drivers import update
    while True:
        try:
            await update.serve(link)
        except OSError:
            pass


async def run(link, serve, stream):
    await asyncio.gather(link.pump(), command_sessions(link, serve), log_sessions(link, stream),
                         update_sessions(link))
