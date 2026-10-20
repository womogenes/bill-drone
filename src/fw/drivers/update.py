"""
Apply native image sectors and Python files sent over the direct link.
"""

import asyncio
import binascii
import esp32
import hashlib
import json
import machine
import os
import gc
import struct
import flight_rt
from drivers import control

SECTOR = 4096
HASH_SIZE = 8
# Sector buffers are allocated on the first request; boot memory is tight.
buffer = None
readback = None


def require_idle():
    state = flight_rt.snapshot()
    if control.settings["enabled"] or state["enabled"] or any(state["throttle"]):
        raise ValueError("Disable control and zero throttle before updating")


def slots():
    running = esp32.Partition(esp32.Partition.RUNNING)
    if not 0x10 <= running.info()[1] <= 0x1F:
        raise ValueError("Running from the factory slot; migrate over USB first")
    return running, running.get_next_update()


def sector_hash(data):
    return hashlib.sha256(data).digest()[:HASH_SIZE]


def write_sector(partition, sector, data):
    require_idle()
    # Writing over the fallback slot requires the running image to stay selected.
    esp32.Partition.mark_app_valid_cancel_rollback()
    partition.writeblocks(sector, data)
    partition.readblocks(sector, readback)
    if readback != data:
        raise ValueError(f"Sector {sector} readback mismatch")


def file_hash(path):
    digest = hashlib.sha256()
    try:
        stream = open(path, "rb")
    except OSError as error:
        if error.errno == 2:
            return bytes(32)
        raise
    with stream:
        while True:
            count = stream.readinto(buffer)
            if not count:
                return digest.digest()
            digest.update(memoryview(buffer)[:count])


async def reset():
    await asyncio.sleep_ms(300)
    machine.reset()


async def handle(request):
    global buffer, readback
    if buffer is None:
        gc.collect()
        buffer = bytearray(SECTOR)
        readback = bytearray(SECTOR)
    op, body = request[:1], request[1:]
    if op == b"I":
        running = esp32.Partition(esp32.Partition.RUNNING).info()
        target = slots()[1].info() if 0x10 <= running[1] <= 0x1F else (0, 0, 0, 0, None)
        return json.dumps({"running": running[4], "target": target[4], "size": target[3]}).encode()
    if op == b"H":
        slot, first, count = struct.unpack("<BHH", body)
        partition = slots()[slot]
        hashes = bytearray()
        for sector in range(first, first + count):
            partition.readblocks(sector, buffer)
            hashes.extend(sector_hash(buffer))
            await asyncio.sleep_ms(0)
        return hashes
    if op == b"C":
        sector = struct.unpack("<H", body)[0]
        running, target = slots()
        running.readblocks(sector, buffer)
        write_sector(target, sector, buffer)
        return b""
    if op == b"W":
        sector = struct.unpack_from("<H", body)[0]
        expected, data = body[2:2 + HASH_SIZE], body[2 + HASH_SIZE:]
        if len(data) != SECTOR or sector_hash(data) != expected:
            raise ValueError(f"Sector {sector} arrived corrupted")
        write_sector(slots()[1], sector, data)
        return b""
    if op == b"B":
        require_idle()
        # ESP-IDF checks the whole image and its SHA-256 before selecting it.
        slots()[1].set_boot()
        return b""
    if op == b"V":
        esp32.Partition.mark_app_valid_cancel_rollback()
        return b""
    if op == b"R":
        require_idle()
        asyncio.create_task(reset())
        return b""
    if op == b"F":
        return b"".join(file_hash(path) for path in body.decode().split("\n"))
    if op == b"P":
        require_idle()
        offset, length = struct.unpack_from("<IB", body)
        path = body[5:5 + length].decode() + ".upload"
        if offset == 0:
            parts = path.split("/")[:-1]
            for index in range(1, len(parts) + 1):
                try:
                    os.mkdir("/".join(parts[:index]))
                except OSError as error:
                    if error.errno != 17:
                        raise
        elif os.stat(path)[6] != offset:
            raise ValueError(f"{path} is not {offset} bytes long")
        with open(path, "ab" if offset else "wb") as stream:
            stream.write(body[5 + length:])
        return b""
    if op == b"M":
        require_idle()
        expected, path = body[:32], body[32:].decode()
        if file_hash(path + ".upload") != expected:
            raise ValueError(f"{path} verification failed")
        os.rename(path + ".upload", path)
        return b""
    raise ValueError(f"Unknown update request {binascii.hexlify(op)}")


async def serve(link):
    """
    Answer each request once; a retried request id gets the cached reply.
    """
    answered = None
    while True:
        request = await link.next_request()
        identifier = request[:4]
        if answered is None or answered[0] != identifier:
            try:
                reply = b"\x00" + await handle(request[4:])
            except Exception as error:
                reply = b"\x01" + repr(error).encode()
            answered = (identifier, identifier + reply)
        await link.send_message(6, answered[1])
