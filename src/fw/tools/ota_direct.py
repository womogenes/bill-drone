# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Update the drone's application over the direct ESP-NOW link through the laptop service.

firmware: diff the new image against the images this laptop last installed in each OTA slot; send only changed sectors (deflate-compressed when that is smaller, over the fast rate while writing), have the drone copy sectors the running image already has, then boot and confirm it.
confirm: keep the running image after a manual check.
camera: write a new ESP32-CAM image to the camera's spare slot through the drone (which forwards each request over its wired link, fw/esp32cam/src/update.h), boot it and check that it runs. The whole image is sent; the camera checks each sector's hash and reads it back, and checks the whole image before selecting it.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import time
import urllib.request
import zlib

from websockets.exceptions import ConnectionClosed, InvalidStatus
from websockets.sync.client import connect

FIRMWARE = Path(__file__).resolve().parents[1]
# The last image installed in each slot, by slot label; the drone reports each slot's ELF SHA-256 prefix to check it.
INSTALLED = FIRMWARE / "drone/installed"
SECTOR = 4096
HASH_SIZE = 8


class Drone:
    def __init__(self, url):
        self.url = url
        self.ws = None

    def request(self, op, body=b"", timeout=2, patience=15):
        """
        Send until answered; the drone replays its reply to a repeated id.
        """
        identifier = os.urandom(4)
        deadline = time.monotonic() + patience
        while time.monotonic() < deadline:
            try:
                if self.ws is None:
                    self.ws = connect(self.url, max_size=8192, compression=None, open_timeout=2)
                self.ws.send(identifier + op + body)
                answer = time.monotonic() + timeout
                reply = b""
                while reply[:4] != identifier:
                    reply = self.ws.recv(timeout=max(0.01, answer - time.monotonic()))
            except TimeoutError:
                continue
            except (ConnectionClosed, InvalidStatus, ConnectionRefusedError):
                # The service refuses or drops clients while the radio link is down, as while the drone restarts, and is not there at all while it restarts.
                self.ws = None
                time.sleep(0.5)
                continue
            if reply[4] != 0:
                # The drone waits 2 s for the camera, which may be busy (hashing its image while streaming video); a new id asks again rather than replaying that answer.
                if reply[5:] == b"The camera did not answer":
                    identifier = os.urandom(4)
                    continue
                raise RuntimeError(f"Drone rejected {op.decode()}: {reply[5:].decode()}")
            return reply[5:]
        raise TimeoutError(f"No reply to {op.decode()} within {patience} s")

    def restart(self):
        # Sent once: a retry reaching the new image would reset it unconfirmed.
        self.ws.send(os.urandom(4) + b"R")
        try:
            while True:
                self.ws.recv(timeout=30)
        except ConnectionClosed:
            self.ws = None


def readings(url):
    """
    New status readings with IMU data from the laptop service's /status, polled every 20 ms.
    """
    last = None
    while True:
        reading = json.load(urllib.request.urlopen(url))["reading"]
        if reading["time"] != last and reading["imu"]:
            last = reading["time"]
            yield reading
        time.sleep(0.02)


def elf_sha(image):
    """
    The ELF SHA-256 prefix from the image's app description, as the drone reports it.
    """
    return image[0xB0:0xB8].hex()


def installed(label, sha):
    path = INSTALLED / f"{label}.bin"
    if not path.exists():
        return None
    image = path.read_bytes()
    return image if elf_sha(image) == sha else None


def sectors(image, count):
    """
    The image's sectors as written, padded with 0xff; sectors past its end are unknown (None).
    """
    if image is None:
        return [None] * count
    known = (len(image) + SECTOR - 1) // SECTOR
    image = image.ljust(known * SECTOR, b"\xff")
    return [image[index * SECTOR:(index + 1) * SECTOR] if index < known else None for index in range(count)]


def firmware(drone, path):
    image = path.read_bytes()
    info = json.loads(drone.request(b"I"))
    if info["running_sha"] == elf_sha(image):
        print(f"{info['running']} already runs this image")
        return
    count = (len(image) + SECTOR - 1) // SECTOR
    local = sectors(image, count)
    target = sectors(installed(info["target"], info["target_sha"]), count)
    running = sectors(installed(info["running"], info["running_sha"]), count)
    stale = [index for index in range(count) if target[index] != local[index]]
    copies = {index for index in stale if running[index] == local[index]}
    print(f"{info['target']}: {len(stale) - len(copies)} sectors to send, {len(copies)} to copy from "
          f"{info['running']}, {count - len(stale)} already current", flush=True)
    INSTALLED.mkdir(parents=True, exist_ok=True)
    record = INSTALLED / f"{info['target']}.bin"
    # The slot no longer holds a known image once writing starts.
    record.unlink(missing_ok=True)
    write(drone, local, stale, copies)
    drone.request(b"B", timeout=10)
    record.write_bytes(image)
    print(f"Booting {info['target']}; it rolls back on the next reset unless confirmed", flush=True)
    drone.restart()
    booted = json.loads(drone.request(b"I", patience=60))
    if booted["running"] != info["target"]:
        raise SystemExit(f"Drone runs {booted['running']}, not {info['target']}: the new image was rolled back")
    drone.request(b"V")
    print(f"Confirmed {info['target']}")


def camera(drone, path):
    image = path.read_bytes()
    # The camera reports the SHA-256 the build appends to the image.
    expected = image[-32:-24].hex()
    running, target, size, sha = drone.request(b"K", b"I").decode().split()
    if sha == expected:
        print(f"The camera's {running} already runs this image")
        return
    if len(image) > int(size):
        raise SystemExit(f"Image of {len(image)} bytes does not fit the camera's {target} ({size} bytes)")
    count = (len(image) + SECTOR - 1) // SECTOR
    local = sectors(image, count)
    print(f"camera {running} -> {target}: {count} sectors", flush=True)
    started = time.monotonic()
    for index in range(count):
        digest = hashlib.sha256(local[index]).digest()[:HASH_SIZE]
        drone.request(b"K", b"W" + struct.pack("<H", index) + digest + local[index], timeout=4, patience=40)
        rate = (index + 1) * SECTOR / 1024 / (time.monotonic() - started)
        print(f"\rWrote {index + 1}/{count} sectors ({rate:.0f} KB/s)", end="", flush=True)
    print()
    drone.request(b"K", b"B", timeout=10)
    print(f"Booting the camera's {target}", flush=True)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        time.sleep(1)
        try:
            booted = drone.request(b"K", b"I", patience=3).decode().split()
        except (TimeoutError, RuntimeError):
            continue
        if booted[0] != target or booted[3] != expected:
            raise SystemExit(f"The camera runs {booted[0]} ({booted[3]}), not the new image")
        print(f"The camera runs the new image from {target}")
        return
    raise SystemExit("The camera did not answer after booting")


def fast(drone, on):
    """Switch the link to the fast rate for writing (or back to the usual rate); False when the drone's firmware predates it."""
    try:
        drone.request(b"F", bytes([on]), patience=5)
        return True
    except RuntimeError:
        return False


def compressed(sector):
    """The sector as raw deflate, or None when that saves nothing."""
    packer = zlib.compressobj(9, zlib.DEFLATED, -15)
    data = packer.compress(sector) + packer.flush()
    return data if len(data) < SECTOR - 64 else None


def write(drone, local, indices, copies):
    started = time.monotonic()
    # Firmware without the fast rate or compressed writes answers F or Y as unknown: then sectors go plain.
    hurried = fast(drone, 1)
    plain = not hurried
    sent = 0
    for done, index in enumerate(indices, 1):
        digest = hashlib.sha256(local[index]).digest()[:HASH_SIZE]
        packed = None if plain or index in copies else compressed(local[index])
        if index in copies:
            drone.request(b"C", struct.pack("<H", index) + digest)
        else:
            if packed:
                try:
                    drone.request(b"Y", struct.pack("<H", index) + digest + packed)
                    sent += len(packed)
                except RuntimeError as error:
                    if "Unknown" not in str(error):
                        raise
                    plain, packed = True, None
            if not packed:
                drone.request(b"W", struct.pack("<H", index) + digest + local[index])
                sent += SECTOR
        print(f"\rWrote {done}/{len(indices)} sectors ({sent / 1024 / (time.monotonic() - started):.0f} KB/s sent)", end="", flush=True)
    print()
    if hurried:
        fast(drone, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    commands = parser.add_subparsers(dest="command", required=True)
    image = commands.add_parser("firmware")
    image.add_argument("image", type=Path)
    commands.add_parser("confirm", help="Keep the running OTA image after a manual check")
    camera_image = commands.add_parser("camera", help="Update the ESP32-CAM through the drone")
    camera_image.add_argument("image", type=Path)
    args = parser.parse_args()

    started = time.monotonic()
    drone = Drone(args.bridge)
    if args.command == "firmware":
        firmware(drone, args.image)
    elif args.command == "camera":
        camera(drone, args.image)
    else:
        drone.request(b"V")
    print(f"Done in {time.monotonic() - started:.1f}s")


if __name__ == "__main__":
    main()
