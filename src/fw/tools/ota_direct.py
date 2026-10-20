# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Update the drone over the direct ESP-NOW link through the controller bridge.

firmware: write the sectors of the inactive OTA slot that differ from the new
image, copying sectors the running image already has, then boot and confirm it.
files: replace changed Python files, then reset.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import time

from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

FIRMWARE = Path(__file__).resolve().parents[1]
SECTOR = 4096
HASH_SIZE = 8
FILE_CHUNK = 1024  # Five radio fragments; the drone receive buffer holds 4096 bytes.


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
            except ConnectionClosed:
                # The bridge refuses or drops clients while the radio link is down.
                self.ws = None
                time.sleep(0.5)
                continue
            if reply[4] != 0:
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


def firmware(drone, path):
    image = path.read_bytes()
    if image[:1] != b"\xe9":
        raise SystemExit("Expected an ESP application image (micropython.bin)")
    info = json.loads(drone.request(b"I"))
    if info["target"] is None:
        raise SystemExit("Drone runs the factory layout; run make migrate_ota over USB first")
    if len(image) > info["size"]:
        raise SystemExit(f"Image is {len(image)} bytes; {info['target']} holds {info['size']}")
    image += b"\xff" * (-len(image) % SECTOR)
    count = len(image) // SECTOR
    sectors = [image[index * SECTOR:(index + 1) * SECTOR] for index in range(count)]
    local = [hashlib.sha256(sector).digest()[:HASH_SIZE] for sector in sectors]
    # Small replies: the drone heap cannot always allocate one hash list for the whole image.
    remote = [b"".join(drone.request(b"H", struct.pack("<BHH", slot, first, min(32, count - first)), timeout=10)
                       for first in range(0, count, 32)) for slot in (0, 1)]
    running, target = ([hashes[index * HASH_SIZE:(index + 1) * HASH_SIZE] for index in range(count)]
                       for hashes in remote)
    if running == local:
        print(f"{info['running']} already runs this image")
        return
    stale = [index for index in range(count) if target[index] != local[index]]
    copies = [index for index in stale if running[index] == local[index]]
    writes = [index for index in stale if running[index] != local[index]]
    print(f"{info['target']}: {len(writes)} sectors to send, {len(copies)} to copy from {info['running']}, "
          f"{count - len(stale)} already current", flush=True)
    for index in copies:
        drone.request(b"C", struct.pack("<H", index))
    for done, index in enumerate(writes, 1):
        drone.request(b"W", struct.pack("<H", index) + local[index] + sectors[index])
        print(f"\rSent {done}/{len(writes)} sectors", end="", flush=True)
    print()
    drone.request(b"B", timeout=10)
    print(f"Booting {info['target']}; it rolls back on the next reset unless confirmed", flush=True)
    drone.restart()
    booted = json.loads(drone.request(b"I", patience=60))
    if booted["running"] != info["target"]:
        raise SystemExit(f"Drone runs {booted['running']}, not {info['target']}: the new image was rolled back")
    drone.request(b"V")
    print(f"Confirmed {info['target']}")


def files(drone, names, reset):
    names = names or [
        "config.py",
        "main.py",
        *sorted(str(path.relative_to(FIRMWARE)) for path in (FIRMWARE / "drivers").glob("*.py")),
        "drivers/flight_native.mpy",
        *sorted(str(path.relative_to(FIRMWARE)) for path in (FIRMWARE / "lib/microdot").glob("*.mpy")),
    ]
    contents = {name: (FIRMWARE / name).read_bytes() for name in names}
    hashes = drone.request(b"F", "\n".join(names).encode())
    changed = 0
    for index, name in enumerate(names):
        data = contents[name]
        digest = hashlib.sha256(data).digest()
        if hashes[index * 32:(index + 1) * 32] == digest:
            continue
        encoded = name.encode()
        for offset in range(0, max(len(data), 1), FILE_CHUNK):
            drone.request(b"P", struct.pack("<IB", offset, len(encoded)) + encoded + data[offset:offset + FILE_CHUNK])
        drone.request(b"M", digest + encoded)
        print(f"Updated {name} ({len(data)} bytes)", flush=True)
        changed += 1
    print(f"{changed}/{len(names)} files updated")
    if changed and reset:
        drone.restart()
        drone.request(b"I", patience=60)
        print("Drone restarted")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    commands = parser.add_subparsers(dest="command", required=True)
    image = commands.add_parser("firmware")
    image.add_argument("image", type=Path)
    upload = commands.add_parser("files")
    upload.add_argument("--no-reset", action="store_true")
    upload.add_argument("names", nargs="*", help="Paths relative to src/fw")
    commands.add_parser("confirm", help="Keep the running OTA image after a manual check")
    args = parser.parse_args()

    started = time.monotonic()
    drone = Drone(args.bridge)
    if args.command == "firmware":
        firmware(drone, args.image)
    elif args.command == "files":
        files(drone, args.names, not args.no_reset)
    else:
        drone.request(b"V")
    print(f"Done in {time.monotonic() - started:.1f}s")


if __name__ == "__main__":
    main()
