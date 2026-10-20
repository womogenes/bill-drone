# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Update the drone over the direct ESP-NOW link through the controller bridge.

firmware: diff the new image against the images this laptop last installed in each OTA
slot; send only changed sectors, have the drone copy sectors the running image already
has, then boot and confirm it. If ESP-IDF rejects the image, send every sector.
files: replace changed Python files, then reset.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import time

from websockets.exceptions import ConnectionClosed, InvalidStatus
from websockets.sync.client import connect

from mpy import compile_driver

FIRMWARE = Path(__file__).resolve().parents[1]
# The last image installed in each slot, by slot label; the drone reports each slot's ELF SHA-256 prefix to check it.
INSTALLED = FIRMWARE / "native/build-runtime-ota/installed"
SECTOR = 4096
HASH_SIZE = 8
FILE_CHUNK = 3840  # The drone reassembles requests of up to 4176 bytes.
HASH_BATCH = 7  # 32-byte file hashes per reply; a reply is one 232-byte radio frame.


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
            except (ConnectionClosed, InvalidStatus):
                # The service refuses or drops clients while the radio link is down, as while the drone restarts.
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
    if image[:1] != b"\xe9":
        raise SystemExit("Expected an ESP application image (micropython.bin)")
    info = json.loads(drone.request(b"I"))
    if info["target"] is None:
        raise SystemExit("Drone runs the factory layout; run make migrate_ota over USB first")
    if len(image) > info["size"]:
        raise SystemExit(f"Image is {len(image)} bytes; {info['target']} holds {info['size']}")
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
    try:
        drone.request(b"B", timeout=10)
    except RuntimeError as error:
        print(f"{error}; sending every sector", flush=True)
        write(drone, local, range(count), set())
        drone.request(b"B", timeout=10)
    record.write_bytes(image)
    print(f"Booting {info['target']}; it rolls back on the next reset unless confirmed", flush=True)
    drone.restart()
    booted = json.loads(drone.request(b"I", patience=60))
    if booted["running"] != info["target"]:
        raise SystemExit(f"Drone runs {booted['running']}, not {info['target']}: the new image was rolled back")
    drone.request(b"V")
    print(f"Confirmed {info['target']}")


def write(drone, local, indices, copies):
    for done, index in enumerate(indices, 1):
        digest = hashlib.sha256(local[index]).digest()[:HASH_SIZE]
        copied = False
        if index in copies:
            try:
                drone.request(b"C", struct.pack("<H", index) + digest)
                copied = True
            except RuntimeError as error:
                print(f"\n{error}; sending sector {index}", flush=True)
        if not copied:
            drone.request(b"W", struct.pack("<H", index) + digest + local[index])
        print(f"\rWrote {done}/{len(indices)} sectors", end="", flush=True)
    print()


def files(drone, names, reset):
    names = names or [
        "config.py",
        "main.py",
        *sorted(str(path.relative_to(FIRMWARE)) for path in (FIRMWARE / "drivers").glob("*.py")),
        "drivers/flight_native.mpy",
    ]
    names = [compile_driver(name) for name in names]
    contents = {name: (FIRMWARE / name).read_bytes() for name in names}
    shadows = [name[:-4] + ".py" for name in names if name.endswith(".mpy")]
    paths = names + shadows
    hashes = b"".join(drone.request(b"F", "\n".join(paths[first:first + HASH_BATCH]).encode())
                      for first in range(0, len(paths), HASH_BATCH))
    hashes = {name: hashes[index * 32:(index + 1) * 32] for index, name in enumerate(paths)}
    changed = 0
    removed = 0
    for name in names:
        data = contents[name]
        digest = hashlib.sha256(data).digest()
        if hashes[name] != digest:
            encoded = name.encode()
            for offset in range(0, max(len(data), 1), FILE_CHUNK):
                drone.request(b"P", struct.pack("<IB", offset, len(encoded)) + encoded + data[offset:offset + FILE_CHUNK])
            drone.request(b"M", digest + encoded)
            print(f"Updated {name} ({len(data)} bytes)", flush=True)
            changed += 1
        if name.endswith(".mpy"):
            shadow = name[:-4] + ".py"
            if hashes[shadow] != bytes(32):
                drone.request(b"K", shadow.encode())
                print(f"Removed {shadow}", flush=True)
                removed += 1
    print(f"{changed}/{len(names)} files updated; {removed} source shadows removed")
    if (changed or removed) and reset:
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
