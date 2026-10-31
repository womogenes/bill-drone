# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Talks to the camera's ESP32 ROM bootloader over its wire to the drone (request U, fw/drone/main/radio.c), for a camera that stopped answering and cannot be power-cycled from here: on 2026-10-31 it went silent after line errors on the link, which is what the ROM's boot message at 115200 looks like to the drone's 2 Mbaud receiver.

uv run tools/camera_rom.py listen    # what the camera sends at 115200
uv run tools/camera_rom.py sync      # the ROM's SYNC handshake
uv run tools/camera_rom.py run       # sync, then leave the loader and run the application in flash (as esptool run)

The bootloader protocol (SLIP frames: direction, opcode, length, checksum, data) is Espressif's serial protocol; the drone must be disarmed.
"""

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_direct import Drone  # noqa: E402

ROM_BAUD = 115200
SYNC, FLASH_BEGIN, FLASH_END = 0x08, 0x02, 0x04


def slip(packet):
    return b"\xc0" + packet.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"


def command(op, data=b"", checksum=0):
    return slip(struct.pack("<BBHI", 0, op, len(data), checksum) + data)


def responses(raw):
    """The ROM's replies in raw bytes: (opcode, value, data)."""
    out = []
    for frame in raw.split(b"\xc0"):
        frame = frame.replace(b"\xdb\xdc", b"\xc0").replace(b"\xdb\xdd", b"\xdb")
        if len(frame) >= 8 and frame[0] == 1:
            op, size, value = struct.unpack_from("<BHI", frame, 1)
            out.append((op, value, frame[8:8 + size]))
    return out


def raw(drone, data, wait_ms):
    return drone.request(b"U", struct.pack("<IH", ROM_BAUD, wait_ms) + data, timeout=5, patience=20)


def sync(drone):
    packet = command(SYNC, b"\x07\x07\x12\x20" + b"\x55" * 32)
    for attempt in range(10):
        got = responses(raw(drone, packet, 150))
        if any(op == SYNC for op, _, _ in got):
            print(f"ROM answered SYNC (attempt {attempt + 1})")
            return True
    print("No SYNC reply")
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("action", choices=("listen", "sync", "run"))
    args = parser.parse_args()
    drone = Drone("ws://127.0.0.1:8765/update")
    if args.action == "listen":
        data = raw(drone, b"", 1500)
        print(len(data), "bytes:", data[:300])
        return
    if not sync(drone):
        sys.exit(1)
    if args.action == "run":
        # As esptool run: an empty flash begin, then flash end with 1 (leave the loader and jump to the application).
        print(responses(raw(drone, command(FLASH_BEGIN, struct.pack("<IIII", 0, 0, 0x4000, 0)), 300)))
        print(responses(raw(drone, command(FLASH_END, struct.pack("<I", 1)), 300)))


if __name__ == "__main__":
    main()
