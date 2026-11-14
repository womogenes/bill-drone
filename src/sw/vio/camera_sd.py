# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Control the camera's black box on its microSD (fw/esp32cam/src/sd.h, camera request D through the drone) for visual-inertial datasets.

uv run sw/vio/camera_sd.py start [--full-every 3]   a new card file with every Nth frame's whole 320x240 image, and the drone's flight log
uv run sw/vio/camera_sd.py stop                      ends the flight log; the card goes back to records and reduced frames only
uv run sw/vio/camera_sd.py status

The drone must be disarmed. Every frame's record (pose, flow, capture time on the camera's clock) and its 80x60 reduced image go to the card whenever it records, as it does from boot; a full frame adds about 77 KB, and at the tracker's 44 fps every 3rd frame (about 15 fps, 1.7 MB/s in all) stays inside the card's 2.5 MB/s. The flight log (request O while disarmed; arming starts its own) carries each sample's newest camera frame, which ties the card's frames to the IMU (sw/vio/blackbox_dataset.py).
"""

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fw/tools"))
from ota_direct import Drone  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("--full-every", type=int, default=3)
    parser.add_argument("--url", default="ws://127.0.0.1:8765/update")
    args = parser.parse_args()
    drone = Drone(args.url)
    if args.action == "stop":
        drone.request(b"O", b"\x00")
    every, mode = {"start": (args.full_every, 2), "stop": (0, 1), "status": (0xFFFF, 0xFF)}[args.action]
    print(drone.request(b"K", b"D" + struct.pack("<HB", every, mode), timeout=3).decode())
    if args.action == "start":
        drone.request(b"O", b"\x01")


if __name__ == "__main__":
    main()
