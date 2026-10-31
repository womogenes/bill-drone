# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Record the camera's FPV stream from the dashboard (sw/dashboard/app/api/camera/route.ts) as a folder of JPEGs named by capture time, for visual-inertial and vision tests on real footage.

uv run src/sw/vio/record_camera.py --out ~/sw/vio/footage/room [--seconds 60] [--imu]

With --imu the drone records its flight log while disarmed for the same span (request O, fw/drone/main/radio.c): the laptop service saves it under sw/recordings like a flight, with the IMU at 100 Hz on the drone's clock; the two clocks are aligned afterwards from the motion. Each frame is <capture_us>.jpg, its capture start on the camera's clock (fw/esp32cam/src/fpv.h), and frames.csv lists capture_us, the laptop's receive time (Unix seconds) and the file. Asking for the stream turns the camera to FPV (tag tracking off) until 3 s after the recording ends.
"""

import argparse
import csv
import re
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fw/tools"))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--url", default="http://localhost:3000/api/camera?fpv=1")
    parser.add_argument("--imu", action="store_true", help="record the drone's flight log (IMU) while disarmed alongside")
    args = parser.parse_args()
    drone = None
    if args.imu:
        from ota_direct import Drone

        drone = Drone("ws://127.0.0.1:8765/update")
        drone.request(b"O", b"\x01")
    out = args.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)
    stream = urllib.request.urlopen(args.url, timeout=10)
    buffer = b""
    count = 0
    end = time.monotonic() + args.seconds
    with open(out / "frames.csv", "w", newline="") as index:
        rows = csv.writer(index)
        rows.writerow(["capture_us", "received_unix_s", "file"])
        while time.monotonic() < end:
            buffer += stream.read1(65536)
            while True:
                header = re.search(rb"Content-Length: (\d+)\r\nX-Time: (\d+) \d+\r\n\r\n", buffer)
                if not header:
                    break
                start, length = header.end(), int(header.group(1))
                if len(buffer) < start + length:
                    break
                name = f"{int(header.group(2))}.jpg"
                (out / name).write_bytes(buffer[start:start + length])
                rows.writerow([header.group(2).decode(), f"{time.time():.6f}", name])
                count += 1
                buffer = buffer[start + length:]
    if drone:
        drone.request(b"O", b"\x00")
        print("Flight log stopped; the laptop service saves it under sw/recordings")
    print(f"{count} frames in {args.seconds:.0f} s to {out}")


if __name__ == "__main__":
    main()
