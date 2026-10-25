"""
Servo bench test through the laptop service: uv run tools/servo_test.py [--amp 5] [--cycles 8]
The drone, disarmed with zero throttle (motors stay off even with the ESCs powered), steps each servo in turn by +amp and -amp degrees and measures the peak gyro jolt after each step; a sloppier servo or linkage shows a larger, later, or more scattered jolt. Prints per-side mean peak pitch and yaw rate (deg/s).
"""

import argparse
import json
import struct
import time

from ota_direct import Drone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    parser.add_argument("--amp", type=float, default=5.0)
    parser.add_argument("--cycles", type=int, default=8)
    args = parser.parse_args()
    drone = Drone(args.bridge)
    result = json.loads(drone.request(b"X", struct.pack("<fB", args.amp, args.cycles)))
    print(f"started: {result['of']} steps of {result['amp']}°")
    while result["active"]:
        time.sleep(1)
        result = json.loads(drone.request(b"X", b"?"))
        print(f"  {result['done']}/{result['of']}", flush=True)
    for side in ("left", "right"):
        r = result[side]
        print(f"{side:5s}: {r['steps']} steps, mean peak pitch rate {r['pitch_rate']:.2f} deg/s, yaw rate {r['yaw_rate']:.2f} deg/s")


if __name__ == "__main__":
    main()
