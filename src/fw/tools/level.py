# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "websockets>=15,<17"]
# ///
"""
Set the level reference: rest the disarmed drone still on a flat spot, then turn it 180° in place on the same spot. The floor's tilt flips sign between the two readings while the IMU's mounting angle does not, so their mean is the drone's level attitude as the IMU sees it; it is stored on the drone as pose_zero.
"""

import argparse

import numpy as np

from ota_direct import Drone, readings

WINDOW = 40  # Status samples (2 s at 20 Hz) that must be still.


def still_accel(url):
    window = []
    for reading in readings(url):
        window = (window + [reading["imu"]["accel"] + reading["imu"]["gyro"]])[-WINDOW:]
        samples = np.array(window)
        if len(window) == WINDOW and samples[:, :3].std(0).max() <= 0.005 and samples[:, 3:].std(0).max() <= 0.2:
            return samples[:, :3].mean(0)


def attitude(accel):
    ax, ay, az = accel
    return np.degrees([np.arctan2(ay, az), np.arctan2(-ax, np.hypot(ay, az))])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="http://127.0.0.1:8765")
    args = parser.parse_args()
    url = f"{args.service}/status"
    first = attitude(still_accel(url))
    print(f"First: roll {first[0]:.2f}°, pitch {first[1]:.2f}°")
    input("Turn the drone 180° in place on the same spot, then press Enter")
    second = attitude(still_accel(url))
    print(f"Second: roll {second[0]:.2f}°, pitch {second[1]:.2f}°")
    level, tilt = (first + second) / 2, (first - second) / 2
    print(f"Level: roll {level[0]:.2f}°, pitch {level[1]:.2f}°; floor tilt: roll {tilt[0]:.2f}°, pitch {tilt[1]:.2f}°")
    Drone(args.service.replace("http", "ws") + "/update").request(b"L", level.astype("<f4").tobytes())
    print("Stored")


if __name__ == "__main__":
    main()
