# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "websockets>=15,<17"]
# ///
"""
Calibrate the accelerometer: rest the disarmed drone still (propped, not hand-held) in different orientations (each axis up and down, then a few in between) until enough poses are collected. Fits per-axis offset and gain so every still pose reads 1 g, then stores them on the drone. Recalibrate the IMU (gyro bias and level) afterwards.
"""

import argparse
import json
import time
import urllib.request

import numpy as np
from scipy.optimize import least_squares

from ota_direct import Drone

WINDOW = 30  # Status samples (1.5 s at 20 Hz) that must be still to count as a pose.
SEPARATION = np.radians(30)


def readings(url):
    last = None
    while True:
        reading = json.load(urllib.request.urlopen(url))["reading"]
        if reading["time"] != last and reading["imu"]:
            last = reading["time"]
            yield reading
        time.sleep(0.02)


def collect(url, poses, settings):
    """
    Still poses as raw body-axis accelerometer means, undoing the correction the drone currently applies.
    """
    offset = np.array([settings[f"accel_offset_{axis}"] for axis in "xyz"])
    gain = np.array([settings[f"accel_gain_{axis}"] for axis in "xyz"])
    window, found = [], []
    for reading in readings(url):
        window = (window + [reading["imu"]["accel"] + reading["imu"]["gyro"]])[-WINDOW:]
        samples = np.array(window)
        # At rest the accelerometer noise is about 0.002 g and the gyro's about 0.03 °/s.
        if len(window) < WINDOW or samples[:, :3].std(0).max() > 0.005 or samples[:, 3:].std(0).max() > 0.2:
            continue
        mean = samples[:, :3].mean(0) / gain + offset
        direction = mean / np.linalg.norm(mean)
        if all(np.arccos(np.clip(direction @ other / np.linalg.norm(other), -1, 1)) > SEPARATION for other in found):
            found.append(mean)
            print(f"Pose {len(found)}/{poses}: {np.round(mean, 3)} ({np.linalg.norm(mean):.3f} g)", flush=True)
            if len(found) == poses:
                return np.array(found)
        window = []


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="http://127.0.0.1:8765")
    parser.add_argument("--poses", type=int, default=10)
    args = parser.parse_args()
    settings = json.load(urllib.request.urlopen(f"{args.service}/settings"))["settings"]
    measured = collect(f"{args.service}/status", args.poses, settings)
    fit = least_squares(lambda p: np.linalg.norm((measured - p[:3]) * p[3:], axis=1) - 1, [0, 0, 0, 1, 1, 1])
    offset, gain = fit.x[:3], fit.x[3:]
    before = np.linalg.norm(measured, axis=1) - 1
    after = np.linalg.norm((measured - offset) * gain, axis=1) - 1
    print(f"Offset {np.round(offset, 4)} g, gain {np.round(gain, 4)}")
    print(f"|a| - 1 g: before max {np.abs(before).max() * 1000:.0f} mg, after max {np.abs(after).max() * 1000:.1f} mg")
    Drone(args.service.replace("http", "ws") + "/update").request(b"Q", np.concatenate([offset, gain]).astype("<f4").tobytes())
    print("Stored; now recalibrate the IMU from the dashboard")


if __name__ == "__main__":
    main()
