"""
A recording of the camera's FPV frames with the drone's IMU (sw/vio/record_camera.py --imu) as a visual-inertial dataset (sw/vio/dataset.py) with its OpenVINS configuration (sw/vio/openvins/configure.py).

uv run --with opencv-python-headless -m sw.vio.recorded --footage ~/sw/vio/footage/room_imu --log sw/recordings/<flight>.csv --out ~/sw/vio/data/room_imu [--offset S]

offset: drone time minus camera time in seconds; without it, found from the motion (find_offset). Frames are turned upright (the camera is mounted upside down, hover_cam_roll 180) and stamped at the middle of their readout. The camera's calibration is the tracking one (fw/esp32cam/src/config.h: fx 369.5 at 320x240 over the whole array) scaled to FPV's 480x320, which takes the array's full width and its middle 2/3 of the height; its distortion coefficients act on normalised coordinates, so they carry over.
"""

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from sw.vio import dataset, scene
from sw.vio.openvins import configure

GRAVITY = 9.80665
# Handheld, without rotors: the MPU6050's own noise (about 1e-4 rad/s/sqrt(Hz) and 4e-3 m/s^2/sqrt(Hz)) widened tenfold for the low-pass filter and handling.
GYRO_NOISE, ACCEL_NOISE = 2e-3, 4e-2


def camera():
    scale = 480 / 320
    fx, fy, cx, cy = 369.494757463 * scale, 370.922837184 * scale, 160.420727397 * scale, 160 + (110.831046213 - 120) * scale
    # Turning the image by 180 degrees moves the principal point to the opposite side.
    return {"fx": fx, "fy": fy, "cx": 479 - cx, "cy": 319 - cy, "k1": -0.031215286, "k2": 0.896256747}


def find_offset(footage, frames, log):
    """Drone time minus camera time (s): where the image's sideways and vertical shift between frames best match the gyro's yaw and pitch rates (they correlated -0.56 and +0.59 on 2026-10-31), within 1.5 s of the recording's start on each clock, which start together."""
    t = np.array([int(f["capture_us"]) for f in frames]) / 1e6
    shift, previous = [], None
    for frame in frames:
        grey = cv2.resize(cv2.imread(str(footage / frame["file"]), cv2.IMREAD_GRAYSCALE), (160, 106))
        flow = None if previous is None else cv2.calcOpticalFlowFarneback(previous, grey, None, 0.5, 2, 9, 2, 5, 1.1, 0)
        shift.append((np.nan, np.nan) if flow is None else (np.median(flow[..., 0]), np.median(flow[..., 1])))
        previous = grey
    dt = np.diff(t, prepend=np.nan)
    ok = np.isfinite(dt) & (dt < 0.07)
    middle = t - dt / 2
    rate = np.array(shift) / dt[:, None]
    grid = np.arange(t[2], t[-1], 0.005)
    image_x, image_y = (np.interp(grid, middle[ok], rate[ok, k]) for k in range(2))
    drone = log.esp_time_us.values / 1e6

    def correlation(a, b):
        m = np.isfinite(b)
        a, b = a[m] - a[m].mean(), b[m] - b[m].mean()
        return np.sum(a * b) / np.sqrt(np.sum(a * a) * np.sum(b * b))

    def score(offset):
        yaw = np.interp(grid + offset, drone, log.gz_deg_s.values, left=np.nan, right=np.nan)
        pitch = np.interp(grid + offset, drone, log.gy_deg_s.values, left=np.nan, right=np.nan)
        return -correlation(image_x, yaw) + correlation(image_y, pitch)

    coarse = drone[0] - t[0]
    candidates = np.arange(coarse - 1.5, coarse + 1.5, 0.002)
    scores = [score(c) for c in candidates]
    best = int(np.argmax(scores))
    print(f"clock offset {candidates[best]:.4f} s, match {scores[best] / 2:.2f} (best elsewhere {max(s for c, s in zip(candidates, scores) if abs(c - candidates[best]) > 0.3) / 2:.2f})")
    return candidates[best]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--footage", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--offset", type=float)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = dataset.Writer(args.out.expanduser())
    log = pd.read_csv(args.log)
    for row in log.itertuples():
        gyro = [math.radians(v) for v in (row.gx_deg_s, row.gy_deg_s, row.gz_deg_s)]
        out.inertial(row.esp_time_us / 1e6, gyro, [v * GRAVITY for v in (row.ax_g, row.ay_g, row.az_g)])
    start, end = log.esp_time_us.iloc[0] / 1e6, log.esp_time_us.iloc[-1] / 1e6
    footage = args.footage.expanduser()
    frames = list(csv.DictReader(open(footage / "frames.csv")))
    offset = find_offset(footage, frames, log) if args.offset is None else args.offset
    for frame in frames:
        time = int(frame["capture_us"]) / 1e6 + offset + scene.READOUT_S / 2
        if start < time < end:
            grey = cv2.rotate(cv2.imread(str(footage / frame["file"]), cv2.IMREAD_GRAYSCALE), cv2.ROTATE_180)
            out.frame(time, cv2.imencode(".png", grey)[1].tobytes())
    out.close()
    print(configure.write(args.out.expanduser(), GYRO_NOISE, ACCEL_NOISE, camera=camera(), dynamic=True), f"{len(out.frames)} frames, {len(out.imu)} IMU samples")


if __name__ == "__main__":
    main()
