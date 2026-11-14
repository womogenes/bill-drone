"""
A recording of the camera's FPV frames with the drone's IMU (sw/vio/record_camera.py --imu) as a visual-inertial dataset (sw/vio/dataset.py) with its OpenVINS configuration (sw/vio/openvins/configure.py).

uv run --with opencv-python-headless -m sw.vio.recorded --footage ~/sw/vio/footage/room_imu --log sw/recordings/<flight>.bin --out ~/sw/vio/data/room_imu [--offset S] [--calibration <dataset>] [--readout-ms 35.6] [--rectify]

Frames are stamped through the log when it names the camera's frames (camera_sequence and camera_age_us, the camera sending each FPV frame's sequence and SYNC pulse to the drone): the frame numbers match the log's frames and a straight line maps the camera's clock onto the drone's (sw/vio/blackbox_dataset.py clock_line), to the frame's start as the drone dated it, plus half the readout (READOUT_MS). Otherwise offset: drone time minus camera time in seconds; without it, found from the motion (find_offset). Frames are turned upright (the camera is mounted upside down, hover_cam_roll 180) and stamped at the middle of their readout. The camera's calibration is another dataset's (--calibration, fitted from tags by sw/vio/tags.py --save), or else the tracking one (fw/esp32cam/src/config.h: fx 369.5 at 320x240 over the whole array) scaled to the frames' size (camera); its distortion coefficients act on normalized coordinates, so they carry over.
"""

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml

from sw.flight_log import read_flight
from sw.vio import dataset, rolling, scene
from sw.vio.blackbox_dataset import clock_line
from sw.vio.openvins import configure

GRAVITY = 9.80665
# Handheld, without rotors: the MPU6050's own noise (about 1e-4 rad/s/sqrt(Hz) and 4e-3 m/s^2/sqrt(Hz)) widened tenfold for the low-pass filter and handling.
GYRO_NOISE, ACCEL_NOISE = 2e-3, 4e-2
# FPV's rolling-shutter readout, first row to last (ms), by frame size: 640x480 from the sensor's registers during VGA FPV (Bill's agent, 2026-11-14: 774 rows at 46.05 us). Other sizes take the simulated camera's (scene.READOUT_S).
READOUT_MS = {(640, 480): 35.6}


def camera(width, height):
    """The tracking calibration (320x240 over the whole array) scaled to an FPV frame of this size, which takes the array's full width and, when it is wider than 4:3, its middle rows (480x320: 2/3 of them); 640x480 is the whole array. Turned upright."""
    scale = width / 320
    fx, fy, cx, cy = 369.494757463 * scale, 370.922837184 * scale, 160.420727397 * scale, height / 2 + (110.831046213 - 120) * scale
    # Turning the image by 180 degrees moves the principal point to the opposite side.
    return {"fx": fx, "fy": fy, "cx": width - 1 - cx, "cy": height - 1 - cy, "k1": -0.031215286, "k2": 0.896256747}


def calibrated(folder):
    """The camera calibration of another dataset (sw/vio/tags.py --save writes the one it fits from the tags)."""
    cam = yaml.safe_load((Path(folder).expanduser() / "kalibr_imucam_chain.yaml").read_text().replace("%YAML:1.0", ""))["cam0"]
    fx, fy, cx, cy = cam["intrinsics"]
    k1, k2, p1, p2 = cam["distortion_coeffs"]
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "k1": k1, "k2": k2, "p1": p1, "p2": p2}


def find_offset(footage, frames, log):
    """Drone time minus camera time (s): where the image's sideways and vertical shift between frames best match the gyro's yaw and pitch rates (they correlated -0.56 and +0.59 on 2026-10-31), within 1.5 s of the recording's start on each clock, which start together."""
    t = np.array([int(f["capture_us"]) for f in frames]) / 1e6
    shift, previous = [], None
    for frame in frames:
        gray = cv2.resize(cv2.imread(str(footage / frame["file"]), cv2.IMREAD_GRAYSCALE), (160, 106))
        flow = None if previous is None else cv2.calcOpticalFlowFarneback(previous, gray, None, 0.5, 2, 9, 2, 5, 1.1, 0)
        shift.append((np.nan, np.nan) if flow is None else (np.median(flow[..., 0]), np.median(flow[..., 1])))
        previous = gray
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
    parser.add_argument("--calibration", type=Path, help="a dataset whose camera calibration to use (sw/vio/tags.py --save)")
    parser.add_argument("--readout-ms", type=float, help="the sensor's readout, first row to last (default READOUT_MS for the frames' size)")
    parser.add_argument("--rectify", action="store_true", help="rectify the rolling shutter from the gyro (sw/vio/rolling.py)")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = dataset.Writer(args.out.expanduser())
    log = read_flight(args.log)[0] if args.log.suffix == ".bin" else pd.read_csv(args.log)
    for row in log.itertuples():
        gyro = [math.radians(v) for v in (row.gx_deg_s, row.gy_deg_s, row.gz_deg_s)]
        out.inertial(row.esp_time_us / 1e6, gyro, [v * GRAVITY for v in (row.ax_g, row.ay_g, row.az_g)])
    start, end = log.esp_time_us.iloc[0] / 1e6, log.esp_time_us.iloc[-1] / 1e6
    footage = args.footage.expanduser()
    frames = list(csv.DictReader(open(footage / "frames.csv")))
    height, width = cv2.imread(str(footage / frames[0]["file"]), cv2.IMREAD_GRAYSCALE).shape
    readout = (args.readout_ms if args.readout_ms is not None else READOUT_MS.get((width, height), scene.READOUT_S * 1e3)) / 1e3
    capture = np.array([int(f["capture_us"]) for f in frames], np.int64)
    capture += np.concatenate([[0], np.cumsum(np.diff(capture) < -(1 << 31))]) * (1 << 32)
    if "frame" in frames[0] and "camera_age_us" in log and log.camera_age_us.notna().any():
        numbers = np.array([int(f["frame"]) for f in frames], np.int64)
        numbers += np.concatenate([[0], np.cumsum(np.diff(numbers) < -(1 << 15))]) << 16
        slope, intercept, residual, wrong = clock_line(numbers, capture, log)
        print(f"clock from the log: {len(residual)} frames matched ({wrong} refused), drift {(slope - 1) * 1e6:+.1f} ppm, residual RMS {np.sqrt(np.mean(residual ** 2)):.1f} us")
        # FPV frames are dated at their start (fw/esp32cam/src/fpv.h), as capture_us is: the middle of the readout is half of it later.
        stamps = (slope * capture + intercept) / 1e6 + readout / 2
    else:
        offset = find_offset(footage, frames, log) if args.offset is None else args.offset
        stamps = capture / 1e6 + offset + readout / 2
    intrinsics = calibrated(args.calibration) if args.calibration else camera(width, height)
    matrix = np.array([[intrinsics["fx"], 0, intrinsics["cx"]], [0, intrinsics["fy"], intrinsics["cy"]], [0, 0, 1]])
    gyro = np.radians(log[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].values)
    for frame, time in zip(frames, stamps):
        if start < time < end:
            gray = cv2.rotate(cv2.imread(str(footage / frame["file"]), cv2.IMREAD_GRAYSCALE), cv2.ROTATE_180)
            if args.rectify:
                # Turned upright, the sensor's first row is the image's bottom one.
                omega = [np.interp(time * 1e6, log.esp_time_us.values, gyro[:, k]) for k in range(3)]
                gray = rolling.rectify(gray, readout, omega, scene.CAMERA_TO_BODY, matrix, bottom_first=True)
            out.frame(time, cv2.imencode(".png", gray)[1].tobytes())
    out.close()
    print(configure.write(args.out.expanduser(), GYRO_NOISE, ACCEL_NOISE, camera=intrinsics, dynamic=True, size=(width, height)), f"{len(out.frames)} frames, {len(out.imu)} IMU samples")


if __name__ == "__main__":
    main()
