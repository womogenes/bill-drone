"""
A synthetic visual-inertial dataset (sw/vio/dataset.py) on a smooth, lively path through the simulated outdoor scene (sw/vio/scene.py), with the IMU computed exactly from the path: a check of the estimator, its configuration and the dataset conventions, where the simulated hover (sw/vio/simdata.py) moves too little to initialize.

uv run --with mujoco --with opencv-python-headless -m sw.vio.synthetic --out ~/sw/vio/data/synthetic [--seconds 40] [--noise]
"""

import argparse
import os
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

from sw.vio import dataset, scene  # noqa: E402
from sw.vio.openvins import configure  # noqa: E402

GRAVITY = 9.80665
FPS = 25.0


def path(t):
    """Body position (m) and roll, pitch, yaw (rad) at t: walking-pace loops at 1.2 m with turns and some tilt."""
    position = np.array([2.0 * np.sin(0.35 * t), 1.5 * np.sin(0.5 * t + 1.0), 1.2 + 0.25 * np.sin(0.8 * t)])
    angles = np.array([0.15 * np.sin(1.1 * t), 0.12 * np.sin(0.9 * t + 0.5), 0.6 * np.sin(0.3 * t)])
    return position, angles


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose(t):
    position, angles = path(t)
    return position, rotation(*angles)


def inertial(t, h=1e-4):
    """Gyro (rad/s, body) and specific force (m/s², body) from the path by central differences."""
    r0, r1 = pose(t - h)[1], pose(t + h)[1]
    omega = cv2.Rodrigues(r0.T @ r1)[0].ravel() / (2 * h)
    acceleration = (path(t + h)[0] - 2 * path(t)[0] + path(t - h)[0]) / h ** 2
    return omega, pose(t)[1].T @ (acceleration + np.array([0.0, 0.0, GRAVITY]))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", required=True)
    parser.add_argument("--seconds", type=float, default=40)
    parser.add_argument("--noise", action="store_true", help="add MPU6050-like IMU noise and bias")
    parser.add_argument("--global-shutter", action="store_true", help="read every row at once (OpenVINS models no rolling shutter)")
    args = parser.parse_args()
    if args.global_shutter:
        scene.READOUT_S = 0.0
    out_dir = Path(args.out).expanduser()
    out = dataset.Writer(out_dir)
    rng = np.random.default_rng(0)
    bias_g, bias_a = (rng.normal(0, 0.01, 3), rng.normal(0, 0.1, 3)) if args.noise else (np.zeros(3), np.zeros(3))
    start = 1.0
    for t in np.arange(start, start + args.seconds, 0.01):
        omega, force = inertial(t)
        if args.noise:
            omega, force = omega + bias_g + rng.normal(0, 0.003, 3), force + bias_a + rng.normal(0, 0.04, 3)
        out.inertial(t, omega, force)
        position, rot = pose(t)
        out.ground_truth(t, position, rot, (path(t + 1e-3)[0] - path(t - 1e-3)[0]) / 2e-3)
    camera = scene.Camera(out_dir / "scene", 0)
    t = start + 0.05
    while t + scene.READOUT_S < start + args.seconds:
        grey = camera.frame(t, pose)
        out.frame(t + scene.READOUT_S / 2, cv2.imencode(".png", grey)[1].tobytes())
        t += 1 / FPS
    camera.close()
    out.close()
    print(configure.write(out_dir, 0.004, 0.06, dynamic=True), f"{len(out.frames)} frames, {len(out.imu)} IMU samples")


if __name__ == "__main__":
    main()
