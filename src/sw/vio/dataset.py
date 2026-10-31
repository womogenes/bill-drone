"""
Visual-inertial data in the EuRoC MAV layout, which OpenVINS, ORB-SLAM3 and most other estimators read, so the same estimators run on simulated flights (sw/vio/simdata.py) and recorded ones:

    <folder>/mav0/cam0/data/<ns>.png, cam0/data.csv: #timestamp [ns],filename
    <folder>/mav0/imu0/data.csv: #timestamp [ns],w_x,w_y,w_z [rad/s],a_x,a_y,a_z [m/s^2]
    <folder>/mav0/state_groundtruth_estimate0/data.csv: #timestamp, p_x,p_y,p_z [m], q_w,q_x,q_y,q_z, v_x,v_y,v_z [m/s] (simulated flights only)

Times are on the drone's clock. The IMU frame is the drone's body (x forward, y left, z up); a camera frame's time is the middle of its readout.
"""

import csv
from pathlib import Path

import numpy as np


def quaternion(rotation):
    """A rotation matrix as (w, x, y, z)."""
    m = np.asarray(rotation)
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return w, x, y, z


class Writer:
    def __init__(self, folder):
        self.root = Path(folder) / "mav0"
        (self.root / "cam0/data").mkdir(parents=True, exist_ok=True)
        (self.root / "imu0").mkdir(parents=True, exist_ok=True)
        (self.root / "state_groundtruth_estimate0").mkdir(parents=True, exist_ok=True)
        self.frames, self.imu, self.truth = [], [], []

    def frame(self, time, png):
        ns = round(time * 1e9)
        (self.root / f"cam0/data/{ns}.png").write_bytes(png)
        self.frames.append((ns, f"{ns}.png"))

    def inertial(self, time, gyro, accel):
        self.imu.append((round(time * 1e9), *gyro, *accel))

    def ground_truth(self, time, position, rotation, velocity):
        self.truth.append((round(time * 1e9), *position, *quaternion(rotation), *velocity))

    def close(self):
        for name, header, rows in (("cam0/data.csv", "#timestamp [ns],filename", self.frames),
                                   ("imu0/data.csv", "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z", self.imu),
                                   ("state_groundtruth_estimate0/data.csv", "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z", self.truth)):
            with open(self.root / name, "w", newline="") as file:
                file.write(header + "\n")
                csv.writer(file, lineterminator="\n").writerows(rows)


def read(folder, name):
    """One of the dataset's CSV files as a float array (the camera's as (ns, filename) rows)."""
    path = Path(folder) / "mav0" / name
    rows = [line.strip().split(",") for line in path.read_text().splitlines() if line and not line.startswith("#")]
    if name.startswith("cam0"):
        return [(int(ns), file) for ns, file in rows]
    return np.array(rows, dtype=float)
