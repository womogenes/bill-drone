"""
A camera black-box file recorded with full frames (sw/vio/camera_sd.py) and the drone's flight log of the same span as a visual-inertial dataset (sw/vio/dataset.py) with its OpenVINS configuration (sw/vio/openvins/configure.py).

uv run --with opencv-python-headless -m sw.vio.blackbox_dataset --blackbox <card>/blackbox/0007.bin --log sw/recordings/<flight>.bin --out ~/sw/vio/data/<name>

Every flight-log sample names the camera's newest frame (its sequence's low 16 bits) and when that frame's centre row was read on the drone's clock, dated by the SYNC edge; the black box has every frame's centre-row time on the camera's clock. A straight line through the matched frames maps the camera's clock onto the log's (offset and drift), and each full frame is stamped through it at its centre row. Frames are turned upright (the camera is mounted upside down, hover_cam_roll 180) with the calibration the file's header carries, adjusted for the turn. They are rolling shutter: the rows are read over about 23 ms.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from sw.flight_log import read_flight
from sw.vio import dataset
from sw.vio.openvins import configure

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fw/esp32cam"))
from blackbox import entries, read_header  # noqa: E402

GRAVITY = 9.80665
# The MPU6050's noise measured still for 30 minutes (2026-10-31: 8.2e-5 rad/s/sqrt(Hz), 1.8e-3 m/s^2/sqrt(Hz)), widened as sw/vio/recorded.py does for the low-pass filter and handling; the rotors shake it far more in flight (configure.py).
GYRO_NOISE, ACCEL_NOISE = 2e-3, 4e-2
# A matched frame further than this from the clock line (us) is a wrong match.
MATCH_US = 1000


def camera_times(frames):
    """Each frame's centre-row readout on the camera's clock (us), unwrapped past 2^32."""
    raw = np.array([(f["record"]["capture_us"] + f["record"]["flow_offset_us"]) & 0xFFFFFFFF for f in frames], np.int64)
    return raw + np.concatenate([[0], np.cumsum(np.diff(raw) < -(1 << 31))]) * (1 << 32)


def log_frames(log):
    """(sequence low 16 bits, centre-row time in log microseconds) for each frame the log names, from the first sample that names it."""
    named = log[log.camera_age_us.notna()]
    first = named[named.camera_sequence.ne(named.camera_sequence.shift())]
    return first.camera_sequence.astype(np.int64).values, (first.esp_time_us - first.camera_age_us).values


def clock_line(frames, times, log):
    """Log time (us) as a function of camera time (us): the fit through every log frame matched to the black box by sequence, with its residuals."""
    sequences = np.array([f["record"]["sequence"] for f in frames], np.int64)
    by_low = {}
    for index, sequence in enumerate(sequences):
        by_low.setdefault(sequence & 0xFFFF, []).append(index)
    lows, logged = log_frames(log)
    pairs = [(by_low[low][0], t) for low, t in zip(lows, logged) if len(by_low.get(low, ())) == 1]
    if len(pairs) < 10:
        raise ValueError(f"Only {len(pairs)} frames in the log match the black box: not the same span, or a log from before camera_sequence")
    index, logged_t = np.array(pairs).T
    slope, offset = np.polyfit(times[index.astype(int)], logged_t, 1)
    # A 16-bit sequence repeats every 25 minutes at 44 fps: longer files match each log frame to the candidate nearest the first line.
    pairs = []
    for low, t in zip(lows, logged):
        candidates = by_low.get(low, [])
        if candidates:
            best = min(candidates, key=lambda i: abs(slope * times[i] + offset - t))
            pairs.append((best, t))
    index, logged_t = np.array(pairs).T
    index = index.astype(int)
    residual = logged_t - (slope * times[index] + offset)
    keep = np.abs(residual) < MATCH_US
    slope, offset = np.polyfit(times[index[keep]], logged_t[keep], 1)
    residual = logged_t[keep] - (slope * times[index[keep]] + offset)
    return slope, offset, residual, int((~keep).sum())


def upright(header):
    """The header's calibration for the image turned by 180 degrees: the principal point moves to the opposite side and the tangential terms change sign."""
    width, height = header["full"]
    k1, k2, p1, p2, _ = header["distortion"]
    return {"fx": header["fx"], "fy": header["fy"], "cx": width - 1 - header["cx"], "cy": height - 1 - header["cy"], "k1": k1, "k2": k2, "p1": -p1, "p2": -p2}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--blackbox", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    data = args.blackbox.read_bytes()
    header = read_header(data)
    frames = [e for e in entries(data) if "record" in e]
    log, _ = read_flight(args.log)
    times = camera_times(frames)
    slope, offset, residual, wrong = clock_line(frames, times, log)
    print(f"clock: {len(residual)} frames matched ({wrong} refused), drift {(slope - 1) * 1e6:+.1f} ppm, residual RMS {np.sqrt(np.mean(residual ** 2)):.1f} us, max {np.abs(residual).max():.1f} us")
    out = dataset.Writer(args.out.expanduser())
    for row in log.itertuples():
        gyro = [math.radians(v) for v in (row.gx_deg_s, row.gy_deg_s, row.gz_deg_s)]
        out.inertial(row.esp_time_us / 1e6, gyro, [v * GRAVITY for v in (row.ax_g, row.ay_g, row.az_g)])
    start, end = log.esp_time_us.iloc[0], log.esp_time_us.iloc[-1]
    for frame, t in zip(frames, times):
        stamp = slope * t + offset
        if frame["full"] is not None and start < stamp < end:
            out.frame(stamp / 1e6, cv2.imencode(".png", cv2.rotate(frame["full"], cv2.ROTATE_180))[1].tobytes())
    out.close()
    width, height = header["full"]
    config = configure.write(args.out.expanduser(), GYRO_NOISE, ACCEL_NOISE, camera=upright(header), dynamic=True, size=(width, height))
    print(config, f"{len(out.frames)} frames, {len(out.imu)} IMU samples")


if __name__ == "__main__":
    main()
