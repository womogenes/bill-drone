"""
Measure control effectiveness from recordings: uv run -m sw.fit sw/recordings/<flight>.bin [...]
Regresses each axis's angular acceleration on its actuator torque term at several delays and compares the slope with the controller's model (1.0 means the model is right). Ground contact pulls slopes toward zero, so trust free-flight segments.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.flight_log import read_flight
from sw.sim.flight import THRUST_TABLE

DEG = np.pi / 180
TABLE = np.array(THRUST_TABLE, float)


def settings(path):
    merged = {}
    for line in path.with_suffix(".jsonl").read_text().splitlines():
        merged |= json.loads(line).get("settings", {})
    return merged


def acceleration(rate, window):
    """
    Angular acceleration in rad/s² from a 100 Hz rate in deg/s, smoothed over window samples.
    """
    smooth = np.convolve(rate * DEG, np.ones(window) / window, mode="same")
    return np.gradient(smooth, 0.01)


def terms(data, setting):
    """
    Per-axis torque terms in N·m (actuator part only) and the model's angular acceleration per N·m.
    """
    left = np.interp(data.left_throttle, TABLE[:, 0], TABLE[:, 1]) * setting["left_thrust_scale"]
    right = np.interp(data.right_throttle, TABLE[:, 0], TABLE[:, 1]) * setting["right_thrust_scale"]
    # Physical tilt: the right servo is mounted mirrored.
    left_tilt, right_tilt = data.left_servo_deg.values * DEG, -data.right_servo_deg.values * DEG
    arm, height = setting["arm"], setting["pivot_height"]
    return left + right, {
        "roll": ((left * np.cos(left_tilt) - right * np.cos(right_tilt)) * arm / 2, 1 / setting["roll_inertia"]),
        "pitch": ((left * np.sin(left_tilt) + right * np.sin(right_tilt)) * height, 1 / setting["pitch_inertia"]),
        "yaw": ((right * np.sin(right_tilt) - left * np.sin(left_tilt)) * arm / 2, 1 / setting["yaw_inertia"]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", type=Path, nargs="+")
    parser.add_argument("--weight", type=float, default=8.83, help="N; samples below 0.95 of it are skipped")
    parser.add_argument("--window", type=int, default=25, help="samples of rate smoothing")
    args = parser.parse_args()
    rows = {axis: [] for axis in ("roll", "pitch", "yaw")}
    for path in args.files:
        data, _ = read_flight(path)
        total, axes = terms(data, settings(path))
        mask = total > 0.95 * args.weight
        rates = {"roll": data.gx_deg_s, "pitch": data.gy_deg_s, "yaw": data.gz_deg_s}
        for axis, (torque, model) in axes.items():
            rows[axis].append((torque, model, acceleration(rates[axis].values, args.window), mask))
    for axis, flights in rows.items():
        best = None
        for delay in range(0, 16, 2):
            x, y = [], []
            for torque, model, accel, mask in flights:
                shifted = np.roll(torque * model, delay)
                keep = mask.copy()
                keep[: delay + args.window] = False
                x.append(shifted[keep])
                y.append(accel[keep])
            x, y = np.concatenate(x), np.concatenate(y)
            design = np.column_stack([x, np.ones_like(x)])
            (slope, offset), *_ = np.linalg.lstsq(design, y, rcond=None)
            fit = 1 - np.var(y - design @ [slope, offset]) / np.var(y)
            if best is None or fit > best[3]:
                best = (delay, slope, offset, fit, len(y))
        delay, slope, offset, fit, count = best
        print(f"{axis:5} effectiveness {slope:5.2f} x model, delay {delay * 10:3d} ms, disturbance {offset:+.2f} rad/s², R² {fit:.2f}, {count} samples")


if __name__ == "__main__":
    main()
