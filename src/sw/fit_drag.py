"""
Rotor-drag identification from airborne recordings: uv run -m sw.fit_drag sw/recordings/<flight>.bin [...]
Multirotor blades flap and drag in proportion to the airspeed, so the accelerometer's horizontal specific force reads about -k times the body velocity on top of the thrust vector's own forward part (the servos tilt it in body x). Over short windows the velocity is known well from inertial dead reckoning (the accelerometer turned level by the logged attitude, integrated), up to a constant and a ramp from attitude error that each window projects out; k, the thrust-vector gain and the IMU's lever arm above the pitch axis are then regressed. Flight means test what the band cannot see: the steady offset of the drag-model velocity (servo zero and mounting), which on hovers that held their place should average to zero, cross-checked against the attitude (pitch plus thrust tilt averages zero in a hover that does not accelerate).
With --blackbox <file.bin> (the camera's black box, fw/esp32cam/blackbox.py --fetch), trusted sightings give ground-truth velocity: k and the offset are fitted against it directly and the drag-model velocity's error is reported.
"""

import argparse
from pathlib import Path

import numpy as np

from sw.fit_actuator import DEG, DT, paths, servo, window
from sw.flight_log import read_flight

G = 9.81
# Servo command to physical tilt (sw/sim/sweep.py FITTED): dead time in samples, lag (s), slew (°/s).
SERVO = (7, 0.01, 340)


def load(path):
    """
    One airborne span: calibrated specific force (g, body forward/left/up), the thrust vector's forward part per unit of its upward part, attitude, body rates and the camera-sighting flag.
    """
    flight = window(path)
    if not flight:
        return None
    data, _ = read_flight(path)
    setting = flight["setting"]
    # The drone calibrates the accelerometer before use (flight_rt.c); the log keeps it raw.
    offset = [setting.get("accel_offset_" + axis, 0.0) for axis in "xyz"]
    gain = [setting.get("accel_gain_" + axis, 1.0) for axis in "xyz"]
    force = np.column_stack([(data[column].values - o) * g for column, o, g in zip(("ax_g", "ay_g", "az_g"), offset, gain)])
    # Into the frame the level reference defines (the logged attitude is relative to it): the IMU sits rolled and pitched on the frame by pose_zero, which shows in hover as a steady sideways or forward specific force otherwise.
    zero_roll, zero_pitch = setting.get("pose_zero_roll", 0.0) * DEG, setting.get("pose_zero_pitch", 0.0) * DEG
    fy = force[:, 1] * np.cos(zero_roll) - force[:, 2] * np.sin(zero_roll)
    fz = force[:, 1] * np.sin(zero_roll) + force[:, 2] * np.cos(zero_roll)
    force = np.column_stack([force[:, 0] * np.cos(zero_pitch) + fz * np.sin(zero_pitch), fy, -force[:, 0] * np.sin(zero_pitch) + fz * np.cos(zero_pitch)])
    delay, tau, slew = SERVO
    tilt = np.column_stack([servo(flight["command"][:, side], delay, tau, slew, 0.0) for side in range(2)]) * DEG
    thrust = flight["thrust"]
    vector = (thrust * np.sin(tilt)).sum(1) / np.maximum((thrust * np.cos(tilt)).sum(1), 1e-3)
    return {
        "name": flight["name"], "lift": flight["lift"], "end": flight["end"], "setting": setting, "force": force, "vector": vector,
        "roll": data.roll_deg.values * DEG, "pitch": data.pitch_deg.values * DEG,
        "gyro": np.column_stack([data.gx_deg_s.values, data.gy_deg_s.values, data.gz_deg_s.values]) * DEG,
        "seen": (data["flags"].values & 8) > 0, "time_us": data.esp_time_us.values,
    }


def dead_reckon(flight, low, high):
    """
    Horizontal velocity (m/s; forward, left in the heading frame) from low, starting at zero: the specific force turned level and integrated, in a frame turning with the heading.
    """
    velocity = np.zeros((high - low, 2))
    forward = left = 0.0
    for row, i in enumerate(range(low, high)):
        roll, pitch = flight["roll"][i], flight["pitch"][i]
        cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
        fx, fy, fz = flight["force"][i]
        ahead = G * (cp * fx + sp * sr * fy + sp * cr * fz)
        side = G * (cr * fy - sr * fz)
        turn = (flight["gyro"][i, 1] * sr + flight["gyro"][i, 2] * cr) / cp
        forward += (ahead + turn * left) * DT
        left += (side - turn * forward) * DT
        velocity[row] = forward, left
    return velocity


def smooth(values, samples):
    kernel = np.ones(samples) / samples
    return np.apply_along_axis(lambda column: np.convolve(column, kernel, mode="same"), 0, values)


def detrend(block):
    t = np.arange(len(block)) - (len(block) - 1) / 2
    basis = np.column_stack([np.ones(len(block)), t])
    coefficients, *_ = np.linalg.lstsq(basis, block, rcond=None)
    return block - basis @ coefficients


def blocks(flight, length, step, samples, trim):
    """
    Per window, detrended and smoothed alike: forward and left specific force (m/s²), dead-reckoned velocity forward and left, the thrust vector's forward specific force, pitch and roll angular acceleration.
    """
    out = []
    for low in range(flight["lift"] + 20, flight["end"] - 5 - length + 1, step):
        high = low + length
        velocity = dead_reckon(flight, low, high)
        force = G * flight["force"][low:high]
        vector = force[:, 2] * flight["vector"][low:high]
        spin = np.gradient(smooth(flight["gyro"][low:high, :2], 5), DT, axis=0)
        block = np.column_stack([force[:, 0], force[:, 1], velocity, vector, spin[:, 1], spin[:, 0]])
        out.append(detrend(smooth(block, samples))[trim:-trim])
    return out


def regress(rows, gain=None):
    """
    Forward: specific force = -k v + gain * thrust vector + lever * pitch acceleration (gain fitted, or fixed when given). Sideways: specific force = -k v + lever * roll acceleration. Returns each axis's k with its standard error, the coefficients and R².
    """
    rows = np.concatenate(rows)
    result = {}
    for axis, (target, columns) in {"forward": (0, [2, 4, 5]), "left": (1, [3, 6])}.items():
        x, y = rows[:, columns], rows[:, target]
        if axis == "forward" and gain is not None:
            x, y = rows[:, [2, 5]], y - gain * rows[:, 4]
        coefficients, *_ = np.linalg.lstsq(x, y, rcond=None)
        residual = y - x @ coefficients
        # Standard error from the residual, inflated for the smoothing's correlation (about one independent sample per eight).
        dof = max(1, len(y) / 8 - x.shape[1])
        covariance = np.linalg.inv(x.T @ x) * residual.var() * len(y) / dof
        result[axis] = {"k": -coefficients[0], "k_se": np.sqrt(covariance[0, 0]), "coefficients": coefficients, "r2": 1 - residual.var() / y.var()}
    return result


def steady(flight):
    """
    Over the airborne span after liftoff: mean forward specific force and its thrust-vector part (g), mean pitch and mean commanded thrust tilt (degrees). In a hover that holds its place the world-horizontal force averages zero, so pitch + tilt shows any tilt the servo model misses, independently of the accelerometer's forward reading.
    """
    span = slice(flight["lift"] + 20, flight["end"] - 5)
    force = flight["force"][span]
    return force[:, 0].mean(), (force[:, 2] * flight["vector"][span]).mean(), flight["pitch"][span].mean() / DEG, np.degrees(np.arctan(flight["vector"][span])).mean()


def drag_velocity(flight, k, offset):
    """
    The drag-model velocity (m/s; forward, left in body axes) every sample: forward specific force less the thrust vector's part and the steady offset (g).
    """
    force = flight["force"]
    return np.column_stack([-G * (force[:, 0] - force[:, 2] * flight["vector"] - offset) / k, -G * force[:, 1] / k])


def camera_velocity(path, flight, cam_pitch):
    """
    Ground truth from the camera's black box (fw/esp32cam/blackbox.py --fetch): each trusted sighting's tag centre (camera frame) levelled by the logged attitude at its capture, differenced into the drone's velocity in body heading axes and smoothed over 0.3 s. The CAM clock is matched to the drone's by the shift (±5 s) that best lines the sightings up with the log's tag-seen flag. Returns (sample index, forward, left) rows.
    """
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "fw/esp32cam"))
    from record import load as load_frames

    frames = load_frames(path)
    keep = frames["trusted"] > 0
    capture, tag = frames["capture_us"][keep].astype(float), frames["tag_position"][keep].astype(float)
    times = flight["time_us"].astype(float)
    seen = flight["seen"]

    def agreement(shift):
        index = np.searchsorted(times, capture + shift)
        inside = (index > 0) & (index < len(times))
        return seen[index[inside]].sum() - (~seen[index[inside]]).sum()

    centre = times[np.flatnonzero(seen)].mean() - capture.mean() if seen.any() else times.mean() - capture.mean()
    shift = max((centre + step for step in np.arange(-5e6, 5e6, 5e3)), key=agreement)
    index = np.clip(np.searchsorted(times, capture + shift), 0, len(times) - 1)
    ca, sa = np.cos(cam_pitch), np.sin(cam_pitch)
    x, y, z = tag[:, 1] * sa + tag[:, 2] * ca, -tag[:, 0], tag[:, 2] * sa - tag[:, 1] * ca
    roll, pitch = flight["roll"][index], flight["pitch"][index]
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    y1, z1 = cr * y - sr * z, sr * y + cr * z
    level = np.column_stack([cp * x + sp * z1, y1])
    # The drone's heading moves the tag's bearing; the gyro's integrated heading takes it out, and the velocity is turned back into the heading at each sample.
    heading = np.cumsum((flight["gyro"][:, 1] * np.sin(flight["roll"]) + flight["gyro"][:, 2] * np.cos(flight["roll"])) / np.cos(flight["pitch"])) * DT
    turn = heading[index]
    world = np.column_stack([np.cos(turn) * level[:, 0] - np.sin(turn) * level[:, 1], np.sin(turn) * level[:, 0] + np.cos(turn) * level[:, 1]])
    rows = []
    span = 0.3e6
    for i, t in enumerate(capture):
        near = np.flatnonzero(np.abs(capture - t) <= span / 2)
        if len(near) < 4:
            continue
        # The tag stands still: its slope in the level frame is the drone's velocity reversed.
        slope = np.polyfit((capture[near] - t) / 1e6, world[near], 1)[0]
        velocity = -slope
        c, s = np.cos(turn[i]), np.sin(turn[i])
        rows.append((index[i], c * velocity[0] + s * velocity[1], -s * velocity[0] + c * velocity[1]))
    return np.array(rows), shift


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--min-seconds", type=float, default=2.5)
    parser.add_argument("--blackbox", help="a camera black-box file covering one recording named in files")
    args = parser.parse_args()
    flights = [flight for flight in (load(path) for path in paths(args.files)) if flight and flight["end"] - flight["lift"] >= args.min_seconds / DT]
    print(f"{len(flights)} flights of at least {args.min_seconds} s")
    # The thrust vector and the IMU's lever arm both follow the servo (pitch acceleration is the vector's torque), so a free fit trades them against each other at short smoothing; at long smoothing the free gain approaches 1, which the steady check below supports.
    for length, samples in ((150, 30), (150, 50), (300, 30), (300, 50)):
        rows = [block for flight in flights for block in blocks(flight, length=length, step=length // 2, samples=samples, trim=samples // 2 + 5)]
        free, fixed = regress(rows), regress(rows, gain=1.0)
        print(f"windows {length / 100:.1f} s, smoothing {samples * 10} ms: gain 1: k {fixed['forward']['k']:.2f} ± {fixed['forward']['k_se']:.2f} 1/s, R² {fixed['forward']['r2']:.2f} | free: k {free['forward']['k']:.2f} ± {free['forward']['k_se']:.2f}, gain {free['forward']['coefficients'][1]:.2f}, R² {free['forward']['r2']:.2f} | left: k {fixed['left']['k']:.2f} ± {fixed['left']['k_se']:.2f}, R² {fixed['left']['r2']:.2f}")
    rows = np.array([steady(flight) + (flight["end"] - flight["lift"],) for flight in flights])
    weight = np.sqrt(rows[:, 4])[:, None]
    (slope, intercept), *_ = np.linalg.lstsq(np.column_stack([rows[:, 1], np.ones(len(rows))]) * weight, rows[:, 0] * weight[:, 0], rcond=None)
    residual = rows[:, 0] - rows[:, 1]
    offset = np.average(residual, weights=rows[:, 4])
    missing = -np.average(rows[:, 2] + rows[:, 3], weights=rows[:, 4])
    print(f"steady: forward force = {slope:.2f} x vector + {intercept:+.3f} g across flights; with gain 1 the offset is {offset:+.3f} ± {residual.std():.3f} g ({np.degrees(np.arcsin(offset)):.1f}° of extra forward thrust tilt), and pitch + commanded tilt averages {-missing:+.1f}° (thrust {missing:.1f}° further forward than modelled would zero it)")
    k = regress([block for flight in flights for block in blocks(flight, length=150, step=75, samples=50, trim=30)], gain=1.0)["forward"]["k"]
    for flight, row in zip(flights, rows):
        velocity = drag_velocity(flight, k, offset)[flight["lift"] + 20:flight["end"] - 5].mean(0)
        span = slice(flight["lift"], flight["end"])
        print(f"{flight['name']} {(flight['end'] - flight['lift']) / 100:4.1f} s, tag seen {flight['seen'][span].mean():.2f}: offset {row[0] - row[1]:+.3f} g, pitch + tilt {row[2] + row[3]:+.1f}°, mean drag-model velocity (k {k:.2f}) forward {velocity[0]:+.2f} left {velocity[1]:+.2f} m/s")
    if args.blackbox:
        for flight in flights:
            truth, shift = camera_velocity(args.blackbox, flight, flight["setting"].get("hover_cam_pitch", 0.0) * DEG)
            if len(truth) < 20:
                continue
            index = truth[:, 0].astype(int)
            force = G * (flight["force"][index, 0] - flight["force"][index, 2] * flight["vector"][index])
            (slope, intercept), *_ = np.linalg.lstsq(np.column_stack([truth[:, 1], np.ones(len(truth))]), force, rcond=None)
            model = drag_velocity(flight, k, offset)[index]
            print(f"{flight['name']} camera ({len(truth)} sightings, CAM clock shift {shift / 1e6:+.3f} s): forward k {-slope:.2f} 1/s, offset {intercept / G:+.3f} g; drag-model velocity error forward {np.mean(model[:, 0] - truth[:, 1]):+.2f} ± {np.std(model[:, 0] - truth[:, 1]):.2f}, left {np.mean(model[:, 1] - truth[:, 2]):+.2f} ± {np.std(model[:, 1] - truth[:, 2]):.2f} m/s")


if __name__ == "__main__":
    main()
