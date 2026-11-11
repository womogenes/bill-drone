"""
Replay a recording's tag hold through the firmware's hover.c, open loop: uv run -m sw.sim.hold_replay <recording prefix> [...] [--set name=value,...] [--define NAME=value ...]

Each tick the hold gets what flight_rt.c gave it, rebuilt from the log: the attitude, the heading rate from the gyro, the vertical specific force, the drag model's inputs (the calibrated accelerometer turned by pose_zero and the commanded servo tilt weighted by each motor's thrust), and each new sighting (the logged tag position turned back into the camera frame by the attitude at its capture). The hold's own outputs are not flown (the drone flew what it flew), so this shows what the estimators learn from the real flight, including the terms the log does not carry (push, trim, thrust_tilt, drag_offset, drag_bias), and what other settings or another hover.c (HOVER_SOURCE, HOVER_DEFINES) would have asked for on the same data.
Prints how closely the replay reproduces the logged velocity, carry and pitch target, and the steady terms over the tracked stretch.
"""

import argparse
import math

import numpy as np

from sw.flight_log import metadata, parse_settings, read_flight, recording, settings
from sw.sim import hover

DEG = math.pi / 180


def camera_vector(gains, roll, pitch, yaw, out):
    """hover.c level() undone: a level-frame vector turned by yaw back into the camera's (right, down, ahead) at this attitude."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    x2, y1 = cy * out[0] + sy * out[1], -sy * out[0] + cy * out[1]
    z2 = out[2]
    cp, sp = math.cos(pitch), math.sin(pitch)
    x, z1 = cp * x2 - sp * z2, sp * x2 + cp * z2
    cr, sr = math.cos(roll), math.sin(roll)
    y, z = cr * y1 + sr * z1, -sr * y1 + cr * z1
    ca, sa = math.cos(gains.cam_pitch), math.sin(gains.cam_pitch)
    down, ahead = x * sa - z * ca, x * ca + z * sa
    right = -y
    cc, sc = math.cos(gains.cam_roll), math.sin(gains.cam_roll)
    return [cc * right + sc * down, -sc * right + cc * down, ahead]


def replay(path, overrides=None, tilt_offset=0.0):
    """
    The hold replayed on a recording: a dict of per-tick arrays (time, the replay's and the log's velocity, carry and pitch target, and the unlogged terms) over the hold's ticks. overrides are settings flown instead of the recording's; tilt_offset (degrees) is added to the commanded servo tilt the hold is told, as if the servo trims were moved by it.
    """
    data, _ = read_flight(path)
    meta = metadata(path)
    s = settings(path) | (overrides or {})
    table = np.array(meta["thrust_table"])
    gains = hover.gains(s)
    run = hover.Hover(s)
    accel_offset = np.array([s["accel_offset_x"], s["accel_offset_y"], s["accel_offset_z"]])
    accel_gain = np.array([s["accel_gain_x"], s["accel_gain_y"], s["accel_gain_z"]])
    zero_roll, zero_pitch = math.radians(s["pose_zero_roll"]), math.radians(s["pose_zero_pitch"])
    times = data.esp_time_us.to_numpy().astype(np.int64)
    accel = (data[["ax_g", "ay_g", "az_g"]].to_numpy() - accel_offset) * accel_gain
    gyro = np.radians(data[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].to_numpy())
    roll, pitch = np.radians(data.roll_deg.to_numpy()), np.radians(data.pitch_deg.to_numpy())
    yaw_logged = np.radians(data.hover_yaw_deg.to_numpy())
    mode = (data["flags"].to_numpy() >> 1) & 3
    left_thrust = np.interp(data.left_throttle, table[:, 0], table[:, 1])
    right_thrust = np.interp(data.right_throttle, table[:, 0], table[:, 1])
    left_tilt = np.radians(data.left_servo_deg - s["left_trim"] + tilt_offset)
    right_tilt = -np.radians(data.right_servo_deg - s["right_trim"] - tilt_offset)
    forward = left_thrust * np.sin(left_tilt) + right_thrust * np.sin(right_tilt)
    up = left_thrust * np.cos(left_tilt) + right_thrust * np.cos(right_tilt)
    thrust_tilt = np.where(up > 0, np.arctan2(forward, up), 0)
    tag = data[["hover_x_m", "hover_y_m", "hover_z_m"]].to_numpy()
    age = data.hover_tag_age_ms.to_numpy()
    range_target = data.hover_range_target_m.to_numpy()
    names = ("t", "vx", "vx_log", "carry", "carry_log", "pitch", "pitch_log", "push", "trim", "thrust_tilt", "drag_offset", "drag_bias", "drag_vx", "drag_vx_log", "tracking", "range", "setpoint", "height")
    out = {name: [] for name in names}
    sequence, last_capture = 0, None
    previous = None
    for i in range(len(data)):
        dt = 0.01 if previous is None else max((times[i] - previous) / 1e6, 1e-4)
        previous = times[i]
        # The attitude estimate's own (absolute) pitch and roll for the vertical, as flight_rt.c turns the accelerometer.
        ar, ap = roll[i] + zero_roll, pitch[i] + zero_pitch
        a = accel[i]
        vertical = -a[0] * math.sin(ap) + math.cos(ap) * (a[1] * math.sin(ar) + a[2] * math.cos(ar))
        if mode[i] == 0:
            run.rest(vertical, data.base_throttle.iloc[i] > 0, dt)
            continue
        y = a[1] * math.cos(zero_roll) - a[2] * math.sin(zero_roll)
        z = a[1] * math.sin(zero_roll) + a[2] * math.cos(zero_roll)
        specific = (a[0] * math.cos(zero_pitch) + z * math.sin(zero_pitch), y, -a[0] * math.sin(zero_pitch) + z * math.cos(zero_pitch))
        heading_rate = (gyro[i, 1] * math.sin(roll[i]) + gyro[i, 2] * math.cos(roll[i])) / math.cos(pitch[i])
        pose = None
        if not math.isnan(age[i]):
            capture = int(times[i] - age[i] * 1000)
            if last_capture is None or abs(capture - last_capture) > 3000:
                last_capture = capture
                sequence += 1
                k = max(0, int(np.searchsorted(times, capture)) - 1)
                vector = camera_vector(gains, roll[k], pitch[k], yaw_logged[k], tag[i])
                pose = hover.Pose(sequence=sequence, capture_us=capture, tag=tuple(vector))
        # The pilot's hold-point nudge toward the tag, from the logged range target (the setpoint less the nudge).
        if run.state.have_setpoint and not math.isnan(range_target[i]):
            run.gains.nudge_forward = run.state.setpoint - range_target[i]
        output = run.update(mode[i], pose, int(times[i]), dt, roll[i], pitch[i], heading_rate, vertical, run.state.force, drag=(specific, thrust_tilt[i]))
        st = run.state
        cy, sy = math.cos(st.yaw), math.sin(st.yaw)
        values = (
            (times[i] - times[0]) / 1e6, st.velocity[0], data.hover_vx_m_s.iloc[i], st.carry[0], data.hover_trim_x_m_s2.iloc[i], math.degrees(output.pitch), data.target_pitch_deg.iloc[i],
            st.push[0] * cy + st.push[1] * sy, st.trim[0], math.degrees(st.thrust_tilt), st.drag_offset[0], st.drag_bias[0], st.drag_velocity[0], data.hover_drag_vx_m_s.iloc[i], st.tracking,
            math.hypot(tag[i, 0], tag[i, 1]), range_target[i], st.height,
        )
        for name, value in zip(names, values):
            out[name].append(value)
    return {name: np.array(value, dtype=float) for name, value in out.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", help="recording name prefixes (UTC time), e.g. 2026-11-11T23-35")
    parser.add_argument("--set", default="", help="settings flown instead, name=value,...")
    parser.add_argument("--tilt-offset", type=float, default=0.0, help="degrees added to the commanded servo tilt the hold is told")
    args = parser.parse_args()
    for name in args.recordings:
        r = replay(recording(name), parse_settings(args.set), args.tilt_offset)
        tracked = (r["tracking"] > 0) & (r["height"] > 0.15)
        if not tracked.any():
            print(name, "never tracked")
            continue

        def rms(a, b):
            return float(np.sqrt(np.nanmean((r[a][tracked] - r[b][tracked]) ** 2)))

        last = np.flatnonzero(tracked)[-1]
        print(f"{name}: {tracked.sum() / 100:.1f} s tracked; replay against log RMS: vx {rms('vx', 'vx_log'):.3f} m/s, carry {rms('carry', 'carry_log'):.2f} m/s², pitch target {rms('pitch', 'pitch_log'):.2f} deg")
        print(f"  at the end: push {r['push'][last]:+.2f} trim {r['trim'][last]:+.2f} carry {r['carry'][last]:+.2f} (log {r['carry_log'][last]:+.2f}) m/s², thrust_tilt {r['thrust_tilt'][last]:+.2f} deg, drag_offset {r['drag_offset'][last]:+.2f} m/s, drag_bias {r['drag_bias'][last]:+.2f} m/s²")
        print(f"  mean: push {np.mean(r['push'][tracked]):+.2f} trim {np.mean(r['trim'][tracked]):+.2f} pitch target {np.mean(r['pitch'][tracked]):+.2f} (log {np.nanmean(r['pitch_log'][tracked]):+.2f}) deg")


if __name__ == "__main__":
    main()
