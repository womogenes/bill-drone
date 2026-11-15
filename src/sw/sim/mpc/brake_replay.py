"""
The 2026-11-15 07:03 UTC burst60's brake replayed open loop through flip.c's greedy allocation and the MPC: python -m sw.sim.mpc.brake_replay [--mpc '{}'] [--start 12.30] [--end 12.62] [--plot out.png] (from src; needs make_mpc.py's ffmpc.py).

The wanted angular accelerations and collective are read back from the flown commands by inverting flip_allocate (roll from the thrust difference, pitch from the common tilt at its lagged thrust, yaw from the differential), so the greedy allocation of them is what flew. From --start the MPC (mpc_hook.c's flip_allocate, mode 3 unless set) allocates the same wanted sequence instead, its actuator estimate warmed on what flew before. Both command sequences then drive one actuator plant as the burst fits it (the servos 100 ms late on the left and 70 ms on the right, then indi.c's horn and slew; the motors one tick late and 66 ms behind; the tilt's pitch authority twice FLIP_PITCH_AUTHORITY; the assemblies' swing reaction 0.27), and the body pitch and roll acceleration each gives are integrated into rates. Open loop: the wanted sequence is the one flown, so after the brake it carries the controller's reaction to the flown pitch error; the comparison is meaningful up to about 12.60 s.
"""

import argparse
import ctypes
import json
import math

import numpy as np

from sw.flight_log import read_flight, recording, settings as recording_settings
from sw.sim import flight
from sw.sim.mpc import ffmpc as FF

RECORDING = "2026-11-15T07-02-48"
DT = 0.01
# flip.c's authority and its allocation's terms.
ROLL, PITCH, YAW = 13.5, 7.08, 4.66
FLIP_MOTOR_TAU = 0.1
# The plant as the burst fits it.
DELAY = (10, 7)
WN, ZETA, SLEW, STEPS = 37.5, 0.87, 4.97, 4
MOTOR_TAU = 0.066
PITCH_GAIN = 2.0
SWING = 0.27


class Righting(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("kp", "brake", "max_rate", "k_roll", "k_pitch", "kd_yaw", "roll_over", "servo_max", "floor", "ceiling", "gyro_ff", "yaw_ff")]


def flown():
    """The burst's commands per tick: time (s), each motor's thrust (N) and each assembly's physical forward tilt (radians), and the gyro (deg/s)."""
    df = read_flight(recording(RECORDING))[0].reset_index(drop=True)
    s = recording_settings(recording(RECORDING))
    t = ((df.esp_time_us - df.esp_time_us.iloc[0]) / 1e6).to_numpy()
    ratio = np.clip(df.battery_v.to_numpy() / s["thrust_voltage"], 0.85, 1.15)
    scale = np.column_stack([s["left_thrust_scale"] * ratio ** 2, s["right_thrust_scale"] * ratio ** 2])
    thrust = np.column_stack([[flight.thrust(v) for v in df.left_throttle], [flight.thrust(v) for v in df.right_throttle]]) * scale
    directions = (flight.MODEL["left_servo_direction"], flight.MODEL["right_servo_direction"])
    tilt = np.radians(np.column_stack([directions[0] * (df.left_servo_deg - s["left_trim"]), directions[1] * (df.right_servo_deg - s["right_trim"])]))
    gyro = df[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].to_numpy()
    return t, thrust, tilt, gyro, s


def wanted(thrust, tilt):
    """flip_allocate inverted: per tick the roll, pitch and yaw wanted (rad/s²), the collective (N) and the lagged thrust it allocated against."""
    keep = math.exp(-DT / FLIP_MOTOR_TAU)
    have = thrust[0].sum()
    out = []
    for (left, right), (tl, tr) in zip(thrust, tilt):
        total = left + right
        have += (total - have) * (1 - keep)
        common, differential = (tl + tr) / 2, (tr - tl) / 2
        h = max(have, 0.5)
        out.append((ROLL * (left - right), PITCH * h * math.sin(common), YAW * h * math.cos(common) * differential, total, have))
    return np.array(out)


def plant(thrust, tilt):
    """The body's pitch and roll acceleration (rad/s²) each tick from these commands through the burst's actuators."""
    horn, rate, motor = tilt[0].copy(), np.zeros(2), thrust[0].copy()
    pitch, roll = [], []
    h = DT / STEPS
    for k in range(len(thrust)):
        before = rate.copy()
        for side in range(2):
            goal = tilt[max(k - DELAY[side], 0), side]
            for _ in range(STEPS):
                rate[side] = np.clip(rate[side] + (WN * WN * (goal - horn[side]) - 2 * ZETA * WN * rate[side]) * h, -SLEW, SLEW)
                horn[side] += rate[side] * h
        motor += (thrust[max(k - 1, 0)] - motor) * (1 - math.exp(-DT / MOTOR_TAU))
        pitch.append(PITCH_GAIN * PITCH * (motor * np.sin(horn)).sum() - SWING * (rate - before).sum() / DT)
        roll.append(ROLL * (motor[0] * math.cos(horn[0]) - motor[1] * math.cos(horn[1])))
    return np.array(pitch), np.array(roll)


def replay(t, thrust, tilt, want, s, start, config):
    """The MPC's commands for the wanted sequence from start on (what flew before it)."""
    library = FF.library
    config = dict(config)
    library.mpc_hook_init(int(config.pop("mode", 3)))
    for key, value in config.items():
        if isinstance(value, (list, tuple)):
            getattr(FF.MPC_CONFIG, key)[:] = value
        else:
            setattr(FF.MPC_CONFIG, key, value)
    weight = s["hover_mass"] * 9.81
    defaults = FF.DEFAULTS
    righting = Righting(defaults.kp, defaults.brake, defaults.max_rate, defaults.k_roll, defaults.k_pitch, defaults.kd_yaw, defaults.roll_over, defaults.servo_max, defaults.floor, defaults.ceiling, 0.0, 0.0)
    authority = FF.Authority(ROLL, PITCH, YAW, weight)
    cap = flight.thrust(defaults.throttle_cap) * s["left_thrust_scale"]
    library.mpc_hook_select(0, cap, None)
    library.flip_allocate.argtypes = [ctypes.POINTER(Righting), ctypes.POINTER(FF.Authority)] + [ctypes.c_float] * 5 + [ctypes.POINTER(ctypes.c_float), ctypes.c_void_p, ctypes.c_float, ctypes.POINTER(FF.Output)]
    out_thrust, out_tilt = thrust.copy(), tilt.copy()
    have = ctypes.c_float(want[0, 4])
    for k in range(len(t)):
        if t[k] >= start:
            out = FF.Output()
            library.flip_allocate(ctypes.byref(righting), ctypes.byref(authority), *want[k, :3], want[k, 3], 0.0, ctypes.byref(have), None, DT, ctypes.byref(out))
            out_thrust[k] = (out.left, out.right)
            out_tilt[k] = (out.left_tilt, out.right_tilt)
        else:
            have.value = want[k, 4]
        library.mpc_hook_sent(0, (ctypes.c_float * 2)(*out_tilt[k]), (ctypes.c_float * 2)(*out_thrust[k]), DT)
    return out_thrust, out_tilt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mpc", default="{}")
    parser.add_argument("--start", type=float, default=12.30)
    parser.add_argument("--end", type=float, default=12.62)
    parser.add_argument("--plot")
    args = parser.parse_args()
    t, thrust, tilt, gyro, s = flown()
    window = (t >= args.start - 0.5) & (t <= args.end + 0.2)
    t, thrust, tilt, gyro = t[window], thrust[window], tilt[window], gyro[window]
    want = wanted(thrust, tilt)
    mpc_thrust, mpc_tilt = replay(t, thrust, tilt, want, s, args.start, json.loads(args.mpc))
    greedy_pitch, greedy_roll = plant(thrust, tilt)
    mpc_pitch, mpc_roll = plant(mpc_thrust, mpc_tilt)
    inside = (t >= args.start) & (t <= args.end)
    print(" t     | flown: L R N, tilt l r deg, pitch accel | MPC: L R N, tilt l r deg, pitch accel | wanted roll pitch | gyro pitch")
    for k in np.flatnonzero(inside):
        print(f"{t[k]:6.2f} | {thrust[k, 0]:4.1f} {thrust[k, 1]:4.1f} {np.degrees(tilt[k, 0]):6.1f} {np.degrees(tilt[k, 1]):6.1f} {greedy_pitch[k]:6.0f} | {mpc_thrust[k, 0]:4.1f} {mpc_thrust[k, 1]:4.1f} {np.degrees(mpc_tilt[k, 0]):6.1f} {np.degrees(mpc_tilt[k, 1]):6.1f} {mpc_pitch[k]:6.0f} | {want[k, 0]:5.0f} {want[k, 1]:5.0f} | {gyro[k, 1]:5.0f}")
    gp, mp = np.degrees(np.cumsum(greedy_pitch[inside]) * DT), np.degrees(np.cumsum(mpc_pitch[inside]) * DT)
    gr, mr = np.degrees(np.cumsum(greedy_roll[inside]) * DT), np.degrees(np.cumsum(mpc_roll[inside]) * DT)
    print(f"pitch rate change {args.start:.2f}-{args.end:.2f} s from the allocation (deg/s): greedy {gp[-1]:.0f} (most nose-up {gp.min():.0f}), MPC {mp[-1]:.0f} (most nose-up {mp.min():.0f}); roll rate change: greedy {gr[-1]:.0f}, MPC {mr[-1]:.0f}; flown gyro pitch {gyro[inside][0, 1]:.0f} to {gyro[inside][-1, 1]:.0f}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        tt = t[inside]
        fig, axes = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
        axes[0].plot(tt, thrust[inside], label=["flown left", "flown right"])
        axes[0].plot(tt, mpc_thrust[inside], "--", label=["MPC left", "MPC right"])
        axes[0].set_ylabel("thrust asked (N)")
        axes[1].plot(tt, np.degrees(tilt[inside]), label=["flown left", "flown right"])
        axes[1].plot(tt, np.degrees(mpc_tilt[inside]), "--", label=["MPC left", "MPC right"])
        axes[1].set_ylabel("tilt asked (deg)")
        axes[2].plot(tt, gp, label="greedy (flown)")
        axes[2].plot(tt, mp, label="MPC")
        axes[2].set_ylabel("pitch rate from the\nallocation (deg/s)")
        axes[3].plot(tt, gr, label="greedy (flown)")
        axes[3].plot(tt, mr, label="MPC")
        axes[3].set_ylabel("roll rate from the\nallocation (deg/s)")
        axes[3].set_xlabel(f"s into {RECORDING}")
        for ax in axes:
            ax.legend(fontsize=7)
            ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)


if __name__ == "__main__":
    main()
