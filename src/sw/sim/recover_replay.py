"""
The firmware's recovery trigger (fw/drone/main/recover.c recover_tick) on a recording's own gyro, accelerometer, attitude and flip phases: uv run -m sw.sim.recover_replay [2026-11-12T02-27-04] [--tip-limit 45]

Open loop: each tick takes the recorded values, as flight_rt.c would have handed them to recover_tick with recover_enable 1, and the recovery's commands are printed, not flown, so everything after the first trigger is what the drone did without it. It shows when and why a recovery would have started, the height then, how the righting's attitude (its quaternion on the gyro alone from the last ordinary tick) compares with the recorded Euler estimate, and the first commands. The flip's exemption and gyro-only window are rebuilt from the recorded phase (flip.h: exempt while spinning or righting and FLIP_GRACE after the hand-back, gyro-only from the climb to FLIP_GRACE after the hand-back).
"""

import argparse
import ctypes
import math

import numpy as np

from sw.flight_log import read_flight, recording, settings as recording_settings
from sw.sim import flip_firmware as F
from fw.controller.log_format import RECOVERY_CAUSES

FLIP_GRACE = 1.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", nargs="?", default="2026-11-12T02-27-04")
    parser.add_argument("--tip-limit", type=float, help="degrees (default: the recording's setting)")
    parser.add_argument("--show", type=float, default=0.3, help="seconds of commands to print after the trigger")
    args = parser.parse_args()
    path = recording(args.name)
    settings = recording_settings(path)
    data = read_flight(path)[0]
    tip_limit = math.radians(args.tip_limit or settings.get("tip_limit", 45.0))
    model = F.flight.MODEL
    mass = settings.get("hover_mass", 0.9)
    authority = F.Authority(model["arm"] / model["roll_inertia"], model["pivot_height"] / model["pitch_inertia"], model["arm"] / model["yaw_inertia"], mass * 9.81)
    run = F.RecoverRun()
    F.library.recover_reset(ctypes.byref(run))
    controller = (ctypes.c_float * 18)()
    integral = ctypes.c_float(0.0)
    time = (data.esp_time_us - data.esp_time_us.iloc[0]) / 1e6
    armed = data["flags"].astype(int) & 1
    handed_back = -math.inf
    previous_phase = 0
    started = None
    shown = 0
    for k in range(1, len(data)):
        row = data.iloc[k]
        if not armed.iloc[k]:
            continue
        now = time.iloc[k]
        phase = int(row.flip) if not math.isnan(row.flip) else 0
        if phase == 3 and previous_phase != 3:
            handed_back = now
        previous_phase = phase
        grace = 0 <= now - handed_back < FLIP_GRACE
        exempt = phase == 2 or grace
        roll, pitch = math.radians(row.roll_deg), math.radians(row.pitch_deg)
        gyro = (ctypes.c_float * 3)(*np.radians([row.gx_deg_s, row.gy_deg_s, row.gz_deg_s]))
        accel = (ctypes.c_float * 3)(row.ax_g, row.ay_g, row.az_g)
        attitude = (ctypes.c_float * 2)(roll, pitch)
        given = F.RecoverInput(enabled=True, tipped=not exempt and max(abs(roll), abs(pitch)) > tip_limit, allowed=True, gyro=gyro, accel=accel, dt=float(time.iloc[k] - time.iloc[k - 1]), tip_limit=tip_limit,
                               flip_active=phase > 0, flip_gyro_only=phase > 0 or grace, flip_phase=phase, climb_rate=float(row.hover_climb_m_s), collective=2 * F.flight.thrust(float(row.base_throttle)), authority=authority)
        out = F.RecoverOutput()
        F.library.recover_tick(ctypes.byref(run), ctypes.byref(F.RECOVER_DEFAULTS), ctypes.byref(given), ctypes.addressof(attitude), ctypes.addressof(controller), ctypes.byref(integral), ctypes.byref(out))
        q = run.q
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2)))))
        if started is None and run.stage:
            started = now
            print(f"{args.name}: recovery would start at {now:.2f} s ({RECOVERY_CAUSES[run.cause]}), flip phase {phase}, hold height {row.hover_height_m:.2f} m, climb {row.hover_climb_m_s:.2f} m/s; "
                  f"recorded roll {row.roll_deg:.0f} pitch {row.pitch_deg:.0f}, the righting's tilt {tilt:.0f} deg; rates {row.gx_deg_s:.0f} {row.gy_deg_s:.0f} {row.gz_deg_s:.0f} deg/s")
            print("   t  stage  tilt  q roll  q pitch  rec roll  rec pitch  left N  right N  left tilt  right tilt  collective N  level  cut")
        if started is not None and now - started <= args.show and shown < 40:
            shown += 1
            m = out.motors
            print(f"{now:6.2f}  {run.stage}  {tilt:5.0f}  {math.degrees(attitude[0]):6.0f}  {math.degrees(attitude[1]):7.0f}  {row.roll_deg:8.0f}  {row.pitch_deg:9.0f}  {m.left:6.2f}  {m.right:7.2f}  {math.degrees(m.left_tilt):9.1f}  {math.degrees(m.right_tilt):10.1f}  {m.collective:12.2f}  {out.level:5d}  {out.cut}")
        if out.cut:
            print(f"cut ({out.cut}) at {now:.2f} s")
            break
    if started is None:
        print(f"{args.name}: no recovery would have started")


if __name__ == "__main__":
    main()
