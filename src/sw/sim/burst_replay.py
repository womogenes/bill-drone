"""
The bursts' plant (models/airframe_2026-11-15_bursts.json) from the logs: each window's throttles and servo angles drive the full-attitude plant (sw/sim/rigid.py) open loop on the 2026-11-14 thrust stand's table (sw/sim/flip_esc.py STAND) at the window's own hover trim, the body's rates and attitude set back to the log's every --every s (multiple shooting), and the named airframe terms fitted by least squares to the rate errors on --axes: uv run -m sw.sim.burst_replay [--fit tilt_authority,servo_delay,...] [--airframe airframe_2026-11-12_raised.json] [--set k=v,...] [--every 0.05] [--axes 0,1,2] [WINDOW ...]; --bins NAME:START:END ... prints the tilt's pitch authority by how far the swings go (the hovers' check).

A window is RECORDING:START:END (s from the recording's first sample), with :bands for a flight with the rubber bands on (their preload and the -2.5 degree tilt offset). The default windows are the three bursts: 2026-11-15 05:29 UTC burst90 (bands on, the Euler flip), 07:03 burst60 and 07:13 burst30 (bands off, the quaternion bursts). The plant is the airframe with the bands off unless a window says otherwise (left_band and right_band 0).

What it found (models/NOTES.md "The bursts' plant"): on the raised pivots' hover fit the three bursts' pitch rate errors were 32, 71 and 53 deg/s RMS; with the tilt's pitch authority 2.3 times the geometry's, the servos 40 ms late, the assemblies' swing reaction at about the CAD's and yaw's tilt authority 0.74 (it was 0.30) they are 20, 16 and 25. --bins on the night's hovers: the tilt's pitch authority fits 0.9 of the model's over spans whose swings stay under 2 degrees, 2.3-2.9 over those past 4: the hover fits' 1 was the small swings' (diluted by their own error), not the drone's.
"""

import argparse
import math

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.optimize import least_squares
from scipy.signal import butter, filtfilt

from sw.flight_log import parse_settings, read_flight, recording
from sw.sim import airframe as A
from sw.sim import flip_replay as F
from sw.sim import rigid as R
from sw.sim.flip_esc import STAND

BURSTS = ["2026-11-15T05-29-38:17.4:19.6:bands", "2026-11-15T07-02-48:12.2:12.9", "2026-11-15T07-13-32:13.2:15.2"]
# The bands' preload as the raised pivots' fit has it (N·m) and their tilt offset (degrees).
BANDS = {"left_band": 0.07189, "right_band": 0.003407}
BAND_TILT = -2.5
# Bounds of what --fit may move: (low, high).
BOUNDS = {"tilt_authority": (0.3, 5.0), "servo_delay": (0.0, 0.12), "servo_inertia": (0.0, 0.003), "servo_wn": (8.0, 150.0), "servo_slew": (100.0, 3000.0), "rotor_momentum": (0.0, 0.08), "wash": (-0.5, 1.2), "prop_torque": (-0.04, 0.04),
          "yaw_from_roll": (-0.05, 0.05), "inflow_damping": (0.0, 2.0), "differential_slope": (0.2, 1.5), "link_c": (0.0, 0.3), "climb_inflow": (0.0, 0.3)}
# Terms fitted as one for both sides.
PAIRED = {"servo_delay": "right_servo_delay", "servo_wn": "right_servo_wn", "motor_tau": "right_motor_tau"}


class Window(F.Log):
    """flip_replay.Log over a window, with the hover throttles before it (for the trim on the stand's table) and whether the bands were on."""

    def __init__(self, spec):
        name, start, end, *rest = spec.split(":")
        super().__init__(name, float(start), float(end))
        self.spec, self.bands = spec, "bands" in rest
        df = read_flight(recording(name))[0]
        time = (df["esp_time_us"] - df["esp_time_us"].iloc[0]).to_numpy() / 1e6
        hover = (df["flip"].fillna(0) == 0) & (df["left_throttle"] > 300) & (df["right_throttle"] > 300) & (time < float(start))
        self.hover_throttle = df.loc[hover, ["left_throttle", "right_throttle"]].to_numpy(float)
        self.usable = np.abs(self.gyro).max(1) < 0.97 * self.gyro_range


def plant(window, airframe, overrides):
    """The rigid.Sim plant for a window: the airframe with the overrides (the bands off unless the window has them), the stand's table at the window's hover trim, no disturbances."""
    params = A.model(airframe) | {"left_band": 0.0, "right_band": 0.0} | (BANDS if window.bands else {}) | overrides
    for key, other in PAIRED.items():
        if key in overrides:
            params[other] = overrides[key]
    geometry = A.geometry_of(params)
    hover = np.column_stack([np.interp(window.hover_throttle[:, s], STAND[:, 0], STAND[:, 1]) for s in range(2)]) * np.array([1.0, params.get("right_thrust_gain", 1.0)])
    level, mean = A.trim_for(hover, geometry)
    params |= {"thrust_level": float(level), "hover_thrust": [float(m) for m in mean]}
    return {"airframe": params, "geometry": geometry, "disturbances": [], "rotor_drag": (0.07, 0.2), "tilt_offset": BAND_TILT if window.bands else 0.0, "table": STAND}


def replay(window, plant, every):
    """Each tick's mean body rates (rad/s) the plant predicts from the window's commands, its rates and attitude set back to the log's every `every` s."""
    sim = R.Sim(1, plant, seed=0)
    sim.gyro_bias[:] = 0.0
    tilt = window.tilt()
    p = sim.p
    gain = np.array([[1.0, p["right_thrust_gain"]]])
    start = A.actual_thrust(p, A.table_thrust(p, window.throttle[:1], np.ones(1), table=sim.table, gain=gain), sim.trim)
    sim.reset(np.array([[0, 0, 10.0]]), np.array([[0, 0, window.climb[0]]]), F.quaternion(*window.attitude[0])[None], window.gyro[:1], motor=start, tilt=tilt[:1] - sim.tilt_offset)
    sim.throttles[:] = window.throttle[0]
    sim.bind()
    stride = max(1, round(every / A.DT))
    rates = np.zeros((len(window.time), 3))
    rates[0] = window.gyro[0]
    for k in range(1, len(window.time)):
        if k % stride == 0:
            sim.omega[:] = window.gyro[k - 1]
            sim.quat[:] = F.quaternion(*window.attitude[k - 1])
        sim.step(window.throttle[k][None], tilt[k][None])
        rates[k] = sim.turned[0]
    return rates


def errors(windows, airframe, overrides, every, axes):
    """Each window's rate errors (deg/s) on the axes where the gyro did not clip."""
    return [np.degrees(replay(w, plant(w, airframe, overrides), every)[w.usable][:, axes] - w.gyro[w.usable][:, axes]).ravel() for w in windows]


def fit(windows, airframe, fixed, names, every, axes):
    """Least squares on all windows at once, each weighted by its length; returns the fitted terms and each window's RMS before and after."""
    low, high = np.array([BOUNDS[k][0] for k in names]), np.array([BOUNDS[k][1] for k in names])
    start = A.complete(A.model(airframe)) | fixed
    x0 = np.clip([start[k] for k in names], low + 1e-9, high - 1e-9)
    residuals = lambda x: np.concatenate([e / math.sqrt(len(e)) for e in errors(windows, airframe, fixed | dict(zip(names, map(float, x))), every, axes)])
    rms = lambda x: [float(np.sqrt(np.mean(e ** 2))) for e in errors(windows, airframe, fixed | dict(zip(names, map(float, x))), every, axes)]
    result = least_squares(residuals, x0, bounds=(low, high), diff_step=1e-2, x_scale="jac", max_nfev=300)
    return dict(zip(names, map(float, result.x))), rms(x0), rms(result.x)


def bins(windows, airframe, overrides, edges=(0, 2, 4, 7, 12)):
    """The tilt's pitch authority over the model's by how far the swings go: one-step pitch accelerations (the rates set back every tick) less what the plant gives with the tilt's pitch torque removed, regressed on that torque, in bins of the common tilt command's swing (RMS over 0.3 s about its 1 s mean, degrees)."""
    lowpass = butter(2, 10 / 50)
    rows = []
    for w in windows:
        once = {g: replay(w, plant(w, airframe, overrides | {"tilt_authority": g * A.complete(A.model(airframe) | overrides)["tilt_authority"]}), A.DT) for g in (1.0, 2.0)}
        accel = lambda rates: filtfilt(*lowpass, np.vstack([np.zeros(3), (rates[1:] - w.gyro[:-1]) / A.DT]), axis=0)[:, 1]
        measured = filtfilt(*lowpass, np.vstack([np.zeros(3), np.diff(w.gyro, axis=0) / A.DT]), axis=0)[:, 1]
        tilt_term = accel(once[2.0]) - accel(once[1.0])
        common = w.tilt().mean(1)
        departure = common - uniform_filter1d(common, 100)
        swing = np.degrees(np.sqrt(uniform_filter1d(departure ** 2, 30)))
        keep = w.usable & (w.time >= w.time[0] + 0.3)
        rows.append(np.column_stack([swing, measured - (accel(once[1.0]) - tilt_term), tilt_term])[keep])
    rows = np.concatenate(rows)
    for low, high in zip(edges, edges[1:] + (math.inf,)):
        chosen = (rows[:, 0] >= low) & (rows[:, 0] < high)
        if chosen.sum() < 30:
            continue
        x, y = rows[chosen, 2], rows[chosen, 1]
        print(f"swings {low}-{high} deg: {chosen.sum()} ticks, the tilt's pitch authority {(x @ y) / (x @ x):.2f} of the model's (correlation {np.corrcoef(x, y)[0, 1]:+.2f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("windows", nargs="*", default=BURSTS)
    parser.add_argument("--airframe", default="airframe_2026-11-12_raised.json")
    parser.add_argument("--set", default="", help="airframe terms over the airframe's, name=value,...")
    parser.add_argument("--fit", default="", help="terms to fit, comma-separated (BOUNDS)")
    parser.add_argument("--every", type=float, default=0.05)
    parser.add_argument("--axes", default="0,1,2")
    parser.add_argument("--bins", action="store_true", help="the tilt's pitch authority by swing size instead")
    args = parser.parse_args()
    windows = [Window(spec) for spec in args.windows]
    fixed = parse_settings(args.set) if args.set else {}
    axes = [int(a) for a in args.axes.split(",")]
    if args.bins:
        bins(windows, args.airframe, fixed)
        return
    if args.fit:
        values, before, after = fit(windows, args.airframe, fixed, args.fit.split(","), args.every, axes)
        print(", ".join(f"{k} {v:.4g}" for k, v in values.items()))
    else:
        before = after = [float(np.sqrt(np.mean(e ** 2))) for e in errors(windows, args.airframe, fixed, args.every, axes)]
    for w, b, a in zip(windows, before, after):
        print(f"  {w.spec}: rate error RMS (axes {args.axes}) {b:.1f} -> {a:.1f} deg/s")


if __name__ == "__main__":
    main()
