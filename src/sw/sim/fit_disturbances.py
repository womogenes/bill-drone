"""
Fit the sim's disturbances (sw/sim/disturbance.py) from hover recordings: uv run --with scipy -m sw.sim.fit_disturbances <recordings> [--airframe models/airframe_<day>.json] [--out disturbances.json]

Torques: the airframe model (sw/sim/airframe.py) is run one step ahead along each flight's logged commands, as identify_airframe's teacher does; what it leaves of each axis's angular acceleration, below CUTOFF Hz (the rest is vibration and sensor noise that does not move the attitude), is a drift whose standard deviation and correlation time are fitted. Its mean is the airframe's push, already in the model, so no constant is added.
Forces: the horizontal acceleration the camera saw (toward the tag and to the left, from the range and the place on the circle, smoothed over SMOOTH s) less what thrust tilt and the airframe's drag explain (g tan of body pitch plus the motors' mean tilt, or of roll), split into a constant (the room's wash, or a thrust the servos do not report) and a drift.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.signal import butter, filtfilt

from sw.sim import airframe as A
from sw.sim import identify_airframe as N

CUTOFF = 2.0
SMOOTH = 0.5
MODELS = Path(__file__).with_name("models")


def drift(series, dt=A.DT):
    """Standard deviation and the lag where the autocorrelation first falls below 1/e (s)."""
    x = np.concatenate([s - s.mean() for s in series])
    sd = x.std()
    lags = []
    for s in series:
        s = s - s.mean()
        ac = np.correlate(s, s, "full")[len(s) - 1:]
        ac = ac / ac[0]
        below = np.flatnonzero(ac < 1 / math.e)
        if len(below):
            lags.append(below[0] * dt)
    return sd, float(np.median(lags)) if lags else 1.0


def torques(params, data):
    flights = list(range(len(data.records)))
    keep, _, trim = N.teacher(params, data, flights)
    b, a = butter(2, CUTOFF / (0.5 / A.DT))
    out = {axis: [] for axis in ("roll", "pitch", "yaw")}
    f = np.array(flights)
    state = A.State(len(f))
    zero = np.zeros((len(f), 3))
    residual = np.zeros((len(f), data.size, 3))
    for k in range(data.size - 1):
        for key in keep:
            setattr(state, key, keep[key][:, k].copy())
        state.rate, state.att = data.rate[f, k].copy(), data.att[f, k].copy()
        state, _ = A.step(params, state, data.throttle[f], data.command[f], np.full(len(f), k), data.voltage[f, k], zero, trim, geometry=N.GEOM, table=data.table)
        residual[:, k] = (data.rate[f, k + 1] - state.rate) / A.DT
    for j, r in enumerate(data.records):
        window = residual[j, r["lift"] + 50:r["end"] - 1]
        if len(window) < 200:
            continue
        smooth = filtfilt(b, a, window, axis=0)
        for i, axis in enumerate(out):
            out[axis].append(smooth[:, i])
    return {axis: drift(series) for axis, series in out.items()}


def forces(params, records):
    """The horizontal acceleration the camera saw less what tilt and drag explain, per flight [sample, (toward the tag, left)] over the tag-tracked hold."""
    series = []
    for r in records:
        s = r["setting"]
        x, y = r["hover_x_m"], r["hover_y_m"]
        fresh = np.isfinite(x) & np.isfinite(y)
        mode = r["flags"].astype(int) >> 1 & 3
        k = np.arange(len(x))
        hold = (k > r["lift"] + 300) & (k < r["end"]) & (mode == 1) & fresh
        if hold.sum() < 500:
            continue
        span = slice(np.flatnonzero(hold)[0], np.flatnonzero(hold)[-1])
        rng = np.hypot(x, y)
        around = np.full(len(x), np.nan)
        around[fresh] = np.unwrap(np.arctan2(y[fresh], x[fresh]))
        n = round(SMOOTH / A.DT)
        # Toward the tag: the range shrinking; left: the drone moving left round the tag turns the tag's direction to the right (around falls).
        toward = -uniform_filter1d(np.interp(k, k[fresh], rng[fresh]), n)
        left = -uniform_filter1d(np.interp(k, k[fresh], around[fresh] * rng[fresh]), n)
        result = []
        for position, drag in ((toward, params["vx_drag"]), (left, params["vy_drag"])):
            v = np.gradient(position, A.DT)
            result.append((uniform_filter1d(np.gradient(v, A.DT), n), v, drag))
        tilt = np.radians(((r["left_servo_deg"] - s.get("left_trim", 0.0)) - (r["right_servo_deg"] - s.get("right_trim", 0.0))) / 2)
        pitch, roll = np.radians(r["pitch_deg"]), np.radians(r["roll_deg"])
        forward_lean = uniform_filter1d(9.81 * np.tan(pitch + tilt), n)
        # Positive roll (right side down) moves right.
        left_lean = uniform_filter1d(-9.81 * np.tan(roll), n)
        (ax, vx, cx), (ay, vy, cy) = result
        series.append(np.column_stack([ax - forward_lean + cx * vx, ay - left_lean + cy * vy])[span])
    return series


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--airframe", type=Path, help="default: the newest sw/sim/models/airframe_*.json")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    saved = json.loads((args.airframe or sorted(MODELS.glob("airframe_2*.json"))[-1]).read_text())
    params = A.complete(saved)
    N.GEOM = A.geometry_of(saved)
    records = [r for r in (N.load(p, 3.0) for p in N.paths(args.recordings)) if r]
    print(f"{len(records)} flights")
    terms = []
    for axis, (sd, tau) in torques(params, N.Data(records)).items():
        print(f"{axis:8s} torque drift {sd:.2f} rad/s² over {tau:.2f} s")
        terms.append({"channel": axis, "kind": "drift", "sd": round(sd, 3), "tau": round(tau, 3)})
    series = forces(params, records)
    for i, channel in enumerate(("forward", "left")):
        values = [s[:, i] for s in series]
        constant = float(np.mean(np.concatenate(values)))
        sd, tau = drift(values)
        print(f"{channel:8s} force constant {constant:+.2f} m/s², drift {sd:.2f} m/s² over {tau:.2f} s")
        terms += [{"channel": channel, "kind": "constant", "value": round(constant, 3)}, {"channel": channel, "kind": "drift", "sd": round(sd, 3), "tau": round(tau, 3)}]
    if args.out:
        args.out.write_text(json.dumps(terms, indent=1) + "\n")
        print(f"saved {args.out}")


if __name__ == "__main__":
    main()
