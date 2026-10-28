"""
Pitch effectiveness, actuator delay and steady push from the airborne part of free-flight recordings: uv run -m sw.fit_airborne sw/recordings/<flight>.bin [...]
Each window runs from liftoff (first az above 1.25 g once collective reaches 600) to |pitch| or |roll| past 35°, an impact (|a| > 2.5 g) or motor stop, so ground contact never enters the fit.
"""

import sys
from pathlib import Path

import numpy as np

from sw.fit import acceleration, settings, terms
from sw.flight_log import read_flight


def windows(paths):
    for path in paths:
        data, _ = read_flight(path)
        on = np.flatnonzero(data.base_throttle.values >= 600)
        if not len(on) or not (data.az_g.values[on[0]:] > 1.25).any():
            continue
        lift = on[0] + np.argmax(data.az_g.values[on[0]:] > 1.25)
        shock = np.sqrt(data.ax_g**2 + data.ay_g**2 + data.az_g**2).values
        end = next((i for i in range(lift + 3, len(data)) if abs(data.pitch_deg[i]) > 35 or abs(data.roll_deg[i]) > 35 or shock[i] > 2.5 or data.base_throttle[i] == 0), len(data))
        _, axes = terms(data, {"left_thrust_scale": 1.0, "right_thrust_scale": 1.0} | settings(path), path)
        torque, model = axes["pitch"]
        yield path, lift, end, torque * model, acceleration(data.gy_deg_s.values, 5)


def main():
    flights = list(windows([Path(p) for p in sys.argv[1:]]))
    for path, lift, end, _, _ in flights:
        print(f"{path.name[:24]} airborne {lift / 100:5.2f}-{end / 100:5.2f} s")
    best = None
    for delay in range(21):
        x = np.concatenate([np.roll(m, delay)[lift:end] for _, lift, end, m, _ in flights])
        y = np.concatenate([a[lift:end] for _, lift, end, _, a in flights])
        design = np.column_stack([x, np.ones_like(x)])
        (slope, push), *_ = np.linalg.lstsq(design, y, rcond=None)
        fit = 1 - np.var(y - design @ [slope, push]) / np.var(y)
        if best is None or fit > best[3]:
            best = (delay, slope, push, fit, len(y))
    delay, slope, push, fit, count = best
    print(f"pitch effectiveness {slope:.2f} x model, delay {delay * 10} ms, push {push:+.2f} rad/s² (+ nose-down), R² {fit:.2f}, {count} samples")


if __name__ == "__main__":
    main()
