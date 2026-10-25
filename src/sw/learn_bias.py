"""
Learn roll_bias and pitch_bias from a flight: uv run -m sw.learn_bias [sw/recordings/<flight>.bin] [--apply] [--gain 0.7]
Rebuilds the attitude controller's integrators and pitch-bias fade-in from the log (flight_math.c), finds the airborne stretches where the attitude stopped moving, and takes the controller's total angular-acceleration command there as the steady disturbance the bias should cancel. --apply moves each bias that fraction of the way and stores it through the running bridge.
"""

import argparse
import json
import math
import urllib.request
from pathlib import Path

import numpy as np

from sw.flight_log import read_flight

BRIDGE = "http://127.0.0.1:8765/settings"
# Airborne samples count as steady when the body rate is below this (deg/s), from this long after liftoff (s).
STEADY_RATE = 15
SETTLE_S = 0.3


def settings_of(path):
    settings = {}
    for line in path.with_suffix(".jsonl").read_text().splitlines():
        event = json.loads(line)
        settings |= event.get("settings", {})
    return settings


def learn(path):
    data, _ = read_flight(path)
    s = settings_of(path)
    t = (data.esp_time_us - data.esp_time_us.iloc[0]).to_numpy() / 1e6
    dt = np.diff(t, prepend=t[0])
    airborne = data.base_throttle.to_numpy() >= s["integrate_throttle"]
    if not airborne.any():
        return None
    lift = t[airborne.argmax()]
    result = {"file": path.name, "airborne_s": round(float(t[airborne].max() - lift), 2)}
    for axis, rate in (("roll", "gx_deg_s"), ("pitch", "gy_deg_s")):
        angle = np.radians(data[f"{axis}_deg"].to_numpy())
        # The controller flies level at pitch_trim (degrees in the settings), added to the logged target.
        target = np.radians(data[f"target_{axis}_deg"].to_numpy() + (s.get("pitch_trim", 0) if axis == "pitch" else 0))
        # The setpoint holds the ground attitude until liftoff, then ramps to the target at target_rate (rad/s in the log's settings is degrees).
        setpoint = np.where(airborne, target, angle)
        step = math.radians(s["target_rate"]) * dt
        for i in range(1, len(setpoint)):
            if airborne[i]:
                setpoint[i] = setpoint[i - 1] + np.clip(target[i] - setpoint[i - 1], -step[i], step[i])
        error = setpoint - angle
        # As flight_math.c: the integrators see at most 8° of error; roll's starts at its bias, while pitch's starts at zero and the pitch bias fades in over 0.3 s from the moment collective reaches integrate_throttle.
        seen = np.clip(error, -math.radians(8), math.radians(8))
        bias = np.clip(s[f"{axis}_bias"], -s["integral_limit"], s["integral_limit"])
        integral = np.clip((bias if axis == "roll" else 0) + np.cumsum(np.where(airborne, s[f"{axis}_i"] * seen * dt, 0)), -s["integral_limit"], s["integral_limit"])
        if axis == "pitch":
            integral = integral + bias * np.clip((t - lift) / 0.3, 0, 1)
        gyro = np.radians(data[rate].to_numpy())
        command = s[f"{axis}_p"] * error + integral - s[f"{axis}_damping"] * gyro
        steady = airborne & (t - lift >= SETTLE_S) & (np.abs(data[rate].to_numpy()) < STEADY_RATE)
        if steady.sum() < 5:
            result[axis] = {"steady_samples": int(steady.sum()), "bias": s[f"{axis}_bias"]}
            continue
        # At a steady attitude the command balances the disturbance; the bias should start there.
        needed = float(np.median(command[steady]))
        result[axis] = {"steady_samples": int(steady.sum()), "bias": s[f"{axis}_bias"], "needed": round(needed, 2), "tilt_deg": round(float(np.degrees(np.median(angle[steady] - setpoint[steady]))), 1)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path, nargs="?")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--gain", type=float, default=0.7)
    args = parser.parse_args()
    path = args.file or max(Path(__file__).parent.joinpath("recordings").glob("*.bin"), key=lambda p: p.stat().st_mtime)
    result = learn(path)
    print(json.dumps(result, indent=1))
    if not result or not args.apply:
        return
    update = {}
    for axis in ("roll", "pitch"):
        if "needed" in result[axis]:
            old = result[axis]["bias"]
            update[f"{axis}_bias"] = round(old + args.gain * (result[axis]["needed"] - old), 2)
    if update:
        request = urllib.request.Request(BRIDGE, json.dumps({"settings": update}).encode(), {"Content-Type": "application/json"})
        stored = json.load(urllib.request.urlopen(request, timeout=5))["settings"]
        print("stored", {k: stored[k] for k in update})


if __name__ == "__main__":
    main()
