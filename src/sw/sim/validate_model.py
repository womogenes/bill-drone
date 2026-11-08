"""
Score airframe models on flights without fitting anything: uv run -m sw.sim.validate_model <recordings> --models models/airframe_A.json [models/airframe_B.json ...] [--workers N]

For each model, on the given flights:
- the closed-loop multi-step prediction (identify_airframe's score: the firmware controller flies the model on the logged references from every start, and the attitude error at each horizon is compared with the real one), as the held-out report prints it;
- the stick steps: every jump of over 8 degrees in a logged attitude target held for 0.6 s or more while airborne, flown closed loop from the step's start for as long as the target holds (at most 3 s), with the model's and the real response's overshoot (peak over the step size), time to peak and rise to 90%, and the RMS of the model's response error over the step.
Added 2026-11-08 to check the 2026-11-08 model on flights it was not fitted to (the first outdoor flights, with many arrow-key steps) before its gain predictions are trusted. Fitting nothing, it runs on a laptop for a few flights.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import identify_airframe as I

DEG = np.pi / 180
STEP = 8 * DEG
HOLD = 0.6
LONGEST = 3.0


def steps(record_index, data):
    """The stick steps in one flight: (axis, start sample, held samples, size in radians)."""
    found = []
    record = data.records[record_index]
    reference = data.targets[record_index, :, 1:3]
    airborne = np.zeros(data.size, bool)
    airborne[record["lift"] : record["end"]] = True
    for axis in range(2):
        target = reference[:, axis]
        k = 1
        while k < len(target) - 1:
            jump = target[k] - target[k - 1]
            if airborne[k] and abs(jump) > STEP:
                end = k
                while end < len(target) - 1 and airborne[end] and abs(target[end] - target[k]) < DEG and (end - k) * I.DT < LONGEST:
                    end += 1
                if (end - k) * I.DT >= HOLD:
                    found.append((axis, k, end - k, jump))
                k = max(end, k + 1)
            else:
                k += 1
    return found


def response(values, size):
    """Overshoot (peak over size), time to peak (s) and rise to 90% (s) of a step response from its start value."""
    scaled = (values - values[0]) / size
    peak = int(np.argmax(scaled))
    rise = np.argmax(scaled >= 0.9) * I.DT if (scaled >= 0.9).any() else np.nan
    return scaled[peak], peak * I.DT, rise


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--shortest", type=float, default=0.5)
    args = parser.parse_args()
    records = [r for r in (I.load(p, args.shortest) for p in I.paths(args.recordings)) if r]
    print(f"{len(records)} flights: " + ", ".join(r["name"] for r in records), flush=True)
    for model_path in args.models:
        loaded = json.loads(Path(model_path).read_text())
        params = A.complete(loaded)
        geometry = A.load_geometry(loaded.get("geometry") or A.GEOMETRY_FILE)
        ground = loaded.get("ground_by")
        data = I.Data(records, ground)
        params["right_thrust_gain"] = I.thrust_split(records)
        base = {key: params[key] for key in A.KEYS}
        I.initialise(data, [], base, geometry=geometry)
        everything = list(range(len(records)))
        I.report([I.score(np.array([]), everything, True)], f"\n{model_path}: closed loop on the logged references")
        # Every step, flown from its start for as long as its target holds.
        flights = np.array(everything)
        horizon = int(LONGEST / I.DT)
        F, S0, valid, rates, attitudes = I.predict(params, data, flights, [1, horizon], 1, True)
        rows = []
        for i in everything:
            for axis, start, held, size in steps(i, data):
                lane = np.flatnonzero((F == i) & (S0 == start))
                if not len(lane):
                    continue
                n = min(held, horizon)
                model = np.concatenate([[data.att[i, start, axis]], attitudes[: n - 1, lane[0], axis]])
                real = data.att[i, start : start + n, axis]
                rows.append((records[i]["name"][11:19], "roll" if axis == 0 else "pitch", size / DEG, response(real, size), response(model, size), np.sqrt(np.mean((model - real) ** 2)) / DEG))
        print(f"  {len(rows)} stick steps (overshoot, time to peak s, rise to 90% s; real -> model; response RMS error)")
        for name, axis, size, real, model, error in rows:
            print(f"    {name} {axis:5s} {size:+6.1f}°: {real[0]:.2f} {real[1]:.2f} {real[2]:.2f} -> {model[0]:.2f} {model[1]:.2f} {model[2]:.2f}; {error:.1f}°")
        if rows:
            real = np.array([r[3] for r in rows])
            model = np.array([r[4] for r in rows])
            errors = np.array([r[5] for r in rows])
            print(f"  medians: overshoot {np.nanmedian(real[:, 0]):.2f} -> {np.nanmedian(model[:, 0]):.2f}, time to peak {np.nanmedian(real[:, 1]):.2f} -> {np.nanmedian(model[:, 1]):.2f} s, rise {np.nanmedian(real[:, 2]):.2f} -> {np.nanmedian(model[:, 2]):.2f} s; response error {np.median(errors):.1f}° (mean {errors.mean():.1f}°)", flush=True)


if __name__ == "__main__":
    main()
