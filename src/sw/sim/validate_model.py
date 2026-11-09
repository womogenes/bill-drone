"""
Score airframe models on flights without fitting anything: uv run -m sw.sim.validate_model <recordings> --models models/airframe_A.json [models/airframe_B.json ...] [--truth DIR]

For each model, on the given flights:
- the closed-loop multi-step prediction (identify_airframe's score: the firmware controller flies the model on the logged references from every start, and the attitude error at each horizon is compared with the real one), as the held-out report prints it;
- the stick steps: every jump of over 8 degrees in a logged attitude target held for 0.6 s or more while airborne, flown closed loop from the step's start for as long as the target holds (at most 3 s), with the model's and the real response's overshoot (peak over the step size), time to peak and rise to 90%, and the RMS of the model's response error over the step;
- with --truth (added 2026-11-09), on flights with a smoothed GPS truth (sw/sim/gps_truth.py --save), the horizontal velocity 1-3 s ahead (velocity_scores): the airframe's own horizontal model integrated from the truth on the closed-loop prediction and on the logged attitude, against the truth.
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
# The velocity horizons (samples) scored against the GPS truth: 1-3 s, the outer loop's time scale (added 2026-11-09).
VELOCITY_HORIZONS = (100, 200, 300)


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


def velocity_scores(params, data, flights, folder, horizons=VELOCITY_HORIZONS, stride=50):
    """
    The horizontal velocity change against the smoothed GPS truth (identify_airframe.truth_world) on the flights that have one: from every stride-th airborne start the world velocity is integrated from the truth's at the start with the airframe's horizontal model (vx_pitch (pitch + tilt_push tilt) - vx_drag v forward, vy_roll roll - vy_drag v left, turned into the world by the truth's heading), on the closed-loop predicted attitude and tilt ("closed") or on the logged attitude and the one-step tilt ("logged": the translational model alone), the latter also with each flight's constant body-frame acceleration removed ("logged, offset": the level reference's error, which gps_truth 1b measures at 0.1-0.5 m/s²). Returns {variant: {horizon s: (lanes, RMS forward, RMS left error in m/s at the end's body axes)}} plus "hold" (the truth's own change: what a model predicting no change misses) and "fast" (closed loop from starts over 2 m/s), and the lanes for plotting.
    """
    worlds = {i: I.truth_world(data.records[i], folder) for i in flights if data.truth[i]}
    use = np.array(sorted(worlds))
    if not len(use):
        return None
    world = np.full((len(data.records), data.size), np.nan, complex)
    heading = np.full((len(data.records), data.size), np.nan)
    for i, (w, h) in worlds.items():
        world[i, :len(w)], heading[i, :len(h)] = w, h
    p = params
    keep, _, _ = I.teacher(p, data, use)
    side_gain = np.array([1.0, p["right_gain"]])
    taught_tilt = (keep["tilt"] * side_gain).mean(-1) * DEG
    F, S0, valid, rates, attitudes, tilts = I.predict(p, data, use, [1, *horizons], stride, True, velocities=True)
    J = np.searchsorted(use, F)
    # Each flight's constant body-frame acceleration the translational model misses over its airborne span (the truth's acceleration less the model's on the logged attitude).
    offset = np.zeros(len(data.records), complex)
    for j, i in enumerate(use):
        r = data.records[i]
        span = slice(r["lift"], r["end"])
        turn = np.exp(-1j * heading[i, span])
        measured = np.gradient(world[i, span]) / I.DT * turn
        body = world[i, span] * turn
        model = p["vx_pitch"] * (data.att[i, span, 1] + p["tilt_push"] * taught_tilt[j, span]) - p["vx_drag"] * body.real + 1j * (p["vy_roll"] * data.att[i, span, 0] - p["vy_drag"] * body.imag)
        good = np.isfinite(measured) & np.isfinite(model)
        offset[i] = np.mean((measured - model)[good]) if good.any() else 0.0
    longest = max(horizons)
    runs = {"closed": world[F, S0].copy(), "logged": world[F, S0].copy(), "logged, offset": world[F, S0].copy()}
    traces = {name: np.zeros((longest + 1, len(F)), complex) for name in runs}
    for name in runs:
        traces[name][0] = runs[name]
    for j in range(longest):
        k = np.minimum(S0 + j, data.size - 1)
        theta = heading[F, k]
        turn = np.exp(1j * theta)
        for name, v in runs.items():
            if name == "closed":
                roll, pitch = (data.att[F, S0, 0], data.att[F, S0, 1]) if j == 0 else (attitudes[j - 1, :, 0], attitudes[j - 1, :, 1])
                tilt = taught_tilt[J, S0] if j == 0 else tilts[j - 1]
            else:
                roll, pitch, tilt = data.att[F, k, 0], data.att[F, k, 1], taught_tilt[J, k]
            body = v * np.conj(turn)
            accel = p["vx_pitch"] * (pitch + p["tilt_push"] * tilt) - p["vx_drag"] * body.real + 1j * (p["vy_roll"] * roll - p["vy_drag"] * body.imag)
            if name == "logged, offset":
                accel = accel + offset[F]
            runs[name] = v + accel * turn * I.DT
            traces[name][j + 1] = runs[name]
    fast = np.abs(world[F, S0]) > 2.0
    out = {"lanes": (F, S0, traces, world, heading)}
    for name in (*runs, "hold", "fast"):
        out[name] = {}
        for h_index, h in enumerate(horizons):
            end = np.minimum(S0 + h, data.size - 1)
            truth = world[F, end]
            predicted = world[F, S0] if name == "hold" else traces["closed" if name == "fast" else name][h]
            good = valid[1 + h_index] & np.isfinite(truth) & np.isfinite(predicted) & (fast if name == "fast" else True)
            error = (predicted - truth)[good] * np.exp(-1j * heading[F, end][good])
            out[name][h * I.DT] = (int(good.sum()), float(np.sqrt(np.mean(error.real ** 2))) if good.any() else np.nan, float(np.sqrt(np.mean(error.imag ** 2))) if good.any() else np.nan)
    return out


def print_velocity(scores):
    print("  horizontal velocity against the GPS truth (RMS error forward/left, m/s):")
    for name in ("hold", "logged", "logged, offset", "closed", "fast"):
        print(f"    {name:15s} " + "; ".join(f"{h:.0f} s {n} lanes {fx:.2f}/{fy:.2f}" for h, (n, fx, fy) in scores[name].items()), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--shortest", type=float, default=0.5)
    parser.add_argument("--velocity", choices=("logged", "tag"), default="tag", help="the logged hover velocity everywhere, or only where the tag measured it (identify_airframe.Data)")
    parser.add_argument("--measured-input", action="store_true", help="the measured velocity (tag or GPS truth) is the speed terms' input at every step that has one, not only at each prediction's start (identify_airframe.predict)")
    parser.add_argument("--truth", help="a folder of smoothed GPS truths (sw/sim/gps_truth.py --save): their velocity is the airframe's where they have one, and the horizontal velocity prediction is scored against them")
    args = parser.parse_args()
    records = [r for r in (I.load(p, args.shortest) for p in I.paths(args.recordings)) if r]
    print(f"{len(records)} flights: " + ", ".join(r["name"] for r in records), flush=True)
    for model_path in args.models:
        loaded = json.loads(Path(model_path).read_text())
        params = A.complete(loaded)
        geometry = A.load_geometry(loaded.get("geometry") or A.GEOMETRY_FILE)
        ground = loaded.get("ground_by")
        data = I.Data(records, ground, "truth" if args.truth else args.velocity, args.truth)
        data.measured_input = args.measured_input
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
        if args.truth:
            scores = velocity_scores(params, data, everything, args.truth)
            if scores:
                print_velocity(scores)


if __name__ == "__main__":
    main()
