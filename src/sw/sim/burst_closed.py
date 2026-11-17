"""
A recorded burst flown again on a plant by the firmware that flew it, and set against the log: uv run -m sw.sim.burst_closed prepare FLIGHT PLANT.json OUT.json (in this tree: the plant with the flight's own CoM and yaw push from scorecard.flight_terms baked in), then, in a tree of the firmware to fly (the as-flown commit, or the one that will fly), PYTHONPATH=. python burst_closed.py fly PREPARED.json OUT.json [--harness DIR] [--n 64] [--set k=v,...] [--draws JSON] [--workers 16], and uv run -m sw.sim.burst_closed plot OUT.json ... FIGURE.png here.

fly runs the tree's own firmware through its flip_firmware harness (with the pack's sag and thrust_voltage where it has them, as william/consolidate's does; for older trees --harness DIR's ffsag.py, flip_firmware patched with them as william/burst-repro reproduced the 07:03 burst60), from the ground through the takeoff, the climb to the recording's hold and the flip request: the recording's settings as flown (the GPS hold, the burst it flew: flip_angle, or flip_enable before it), asked for the flip once a second from the time after liftoff the log asked (until it starts), the stand's thrust table with the pack's sag, the firmware's thrust scales and hover_mass as set, the plant as saved with no randomization but the draws (--draws: {term: [low, high]} drawn uniformly per run, the plant's uncertainty), and calm air. Each run's trace from the flip's start (2 s), and its measures: the largest roll and its time, the largest nose excursion either way and when, the largest pitch rate, the hand-back time (the flip's phase 3), the height lost and the highest point, whether it was clean or touched.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

# The trace's columns (flip_firmware TRACE, as the harness records it): s from the flip's start, body rates (deg/s), throttles, servo angles (deg), the flip's phase, the height (m), roll and pitch (deg), the pack (V), the assemblies' tilts (deg).
COLUMNS = ("t", "p", "q", "r", "left", "right", "left_servo", "right_servo", "phase", "height", "roll", "pitch", "volts", "left_tilt", "right_tilt")
# The pack's sag (V at no load, V per newton^1.5 summed over the motors) and the thrust_voltage the drone flew (william/burst-repro's harness).
SAG, THRUST_VOLTAGE = (16.196, 0.0161), 16.1
# The burst a recording from before flip_angle (a865823) asked for by flip_enable: 2, 3 and 4 were 60, 90 and 30 degrees.
FLIP_ENABLE_ANGLE = {2: 60.0, 3: 90.0, 4: 30.0}


def log_burst(name):
    """The recording's burst: the flip's start (s from the recording's start), its liftoff, and the log's trace from 0.5 s before the start to 2.5 s after, in the trace's columns where the log has them."""
    from sw.sim import scorecard as S
    flight = S.named(name)[1]
    start = flight.time[np.flatnonzero(flight.flip > 0)[0]]
    lifted = flight.time[np.flatnonzero(flight.airborne)[0]]
    shown = (flight.time >= start - 0.5) & (flight.time <= start + 2.5)
    trace = np.column_stack([flight.time[shown] - start, np.degrees(flight.gyro[shown]), flight.throttle[shown], np.degrees(flight.tilt[shown]), flight.flip[shown], flight.height[shown], np.degrees(flight.attitude[shown]), np.full(shown.sum(), np.nan), np.degrees(flight.tilt[shown])])
    return {"start": float(start), "liftoff": float(lifted), "trace": trace.tolist()}


def prepare(name, plant_path, out):
    """The plant for this flight's rerun: the airframe with the flight's CoM (forward and left) and yaw push baked in, the trim the closed loop flies (thrust level 1 on the stand's table with the sag, the split the CoM asks for), and the log's burst."""
    from sw.sim import airframe as A
    from sw.sim import scorecard as S
    entry, flight = S.named(name)
    plant = S.plant_for(flight, plant_path, {})
    # The CoM from the plant without its disturbance state, which would take the CoM's steady torque into its estimate: the closed loop has no observer.
    terms = S.flight_terms(flight, plant | {"observer": None}, 100)
    if terms is None:
        terms = np.zeros(3)
    params = dict(plant["airframe"])
    geometry = plant["geometry"]
    params["com_x"] = params.get("com_x", A.PARAMS["com_x"][2]) + terms[0]
    params.setdefault("geometry_terms", {})["com_y"] = geometry["com"][1] + terms[1]
    params["yaw_push"] = params.get("yaw_push", 0.0) + terms[2]
    weight = geometry["mass"] * A.G
    arm, offset = geometry["arm"], geometry["com"][1] + terms[1]
    params |= {"thrust_level": 1.0, "hover_thrust": [weight * (arm + offset) / (2 * arm), weight * (arm - offset) / (2 * arm)]}
    params.pop("observer", None)
    params.pop("fit", None)
    burst = log_burst(name)
    Path(out).write_text(json.dumps({"flight": entry["name"], "airframe": params, "terms": [float(t) for t in terms], "log": burst}, indent=1))
    print(f"{entry['name']}: CoM {1000 * terms[0]:+.1f} mm forward, {1000 * terms[1]:+.1f} mm left of the plant's, yaw push {terms[2]:+.2f} rad/s²; flip at {burst['start']:.2f} s, {burst['start'] - burst['liftoff']:.1f} s after liftoff")


def fly(prepared, out, harness, n, overrides, draws, workers, seed):
    """The prepared flight's burst flown n times by this tree's firmware."""
    from multiprocessing import Pool
    if harness:
        sys.path.insert(0, harness)
        import ffsag as FF
        options = {"sag": SAG, "thrust_voltage": THRUST_VOLTAGE}
    else:
        from sw.sim import flip_firmware as FF
        from sw.sim import flip_esc
        # Trees whose flip_firmware carries the sag itself (william/consolidate from 112eb6f): the stand's fit of it and its voltages.
        options = {"sag": flip_esc.SAG, "table_volts": flip_esc.STAND_VOLTS} if hasattr(flip_esc, "SAG") else {}
    from sw.flight_log import recording, settings as recording_settings
    from sw.sim.flip_esc import STAND
    from sw.sim import flight
    # The IMU's ranges as this tree's firmware sets them (runtime.h: ±8 g from william/consolidate, ±2 g before).
    gyro_range, accel_range = math.radians(getattr(flight, "GYRO_RANGE", 2000.0)), getattr(flight, "ACCEL_RANGE", 2.0)
    data = json.loads(Path(prepared).read_text())
    FF.TRACE = True
    name = "burst_closed_plant.json"
    (FF.R.A.MODELS / name).write_text(json.dumps(data["airframe"]))
    # The plant as saved: its own rotor drag (none where the hubs' aerodynamics carry it), no disturbance terms.
    FF.plant_of = lambda airframe: {"airframe": FF.R.A.model(airframe), "geometry": FF.R.A.geometry_of(FF.R.A.model(airframe)), "disturbances": [], "rotor_drag": tuple(FF.R.A.model(airframe).get("rotor_drag", (0.07, 0.2)))}
    flown = recording_settings(recording(data["flight"])) | {"hover_camera": 0, "hover_center": 0}
    if "flip_angle" not in flown and flown.get("flip_enable") in FLIP_ENABLE_ANGLE:
        flown["flip_angle"] = FLIP_ENABLE_ANGLE[flown["flip_enable"]]
    flown |= overrides
    if options:
        flown["thrust_voltage"] = THRUST_VOLTAGE
    log = data["log"]
    flip_at = max(1.0, log["start"] - log["liftoff"] - 0.5)
    rng = np.random.default_rng(seed)
    jobs = []
    for k in range(n):
        drawn = {key: float(rng.uniform(*span)) for key, span in draws.items()}
        job_options = options | {"table": STAND, "thrust_scale": flown.get("left_thrust_scale", 1.3), "hover_mass": flown.get("hover_mass", 1.02), "set": drawn}
        jobs.append((flown, 0.0, seed + 1000 * k, 1, flip_at + 7.0, gyro_range, accel_range, flip_at, True, name, "none", {}, job_options))
    with Pool(workers) as pool:
        rows = [r | {"drawn": j[12]["set"]} for j, batch in zip(jobs, pool.map(FF.fly, jobs, chunksize=1)) for r in batch]
    runs = []
    for r in rows:
        trace = np.array(r.get("trace") or [])
        run = {"started": r["started"], "drawn": r["drawn"], "refusals": r.get("refusals")}
        if len(trace):
            run |= measures(trace) | {"clean": bool(r.get("clean")), "touched": bool(r.get("touched")), "lost": float(r.get("lost", math.nan)), "peak": float(r.get("peak", math.nan)), "handed": float(r.get("handed", math.nan)), "rates": [float(v) for v in r.get("rates", [])], "trace": trace[:, :len(COLUMNS)].round(3).tolist()}
        runs.append(run)
    Path(out).write_text(json.dumps({"flight": data["flight"], "settings": overrides, "draws": draws, "runs": runs, "log": log, "log_measures": measures(np.array(log["trace"], float))}, indent=1))
    summary(out)


def measures(trace):
    """A burst's measures from its trace (the log's or a run's): the largest roll (deg) and when, the nose's largest excursion up and down (deg, the attitude's pitch) and when, the largest pitch rate either way (deg/s), the flip's phase-3 time (the hand-back), and the height lost and gained over the 2 s (m)."""
    t = trace[:, 0]
    after = t >= 0
    roll, pitch, q, height = trace[:, 10], trace[:, 11], trace[:, 2], trace[:, 9]
    out = {}
    k = np.nanargmax(np.where(after, np.abs(roll), -np.inf))
    out |= {"roll_max": float(roll[k]), "roll_max_t": float(t[k])}
    for label, sign in (("pitch_up", 1), ("pitch_down", -1)):
        k = np.nanargmax(np.where(after, sign * pitch, -np.inf))
        out |= {label: float(pitch[k]), f"{label}_t": float(t[k])}
    k = np.nanargmax(np.where(after, np.abs(q), -np.inf))
    out |= {"q_max": float(q[k]), "q_max_t": float(t[k])}
    handed = np.flatnonzero(after & (trace[:, 8] == 3))
    out["handed_t"] = float(t[handed[0]]) if len(handed) else None
    start = height[np.flatnonzero(after)[0]]
    out |= {"height_start": float(start), "height_lost": float(start - np.nanmin(np.where(after, height, np.nan))), "height_gained": float(np.nanmax(np.where(after, height, np.nan)) - start)}
    return out


def summary(path):
    data = json.loads(Path(path).read_text())
    runs = [r for r in data["runs"] if "roll_max" in r]
    log = data["log_measures"]
    print(f"{data['flight']}: {len(runs)} of {len(data['runs'])} runs flipped; clean {np.mean([r['clean'] for r in runs]) * 100:.0f}%, touched {np.mean([r['touched'] for r in runs]) * 100:.0f}%")
    for key in ("roll_max", "roll_max_t", "pitch_up", "pitch_up_t", "pitch_down", "pitch_down_t", "q_max", "handed_t", "height_lost", "height_gained"):
        values = np.array([r[key] for r in runs if r.get(key) is not None], float)
        if len(values):
            low, mid, high = np.percentile(values, [10, 50, 90])
            print(f"  {key:14} log {log.get(key) if log.get(key) is None else round(log[key], 2)!s:>8}   sim median {mid:8.2f}  (10-90%: {low:.2f} to {high:.2f})")


def plot(paths, figure):
    """Each burst's log (black) against its runs (the median run in color, the 10-90% band shaded) for roll, pitch, pitch rate and height from the flip's start."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    channels = (("roll", 10, "deg"), ("pitch", 11, "deg"), ("pitch rate", 2, "deg/s"), ("roll rate", 1, "deg/s"), ("height", 9, "m"))
    fig, axes = plt.subplots(len(channels), len(paths), figsize=(5.5 * len(paths), 2.0 * len(channels)), squeeze=False, sharex=True)
    for column, path in enumerate(paths):
        data = json.loads(Path(path).read_text())
        log = np.array(data["log"]["trace"], float)
        runs = [np.array(r["trace"], float) for r in data["runs"] if r.get("trace")]
        grid = np.arange(0, 2.0, 0.01)
        for row, (label, c, unit) in enumerate(channels):
            ax = axes[row, column]
            if runs:
                stack = np.array([np.interp(grid, r[:, 0], r[:, c]) for r in runs])
                low, mid, high = np.nanpercentile(stack, [10, 50, 90], axis=0)
                ax.fill_between(grid, low, high, color="#d62728", alpha=0.2, lw=0)
                ax.plot(grid, mid, color="#d62728", lw=1.0, label="sim median, 10-90%")
            values = log[:, c] - (log[np.argmin(np.abs(log[:, 0])), c] if c == 9 else 0.0)
            if c == 9 and runs:
                values = values + np.nanmedian([r[0, 9] for r in runs])
            ax.plot(log[:, 0], values, color="black", lw=1.1, label="log")
            ax.set_ylabel(f"{label} ({unit})", fontsize=8)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=7)
            if row == 0:
                clean = np.mean([r["clean"] for r in data["runs"] if "clean" in r]) * 100 if runs else 0
                ax.set_title(f"{data['flight']} {data.get('label', '')}\n{len(runs)} runs, {clean:.0f}% clean", fontsize=8)
                ax.legend(fontsize=6)
        axes[-1, column].set_xlabel("s from the flip's start", fontsize=8)
        axes[-1, column].set_xlim(-0.5, 2.0)
    fig.tight_layout()
    fig.savefig(figure, dpi=80)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("prepare", "fly", "summary", "plot"))
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--harness")
    parser.add_argument("--n", type=int, default=32)
    parser.add_argument("--set", default="")
    parser.add_argument("--draws", default="{}")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=7000)
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(*args.paths)
    elif args.action == "fly":
        overrides = {k: float(v) for k, v in (item.split("=") for item in args.set.split(",") if item)}
        fly(args.paths[0], args.paths[1], args.harness, args.n, overrides, json.loads(args.draws), args.workers, args.seed)
    elif args.action == "summary":
        for path in args.paths:
            summary(path)
    else:
        plot(args.paths[:-1], args.paths[-1])


if __name__ == "__main__":
    main()
