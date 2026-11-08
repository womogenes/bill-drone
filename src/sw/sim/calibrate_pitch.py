"""
Calibrate the airframe's pitch path to the hops' closed-loop pitch behaviour: uv run --with scipy --with cma -m sw.sim.calibrate_pitch --airframe models/airframe_<day>.json --recordings <file listing the identification's recordings> [--generations 30] [--workers 90] [--out file]

identify_airframe's multi-step prediction pins the pitch plant at the horizons the attitude controller dominates but leaves loose what decides whether the closed loop swings or shakes: on drone 1 as flown on 2026-11-02 and 2026-11-08 the identified model flew the hops' settings calm (pitch 18-30 degrees peak to peak, decaying), where the real hops swung at 0.7-0.9 Hz to 35-44 degrees and grew, and it did not shake at pitch_damping 8 where the real drone shook at 2.66 Hz (67 deg/s RMS in 2.2-3.5 Hz, 2026-11-02 04:36 UTC). Probing one term at a time (2026-11-08): the servo's damping alone brings the shake (servo_zeta 0.5: 2.62 Hz, 96 deg/s at pitch_damping 8, none at 6), and the low-frequency pitch path (the forward speed's moment drag_z_x, the held tilt's lagged share pitch_wake) the growing swing.
This CMA-ES search sets those terms so that the hover sim (sweep.run, hover=True, sweep.CURRENT's room with this airframe) flying each matched flight's settings from its resting attitude, with its test pattern, matches the flight's pitch-rate RMS in three bands (SWING, MIDDLE, SHAKE; over the flight's airborne time, median over SEEDS runs), with a penalty on the identification's multi-step cost growing (PENALTY per 100% growth). Searched (SPACE): the servo's damping ratio and natural frequency (both sides scaled together), its reflected inertia, its compliance, the tilt's pitch authority (through wash_z), the held tilt's lagged share and lag, and the forward speed's moment height. Every other term stays as identified; refit them afterwards with identify_airframe --start <this output> --freeze <the searched keys>.
"""

import argparse
import json
import math
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.fit import settings as recording_settings
from sw.sim import airframe as A
from sw.sim import identify_airframe as N
from sw.sim import pitch_check as C

MODELS = Path(__file__).with_name("models")
RECORDINGS = Path(__file__).parents[1] / "recordings"
# The flights matched (2026-11-08 and 2026-11-02, drone 1 with 10-inch props): the 2026-11-08 hops at pitch_p 36 and 28 (00:23, 00:28, 00:31, 00:46, 00:52), the 2026-11-02 chirps at pitch_damping 6 and 8 with the same pattern (04:23, 04:36) and two long tag hovers (04:50, 05:22). 00:05, 00:09, 00:20 and 00:39 (under 4.5 s) and the other 2026-11-02 flights are left for checking.
FLIGHTS = ("2026-11-08T00-23-19-186Z_3160940019", "2026-11-08T00-28-06-995Z_1030209381", "2026-11-08T00-31-38-601Z_3398894371", "2026-11-08T00-46-21-333Z_1569889667", "2026-11-08T00-52-55-578Z_2701204891",
           "2026-11-02T04-23-35-159Z_4231648877", "2026-11-02T04-36-23-641Z_3729769197", "2026-11-02T04-50-11-531Z_4148155385", "2026-11-02T05-22-01-327Z_3157561583")
SWING, MIDDLE, SHAKE = (0.4, 1.5), (1.5, 2.2), (2.2, 3.5)
SEEDS = 6
# Simulated seconds per flight at most (the long hovers are matched on their first 20 s).
LONGEST = 20.0
# Searched: key, low, high, log-scaled. "authority" scales the tilt's pitch arm (airframe.resolve tilt_arm) through wash_z; "servo_speed" scales both servo_wn.
SPACE = (("servo_zeta", 0.2, 1.2, True), ("servo_speed", 0.6, 1.6, True), ("servo_inertia", 1e-5, 0.0015, True), ("servo_compliance", 0.0, 1.0, False), ("authority", 0.8, 1.6, True),
         ("pitch_wake", -1.4, 0.0, False), ("wake_tau", 0.1, 1.5, True), ("drag_z_x", -0.02, 0.09, False))
PENALTY = 2.0


def rate_bands(rate, lost):
    """The pitch rate's RMS (deg/s) in each band; a run that was lost counts its last band values doubled (it diverged)."""
    out = [math.sqrt(np.mean(C.band(rate, *b) ** 2)) for b in (SWING, MIDDLE, SHAKE)]
    return [v * (2.0 if lost else 1.0) for v in out]


def apply(base, x, geometry):
    """The airframe with the searched values x (dict keyed as SPACE) in place."""
    p = dict(base)
    for key in ("servo_zeta", "servo_inertia", "servo_compliance", "pitch_wake", "wake_tau", "drag_z_x"):
        p[key] = x[key]
    p["servo_wn"] = base["servo_wn"] * x["servo_speed"]
    p["right_servo_wn"] = base["right_servo_wn"] * x["servo_speed"]
    c = A.resolve(A.complete(base), geometry)
    cz = geometry["com"][2]
    dz = geometry["pivot_z"] - cz
    p["wash_z"] = cz + (dz - x["authority"] * c["tilt_arm"]) / base["wash"]
    return p


def decode(z):
    """Search coordinates in [0, 1] to values."""
    out = {}
    for (key, low, high, log), v in zip(SPACE, np.clip(z, 0, 1)):
        out[key] = math.exp(math.log(low) + v * (math.log(high) - math.log(low))) if log else low + v * (high - low)
    return out


def encode(x):
    return np.array([(math.log(x[key]) - math.log(low)) / (math.log(high) - math.log(low)) if log else (x[key] - low) / (high - low) for key, low, high, log in SPACE])


def flight_job(job):
    """One sim run of a matched flight: its band RMS."""
    settings, airframe, rest, pattern, seconds, seed = job
    from sw.sim.sweep import CURRENT, run
    plant = CURRENT | {"ground_pitch": rest, "airframe": airframe | {f"{axis}_push": 0.0 for axis in ("roll", "pitch", "yaw")}}
    plant["camera"] = dict(plant["camera"]) | {"tag": (settings.get("hover_distance", 2.0), 0.0, 0.45)}
    if pattern:
        plant["excite"] = pattern
    log = []
    result = run(settings, plant, seconds=seconds + 1.5, seed=seed, log=log, hover=True)
    t = np.array([e[0] for e in log])
    lifted = result.get("liftoff_s") or next((t[k] for k in range(len(t)) if log[k][1][2] > 0.01), t[0])
    keep = (t >= lifted) & (t <= lifted + seconds)
    rate = np.degrees([e[11][1] for e, k in zip(log, keep) if k])
    return rate_bands(rate, not result["stable"])


PREDICTION = None


def prediction_init(recordings, geometry):
    global PREDICTION
    records = [r for r in (N.load(p) for p in recordings) if r]
    PREDICTION = N.Data(records)
    N.initialise(PREDICTION, [], {}, geometry=geometry)


def prediction_cost(airframe):
    """The identification's multi-step cost (no instrument, no prior) of an airframe on the recordings."""
    N.BASE, N.FREE = A.complete(airframe), []
    r = N.residuals(np.zeros(0), np.arange(len(PREDICTION.records)))
    return 0.5 * float((r ** 2).sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--airframe", type=Path, required=True)
    parser.add_argument("--recordings", type=Path, required=True, help="a file listing the identification's recordings, one per line, for the prediction penalty")
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--population", type=int, default=16)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    import cma
    base = {key: value for key, value in json.loads(args.airframe.read_text()).items() if key != "folds"}
    geometry = A.geometry_of(base)
    recordings = [Path(line.strip()) for line in args.recordings.read_text().splitlines() if line.strip()]
    flights = []
    for name in FLIGHTS:
        path = RECORDINGS / f"{name}.bin"
        r = C.real(path)
        seconds = min(LONGEST, max(3.0, len(r["pitch"]) / C.FS))
        n = int(seconds * C.FS)
        real = rate_bands(r["rate"][:n], False)
        flights.append((name, recording_settings(path), r["rest"], r["pattern"], seconds, real))
        print(f"{name[:19]}: real pitch rate RMS {real[0]:.0f}/{real[1]:.0f}/{real[2]:.0f} deg/s in {SWING}/{MIDDLE}/{SHAKE} Hz over {seconds:.1f} s", flush=True)
    start = {"servo_zeta": base["servo_zeta"], "servo_speed": 1.0, "servo_inertia": max(base["servo_inertia"], 2e-5), "servo_compliance": base.get("servo_compliance", 0.0), "authority": 1.0,
             "pitch_wake": base["pitch_wake"], "wake_tau": base["wake_tau"], "drag_z_x": base["drag_z_x"]}
    predictors = Pool(args.population + 1, initializer=prediction_init, initargs=(recordings, geometry))
    simulators = Pool(args.workers)
    reference = predictors.apply(prediction_cost, (base,))
    print(f"multi-step cost of the identified airframe {reference:.1f}", flush=True)

    def evaluate(xs):
        airframes = [apply(base, x, geometry) for x in xs]
        costs = predictors.map_async(prediction_cost, airframes)
        jobs = [(settings, airframe, rest, pattern, seconds, seed) for airframe in airframes for _, settings, rest, pattern, seconds, _ in flights for seed in range(SEEDS)]
        runs = simulators.map(flight_job, jobs, chunksize=1)
        costs = costs.get()
        out = []
        for i, x in enumerate(xs):
            mine = runs[i * len(flights) * SEEDS:(i + 1) * len(flights) * SEEDS]
            sim = [np.median(mine[j * SEEDS:(j + 1) * SEEDS], 0) for j in range(len(flights))]
            mismatch = np.mean([(np.log(np.maximum(s, 0.5) / np.maximum(f[5], 0.5)) ** 2) for s, f in zip(sim, flights)])
            out.append((mismatch + PENALTY * max(0.0, costs[i] / reference - 1), mismatch, costs[i] / reference, sim))
        return out

    first = evaluate([start])[0]
    print(f"start: objective {first[0]:.3f} (mismatch {first[1]:.3f}, prediction cost x{first[2]:.3f})", flush=True)
    es = cma.CMAEvolutionStrategy(encode(start), 0.2, {"bounds": [0, 1], "popsize": args.population, "seed": 1, "verbose": -9})
    best = (first[0], start, first)
    for generation in range(args.generations):
        zs = es.ask()
        xs = [decode(z) for z in zs]
        results = evaluate(xs)
        es.tell(zs, [r[0] for r in results])
        i = int(np.argmin([r[0] for r in results]))
        if results[i][0] < best[0]:
            best = (results[i][0], xs[i], results[i])
        print(f"generation {generation + 1}: best {results[i][0]:.3f} (mismatch {results[i][1]:.3f}, prediction x{results[i][2]:.3f}); overall {best[0]:.3f}: " + ", ".join(f"{k} {v:.4g}" for k, v in best[1].items()), flush=True)
    objective, x, (_, mismatch, growth, sim) = best
    for (name, *_, real), s in zip(flights, sim):
        print(f"{name[:19]}: real {real[0]:.0f}/{real[1]:.0f}/{real[2]:.0f}, sim {s[0]:.0f}/{s[1]:.0f}/{s[2]:.0f} deg/s", flush=True)
    out = args.out or args.airframe.with_name(args.airframe.stem + "_pitch.json")
    result = apply(base, x, geometry) | {"pitch_calibration": {"from": args.airframe.name, "searched": {k: round(v, 6) for k, v in x.items()}, "mismatch": round(mismatch, 4), "prediction_cost_ratio": round(growth, 4)}}
    out.write_text(json.dumps(result, indent=1) + "\n")
    print(f"saved {out}", flush=True)
    predictors.close()
    simulators.close()


if __name__ == "__main__":
    main()
