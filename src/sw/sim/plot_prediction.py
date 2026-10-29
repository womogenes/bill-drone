"""
Predicted against real attitude: uv run --with scipy --with matplotlib -m sw.sim.plot_prediction <recordings> [--airframe models/airframe_<day>.json] [--compare another.json] [--held-out] [--out plot.png]

Every --every seconds of each flight's airborne window, the firmware controller (with its replayed state at that sample) flies the airframe model (sw/sim/airframe.py) on the logged references alone for --horizon seconds from the logged attitude, rates and actuator states, as identify_airframe scores it. Each free run is drawn over the logged roll and pitch, so the plot shows how far the model's own closed-loop motion follows the drone's from each start. With --compare a second model's runs are drawn too. The model was fitted on these flights unless they are new ones: its held-out errors are in models/NOTES.md. With --held-out, a model saved with its folds (identify_airframe) flies each flight with the fold's parameters that left that flight out.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import identify_airframe as N

MODELS = Path(__file__).with_name("models")


def runs(params, data, horizon, every, flights=None):
    """Free runs from every start (of the flights, default all): lanes' flights and starts, and the predicted attitudes [step, lane, axis] (radians)."""
    flights = np.arange(len(data.records)) if flights is None else np.asarray(flights)
    F, S0, valid, rates, attitudes = N.predict(params, data, flights, [horizon], every)
    return F, S0, attitudes


def held_out_runs(saved, data, horizon, every):
    """runs() with each flight flown by the parameters of the fold that left it out (flights in no fold by the full fit's)."""
    base = A.complete(saved) | {"right_thrust_gain": N.thrust_split(data.records)}
    names = [r["name"] for r in data.records]
    groups = [(base | fold["params"], [names.index(name) for name in fold["held_out"] if name in names]) for fold in saved.get("folds", [])]
    rest = [i for i in range(len(names)) if not any(i in flights for _, flights in groups)]
    parts = [runs(params, data, horizon, every, sorted(flights)) for params, flights in groups + [(base, rest)] if flights]
    return tuple(np.concatenate([part[j] for part in parts], axis=1 if j == 2 else 0) for j in range(3))


def errors(data, F, S0, attitudes, horizon):
    """RMS attitude-change error (degrees, roll and pitch) at the horizon over the runs still airborne there."""
    end = np.array([data.records[f]["end"] for f in F])
    ok = S0 + horizon < end
    change = attitudes[horizon - 1][ok] - data.att[F[ok], S0[ok]]
    real = data.att[F[ok], S0[ok] + horizon] - data.att[F[ok], S0[ok]]
    return np.sqrt(((change - real) ** 2).mean(0)) / A.DEG


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--airframe", type=Path, default=MODELS / "airframe_2026-10-29.json")
    parser.add_argument("--compare", type=Path, help="a second model to draw alongside")
    parser.add_argument("--horizon", type=float, default=1.0, help="length of each free run (s)")
    parser.add_argument("--every", type=float, default=1.0, help="seconds between starts")
    parser.add_argument("--window", type=float, default=20.0, help="seconds of each flight to draw, from liftoff")
    parser.add_argument("--held-out", action="store_true", help="fly each flight with the fold that left it out (models saved with folds)")
    parser.add_argument("--out", type=Path, default=Path("airframe_prediction.png"))
    args = parser.parse_args()
    records = [r for r in (N.load(p) for p in N.paths(args.recordings)) if r]
    data = N.Data(records)
    horizon, every = int(round(args.horizon / A.DT)), int(round(args.every / A.DT))
    models = [(args.airframe.stem, args.airframe)] + ([(args.compare.stem, args.compare)] if args.compare else [])
    predicted = []
    for name, path in models:
        saved = json.loads(path.read_text())
        params = A.complete(saved)
        params["right_thrust_gain"] = N.thrust_split(records)
        N.GEOM = A.geometry_of(saved)
        F, S0, attitudes = held_out_runs(saved, data, horizon, every) if args.held_out and "folds" in saved else runs(params, data, horizon, every)
        rms = errors(data, F, S0, attitudes, horizon)
        print(f"{name}{' (held out)' if args.held_out and 'folds' in saved else ''}: {len(F)} runs, attitude-change error at {args.horizon:g} s roll {rms[0]:.2f}° pitch {rms[1]:.2f}°")
        predicted.append((name, F, S0, attitudes, rms))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = ("tab:red", "tab:blue")
    fig, axes = plt.subplots(len(records), 2, figsize=(15, 2.6 * len(records)), squeeze=False, constrained_layout=True)
    for i, record in enumerate(records):
        lift, end = record["lift"], record["end"]
        stop = min(end, lift + int(args.window / A.DT))
        t = (np.arange(lift, stop) - lift) * A.DT
        for axis, ax in enumerate(axes[i]):
            ax.plot(t, data.att[i, lift:stop, axis] / A.DEG, color="black", lw=1.2, label="logged")
            for m, (name, F, S0, attitudes, rms) in enumerate(predicted):
                for lane in np.flatnonzero((F == i) & (S0 + horizon < stop)):
                    s = S0[lane]
                    run = np.concatenate([[data.att[i, s, axis]], attitudes[:, lane, axis]]) / A.DEG
                    ax.plot((np.arange(s, s + horizon + 1) - lift) * A.DT, run, color=colors[m], lw=0.9, alpha=0.8, label=f"{name} ({args.horizon:g} s runs; RMS {rms[axis]:.1f}° at {args.horizon:g} s)" if lane == np.flatnonzero(F == i)[0] else None)
            ax.set_title(f"{record['name']} {'roll' if axis == 0 else 'pitch'}", fontsize=10)
            ax.set_ylabel("degrees")
            ax.grid(alpha=0.3)
            if i == 0:
                ax.legend(fontsize=8, loc="upper right")
    for ax in axes[-1]:
        ax.set_xlabel("seconds since liftoff")
    fig.suptitle(f"Closed-loop free runs of the airframe model (the firmware controller flying it on the logged references) from the logged state every {args.every:g} s, against the logged attitude", fontsize=11)
    fig.savefig(args.out, dpi=110)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
