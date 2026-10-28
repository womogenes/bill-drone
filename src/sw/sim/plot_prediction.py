"""
Predicted against real attitude: uv run --with scipy --with matplotlib -m sw.sim.plot_prediction <recordings> [--airframe models/airframe_<day>.json] [--compare another.json] [--out plot.png]

Every --every seconds of each flight's airborne window, the firmware controller (with its replayed state at that sample) flies the airframe model (sw/sim/airframe.py) on the logged references alone for --horizon seconds from the logged attitude, rates and actuator states, as identify_airframe scores it. Each free run is drawn over the logged roll and pitch, so the plot shows how far the model's own closed-loop motion follows the drone's from each start. With --compare a second model's runs are drawn too. The model was fitted on these flights unless they are new ones: its held-out errors are in models/NOTES.md.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import identify_airframe as N

MODELS = Path(__file__).with_name("models")


def runs(params, data, horizon, every):
    """Free runs from every start: lanes' flights and starts, and the predicted attitudes [step, lane, axis] (radians)."""
    flights = np.arange(len(data.records))
    F, S0, valid, rates, attitudes = N.predict(params, data, flights, [horizon], every)
    return F, S0, attitudes


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
    parser.add_argument("--airframe", type=Path, default=MODELS / "airframe_2026-10-28.json")
    parser.add_argument("--compare", type=Path, help="a second model to draw alongside")
    parser.add_argument("--horizon", type=float, default=1.0, help="length of each free run (s)")
    parser.add_argument("--every", type=float, default=1.0, help="seconds between starts")
    parser.add_argument("--window", type=float, default=20.0, help="seconds of each flight to draw, from liftoff")
    parser.add_argument("--out", type=Path, default=Path("airframe_prediction.png"))
    args = parser.parse_args()
    records = [r for r in (N.load(p) for p in N.paths(args.recordings)) if r]
    data = N.Data(records)
    horizon, every = int(round(args.horizon / A.DT)), int(round(args.every / A.DT))
    models = [(args.airframe.stem, args.airframe)] + ([(args.compare.stem, args.compare)] if args.compare else [])
    predicted = []
    for name, path in models:
        params = A.complete(json.loads(path.read_text()))
        split = np.median(np.concatenate([A.thrust(r["left_throttle"][r["lift"]:r["end"]]) for r in records])) / np.median(np.concatenate([A.thrust(r["right_throttle"][r["lift"]:r["end"]]) for r in records]))
        params["right_thrust_gain"] = float(split)
        F, S0, attitudes = runs(params, data, horizon, every)
        rms = errors(data, F, S0, attitudes, horizon)
        print(f"{name}: {len(F)} runs, attitude-change error at {args.horizon:g} s roll {rms[0]:.2f}° pitch {rms[1]:.2f}°")
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
