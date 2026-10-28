"""
The camera hover, sim against real: uv run --with scipy --with matplotlib -m sw.sim.plot_hover <recordings> [--seeds 4] [--out plot.png]

Each recording's camera hover (its own settings, tracking the tag) next to the sim (sweep.run with hover=True on IDENTIFIED_ESC, the identified airframe with its calibrated disturbances) flying the same settings for --seconds. A random disturbance cannot be replayed sample for sample, so the plot compares what the disturbances should reproduce: the traces' size and pace (top: range error and sideways wander, real and one sim run), their spectra and the rates' spectra (bottom: real per flight, sim averaged over --seeds runs), with each panel's RMS (the sim's the median over its runs). Both positions are the hover's own camera-based estimate, so both carry the camera's noise and lag.
"""

import argparse
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.signal import welch

from sw.fit import settings
from sw.flight_log import read_flight
from sw.sim.sweep import IDENTIFIED_ESC, run

FS = 100.0
SETTLE = 4.0
LANDING = 3.0


def real(path):
    """The tracked part of a camera hover: range error and sideways place (m), and pitch and roll rate (deg/s), at the control rate."""
    d, _ = read_flight(path)
    on = np.flatnonzero(d.base_throttle.values > 0)[0]
    flags = d["flags"].values.astype(int)
    age = np.nan_to_num(d.hover_tag_age_ms.values, nan=65535)
    k = np.arange(len(d))
    ok = (k > on + int(SETTLE * FS) + 300) & ((flags >> 1 & 3) == 1) & (age < 120)
    if not ok.any():
        return None
    first, last = np.flatnonzero(ok)[[0, -1]]
    # Not the landing at the end.
    last -= int(LANDING * FS)
    span = slice(first, last + 1)
    x, y = d.hover_x_m.values[span], d.hover_y_m.values[span]
    reach = np.hypot(x, y)
    side = np.radians(((np.degrees(np.arctan2(y, x)) - d.hover_arc_deg.values[span] + 180) % 360) - 180) * reach
    series = dict(range=reach - d.hover_range_target_m.values[span], side=side, pitch_rate=d.gy_deg_s.values[span], roll_rate=d.gx_deg_s.values[span])
    return {key: np.interp(np.arange(len(v)), np.flatnonzero(~np.isnan(v)), v[~np.isnan(v)]) for key, v in series.items()}


def sim(job):
    path, seed, seconds = job
    s = settings(path) | {"hover_distance": 2.0}
    log = []
    result = run(s, IDENTIFIED_ESC, seconds=seconds, seed=seed, log=log, hover=True)
    if not result["stable"]:
        return None
    t = np.array([e[0] for e in log]) - result["liftoff_s"]
    keep = t > SETTLE
    angles = np.array([e[2] for e in log])[keep]
    # The hover's own estimate, as the drone logs hover_x_m/hover_y_m against hover_range_target_m and hover_arc_deg: like the real traces, with the camera's noise and lag.
    estimate = [e[8] for e, k in zip(log, keep) if k]
    x = np.array([e[0][0] for e in estimate])
    y = np.array([e[0][1] for e in estimate])
    reach = np.hypot(x, y)
    side = (np.angle(np.exp(1j * (np.arctan2(y, x) - np.array([e[1] for e in estimate]))))) * reach
    rate = np.degrees(np.diff(angles, axis=0) * FS)
    return dict(range=reach - np.array([e[2] for e in estimate]), side=side, pitch_rate=np.append(rate[:, 1], rate[-1, 1]), roll_rate=np.append(rate[:, 0], rate[-1, 0]))


def spectrum(series):
    f, power = welch(series - series.mean(), fs=FS, nperseg=min(len(series), 1024))
    return f, power


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--seeds", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--out", type=Path, default=Path("hover_prediction.png"))
    args = parser.parse_args()
    flights = [(path, r) for path in args.recordings if (r := real(path)) is not None]
    with Pool(args.workers) as pool:
        sims = pool.map(sim, [(path, seed, args.seconds) for path, _ in flights for seed in range(args.seeds)])
    sims = [[s for s in sims[i * args.seeds:(i + 1) * args.seeds] if s is not None] for i in range(len(flights))]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keys = (("range", "range error (m)"), ("side", "sideways (m)"), ("pitch_rate", "pitch rate (deg/s)"), ("roll_rate", "roll rate (deg/s)"))
    fig = plt.figure(figsize=(15, 4.0 * len(flights) + 4.5), constrained_layout=True)
    grid = fig.add_gridspec(len(flights) + 1, 4)
    for i, ((path, r), runs) in enumerate(zip(flights, sims)):
        for j, (key, label) in enumerate(keys[:2]):
            ax = fig.add_subplot(grid[i, 2 * j:2 * j + 2])
            ax.plot(np.arange(len(r[key])) / FS, r[key], color="black", lw=1, label=f"real {path.name[11:19]} (RMS {r[key].std():.2f}, mean {r[key].mean():+.2f})")
            if runs:
                spread = [run[key].std() for run in runs]
                s = runs[int(np.argsort(spread)[len(spread) // 2])][key]
                ax.plot(np.arange(len(s)) / FS, s, color="tab:red", lw=1, alpha=0.8, label=f"sim, same settings, median run (RMS {np.median(spread):.2f}; runs {min(spread):.2f}-{max(spread):.2f})")
            ax.set_title(f"{path.name[:19]}: {label}", fontsize=10)
            ax.set_xlabel("seconds tracking")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8, loc="upper right")
    for j, (key, label) in enumerate(keys):
        ax = fig.add_subplot(grid[len(flights), j])
        for (path, r), runs in zip(flights, sims):
            f, power = spectrum(r[key])
            ax.loglog(f[1:], power[1:], color="black", lw=1, alpha=0.8, label=f"real {path.name[11:19]}")
            if runs:
                fs, ps = zip(*(spectrum(run[key]) for run in runs))
                ax.loglog(fs[0][1:], np.mean([p[1:len(fs[0])] for p in ps], 0), color="tab:red", lw=1, alpha=0.8, label=f"sim {path.name[11:19]} settings")
        ax.set_title(f"spectrum: {label}", fontsize=10)
        ax.set_xlabel("Hz")
        ax.grid(alpha=0.3, which="both")
        if j == 0:
            ax.legend(fontsize=7)
    fig.suptitle("Camera hover, real against the identified sim flying each flight's own settings (the disturbances are random, so compare size, pace and spectra, not the samples)", fontsize=11)
    fig.savefig(args.out, dpi=100)
    print(f"saved {args.out}")
    for (path, r), runs in zip(flights, sims):
        print(path.name[:19] + "  " + "  ".join(f"{key} real {r[key].std():.2f} sim {np.median([run[key].std() for run in runs]):.2f}" for key, _ in keys))


if __name__ == "__main__":
    main()
