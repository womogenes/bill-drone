"""
The GPS hold's heading estimate (fw/drone/main/hover.c take_gps) replayed on recorded flights against gps_truth's heading: uv run -m sw.sim.heading_replay [recordings or globs] [--set name=value,...] [--compare old_hover.c] [--plot out.png] [--verbose]

Each GPS-held flight of drone 1 (default: 2026-11-09's, as gps_truth) runs through hover.c tick by tick from the recording's attitude, heading rate, specific force (the level reference's body axes, as flight_rt.c drag_inputs), servo tilt, barometer and GPS solutions (dated as flight_rt.c gps_input dates them: arrival less GPS_LATENCY_US, and only those inside its accuracy masks), on its own settings (plus --set). The hold's outputs are not flown, so it replays the estimator only: the motion stays what the firmware of the day made of it. The truth is gps_truth's Alignment, the nose's heading from the GPS velocity changes against the accelerometer's over the whole flight (the gyro yaw plus a constant and a drift; the same theta gps_truth --save writes as "heading"). It prints per flight the nose's heading at engaging (clockwise from north: with hover_gps_heading 0, the first guess's error is minus this), the hold frame's heading error (the estimate less the truth, degrees) at times from engaging, when it first stays within 20 degrees, its RMS from 10 s on, and for hover.c versions that have it when the heading was first pinned (the GPS steers the hold only from then). With --compare a second hover.c (e.g. git show 4320732:src/fw/drone/main/hover.c) replays alongside as "old"; --plot draws both per flight. On the eleven GPS-held flights of 2026-11-09 the old take_gps (4320732) left the heading at its first guess (hover_gps_heading 0, so off by the nose's direction at takeoff, 20-57 degrees) on six and corrected it within 10-30 s on four; the block evidence pins it 2-13 s after engaging (median 2.3), within 17 degrees then and 2 degrees median RMS after 10 s (worst 6).
"""

import argparse
import ctypes
import math
import subprocess
from pathlib import Path

import numpy as np

from sw.sim import hover as H
from sw.sim.gps_truth import DEFAULT, FS, Alignment, Flight, flights

# flight_rt.c: GPS_LATENCY_US, GPS_MAX_SPEED_ACCURACY, GPS_MAX_ACCURACY, BARO_LATENCY_US.
GPS_LATENCY = 0.06
GPS_MAX_SPEED_ACCURACY = 0.6
GPS_MAX_ACCURACY = 8.0
BARO_LATENCY = 0.075
# Times from engaging the table samples (s), and the error that counts as acquired (degrees).
SAMPLES = (2, 5, 10, 15, 20, 30, 45, 60)
ACQUIRED = 20.0


def library_for(source):
    """hover.c at source compiled into its own library beside it (with this tree's hover.h, whose additions are all at the structures' ends), loaded with this tree's State layout."""
    target = Path(source).with_suffix(".so")
    if not target.exists() or target.stat().st_mtime < max(Path(source).stat().st_mtime, H.HEADER.stat().st_mtime):
        subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-iquote", str(H.HEADER.parent), str(source), "-o", str(target), "-lm"], check=True)
    return H.load(target, H.State)


def replay(f, align, overrides=None, library=None, withhold=()):
    """Run hover.c over the flight from engaging; returns a dict of arrays over the engaged ticks: t (s from engaging), index (their ticks on f.grid), error (degrees), sigma (degrees, NaN for an older hover.c), aligned, replayed and logged hold velocity, frame (the hold frame's true heading, rad counterclockwise from north); and nose, the true nose heading at engaging (degrees clockwise from north). GPS solutions arriving inside any (start, end) of withhold (s on the grid) are kept from it."""
    library = library or H.library
    settings = f.settings | (overrides or {})
    gains = H.gains(settings)
    state, output = H.State(), H.Output()
    engaged = np.flatnonzero(f.mode > 0)
    if not len(engaged):
        return None
    start = engaged[0]
    lt, rt = np.radians(f.left_tilt), np.radians(f.right_tilt)
    tilt = np.arctan2(f.left_n * np.sin(lt) + f.right_n * np.sin(rt), np.maximum(f.left_n * np.cos(lt) + f.right_n * np.cos(rt), 1e-6))
    rate = (f.gyro[:, 1] * np.sin(f.roll) + f.gyro[:, 2] * np.cos(f.roll)) / np.cos(f.pitch)
    vertical = f.up / 9.81
    usable = (f.gps_sacc <= GPS_MAX_SPEED_ACCURACY) & (f.gps_hacc <= GPS_MAX_ACCURACY)
    for low, high in withhold:
        usable &= ~((f.gps_t >= low) & (f.gps_t < high))
    arrivals = np.searchsorted(f.grid, f.gps_t)
    fix_at = {}
    for index in np.flatnonzero(usable):
        fix_at.setdefault(int(arrivals[index]), index)
    rows = []
    dt = 1 / FS
    for i in range(len(f.grid)):
        now_us = int(round(f.grid[i] * 1e6))
        baro = None
        if i % 2 == 0 and np.isfinite(f.baro[i]):
            baro = H.Baro(capture_us=now_us - int(BARO_LATENCY * 1e6), altitude=f.baro[i])
        if i < start:
            library.hover_rest(ctypes.byref(state), float(vertical[i]), bool(f.left[i] > 0), dt, ctypes.byref(baro) if baro else None)
            continue
        if f.mode[i] == 0 or state.landed:
            break
        gps = None
        k = fix_at.get(i)
        if k is not None:
            u, p = f.gps_u[k], f.gps_position[k]
            gps = H.Gps(capture_us=int(round((f.gps_t[k] - GPS_LATENCY) * 1e6)), velocity=(u.real, -u.imag, f.gps_up[k]), position=(p.real, -p.imag, 0.0))
        drag = H.Drag(specific=tuple(f.specific[i]), tilt=float(tilt[i]))
        force = state.force if state.engaged else float(f.thrust_z[i])
        library.hover_update(ctypes.byref(state), ctypes.byref(gains), int(f.mode[i]), None, now_us, dt, float(f.roll[i]), float(f.pitch[i]), float(rate[i]),
                             float(vertical[i]), force, ctypes.byref(drag), None, ctypes.byref(gps) if gps else None, ctypes.byref(baro) if baro else None, ctypes.byref(output))
        new = library is H.library
        rows.append((f.grid[i], state.yaw, state.heading, complex(state.velocity[0], state.velocity[1]), math.degrees(state.heading_sigma) if new else math.nan, state.aligned if new else True))
    t, yaw, heading, velocity, sigma, aligned = (np.array(column) for column in zip(*rows))
    index = np.searchsorted(f.grid, t)
    # The hold frame's x axis in the world, counterclockwise from north (gps_truth's theta is the nose's): the truth's clockwise heading is its negative.
    frame = align.theta[index] - yaw
    error = np.degrees((heading + frame + math.pi) % (2 * math.pi) - math.pi)
    nose = math.degrees(-align.theta[index[0]]) % 360
    return {"t": t - t[0], "index": index, "frame": frame, "error": error, "sigma": sigma, "aligned": aligned.astype(bool), "replayed": velocity, "logged": f.hover_v[index], "nose": nose}


def summary(r):
    """The error at SAMPLES, the time it first stays within ACQUIRED, its RMS from 10 s on, and when the heading was first pinned (NaN if never)."""
    t, error = r["t"], r["error"]
    picks = [f"{error[np.searchsorted(t, s)]:+.0f}" if s < t[-1] else "" for s in SAMPLES]
    outside = np.flatnonzero(np.abs(error) > ACQUIRED)
    settled = 0.0 if not len(outside) else (t[outside[-1] + 1] if outside[-1] + 1 < len(t) else math.nan)
    late = t >= 10
    rms = math.sqrt(np.mean(error[late] ** 2)) if late.any() else math.nan
    pinned = t[np.argmax(r["aligned"])] if r["aligned"].any() and np.isfinite(r["sigma"][0]) else math.nan
    return picks, settled, rms, pinned


def plot(results, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    count = len(results)
    columns = 3
    rows = math.ceil(count / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(5 * columns, 2.8 * rows), squeeze=False, sharey=True)
    for ax, (name, versions) in zip(axes.flat, results.items()):
        for version, r in versions.items():
            style = {"old": dict(color="#888888", linestyle="--"), "new": dict(color="#1f6fd1")}[version]
            ax.plot(r["t"], r["error"], label=f"{version} heading error", **style)
            if version == "new" and np.isfinite(r["sigma"][0]):
                ax.fill_between(r["t"], -180, 180, where=~r["aligned"], color="#f2b8a0", alpha=0.35, linewidth=0, label="new: not pinned (GPS not steering)")
                ax.plot(r["t"], np.minimum(r["sigma"], 180), color="#d1731f", linewidth=0.8, label="new: evidence sd")
        ax.axhline(0, color="black", linewidth=0.5)
        ax.set_title(f"{name} (nose {next(iter(versions.values()))['nose']:.0f} deg cw of N at engage)", fontsize=9)
        ax.set_ylim(-90, 90)
        ax.set_xlabel("s from engaging")
        ax.grid(alpha=0.3)
    for ax in axes.flat[count:]:
        ax.axis("off")
    axes[0, 0].set_ylabel("deg")
    axes[0, 0].legend(fontsize=7, loc="lower right")
    figure.suptitle("Hold frame heading error, replayed through hover.c (estimate less gps_truth)", fontsize=11)
    figure.tight_layout()
    figure.savefig(path, dpi=110)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="*", default=list(DEFAULT))
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--compare", type=Path, help="another hover.c to replay alongside (old)")
    parser.add_argument("--plot", type=Path, help="write the per-flight error plot here (PNG)")
    parser.add_argument("--verbose", action="store_true", help="the error every 2 s")
    args = parser.parse_args()
    overrides = {k: float(v) for k, v in (item.split("=") for item in args.set.split(",") if item)}
    old = library_for(args.compare) if args.compare else None
    print(" | ".join(["flight", "version", "engaged s", "nose cw of N deg", "logged vs replay rms m/s"] + [f"{s}s" for s in SAMPLES] + ["within 20 from s", "rms after 10 s", "pinned at s"]))
    results, totals = {}, {}
    for f in (Flight(path) for path in flights(args.recordings)):
        if not f.has_gps() or f.settings.get("hover_gps_tau", 0) <= 0:
            continue
        a = Alignment(f)
        if not a.good:
            continue
        for name, library in (("old", old), ("new", None)) if old else (("new", None),):
            r = replay(f, a, overrides, library)
            if r is None:
                continue
            results.setdefault(f.time, {})[name] = r
            picks, settled, rms, pinned = summary(r)
            mismatch = math.sqrt(np.mean(np.abs(r["replayed"] - r["logged"]) ** 2))
            print(" | ".join([f.time, name, f"{r['t'][-1]:.0f}", f"{r['nose']:.0f}", f"{mismatch:.2f}", *picks, f"{settled:.1f}", f"{rms:.0f}", f"{pinned:.1f}"]), flush=True)
            totals.setdefault(name, []).append(rms)
            if args.verbose:
                t = r["t"]
                print("   ", " ".join(f"{s:.0f}:{r['error'][np.searchsorted(t, s)]:+.0f}" + (f"/{r['sigma'][np.searchsorted(t, s)]:.0f}" if np.isfinite(r["sigma"][0]) else "") for s in np.arange(0, t[-1], 2.0)))
    for name, rms in totals.items():
        print(f"{name}: heading error rms after 10 s, median over flights {np.nanmedian(rms):.0f} deg, worst {np.nanmax(rms):.0f}")
    if args.plot:
        plot(results, args.plot)
        print(f"Plot: {args.plot}")


if __name__ == "__main__":
    main()
