"""
The Kalman filter (fw/drone/main/ekf.c) against the estimators it would replace (flight_math.c's attitude, hover.c's heading, velocity and height), both replayed on recorded flights: uv run -m sw.sim.ekf_replay [recordings or globs] [--defines "EKF_ACCEL_NOISE=0.05f ..."] [--outage 2 5] [--plot out.png] [--verbose]

Each GPS-held flight of drone 1 (default: 2026-11-09's, as gps_truth) runs through ekf.c (sw/sim/ekf.py replay: the IMU, the GPS solutions and the barometer as flight_rt.c would feed them) and through hover.c (sw/sim/heading_replay.py and height_replay.py). Neither flies the drone, so this scores the estimators alone on the motion the firmware of the day made. It prints per flight:
- Heading: when each pinned it (s from engaging) and the error's RMS from 10 s on against gps_truth's Alignment (the nose's heading from the GPS velocity's changes against the accelerometer's over the whole flight). The truth leans on the firmware's own attitude, as hover.c does, and is itself good to a few degrees.
- Outages: the GPS kept from both for --outage seconds at a time (every outage + 6 s from 14 s after engaging, both headings pinned), and each one's horizontal velocity at the end against the GPS velocity it did not see (m/s). This is the test neither was tuned on: without the GPS the velocity is the attitude, the heading and the accelerometer's bias integrated, so the error after T s is their combined acceleration error times T. The filter's position over the outage against the GPS position's change is printed with it, and both climb rates against the withheld GPS's (hover.c's never takes the GPS's while the barometer holds the height).
- Height: the height and climb-rate error against gps_truth's smoothed trajectory (the barometer and accelerometer, Kalman forward and RTS back) over the first 2.5 s from the motors' start and over the airborne span. That truth takes each barometer reading at 0.12 m as hover.c does, so a pressure disturbance in a hard maneuver (05:53 at 24 s: 1 m for 0.7 s, which the GPS's climb rate does not show) is in it too.
- GPS height: both heights at each GPS solution against the GPS's own height, which nothing on the drone uses, so the one height check that does not lean on the barometer. It is good to about a meter and wanders slowly, so each difference is taken less its running median over GPS_HEIGHT_S either side: what is left is how each follows height changes over a few seconds, on top of the GPS's own noise (the floor both share).
- Consistency: the filter's GPS velocity and barometer innovations over the standard deviation it expected (RMS; 1 if its uncertainty is honest), and how many GPS solutions its gate refused.
"""

import argparse
import math

import numpy as np

from sw.sim import ekf, heading_replay, height_replay
from sw.sim import gps_truth as T
from sw.sim import hover

TAKEOFF_S = 2.5
# Outages start this long after engaging (s; the slowest pinning of 2026-11-09 was 13 s) and leave this long between them for the estimators to recover.
OUTAGE_AFTER = 14.0
OUTAGE_REST = 6.0
HEADING_AFTER = 10.0
GPS_HEIGHT_S = 5.0


def rms(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return math.sqrt(np.mean(values ** 2)) if len(values) else math.nan


def wander(values, size):
    """values less their running median over size samples."""
    padded = np.pad(values, size // 2, mode="edge")
    return values - np.median(np.lib.stride_tricks.sliding_window_view(padded, size), axis=1)


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def outages(f, align, engaged, duration, library, climb):
    """The GPS withheld for duration s at a time: a list per window of (end time, the filter's velocity error, hover.c's, the filter's position error over the window), m/s and m, and the climb-rate errors against every withheld solution's (the filter's, then hover.c's from climb, which never takes the GPS's while the barometer holds the height)."""
    windows = [(start, start + duration) for start in np.arange(engaged + OUTAGE_AFTER, f.end - duration - 1, duration + OUTAGE_REST)]
    if not windows:
        return [], np.zeros((0, 2))
    out = ekf.replay(f, library, withhold=windows)
    old = heading_replay.replay(f, align, withhold=windows)
    at = dict(zip(old["index"].tolist(), range(len(old["index"]))))
    rows, vertical = [], []
    for start, end in windows:
        # The last solution inside the window, at the time it measured.
        inside = np.flatnonzero((f.gps_t >= start) & (f.gps_t < end))
        if len(inside) < 5:
            continue
        k = inside[-1]
        i = int(np.searchsorted(f.grid, f.gps_t[k] - align.latency))
        first = int(np.searchsorted(f.grid, f.gps_t[inside[0]] - align.latency))
        if i not in at or first not in at or not out["aligned"][first] or not old["aligned"][at[first]] or not out["aligned"][i]:
            continue
        gps = f.gps_u[k]
        filtered = complex(out["velocity"][i, 0], out["velocity"][i, 1])
        j = at[i]
        # hover.c's velocity is in its hold frame: the GPS's turned into it by the frame's true heading.
        held = old["replayed"][j] - gps * np.exp(-1j * old["frame"][j])
        moved = complex(*(out["position"][i, :2] - out["position"][first, :2])) - (f.gps_position[k] - f.gps_position[inside[0]])
        rows.append((end, abs(filtered - gps), abs(held), abs(moved)))
        # From half a second in, when the last solution taken has stopped counting.
        for k in inside[f.gps_t[inside] >= start + 0.5]:
            i = int(np.searchsorted(f.grid, f.gps_t[k] - align.latency))
            vertical.append((out["velocity"][i, 2] - f.gps_up[k], climb[i] - f.gps_up[k]))
    return rows, np.array(vertical).reshape(-1, 2)


def score(f, library, durations):
    align = T.Alignment(f)
    out = ekf.replay(f, library)
    old = heading_replay.replay(f, align)
    t = f.grid
    engaged = t[np.flatnonzero(f.mode > 0)[0]]
    aligned = out["aligned"] > 0
    error = np.degrees(wrap(out["yaw"] - align.theta))
    late = f.air & aligned & (t > engaged + HEADING_AFTER)
    row = {"name": f.name, "good": align.good, "engaged": engaged, "out": out, "old": old, "align": align, "error": error,
           "pinned": (t[np.argmax(aligned)] - engaged if aligned.any() else math.nan, old["t"][np.argmax(old["aligned"])] if old["aligned"].any() else math.nan),
           "heading": (rms(error[late]), rms(old["error"][old["t"] > HEADING_AFTER]))}
    # Height against the smoothed trajectory.
    s = T.smoothed(f)
    height, climb = height_replay.replay(f, hover, f.settings)
    withheld = {duration: outages(f, align, engaged, duration, library, climb) for duration in durations}
    row["outages"] = {duration: rows for duration, (rows, vertical) in withheld.items()}
    row["vertical"] = {duration: vertical for duration, (rows, vertical) in withheld.items()}
    m = len(s["t"])
    onset = int(np.searchsorted(t, f.onset))
    takeoff = slice(onset, min(m, onset + int(TAKEOFF_S / T.DT)))
    air = np.flatnonzero(f.air[:m])
    # The filter's climb rate is its vertical velocity; its height is from where it last rested.
    row["smoothed"], row["hover_height"], row["hover_climb"] = s, height, climb
    row["takeoff"] = {"ekf": (rms(out["height"][takeoff] - s["up"][takeoff]), rms(out["velocity"][takeoff, 2] - s["climb"][takeoff])),
                      "hover": (rms(height[takeoff] - s["up"][takeoff]), rms(climb[takeoff] - s["climb"][takeoff]))}
    row["flight"] = {"ekf": (rms(out["height"][air] - s["up"][air]), rms(out["velocity"][air, 2] - s["climb"][air])),
                     "hover": (rms(height[air] - s["up"][air]), rms(climb[air] - s["climb"][air]))}
    # Against the GPS's height at each solution's own moment, from 3 s after liftoff.
    k = np.flatnonzero((f.gps_t >= f.liftoff + 3) & (f.gps_t <= f.end - 1) & (f.gps_t - ekf.GPS_LATENCY < t[m - 1]) & np.isfinite(f.gps_height))
    at = np.searchsorted(t, f.gps_t[k] - ekf.GPS_LATENCY)
    size = int(2 * GPS_HEIGHT_S * 10) | 1
    row["gps_height"] = (wander(out["height"][at] - f.gps_height[k], size), wander(height[at] - f.gps_height[k], size)) if len(k) > size else (np.zeros(0), np.zeros(0))
    used = f.air & aligned
    changed = np.flatnonzero(np.any(np.diff(out["gps_innovation"], axis=0) != 0, axis=1)) + 1
    changed = changed[used[changed]]
    row["consistency"] = (rms(out["gps_innovation"][changed, :2]), rms(out["gps_innovation"][changed, 2]), rms(out["baro_innovation"][f.air]), int(out["rejected"][used].max()) if used.any() else 0)
    row["level"] = (np.degrees(np.mean(wrap(f.roll - out["roll"])[f.air])), np.degrees(np.mean(wrap(f.pitch - out["pitch"])[f.air])), out["accel_bias"][f.air].mean(axis=0), out["gyro_bias"][f.air].mean(axis=0))
    return row


def plot(rows, durations, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    old_color, new_color, ink, muted = "#b8621b", "#2a6fb0", "#222222", "#888888"
    figure, axes = plt.subplots(2, 3, figsize=(18, 9))
    for ax in axes.flat:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(alpha=0.25, linewidth=0.6)
    clock = lambda r: r["name"][11:16].replace("-", ":")

    def pairs(ax, picked, values):
        """One pair of dots per flight: (the filter's, hover.c's)."""
        x = np.arange(len(picked))
        for i, (new, old) in enumerate(values):
            ax.plot([i, i], [new, old], color=muted, linewidth=1, zorder=0)
        ax.plot(x, [old for new, old in values], "o", color=old_color, markersize=8, label="current (hover.c)")
        ax.plot(x, [new for new, old in values], "o", color=new_color, markersize=8, label="Kalman filter")
        ax.set_xticks(x, [clock(r) for r in picked], rotation=45, ha="right")
        ax.set_ylim(0, None)
        ax.legend(frameon=False)

    # Velocity error at the end of each outage, every window sorted.
    ax = axes[0, 0]
    for duration, style in zip(durations, ("-", "--")):
        windows = [w for r in rows for w in r["outages"][duration]]
        if not windows:
            continue
        share = (np.arange(len(windows)) + 0.5) / len(windows) * 100
        ax.plot(np.sort([w[2] for w in windows]), share, style, color=old_color, linewidth=2, label=f"current (hover.c), {duration:g} s without GPS")
        ax.plot(np.sort([w[1] for w in windows]), share, style, color=new_color, linewidth=2, label=f"Kalman filter, {duration:g} s without GPS")
    ax.set_xlabel("horizontal velocity error at the end of the outage (m/s)")
    ax.set_ylabel("share of outages (%)")
    ax.set_title("Velocity after losing the GPS", loc="left")
    ax.legend(frameon=False, loc="lower right")
    # One flight's velocity error through its outages: the longest duration, the flight with the most windows, from when both headings are pinned.
    ax = axes[0, 1]
    duration = durations[-1]
    pick = max((r for r in rows if r["outages"][duration]), key=lambda r: len(r["outages"][duration]), default=None)
    if pick is not None:
        f, align = pick["f"], pick["align"]
        out, old = pick["withheld"][duration]
        index = old["index"]
        both = (out["aligned"][index] > 0) & old["aligned"]
        pinned = f.grid[index][np.argmax(both)]
        low = next(start for start, end in pick["windows"][duration] if start > pinned + duration) - 3
        windows = [w for w in pick["windows"][duration] if w[0] > low][:4]
        high = windows[-1][1] + 3
        # Every tenth tick: the GPS's own rate.
        j = np.flatnonzero((f.grid[index] > low) & (f.grid[index] < high))[::10]
        t = f.grid[index][j]
        gps = align.velocity[index][j]
        ax.plot(t, np.abs(old["replayed"][j] - gps * np.exp(-1j * old["frame"][j])), color=old_color, linewidth=2, label="current (hover.c)")
        ax.plot(t, np.abs(out["velocity"][index][j, 0] + 1j * out["velocity"][index][j, 1] - gps), color=new_color, linewidth=2, label="Kalman filter")
        for start, end in windows:
            ax.axvspan(start, end, color=muted, alpha=0.15, linewidth=0)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("horizontal velocity error against the GPS (m/s)")
        ax.set_ylim(0, None)
        ax.set_title(f"Example: {clock(pick)} UTC, GPS withheld in the shaded spans", loc="left")
        ax.legend(frameon=False)
    # Heading error per flight with a truth to judge it by.
    ax = axes[0, 2]
    good = [r for r in rows if r["good"]]
    pairs(ax, good, [r["heading"] for r in good])
    ax.set_ylabel(f"heading error from {HEADING_AFTER:g} s on (degrees RMS)")
    ax.set_title("Heading against the GPS truth, per flight (UTC)", loc="left")
    # Takeoff height: the flight whose current estimate was worst.
    ax = axes[1, 0]
    worst = max(rows, key=lambda r: r["takeoff"]["hover"][0] if np.isfinite(r["takeoff"]["hover"][0]) else 0)
    f, s = worst["f"], worst["smoothed"]
    k = np.flatnonzero((s["t"] > f.onset - 0.3) & (s["t"] < f.onset + 6))
    ax.plot(s["t"][k] - f.onset, s["up"][k], color=ink, linewidth=1.5, label="barometer and accelerometer, smoothed both ways")
    ax.plot(s["t"][k] - f.onset, worst["hover_height"][k], color=old_color, linewidth=2, label="current (hover.c)")
    ax.plot(s["t"][k] - f.onset, worst["out"]["height"][k], color=new_color, linewidth=2, label="Kalman filter")
    ax.set_xlabel("time from the motors' start (s)")
    ax.set_ylabel("height (m)")
    ax.set_title(f"Takeoff height, {clock(worst)} UTC", loc="left")
    ax.legend(frameon=False)
    # Height against the GPS's own height, per flight.
    ax = axes[1, 1]
    high = [r for r in rows if len(r["gps_height"][0])]
    pairs(ax, high, [(rms(r["gps_height"][0]), rms(r["gps_height"][1])) for r in high])
    ax.set_ylabel("height against the GPS's (m RMS)")
    ax.set_title(f"Height changes over {GPS_HEIGHT_S:g} s against the GPS's own height\n(used by neither; its noise is in both)", loc="left")
    # Climb rate against the GPS's while it was withheld, per flight.
    ax = axes[1, 2]
    climbing = [r for r in rows if len(r["vertical"][duration])]
    pairs(ax, climbing, [(rms(r["vertical"][duration][:, 0]), rms(r["vertical"][duration][:, 1])) for r in climbing])
    ax.set_ylabel("climb rate error (m/s RMS)")
    ax.set_title(f"Climb rate against the GPS's, withheld {duration:g} s at a time", loc="left")
    figure.tight_layout()
    figure.savefig(path, dpi=110)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="*", default=list(T.DEFAULT))
    parser.add_argument("--defines", default="", help="compile-time overrides for ekf.c, space separated")
    parser.add_argument("--outage", nargs="*", type=float, default=[2.0, 5.0], help="GPS outage lengths (s)")
    parser.add_argument("--plot")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    library = ekf.load(tuple(args.defines.split()))
    rows = []
    for path in T.flights(args.recordings):
        f = T.Flight(path)
        if not f.flown or not f.settings.get("hover_gps_tau") or not len(f.gps_t):
            continue
        row = score(f, library, args.outage)
        row["f"] = f
        if args.plot:
            # The withheld replays again for the example trace.
            row["windows"] = {d: [(start, start + d) for start in np.arange(row["engaged"] + OUTAGE_AFTER, f.end - d - 1, d + OUTAGE_REST)] for d in args.outage}
            row["withheld"] = {d: (ekf.replay(f, library, withhold=w), heading_replay.replay(f, row["align"], withhold=w)) for d, w in row["windows"].items() if w}
        rows.append(row)
        gaps = "  ".join(f"{d:g} s: {np.median([w[1] for w in ws]):.2f}/{np.median([w[2] for w in ws]):.2f} worst {max(w[1] for w in ws):.2f}/{max(w[2] for w in ws):.2f} moved {np.median([w[3] for w in ws]):.1f} m ({len(ws)})" if ws else f"{d:g} s: none" for d, ws in row["outages"].items())
        print(f"{f.name[11:19]}{'' if row['good'] else ' (truth poor)'}: pinned {row['pinned'][0]:4.1f}/{row['pinned'][1]:4.1f} s, heading {row['heading'][0]:4.1f}/{row['heading'][1]:4.1f}°; outage velocity {gaps}; "
              f"takeoff height {row['takeoff']['ekf'][0]:.2f}/{row['takeoff']['hover'][0]:.2f} m climb {row['takeoff']['ekf'][1]:.2f}/{row['takeoff']['hover'][1]:.2f} m/s, flight {row['flight']['ekf'][0]:.2f}/{row['flight']['hover'][0]:.2f} m {row['flight']['ekf'][1]:.2f}/{row['flight']['hover'][1]:.2f} m/s; "
              f"innovations {row['consistency'][0]:.2f} {row['consistency'][1]:.2f} baro {row['consistency'][2]:.2f}, refused {row['consistency'][3]}", flush=True)
        if args.verbose:
            roll, pitch, accel, gyro = row["level"]
            print(f"    logged attitude less the filter's: roll {roll:+.1f}° pitch {pitch:+.1f}°; accelerometer bias {accel.round(2)} m/s², gyro bias {(gyro * 1000).round(1)} mrad/s")
    print("\n(each pair: Kalman filter / hover.c)")
    good = [r for r in rows if r["good"]]
    print(f"heading from {HEADING_AFTER:g} s, RMS median over {len(good)} flights with a good truth: {np.median([r['heading'][0] for r in good]):.1f}/{np.median([r['heading'][1] for r in good]):.1f}°, worst {max(r['heading'][0] for r in good):.1f}/{max(r['heading'][1] for r in good):.1f}°")
    for d in args.outage:
        windows = [w for r in rows for w in r["outages"][d]]
        if windows:
            e, h, m = (np.array([w[i] for w in windows]) for i in (1, 2, 3))
            print(f"{d:g} s without the GPS, {len(windows)} outages: velocity error median {np.median(e):.2f}/{np.median(h):.2f} m/s, 90th percentile {np.percentile(e, 90):.2f}/{np.percentile(h, 90):.2f}, worst {e.max():.2f}/{h.max():.2f}; the filter's position off by {np.median(m):.1f} m median, {m.max():.1f} worst")
            v = np.concatenate([r["vertical"][d] for r in rows])
            print(f"    climb rate against the withheld GPS's, {len(v)} solutions: {rms(v[:, 0]):.2f}/{rms(v[:, 1]):.2f} m/s RMS")
    for span in ("takeoff", "flight"):
        print(f"{span}: height error RMS median {np.nanmedian([r[span]['ekf'][0] for r in rows]):.2f}/{np.nanmedian([r[span]['hover'][0] for r in rows]):.2f} m, worst {np.nanmax([r[span]['ekf'][0] for r in rows]):.2f}/{np.nanmax([r[span]['hover'][0] for r in rows]):.2f}; "
              f"climb rate {np.nanmedian([r[span]['ekf'][1] for r in rows]):.2f}/{np.nanmedian([r[span]['hover'][1] for r in rows]):.2f} m/s, worst {np.nanmax([r[span]['ekf'][1] for r in rows]):.2f}/{np.nanmax([r[span]['hover'][1] for r in rows]):.2f}")
    high = [np.concatenate([r["gps_height"][i] for r in rows]) for i in (0, 1)]
    print(f"height changes against the GPS's own height (less its {GPS_HEIGHT_S:g} s running median), {len(high[0])} solutions: {rms(high[0]):.2f}/{rms(high[1]):.2f} m RMS")
    if args.plot:
        plot(rows, args.outage, args.plot)
        print("wrote", args.plot)


if __name__ == "__main__":
    main()
