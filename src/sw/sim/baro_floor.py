"""
The barometer near the floor on drone 1's 2026-11-09 outdoor flights: uv run -m sw.sim.baro_floor [recordings or globs] [--plot out.png]

Against gps_truth's smoothed height (the accelerometer integrated, held to the barometer only loosely near the floor and to the floor before takeoff), the barometer's error (its reading less the smoother's offset, less the height) binned by the true height for the takeoff (from TAKEOFF_FROM to TAKEOFF_TO s after liftoff) and the rest of the flight, and by the time since liftoff. Landings are measured against the floor they came down on instead (the smoother has nothing but the barometer to hold their end to): the barometer less the floor read with the motors off after the contact, less the height the accelerometer gives back from the contact, binned by that height; and per landing how far the floor moved from the takeoff's, the error over the last 0.3 s before contact, and what the hover's height estimate (the barometer less the takeoff floor) read at contact.
"""

import argparse

import numpy as np

from sw.sim import gps_truth as T

TAKEOFF_FROM, TAKEOFF_TO = 0.0, 3.0
LANDING_S = 3.0
HEIGHTS = np.array([-0.05, 0.05, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.5, 2.5])
SINCE = np.arange(0.0, 3.01, 0.2)
# Landings: from REACH s before the end of flight, the contact is the first vertical specific force past CONTACT_G (g), and the floor is read FLOOR_AFTER s after it.
REACH = 2.5
CONTACT_G = 1.4
FLOOR_AFTER = (0.5, 2.5)
MAX_DRIFT = 2.0


def errors(paths):
    """Per flight: the grid times since liftoff and before the end, the smoothed height and climb, and the barometer's error."""
    out = []
    for path in T.flights(paths):
        f = T.Flight(path)
        if not f.flown:
            continue
        s = T.smoothed(f)
        n = len(s["t"])
        error = s["baro"] - s["baro_offset"] - s["up"]
        since = f.grid[:n] - f.liftoff
        before = f.end - f.grid[:n]
        landing = f.mode[n - 5] in (2, 3) and f.stop == "Landed"
        flight = {"name": f.name, "since": since, "before": before, "up": s["up"], "climb": s["climb"], "error": error, "landing": landing, "spinning": f.left[:n] > 0}
        if landing:
            # The smoother has nothing to hold the end of a landing to but the barometer itself, so the landing is measured against the floor instead: the barometer read with the motors off FLOOR_AFTER s after the cut, and the height the accelerometer gives back from the contact (the first jolt past CONTACT_G from REACH s before the end), its climb rate from the smoother's REACH s before the end.
            end = np.searchsorted(f.grid, f.end)
            k0 = max(0, n - int(REACH / T.DT))
            jolts = np.flatnonzero(f.up[k0:end + int(0.5 / T.DT)] / T.G > CONTACT_G)
            contact = k0 + (jolts[0] if len(jolts) else end - k0)
            climb = s["climb"][k0] + np.cumsum((f.up[k0:contact] - T.G - s["bias_up"][k0]) * T.DT)
            height = np.cumsum(climb[::-1])[::-1] * T.DT
            after = (f.grid > f.grid[contact] + FLOOR_AFTER[0]) & (f.grid < f.grid[contact] + FLOOR_AFTER[1]) & (f.left == 0)
            floor = np.nanmean(f.baro[after]) if after.any() else np.nan
            ahead = f.grid[k0:contact] - f.grid[contact]
            flight |= {"floor_drift": floor - s["baro_offset"][0], "to_contact": ahead, "height_back": -height, "floor_error": f.baro[k0:contact] - floor - (-height)}
        out.append(flight)
    return out


def binned(x, y, edges):
    rows = []
    for low, high in zip(edges[:-1], edges[1:]):
        pick = (x >= low) & (x < high) & np.isfinite(y)
        rows.append((low, high, pick.sum(), np.median(y[pick]) if pick.any() else np.nan, np.percentile(y[pick], 10) if pick.any() else np.nan, np.percentile(y[pick], 90) if pick.any() else np.nan))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="*", default=list(T.DEFAULT))
    parser.add_argument("--plot", help="write a figure here (PNG)")
    args = parser.parse_args()
    flights = errors(args.recordings)
    phases = {}
    for name, pick in (("takeoff", lambda e: (e["since"] >= TAKEOFF_FROM) & (e["since"] < TAKEOFF_TO)),
                       ("cruise", lambda e: (e["since"] >= TAKEOFF_TO) & ((e["before"] >= LANDING_S) | (not e["landing"])))):
        up = np.concatenate([e["up"][pick(e)] for e in flights])
        error = np.concatenate([e["error"][pick(e)] for e in flights])
        phases[name] = (up, error)
    # A floor read more than MAX_DRIFT from the takeoff's is a glitch (05:41: 12 m).
    landings = [e for e in flights if e["landing"] and abs(e["floor_drift"]) < MAX_DRIFT]
    phases["landing (against the floor after)"] = (np.concatenate([e["height_back"] for e in landings]), np.concatenate([e["floor_error"] for e in landings]))
    for name, (up, error) in phases.items():
        print("%s: barometer error (m) by height: median [p10, p90] (samples)" % name)
        for low, high, count, median, p10, p90 in binned(up, error, HEIGHTS):
            if count:
                print("  %5.2f-%4.2f m: %+5.2f [%+5.2f, %+5.2f] (%d)" % (low, high, median, p10, p90, count))
    since = np.concatenate([e["since"] for e in flights])
    error = np.concatenate([e["error"] for e in flights])
    print("after liftoff: barometer error (m) by time since liftoff")
    for low, high, count, median, p10, p90 in binned(since, error, SINCE):
        print("  %4.1f-%3.1f s: %+5.2f [%+5.2f, %+5.2f]" % (low, high, median, p10, p90))
    print("landings: the floor after less the takeoff's (m); the barometer against the floor less the height over the last 0.3 s before contact (m) and the height then (m); the barometer less the takeoff floor at contact (m, what the hover's height read)")
    for e in landings:
        last = e["to_contact"] > -0.3
        print("  %s: floor %+5.2f; last 0.3 s %+5.2f at %4.2f m; at contact %+5.2f" % (e["name"], e["floor_drift"], np.nanmean(e["floor_error"][last]), np.mean(e["height_back"][last]), e["floor_drift"] + e["floor_error"][-1]))
    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(1, 3, figsize=(17, 5))
        for (name, (up, err)), color in zip(phases.items(), ("C0", "C7", "C3")):
            step = 5 if len(up) > 20000 else 1
            axes[0].scatter(err[::step], up[::step], s=2, alpha=0.2, color=color)
            rows = binned(up, err, HEIGHTS)
            axes[0].plot([r[3] for r in rows], [(r[0] + r[1]) / 2 for r in rows], "o-", color=color, label=name + " (median per height bin)")
        axes[0].set_xlim(-1.5, 1.5)
        axes[0].set_ylim(-0.1, 2.5)
        axes[0].axvline(0, color="k", lw=0.5)
        axes[0].set_xlabel("barometer error (m)")
        axes[0].set_ylabel("height (m)")
        axes[0].set_title("Barometer error vs height, 2026-11-09 05:38-06:53 UTC")
        axes[0].legend(fontsize=8)
        for e in flights:
            pick = (e["since"] > -0.5) & (e["since"] < 3)
            axes[1].plot(e["since"][pick], e["error"][pick], lw=0.6, alpha=0.6)
        rows = binned(since, error, SINCE)
        axes[1].plot([(r[0] + r[1]) / 2 for r in rows], [r[3] for r in rows], "ko-", lw=2, label="median")
        axes[1].set_ylim(-2, 2)
        axes[1].set_xlabel("time since liftoff (s)")
        axes[1].set_ylabel("barometer less smoothed height (m)")
        axes[1].set_title("After liftoff")
        axes[1].legend(fontsize=8)
        for e in landings:
            axes[2].plot(e["height_back"], e["floor_error"] + e["floor_drift"], lw=0.8, label="%s (floor moved %+.2f)" % (e["name"][11:16], e["floor_drift"]))
        axes[2].set_xlim(1.2, -0.05)
        axes[2].set_ylim(-1.5, 1.5)
        axes[2].axhline(0, color="k", lw=0.5)
        axes[2].set_xlabel("height above the floor it landed on, from the accelerometer back from contact (m)")
        axes[2].set_ylabel("barometer less the takeoff floor, less that height (m)")
        axes[2].set_title("Landings: the hover's height error (floor drift + near-floor offset)")
        axes[2].legend(fontsize=6, ncol=2)
        figure.tight_layout()
        figure.savefig(args.plot, dpi=110)
        print("wrote", args.plot)


if __name__ == "__main__":
    main()
