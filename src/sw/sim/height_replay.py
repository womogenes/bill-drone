"""
The hover's height and climb-rate estimate replayed open loop on recorded flights: uv run -m sw.sim.height_replay [recordings or globs] [--source path/to/hover.c] [--defines "BARO_SETTLE_S=0.5f ..."] [--set name=value ...] [--verbose]

Written 2026-11-09 for the GPS-hold takeoffs that climbed to 1.15-1.62 m against a 0.53-0.77 m target (sw/sim/gps_truth.py section 7). Each flight's hover.c (the firmware's, or --source with --defines) is fed what the drone fed it, tick by tick on gps_truth's 100 Hz grid: the vertical specific force, the attitude, the drag model's inputs, the GPS solutions (dated as flight_rt.c gps_input dates them) and the barometer's readings (each new one once, 75 ms before it was logged); hover_rest while the motors are off. The estimate cannot steer the replayed flight (the thrust flown is the recorded one), so this measures the estimator alone, against gps_truth's smoothed trajectory (the barometer and accelerometer, Kalman forward and RTS back).

Per takeoff it prints the estimate's height and climb-rate error against the smoothed truth over the first 2.5 s from the motors' start (RMS and worst), and the estimate when the true climb peaks; per landing, the same over the last 2 s before the cut.
"""

import argparse
import math

import numpy as np

from sw.sim import gps_truth as T
from sw.sim.landing_test import install

TAKEOFF_S = 2.5
LANDING_S = 2.0
# flight_rt.c: GPS_LATENCY_US and BARO_LATENCY_US, as the firmware dates the fixes and readings.
GPS_LATENCY = 0.06
BARO_LATENCY = 0.075


def replay(f, hover, settings):
    """hover.c's height and climb rate on every grid tick of f (NaN while it rests), as the drone would have estimated them."""
    gains = hover.gains(settings)
    state, out = hover.State(), hover.Output()
    n = len(f.grid)
    height, climb = np.full(n, np.nan), np.full(n, np.nan)
    # A takeoff engages with the motors off (flight_rt.c passes the collective before the hover's first tick, zero on the floor); once engaged hover.c uses its own.
    # GPS fixes by the grid tick they arrive on; north and east from the first fix (m), up from the drone's vertical velocity sign.
    arrive = np.searchsorted(f.grid, f.gps_t)
    position = np.column_stack([f.gps_position.real, -f.gps_position.imag])
    fixes = {}
    for k, i in enumerate(arrive):
        fixes.setdefault(int(i), k)
    raw = np.array(f.baro)
    previous = None
    thrust = np.where(f.left > 0, f.thrust_z, 0.0)
    for i in range(n):
        now = f.grid[i]
        reading = None
        if np.isfinite(raw[i]) and raw[i] != previous:
            previous = raw[i]
            reading = hover.Baro(capture_us=int((now - BARO_LATENCY) * 1e6), altitude=float(raw[i]))
        vertical = float(f.up[i] / T.G)
        if f.mode[i] == 0 or not f.armed[i]:
            hover.library.hover_rest(state, vertical, bool(f.left[i] > 0), T.DT, reading if reading else None)
            continue
        fix = None
        if i in fixes:
            k = fixes[i]
            fix = hover.Gps(capture_us=int((f.gps_t[k] - GPS_LATENCY) * 1e6), velocity=(f.gps_u[k].real, -f.gps_u[k].imag, f.gps_up[k]), position=(position[k, 0], position[k, 1], 0.0))
        drag = hover.Drag(specific=tuple(float(v) for v in f.specific[i]), tilt=float(f.tilt[i]))
        heading_rate = float((f.gyro[i, 1] * math.sin(f.roll[i]) + f.gyro[i, 2] * math.cos(f.roll[i])) / math.cos(f.pitch[i]))
        hover.library.hover_update(state, gains, int(f.mode[i]), None, int(now * 1e6), T.DT, float(f.roll[i]), float(f.pitch[i]), heading_rate, vertical, float(thrust[i]) if state.engaged else 0.0,
                                   drag, None, fix if gains.gps_tau > 0 else None, reading if reading else None, out)
        height[i], climb[i] = state.height, state.climb_rate
    return height, climb


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="*", default=list(T.DEFAULT))
    parser.add_argument("--source", help="hover.c to replay (default the firmware's)")
    parser.add_argument("--defines", default="", help="compile-time overrides for --source, space separated")
    parser.add_argument("--set", nargs="*", default=[], help="settings to override, name=value")
    parser.add_argument("--verbose", action="store_true", help="print each takeoff's estimate and truth every 0.1 s")
    args = parser.parse_args()
    from sw.sim import hover

    install(args.source, args.defines.split())
    overrides = {name: float(value) for name, value in (pair.split("=") for pair in args.set)}
    rows = []
    for path in T.flights(args.recordings):
        f = T.Flight(path)
        if not f.flown:
            continue
        s = T.smoothed(f)
        settings = f.settings | overrides
        height, climb = replay(f, hover, settings)
        m = len(s["t"])
        onset = np.searchsorted(f.grid, f.onset)
        if f.mode[min(onset + 10, len(f.mode) - 1)] == 0:
            continue
        window = slice(onset, min(m, onset + int(TAKEOFF_S / T.DT)))
        dh, dc = height[window] - s["up"][window], climb[window] - s["climb"][window]
        peak = window.start + int(np.argmax(s["climb"][window]))
        # The landing: the last LANDING_S before the end of flight, if the hover was landing.
        end = m - 1
        landing = slice(max(onset, end - int(LANDING_S / T.DT)), end)
        landed = f.mode[end - 5] in (2, 3)
        lh = height[landing] - s["up"][landing] if landed else np.array([np.nan])
        rows.append((f.name, settings.get("hover_gps_tau", 0), settings.get("hover_baro_tau", 0), np.sqrt(np.nanmean(dh ** 2)), np.nanmax(np.abs(dh)), np.sqrt(np.nanmean(dc ** 2)), np.nanmax(np.abs(dc)),
                     s["climb"][peak], climb[peak], s["up"][peak], height[peak], np.nanmean(lh), np.nanmin(lh), np.nanmax(lh)))
        if args.verbose:
            print(f.name)
            for i in range(window.start, window.stop, 10):
                print("  %5.2f s  truth %6.3f m %6.3f m/s  estimate %6.3f m %6.3f m/s  logged %6.3f m %6.3f m/s  baro %6.3f" % (
                    f.grid[i] - f.onset, s["up"][i], s["climb"][i], height[i], climb[i], f.hover_height[i], f.hover_climb[i], f.baro[i] - s["baro_offset"][i]))
    print("%-19s %4s %4s | takeoff %.1f s: height err rms max | climb err rms max | at the climb peak: truth, estimate (m/s); truth, estimate (m) | landing last %.0f s height err mean min max" % ("flight", "gps", "baro", TAKEOFF_S, LANDING_S))
    for r in rows:
        print("%-19s %4.1f %4.1f | %5.2f %5.2f | %5.2f %5.2f | %5.2f %5.2f; %5.2f %5.2f | %5.2f %5.2f %5.2f" % r)
    a = np.array([r[3:] for r in rows], float)
    print("mean%s | %5.2f %5.2f | %5.2f %5.2f | %5.2f %5.2f; %5.2f %5.2f | %5.2f %5.2f %5.2f" % (" " * 25, *np.nanmean(a, 0)))


if __name__ == "__main__":
    main()
