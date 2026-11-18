"""
The robust burst check (sw/sim/tune_flip.py --check --bands: 5 rotor_momentum bands over 0-0.05, every other untested term drawn as flip_firmware.uncertain, from the 5 m hold) on the 2026-11-14 ESCs: uv run -m sw.sim.flip_esc CASE [--kind burst30|burst60|burst90] [--sag] [--n 256] [--workers 90] [--out results.json]

The plant's motors are the thrust stand's (STAND: one motor with its 10-inch prop on the Nuofany 40A calibrated to 1000-2000 us, sw/recordings/stand/2026-11-14T18-07-54, a fresh 4S sagging from 16.3 to 15.3 V; per motor, against the drone's throttle 0-1200) with no ceiling (the table's top is measured now), both motors alike within 3% (the stand found the two pairs within about 2%), while the firmware flies on its own table (flight_math.c) at the drone's thrust scales 1.3 and hover_mass 1.02, as set on 2026-11-14 (SETTINGS, the GPS hold 5 m up as tune_flip.HIGH). Today's 8-10% hover split, left high (2026-11-15T00-19-08 to 00-28-51 UTC), is a CoM offset to the left (COM_LEFT, (L - R) / (L + R) = offset / arm); on this plant the sim's hold needs 641/586 against the flights' 674/607 to 725/647 on a sagging pack. roll_bias is today's 12.87 scaled with the offset (the auto-trim's).

Cases: now (as the drone is), rebalanced (the CoM centered), battery15 (the battery 15 mm right: BATTERY of the mass moves 15 mm), now_left (now with the old weak-left draw on top), now_cal and table_only (the firmware's scales calibrated to each plant at hover, as the robust tuning had them; table_only also centered), and any case with _s12 (thrust scales 1.2 and hover_mass 1.05: what the stand and today's hovers give at hover thrust). For the proposed firmware table, run the same cases in a tree with flight_math.c's table replaced (sw/sim/models/NOTES.md "The new ESCs").
"""

import argparse
import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording, settings as recording_settings
from sw.sim import flip_firmware as FF
from sw.sim import tune_flip as T

# The stand (2026-11-14T18-07-54, the median of each 2 s step's last 0.8 s; the load cell lags about 0.16 s): drone throttle (the stand's times 1.2), thrust per motor (N) and the pack under that load (V).
STAND = np.array([[0, 0.0], [120, 0.575], [240, 1.400], [360, 2.372], [480, 3.438], [600, 4.975], [720, 6.875], [840, 8.973], [960, 11.014], [1080, 13.090], [1200, 15.484]])
STAND_VOLTS = [16.34, 16.29, 16.29, 16.25, 16.19, 16.14, 16.03, 15.89, 15.74, 15.56, 15.33]
# The pack's sag under the stand's load (--sag): V0 - k T^1.5 per motor's thrust T (N), fitted to STAND_VOLTS (0.013 V RMS): 16.31 V and 0.0160 V per N^1.5.
SAG = tuple(float(v) for v in np.polyfit(STAND[:, 1] ** 1.5, STAND_VOLTS, 1)[::-1] * (1, -1))
ARM, MASS = 0.172, 1.05
WEIGHT = MASS * 9.81
COM_LEFT = 0.0155
# The pack's share of the mass (about 0.25 kg; not weighed): a battery moved 15 mm moves the CoM 3.6 mm.
BATTERY = 0.25 / MASS
CASES = {"now": COM_LEFT, "rebalanced": 0.0, "battery15": COM_LEFT - 0.015 * BATTERY, "now_left": COM_LEFT, "now_cal": COM_LEFT, "table_only": 0.0}
# The drone's settings on 2026-11-14 (this recording's equal GET /settings at 18:00 PST).
SETTINGS = "2026-11-15T00-51-18"
ROLL_BIAS = 12.871582
# The bursts by name (flip_angle, degrees).
KINDS = {"burst30": 30.0, "burst60": 60.0, "burst90": 90.0}


def options_for(case, k, seed, band, airframe=FF.AIRFRAME):
    """The plant and firmware terms of one batch: rotor_momentum across the band and the right motor's gain spread as tune_flip.jobs does."""
    golden = (math.sqrt(5) - 1) / 2
    low, high = T.MOMENTUM_BANDS[band]
    spread, u = (seed * 0.001 + k * golden) % 1, (seed * 0.001 + k * golden * golden) % 1
    offset = CASES[case]
    right = 1 / (1 - (1 - T.LEFT_LOW) * u) if case == "now_left" else 0.97 + 0.06 * u
    # The trim's mean thrusts are the hover split the CoM offset asks for, in newtons (level 1: the stand's table is in newtons).
    split = [WEIGHT * (ARM + offset) / (2 * ARM), WEIGHT * (ARM - offset) / (2 * ARM)]
    geometry = FF.R.A.geometry_of(FF.R.A.model(airframe))
    options = {"set": {"rotor_momentum": low + (high - low) * spread, "right_thrust_gain": right, "thrust_level": 1.0, "hover_thrust": split, "motor_ceiling": 0.0},
               "table": STAND, "geometry": {"com": [geometry["com"][0], offset, geometry["com"][2]]}, "thrust_scale": 1.3, "hover_mass": 1.02}
    if case in ("now_cal", "table_only"):
        del options["thrust_scale"], options["hover_mass"]
    return options


def jobs(case, kind, n, seed, band, batch=4, params=None, extra=None, airframe=FF.AIRFRAME, sag=False):
    scaled = case.endswith("_s12")
    case = case.removesuffix("_s12")
    flown = recording_settings(recording(SETTINGS)) | FF.FLIGHT | T.HIGH | T.FAST_CLIMB | {"flip_angle": KINDS[kind], "roll_bias": ROLL_BIAS * max(CASES[case], 0) / COM_LEFT} | (extra or {})
    out = []
    for k, start in enumerate(range(0, n, batch)):
        options = options_for(case, k, seed, band, airframe) | ({"thrust_scale": 1.2, "hover_mass": 1.05} if scaled else {}) | ({"sag": SAG, "table_volts": STAND_VOLTS} if sag else {})
        # The bands' tilt offset drawn as tune_flip does, none on an airframe without the bands (models/airframe_2026-11-15_bursts.json).
        banded = any(FF.R.A.model(airframe).get(key, 0.0) for key in ("left_band", "right_band"))
        out.append((flown, T.BANDS[k % len(T.BANDS)] if banded else 0.0, seed + start, batch, 17.0, math.radians(FF.flight.GYRO_RANGE), FF.flight.ACCEL_RANGE, 7.0, True, airframe, "flip", params or {}, options))
    return out


def stats(rows):
    started = [r for r in rows if r["started"] is not None]
    a = lambda key: np.array([r[key] for r in started], float)
    rates, lost, after = np.array([r["rates"] for r in started]), a("lost"), a("after_tilt")
    return {"n": len(rows), "started": len(started), "clean": float(np.mean(a("clean"))), "touched": float(np.mean(a("touched"))), "lost_median": float(np.median(lost)), "lost_90": float(np.percentile(lost, 90)), "lost_worst": float(lost.max()),
            "rates_median": np.median(rates, 0).round(0).tolist(), "rates_90": np.percentile(rates, 90, axis=0).round(0).tolist(), "after_tilt_median": float(np.median(after)), "after_tilt_90": float(np.percentile(after, 90)),
            "climbed_median": float(np.median(a("climbed"))), "handed_median": float(np.median(a("handed"))), "handed_90": float(np.percentile(a("handed"), 90)), "past90_90": float(np.percentile(a("past90"), 90)), "full_max": float(a("full").max()), "travel_median": float(np.median(a("travel"))), "travel_90": float(np.percentile(a("travel"), 90))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case")
    parser.add_argument("--kind", choices=list(KINDS), default="burst60")
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=95000)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--set", default="", help="settings on top, name=value,...")
    parser.add_argument("--airframe", default=FF.AIRFRAME, help="the plant's airframe in sw/sim/models (airframe_2026-11-15_bursts.json: the 2026-11-15 bursts' plant, the bands off)")
    parser.add_argument("--sag", action="store_true", help="the pack sags under the motors' load as on the stand (SAG), and the firmware compensates where thrust_voltage is set (--set thrust_voltage=16.1)")
    parser.add_argument("--params", default="{}", help="flip parameters over flip_defaults, as JSON (e.g. '{\"yaw_ff\": 2.7}')")
    args = parser.parse_args()
    out = {}
    with Pool(args.workers) as pool:
        for band, (low, high) in enumerate(T.MOMENTUM_BANDS):
            rows = [r for batch in pool.map(FF.fly, jobs(args.case, args.kind, args.n, args.seed, band, params=json.loads(args.params), extra=parse_settings(args.set), airframe=args.airframe, sag=args.sag), chunksize=1) for r in batch]
            s = out[f"{low:.2f}-{high:.2f}"] = stats(rows)
            print(f"{args.case} {args.kind} rotor_momentum {low:.2f}-{high:.2f}: clean {s['clean'] * 100:.1f}%, touched {s['touched'] * 100:.1f}%, height lost 90th {s['lost_90']:.2f} m (worst {s['lost_worst']:.2f}); peak rates roll/pitch/yaw median {s['rates_median']} (90th {s['rates_90']}) deg/s; "
                  f"tilt after the hand-back {s['after_tilt_median']:.0f} deg (90th {s['after_tilt_90']:.0f}); climbed {s['climbed_median']:.2f} m; handed back {s['handed_median']:.2f} s (90th {s['handed_90']:.2f}); past 90 deg 90th {s['past90_90']:.2f} s", flush=True)
    if args.out:
        args.out.write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
