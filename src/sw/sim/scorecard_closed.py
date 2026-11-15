"""
The scorecard's closed-loop check (sw/sim/scorecard.py has the open-loop one): a recorded flight's settings flown again by the firmware's own C (sw/sim/flip_firmware.py fly: the attitude estimate, the controller, the hold, the guards, the tip cut) on a plant, from the ground through the takeoff and the hold, and the drone's measures set against the log's: whether the tip cut fired and how long after liftoff, the pitch rate's dominant frequency and its RMS in the shake band, the largest pitch: uv run -m sw.sim.scorecard_closed [--plant airframe_2026-11-12_raised.json ...] [--flights NAME ...] [--set k=v,...] [--n 4] [--workers 8] [--json out.json] [--plot out.png]

The firmware is this tree's. For flights flown with quaternion 0 and no flip, and for the quaternion hovers, it flies as the drone's did (william/burst-repro changed only the flip's and the recovery's quaternion authority and the bursts' climb: make flight-test BASE=b466dbf matches with the flip off), so these reruns are as flown; the bursts are not (their as-flown reruns are models/NOTES.md "The bursts' plant", on b466dbf's harness). The plant takes the flight's configuration as scorecard.py does (the bands' preload, tilt offset and accelerometer bias, the thrust table), the firmware the flight's own settings with the GPS hold in place of the camera's (the sim has no camera; the indoor tag hops are left out) and the drone's thrust scales and hover_mass as set. Calm air: what the log shows of wind is the plant's error here. The hull is rigid.py's belly (fly's own), not the legs.
"""

import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording, settings as recording_settings
from sw.sim import airframe as A
from sw.sim import flip_firmware as FF
from sw.sim import rigid as R
from sw.sim import scorecard as S
from sw.sim.flip_esc import STAND

# The flights rerun by default: the takeoff divergences and hovers of each configuration flown on the GPS hold.
FLIGHTS = ("2026-11-15T06-53-20", "2026-11-15T06-53-00", "2026-11-15T06-42-27", "2026-11-15T06-30-27", "2026-11-15T06-58-00", "2026-11-15T07-12-25", "2026-11-15T05-18-14", "2026-11-15T04-12-31", "2026-11-12T01-07-43", "2026-11-12T01-38-41", "2026-11-12T02-21-10")
# The shake band (Hz) and the span measured: from this long after liftoff to at most this long after it (s).
BAND = (1.5, 4.0)
SETTLE, SPAN = 1.0, 15.0
# The sim's IMU: the firmware's ranges (deg/s, g).
GYRO_RANGE, ACCEL_RANGE = 2000.0, 2.0


class Tracing(R.Sim):
    """rigid.Sim recording every tick's body rates (deg/s), roll and pitch (deg) and height (m) for all lanes."""

    def step(self, throttle, tilt_command):
        out = super().step(throttle, tilt_command)
        up = R.rotation(self.quat)[:, 2, :]
        attitude = np.degrees(np.column_stack([np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))]))
        self.trace.append(np.column_stack([np.degrees(self.turned), attitude, self.pos[:, 2] - FF.BELLY]))
        return out

    def reset(self, *args, **kwargs):
        self.trace = []
        Tracing.last = self
        return super().reset(*args, **kwargs)


def measures(time, pitch_rate, pitch, height, tipped_at):
    """The log's or a run's measures: liftoff (s, the height past 0.1 m), the tip cut's time after it (None without one), the pitch rate's dominant frequency (Hz) and RMS in BAND (deg/s) from SETTLE to SPAN s after liftoff, the largest |pitch| there (deg)."""
    lifted = np.flatnonzero(height > S.AIRBORNE_HEIGHT)
    if not len(lifted):
        return {"liftoff": None}
    start = time[lifted[0]]
    end = min(start + SPAN, tipped_at if tipped_at is not None else math.inf, time[lifted[-1]])
    chosen = (time >= start + SETTLE) & (time <= end)
    out = {"liftoff": float(start), "tip_after_liftoff": None if tipped_at is None else float(tipped_at - start), "largest_pitch": float(np.nanmax(np.abs(pitch[(time >= start) & (time <= end)]))) if ((time >= start) & (time <= end)).any() else None}
    if chosen.sum() >= 100:
        rate = pitch_rate[chosen] - np.nanmean(pitch_rate[chosen])
        spectrum = np.abs(np.fft.rfft(rate * np.hanning(len(rate)))) ** 2
        frequency = np.fft.rfftfreq(len(rate), A.DT)
        band = (frequency >= BAND[0]) & (frequency <= BAND[1])
        out["peak_hz"] = float(frequency[band][np.argmax(spectrum[band])])
        # The band's RMS by the spectrum's share of the variance (the window's power normalized out).
        out["band_rms"] = float(np.sqrt(np.var(rate) * spectrum[band].sum() / spectrum[1:].sum()))
        out["pitch_rate_rms"] = float(np.sqrt(np.mean(rate ** 2)))
    elif ((time >= start) & (time <= end)).sum() > 10:
        # Too short for a spectrum (the divergences): the pitch rate's zero crossings from the liftoff.
        rate = pitch_rate[(time >= start) & (time <= end)]
        crossings = np.flatnonzero(np.diff(np.sign(rate - rate.mean())) != 0)
        out["peak_hz"] = float((len(crossings) - 1) / 2 / (time[crossings[-1]] - time[crossings[0]])) if len(crossings) > 2 else None
        out["pitch_rate_rms"] = float(np.sqrt(np.mean(rate ** 2)))
    return out


def log_measures(name):
    """The recording's measures (measures), its tip cut from the stop reason."""
    listing = S.flight_list()
    entry = next(e for e in S.entries(listing) if e["name"].startswith(name))
    flight = S.Flight(entry, listing["configurations"][entry["configuration"]])
    stops = [e.get("reason") for e in S.events(recording(name)) if e["type"] == "stop"]
    tipped = flight.last_on if stops and stops[-1] == "Tilt limit exceeded" else None
    return measures(flight.time, np.degrees(flight.gyro[:, 1]), np.degrees(flight.attitude[:, 1]), np.nan_to_num(flight.height, nan=0.0) * flight.on, tipped)


def rerun(task):
    """One flight's settings flown n times on one plant (only the GPS and barometer noise differ between runs): each run's measures."""
    name, plant_name, n, seed, overrides = task
    listing = S.flight_list()
    entry = next(e for e in S.entries(listing) if e["name"].startswith(name))
    configuration = listing["configurations"][entry["configuration"]]
    path = recording(name)
    flown = recording_settings(path)
    params = json.loads(Path(plant_name).read_text()) if Path(plant_name).exists() else A.model(plant_name)
    bands = configuration["bands"]
    own = {"left_band": params.get("left_band", 0.0), "right_band": params.get("right_band", 0.0)}
    params = params | ((own if any(own.values()) else S.BANDS) if bands is True else {} if bands == "plant" else {"left_band": 0.0, "right_band": 0.0}) | overrides
    for key, other in (("servo_delay", "right_servo_delay"), ("servo_wn", "right_servo_wn")):
        if key in overrides and other not in overrides:
            params[other] = overrides[key]
    FF.plant_of = lambda airframe: {"airframe": params, "geometry": A.geometry_of(params), "disturbances": [], "rotor_drag": (0.07, 0.2)}
    FF.R.Sim = Tracing
    # The GPS hold in place of the camera's, the flight's own climb; no flip; the pack's sag is not simulated.
    settings = flown | {"hover_camera": 0, "hover_center": 0, "flip_angle": 0, "thrust_voltage": 0.0}
    settings.setdefault("hover_gps_tau", 0.5)
    settings.setdefault("hover_gps_place_tau", 2.0)
    seconds = min(1.0 + listing_seconds(name) + 3.0, 1.0 + SPAN + 5.0)
    options = {"thrust_scale": flown.get("left_thrust_scale", 1.3), "hover_mass": flown.get("hover_mass", 1.02)}
    if configuration["table"] == "stand":
        options["table"] = STAND
    offset = S.BAND_TILT if bands is True else 0.0
    rows = FF.fly((settings, offset, seed, n, seconds, GYRO_RANGE, ACCEL_RANGE, 1e9, False, plant_name, "none", {}, options))
    trace = np.array(Tracing.last.trace)
    time = np.arange(len(trace)) * A.DT
    out = []
    for i, row in enumerate(rows):
        lane = trace[:, i]
        tipped = None
        if row["tipped"]:
            # The tip cut: the first tick past tip_limit after liftoff.
            tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(lane[:, 3])) * np.cos(np.radians(lane[:, 4])), -1, 1)))
            over = np.flatnonzero((tilt > flown.get("tip_limit", 45.0)) & (lane[:, 5] > S.AIRBORNE_HEIGHT))
            tipped = float(time[over[0]]) if len(over) else None
        out.append(measures(time, lane[:, 1], lane[:, 4], lane[:, 5], tipped) | {"tipped": bool(row["tipped"])})
    # The first run's pitch rate and height, for plot.
    return {"name": name, "plant": plant_name, "runs": out, "pitch_rate": trace[:, 0, 1].tolist(), "height": trace[:, 0, 5].tolist()}


def plot(results, names, plants, out):
    """Each flight's pitch rate from its liftoff: the log (black) and each plant's first run, with the measures in the titles."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    listing = S.flight_list()
    colors = ("#1f77b4", "#d62728", "#2ca02c", "#9467bd")
    fig, axes = plt.subplots(len(names), 1, figsize=(14, 2.2 * len(names)), squeeze=False)
    for ax, name in zip(axes[:, 0], names):
        entry = next(e for e in S.entries(listing) if e["name"].startswith(name))
        flight = S.Flight(entry, listing["configurations"][entry["configuration"]])
        log = log_measures(name)
        if log.get("liftoff") is not None:
            shown = (flight.time >= log["liftoff"] - 0.5) & (flight.time <= log["liftoff"] + SPAN)
            ax.plot(flight.time[shown] - log["liftoff"], np.degrees(flight.gyro[shown, 1]), color="black", lw=1.0, label=f"log: {describe(log)}")
        for plant, color in zip(plants, colors):
            r = next(x for x in results if x["name"] == name and x["plant"] == plant)
            run = r["runs"][0]
            if run.get("liftoff") is None:
                continue
            time = np.arange(len(r["pitch_rate"])) * A.DT - run["liftoff"]
            shown = (time >= -0.5) & (time <= SPAN)
            ax.plot(time[shown], np.array(r["pitch_rate"])[shown], color=color, lw=0.8, label=f"{S.label_of(plant)}: {describe(run)} ({sum(x['tipped'] for x in r['runs'])}/{len(r['runs'])} tipped)")
        ax.set_title(f"{name} ({entry['kind']}, {entry['configuration']})", fontsize=8)
        ax.set_ylabel("pitch rate (deg/s)", fontsize=7)
        ax.legend(fontsize=6, loc="upper right")
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
    axes[-1, 0].set_xlabel("s from liftoff", fontsize=8)
    fig.suptitle("Closed loop: the firmware flying each flight's settings on the plant from the ground (sw/sim/scorecard_closed.py), the pitch rate against the log's", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    plt.close(fig)


def describe(m):
    parts = []
    if m.get("tip_after_liftoff") is not None:
        parts.append(f"tip cut {m['tip_after_liftoff']:.2f} s")
    if m.get("peak_hz") is not None:
        parts.append(f"{m['peak_hz']:.2f} Hz")
    if m.get("band_rms") is not None:
        parts.append(f"{m['band_rms']:.1f} deg/s in 1.5-4 Hz")
    return ", ".join(parts)


def listing_seconds(name):
    """The flight's time with the motors on, from the inventory (s)."""
    rows = json.loads(S.INVENTORY.read_text())["recordings"]
    return next(r["motors_on_s"] for r in rows if r["name"] == name)


def summary(values):
    values = [v for v in values if v is not None]
    return f"{np.median(values):.2f}" if values else "-"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plant", nargs="+", default=list(S.PLANTS))
    parser.add_argument("--flights", nargs="*", default=list(FLIGHTS))
    parser.add_argument("--n", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--set", default="", help="airframe terms over every plant's, name=value,... (servo_delay and servo_wn set both sides)")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--json")
    parser.add_argument("--plot", help="the pitch-rate figure here")
    args = parser.parse_args()
    overrides = parse_settings(args.set) if args.set else {}
    tasks = [(name, plant, args.n, args.seed, overrides) for name in args.flights for plant in args.plant]
    if args.workers > 1:
        with ProcessPoolExecutor(args.workers) as pool:
            results = list(pool.map(rerun, tasks))
    else:
        results = list(map(rerun, tasks))
    logs = {name: log_measures(name) for name in args.flights}
    print(f"{'flight':22} {'plant':28} {'tipped':>7} {'tip after s':>11} {'peak Hz':>8} {'band RMS':>9} {'rate RMS':>9} {'max pitch':>9}")
    for name in args.flights:
        log = logs[name]
        print(f"{name:22} {'log':28} {str(log.get('tip_after_liftoff') is not None):>7} {summary([log.get('tip_after_liftoff')]):>11} {summary([log.get('peak_hz')]):>8} {summary([log.get('band_rms')]):>9} {summary([log.get('pitch_rate_rms')]):>9} {summary([log.get('largest_pitch')]):>9}")
        for r in results:
            if r["name"] != name:
                continue
            runs = r["runs"]
            print(f"{'':22} {S.label_of(r['plant']):28} {sum(x['tipped'] for x in runs):>4}/{len(runs):<2} {summary([x.get('tip_after_liftoff') for x in runs]):>11} {summary([x.get('peak_hz') for x in runs]):>8} {summary([x.get('band_rms') for x in runs]):>9} {summary([x.get('pitch_rate_rms') for x in runs]):>9} {summary([x.get('largest_pitch') for x in runs]):>9}")
    if args.json:
        Path(args.json).write_text(json.dumps({"logs": logs, "runs": [{k: v for k, v in r.items() if k not in ("pitch_rate", "height")} for r in results]}, indent=1))
    if args.plot:
        plot(results, args.flights, args.plant, args.plot)


if __name__ == "__main__":
    main()
