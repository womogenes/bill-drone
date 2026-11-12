"""
A GPS-hold flight's own nudges flown again in the hover sim, beside the flight: uv run -m sw.sim.nudge_replay <recording name> [--span A,B] [--seeds 8] [--set name=value,...] [--airframe models/airframe_<day>.json] [--plot file.png] [--workers N]

Written 2026-11-12 to tell the hold's own ringing from the pilot's nudges on drone 1's flights of 2026-11-11 evening (Pacific). The real side: the recording's pitch and roll and their targets, the GPS ground speed, and the laptop's nudge (its commands put on the drone's clock by the GPS events' arrival). The sim side flies the recording's settings (or --set's changes) on sw/sim/speed_limit.py's plant (the raised airframe with its terms drawn, the bands, a light wind of 0-1 m/s) and the same nudge, as speeds between the commands, from the same time after liftoff (the motors' start plus LIFTOFF). Per nudge (commands moving the nudge, gaps under 0.4 s joined) it prints the largest pitch and roll target from its start to 2.5 s after it, the GPS peak speed and the time from its end to under STOPPED, real against the sim's median over the seeds; and over --span the pitch target's peak to peak and the 0.4-1.6 Hz RMS of the pitch.
"""

import argparse
import json
import os
import random
from datetime import datetime
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import events, metadata, parse_settings, read_flight, recording
from sw.flight_log import settings as recording_settings
from sw.sim.outdoor import plant as outdoor_plant
from sw.sim.pitch_check import SWING, band
from sw.sim.speed_limit import AIRFRAME, BAND_ACCEL, BAND_TILT, STOPPED
from sw.sim.sweep import run

# Liftoff after the motors start (s; sw/sim/gps_truth.py LIFTOFF).
LIFTOFF = 0.35
# Nudge commands this close together (s) are one nudge.
JOIN = 0.4
# A nudge step larger than this (m) is the dashboard's wrap, not a move (fw/drone/main/hover.c NUDGE_JUMP).
JUMP = 0.5


def stamp(text):
    return datetime.fromisoformat(text.replace("Z", "+00:00")).timestamp()


def real(name):
    """The flight on its own clock, in s after liftoff: the series, the GPS speed, and the nudge as (time, forward, left) steps."""
    path = recording(name)
    data, _ = read_flight(path)
    ev = events(path)
    t = data["esp_time_us"].to_numpy() / 1e6
    started = metadata(path)["started_ticks_us"] / 1e6
    gps = [e for e in ev if e["type"] == "gps" and e.get("fix") == 3]
    offset = np.median([e["drone_ms"] / 1000 - started - stamp(e["received_at"]) for e in gps])
    lifted = t[np.argmax(data["left_throttle"].to_numpy() > 300)] + LIFTOFF
    series = {key: data[column].to_numpy() for key, column in (("pitch", "pitch_deg"), ("target_pitch", "target_pitch_deg"), ("roll", "roll_deg"), ("target_roll", "target_roll_deg"))}
    steps = [(stamp(e["received_at"]) + offset - lifted, *e["nudge"][:2]) for e in ev if e["type"] == "command"]
    return {"t": t - lifted, **series, "gps_t": np.array([e["drone_ms"] / 1000 - started for e in gps]) - lifted, "gps_speed": np.array([np.hypot(*e["velocity"][:2]) for e in gps]), "steps": steps}, recording_settings(path)


def moves(steps):
    """The nudge's steps as sweep.run plant["nudge"] segments (s after liftoff, forward and left m/s, s), and the nudges (start, end) they make."""
    segments, changes = [], []
    for (time, forward, left), (after, next_forward, next_left) in zip(steps, steps[1:]):
        df, dl = next_forward - forward, next_left - left
        if (df or dl) and abs(df) < JUMP and abs(dl) < JUMP and after > time:
            # The command at time held until after: the move between them lands at after, flown over the gap before it.
            length = min(after - time, 0.1)
            segments.append((after - length, df / length, dl / length, length))
            changes.append(after)
    nudges = []
    for time in changes:
        if nudges and time - nudges[-1][1] < JOIN:
            nudges[-1][1] = time
        else:
            nudges.append([time - 0.05, time])
    return segments, nudges


def measure(t, target_pitch, target_roll, speed_t, speed, nudge):
    begin, end = nudge
    window = (t >= begin) & (t <= end + 2.5)
    moving = (speed_t >= begin) & (speed_t <= end + 2.5)
    after = np.flatnonzero((speed_t > end) & (speed < STOPPED))
    return {"pitch": float(np.abs(target_pitch[window]).max()) if window.any() else np.nan, "roll": float(np.abs(target_roll[window]).max()) if window.any() else np.nan,
            "peak": float(speed[moving].max()) if moving.any() else np.nan, "stop": float(speed_t[after[0]] - end) if len(after) else np.nan}


def fly(job):
    airframe, flown, segments, seconds, seed = job
    rng = random.Random(seed + 77000)
    p, _ = outdoor_plant(airframe, rng.uniform(0, 1), seed, True, False, flown["hover_climb"], baro=True, level_sd=1.3, pushes=True)
    p |= {"tilt_offset": BAND_TILT, "flight_accel_bias": (BAND_ACCEL, 0.0, 0.0)}
    if segments:
        p["nudge"] = segments
    s = flown | {"hover_gps_heading": p["gps"]["heading"] + random.Random(seed + 1).uniform(-30, 30)}
    log = []
    result = run(s, p, seconds=seconds, seed=seed, log=log, hover=True)
    lifted = result.get("liftoff_s")
    if lifted is None or not result["stable"]:
        return None
    t = np.array([e[0] for e in log]) - lifted
    position = np.array([e[1] for e in log])[:, :2]
    return {"t": t, "pitch": np.degrees([e[2][1] for e in log]), "target_pitch": np.degrees([e[6] for e in log]), "roll": np.degrees([e[2][0] for e in log]), "target_roll": np.degrees([e[9] for e in log]),
            "speed": np.hypot(*np.gradient(position, t, axis=0).T)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording")
    parser.add_argument("--span", type=lambda text: tuple(float(v) for v in text.split(",")), help="s after liftoff for the peak to peak and swing (default: the nudges' first start to 10 s after the last end)")
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--airframe", type=Path, default=AIRFRAME)
    parser.add_argument("--plot", type=Path, help="the real pitch target and pitch beside the sim's (first seed) over the span")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    args = parser.parse_args()
    flight, flown = real(args.recording)
    flown = flown | {"hover_camera": 0, "hover_center": 0} | parse_settings(args.set)
    segments, nudges = moves(flight["steps"])
    span = args.span or (nudges[0][0], nudges[-1][1] + 10)
    airframe = json.loads(args.airframe.read_text())
    with Pool(args.workers) as pool:
        sims = [s for s in pool.map(fly, [(airframe, flown, segments, span[1] + 1, seed) for seed in range(args.seeds)]) if s is not None]
    print(f"{args.recording} (hover_tilt {flown['hover_tilt']:g}, hover_kd {flown['hover_kd']:g}, hover_kp {flown['hover_kp']:g}, hover_thrust_direction {flown['hover_thrust_direction']:g}){' + ' + args.set if args.set else ''}: {len(sims)} of {args.seeds} sim runs held")
    print("nudge (s after liftoff) | largest pitch target, real / sim (deg) | roll target | peak speed (m/s) | stop after (s)")
    for nudge in nudges:
        r = measure(flight["t"], flight["target_pitch"], flight["target_roll"], flight["gps_t"], flight["gps_speed"], nudge)
        s = [measure(m["t"], m["target_pitch"], m["target_roll"], m["t"], m["speed"], nudge) for m in sims]
        med = lambda key: np.nanmedian([x[key] for x in s]) if s else np.nan
        print(f"{nudge[0]:6.1f}-{nudge[1]:5.1f} | {r['pitch']:5.1f} / {med('pitch'):5.1f} | {r['roll']:5.1f} / {med('roll'):5.1f} | {r['peak']:4.2f} / {med('peak'):4.2f} | {r['stop']:4.1f} / {med('stop'):4.1f}")
    inside = lambda m: (m["t"] >= span[0]) & (m["t"] <= span[1])
    swing = lambda m: float(np.sqrt(np.mean(band(m["pitch"], *SWING)[inside(m)] ** 2)))
    print(f"over {span[0]:.1f}-{span[1]:.1f} s: pitch target peak to peak real {np.ptp(flight['target_pitch'][inside(flight)]):.1f}, sim {np.median([np.ptp(m['target_pitch'][inside(m)]) for m in sims]):.1f} deg;"
          f" pitch swing band RMS real {swing(flight):.2f}, sim {np.median([swing(m) for m in sims]):.2f} deg")
    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for m, label, style in ((flight, "flight", "-"), (sims[0], "sim", "--")):
            k = inside(m)
            axes[0].plot(m["t"][k], m["target_pitch"][k], style, label=f"{label} pitch target")
            axes[1].plot(m["t"][k], m["target_roll"][k], style, label=f"{label} roll target")
        g = (flight["gps_t"] >= span[0]) & (flight["gps_t"] <= span[1])
        axes[2].plot(flight["gps_t"][g], flight["gps_speed"][g], label="flight GPS speed")
        axes[2].plot(sims[0]["t"][inside(sims[0])], sims[0]["speed"][inside(sims[0])], "--", label="sim ground speed")
        for nudge in nudges:
            for a in axes:
                a.axvspan(*nudge, color="0.9")
        for a, unit in zip(axes, ("deg", "deg", "m/s")):
            a.set_ylabel(unit)
            a.legend(loc="upper right")
        axes[2].set_xlim(*span)
        axes[2].set_xlabel("s after liftoff (shaded: the pilot's nudges)")
        axes[0].set_title(f"{args.recording} UTC{' + ' + args.set if args.set else ''}")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=80)


if __name__ == "__main__":
    main()
