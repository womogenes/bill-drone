"""
A 2026-11-09 outdoor flight against the hover sim flying it: uv run --with scipy -m sw.sim.outdoor_replay <recording name> --truth DIR --models models/airframe_A.json [...] [--span A,B] [--heading-error DEG] [--stick 'start,pitch,roll,seconds;...'] [--wind 0.3] [--gps-latency S] [--seconds S] [--seeds 8] [--workers N] [--save file.json]

Written 2026-11-09 to check airframe models on what the outer loop must handle: hands-off drift, fly-aways and braking. The real side comes from the flight's smoothed GPS truth (sw/sim/gps_truth.py --save; seconds are counted from the throttle onset): over --span the pitch swing (swing_damping.resonance: frequency, damping ratio, 0.4-1.6 Hz RMS), the horizontal wander (RMS distance from the span's mean position) and the ground speed (mean, peak, and its trace). The sim side flies the recording's settings from takeoff on the GPS hold (outdoor.plant, nominal: the airframe as given, the night's light wind of --wind m/s from a random direction with its turbulence) with the hold's heading belief off by --heading-error degrees (gps_truth analysis 3 measures it per flight: 06:33-06:38 about -60) and the pilot's sticks replayed (s after liftoff, as sweep.run's plant["stick"]), and is measured over the same span after liftoff.
"""

import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings, recording
from sw.flight_log import settings as recording_settings
from sw.sim.outdoor import GPS
from sw.sim.outdoor import plant as outdoor_plant
from sw.sim.swing_damping import resonance
FS = 100.0
# Liftoff after the throttle onset (gps_truth.LIFTOFF), to put the real and sim spans on one clock.
LIFTOFF = 0.35


def measures(t, pitch, position, span):
    """Swing, wander and speed over span (s) of series on t (s after liftoff): pitch in degrees, position [sample, (x, y)] in m (or None)."""
    inside = (t >= span[0]) & (t <= span[1])
    out = {}
    if inside.sum() > 10 * FS:
        frequency, zeta, swing = resonance(pitch[inside])
        out |= {"swing_hz": float(frequency), "zeta": float(zeta), "swing_rms": float(swing)}
    out["pitch_sd"] = float(pitch[inside].std())
    if position is not None and np.isfinite(position[inside]).all():
        p = position[inside]
        speed = np.hypot(*np.gradient(p, 1 / FS, axis=0).T)
        out |= {"wander": float(np.sqrt(np.mean(np.sum((p - p.mean(0)) ** 2, 1)))), "speed_mean": float(speed.mean()), "speed_peak": float(speed.max()), "distance": float(np.hypot(*(p[-1] - p[0])))}
    return out


def real(name, truth, span):
    s = np.load(Path(truth) / f"{name}.npz")
    t = s["t"] - s["t"][np.flatnonzero(s["left_throttle"] > 0)[0]] - LIFTOFF
    position = np.column_stack([s["north"], s["east"]]) if "north" in s.files else None
    out = measures(t, np.degrees(s["pitch"]), position, span)
    keep = t >= 0
    speed = np.hypot(s["v_north"], s["v_east"])[keep] if "v_north" in s.files else None
    return out, t[keep], speed


def fly(job):
    airframe, settings, wind, heading_error, latency, sticks, seconds, span, seed = job
    from sw.sim.sweep import run
    p, _ = outdoor_plant(airframe, wind, seed, True, True, settings.get("hover_climb", 1.0))
    p["gps"] = GPS | {"heading": 0.0, "latency": latency}
    if sticks:
        p["stick"] = sticks
    s = dict(settings) | {"hover_camera": 0, "hover_center": 0, "hover_gps_heading": heading_error}
    log = []
    result = run(s, p, seconds=seconds, seed=seed, log=log, hover=True)
    lifted = result.get("liftoff_s")
    if lifted is None:
        return None
    t = np.array([e[0] for e in log]) - lifted
    position = np.array([e[1] for e in log])
    pitch = np.degrees([e[2][1] for e in log])
    keep = t >= 0
    speed = np.hypot(*np.gradient(position[:, :2], 1 / FS, axis=0).T)
    out = measures(t, pitch, position[:, :2], span) if result["stable"] else {}
    return out | {"lost": None if result["stable"] else float(result["fail_s"] - lifted), "t": t[keep][::10].tolist(), "speed": speed[keep][::10].tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recording")
    parser.add_argument("--truth", required=True)
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--span", type=lambda text: tuple(float(v) for v in text.split(",")), help="s after liftoff (default: 5 s after liftoff to the truth's end)")
    parser.add_argument("--heading-error", type=float, default=0.0)
    parser.add_argument("--stick", type=lambda text: [tuple(float(v) for v in item.split(",")) for item in text.split(";")], help="the pilot's sticks: start s after liftoff, pitch and roll degrees, seconds held; ';' between them")
    parser.add_argument("--wind", type=float, default=0.3)
    parser.add_argument("--gps-latency", type=float, default=GPS["latency"], help="s from the GPS measurement to the hold (gps_truth analysis 3 measures the hold's velocity 0-140 ms behind the GPS on most flights, 240-320 ms at 06:33-06:38)")
    parser.add_argument("--seconds", type=float, help="sim seconds (default: the span's end plus 2)")
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--save", type=Path, help="write the real and simulated measures and speed traces as JSON")
    args = parser.parse_args()
    path = recording(args.recording)
    settings = recording_settings(path) | parse_settings(args.set)
    s = np.load(Path(args.truth) / f"{args.recording}.npz")
    end = s["t"][-1] - s["t"][np.flatnonzero(s["left_throttle"] > 0)[0]] - LIFTOFF - 0.3
    span = args.span or (5.0, end)
    seconds = args.seconds or span[1] + 2.0
    measured, t, speed = real(args.recording, args.truth, span)
    show = lambda m: ", ".join(f"{key} {value:.2f}" for key, value in m.items())
    print(f"{args.recording} over {span[0]:.0f}-{span[1]:.0f} s after liftoff, heading belief off {args.heading_error:+.0f}°{', sticks ' + str(args.stick) if args.stick else ''}{', ' + args.set if args.set else ''}")
    print(f"  real: {show(measured)}", flush=True)
    saved = {"recording": args.recording, "span": span, "real": measured, "real_t": t[::10].tolist(), "real_speed": None if speed is None else speed[::10].tolist(), "models": {}}
    with Pool(min(args.workers, args.seeds)) as pool:
        for model in args.models:
            airframe = json.loads(Path(model).read_text())
            runs = [r for r in pool.map(fly, [(airframe, settings, args.wind, args.heading_error, args.gps_latency, args.stick, seconds, span, seed) for seed in range(args.seeds)]) if r is not None]
            lost = [r["lost"] for r in runs if r["lost"] is not None]
            keys = sorted({key for r in runs for key in r if key not in ("lost", "t", "speed")})
            medians = {key: float(np.median([r[key] for r in runs if key in r])) for key in keys}
            print(f"  {Path(model).stem}: {show(medians)}; lost {len(lost)}/{len(runs)}", flush=True)
            saved["models"][model] = {"medians": medians, "lost": lost, "runs": [{key: value for key, value in r.items()} for r in runs]}
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(json.dumps(saved))


if __name__ == "__main__":
    main()
