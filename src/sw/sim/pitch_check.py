"""
The pitch modes, sim against real: uv run --with scipy -m sw.sim.pitch_check <recordings> [--airframe models/airframe_<day>.json] [--seconds 6] [--seeds 8] [--set key=value,...] [--workers N]

Each recording's hover hop and the hover sim (sweep.run with hover=True on sweep.CURRENT, or CURRENT with --airframe swapped in) flying the same settings from the same resting attitude, with the recording's test pattern if it played one, are measured alike from liftoff:
- the swing: the pitch's strongest frequency in 0.4-1.6 Hz, its peak-to-peak, and its growth (the RMS of the pitch's 0.4-1.6 Hz band over the window's last third over its first third);
- the shake: the pitch rate's RMS in 2.2-3.5 Hz and its strongest frequency in 1.8-4 Hz;
- and for the sim how many runs lost the drone (past 45 degrees) and when.
Added 2026-11-08 to check that the sim reproduces the ~0.7-1 Hz pitch swing of the 2026-11-08 hops and the 2.6-2.8 Hz shake at pitch_damping 8 (2026-11-02 04:36 UTC) before a gain or filter change flies. --set overrides settings (e.g. pitch_damping=8) to fly a recording's hop with other gains; the real columns stay the recording's.
"""

import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

from sw.fit import settings as recording_settings
from sw.flight_log import read_flight

FS = 100.0
SWING = (0.4, 1.6)
SHAKE = (2.2, 3.5)


def band(x, low, high):
    """x filtered to low-high Hz (zero phase), or zeros when too short to filter."""
    if len(x) < 40:
        return np.zeros_like(x)
    return sosfiltfilt(butter(2, (low, high), btype="band", fs=FS, output="sos"), x - x.mean(), padlen=min(len(x) - 1, 30))


def peak(x, low, high):
    """The strongest frequency of x in low-high Hz (zero-padded spectrum, Hann window)."""
    n = 8192
    spectrum = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)), n))
    f = np.fft.rfftfreq(n, 1 / FS)
    inside = (f >= low) & (f <= high)
    return float(f[inside][np.argmax(spectrum[inside])])


def measure(pitch, rate):
    """The swing and shake measures of an airborne pitch (deg) and pitch rate (deg/s) series at FS."""
    swing = band(pitch, *SWING)
    third = max(1, len(swing) // 3)
    return {"swing_hz": peak(pitch, *SWING), "swing_pp": float(pitch.max() - pitch.min()), "growth": float(np.sqrt(np.mean(swing[-third:] ** 2)) / max(1e-6, np.sqrt(np.mean(swing[:third] ** 2)))),
            "shake_rms": float(np.sqrt(np.mean(band(rate, *SHAKE) ** 2))), "shake_hz": peak(rate, 1.8, 4.0)}


def real(path):
    """The recording's airborne pitch and pitch rate (from the identification's liftoff to its end of the airborne window), its resting pitch and its test pattern."""
    from sw.sim.identify_airframe import load
    record = load(path, 0.3)
    if record is None:
        return None
    lift, end = record["lift"], record["end"]
    on = int(np.flatnonzero(record["base_throttle"] > 0)[0])
    events = path.with_suffix(".jsonl")
    patterns = [e for e in map(json.loads, events.read_text().splitlines()) if e.get("type") == "pattern"] if events.exists() else []
    return {"pitch": record["pitch_deg"][lift:end], "rate": record["gy_deg_s"][lift:end], "rest": float(np.median(record["pitch_deg"][max(0, on - 30):on])), "pattern": patterns[-1] if patterns else None}


def simulate(job):
    """One sim run of a hop: the measures over seconds from liftoff, and when it was lost (None if not)."""
    from sw.sim.sweep import CURRENT, run
    settings, airframe, rest, pattern, seconds, seed = job
    plant = CURRENT | {"ground_pitch": rest}
    if airframe is not None:
        plant["airframe"] = airframe | {f"{axis}_push": 0.0 for axis in ("roll", "pitch", "yaw")}
    # The tag where the hold wants to be, so the hop starts on its range as the real ones roughly did.
    plant["camera"] = dict(plant["camera"]) | {"tag": (settings.get("hover_distance", 2.0), 0.0, 0.45)}
    if pattern:
        plant["excite"] = pattern
    log = []
    result = run(settings, plant, seconds=seconds + 1.5, seed=seed, log=log, hover=True)
    t = np.array([e[0] for e in log])
    pitch = np.degrees([e[2][1] for e in log])
    # The body's own pitch rate, as the logged gyro measures it (the filtered gyro the controller reads).
    rate = np.degrees([e[11][1] for e in log])
    # Liftoff as the identification finds it in a recording is near enough the sim's own for these measures.
    lifted = result.get("liftoff_s") or next((t[k] for k in range(len(t)) if log[k][1][2] > 0.01), None)
    if lifted is None:
        return None
    keep = (t >= lifted) & (t <= lifted + seconds)
    return measure(pitch[keep], rate[keep]) | {"lost_s": None if result["stable"] else float(result["fail_s"] - lifted)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--airframe", type=Path, help="an airframe model to fly instead of sweep.CURRENT's")
    parser.add_argument("--seconds", type=float, help="seconds from liftoff the sim flies and is measured over (default: the recording's airborne time, at least 3)")
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--set", default="", help="settings to override, key=value,...")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text()) if args.airframe else None
    overrides = {key: float(value) for key, value in (item.split("=") for item in args.set.split(",") if item)}
    hops = [(path, r) for path in args.recordings if (r := real(path)) is not None]
    span = lambda r: args.seconds or max(3.0, len(r["pitch"]) / FS)
    jobs = [(recording_settings(path) | overrides, airframe, r["rest"], r["pattern"], span(r), seed) for path, r in hops for seed in range(args.seeds)]
    with Pool(min(args.workers, len(jobs))) as pool:
        runs = pool.map(simulate, jobs)
    for i, (path, r) in enumerate(hops):
        m = measure(r["pitch"], r["rate"])
        sims = [s for s in runs[i * args.seeds:(i + 1) * args.seeds] if s is not None]
        lost = [s["lost_s"] for s in sims if s["lost_s"] is not None]
        median = lambda key: np.median([s[key] for s in sims]) if sims else float("nan")
        print(f"{path.name[:19]} ({len(r['pitch']) / FS:.1f} s airborne, rest {r['rest']:+.1f}°{', pattern' if r['pattern'] else ''})")
        print(f"  real: swing {m['swing_hz']:.2f} Hz, {m['swing_pp']:.0f}° p-p, growth x{m['growth']:.1f}; shake {m['shake_rms']:.0f} deg/s RMS at {m['shake_hz']:.2f} Hz")
        print(f"  sim ({len(sims)} runs, medians over {span(r):g} s{', ' + args.set if args.set else ''}): swing {median('swing_hz'):.2f} Hz, {median('swing_pp'):.0f}° p-p, growth x{median('growth'):.1f}; shake {median('shake_rms'):.0f} deg/s RMS at {median('shake_hz'):.2f} Hz; lost {len(lost)} of {len(sims)}" + (f" (median {np.median(lost):.1f} s after liftoff)" if lost else ""), flush=True)


if __name__ == "__main__":
    main()
