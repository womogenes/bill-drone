"""
The pitch swing's frequency and damping, sim against real: uv run --with scipy -m sw.sim.swing_damping --recordings <names> --models models/airframe_A.json [...] [--set key=value,...] [--seconds 30] [--seeds 6] [--workers N]

Fits a second-order resonance driven by white torque (its frequency, damping ratio and level) to the pitch spectrum (0.2-3 Hz, quadratic trend removed) of each recording's hover (named by its UTC start, e.g. 2026-11-02T05-22-01) and of the hover sim (pitch_check.fly) flying the same settings for --seconds, and prints the frequency, the damping ratio and the RMS of the pitch's 0.4-1.6 Hz swing band, with how many runs were lost. --set overrides settings (gains) on every run; the real lines stay the recordings'.
Added 2026-11-08: the long tag hovers of 2026-11-02 swing at 0.80 Hz with a damping ratio of 0.12-0.28, which a model must reproduce at their gains before its gain predictions are trusted (the multi-step prediction alone did not: the best predicting model of the first outdoor flights swung at a damping ratio of 0.5). Long hovers (20 s or more) give usable fits; a few seconds of hop do not.
"""

import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import welch

from sw.flight_log import recording
from sw.flight_log import settings as recording_settings
from sw.sim import pitch_check as PC
BAND = (0.2, 3.0)


def resonance(pitch):
    """Frequency (Hz), damping ratio and swing-band RMS (deg) of a pitch series (deg) at 100 Hz."""
    k = np.arange(len(pitch))
    x = pitch - np.polyval(np.polyfit(k, pitch, 2), k)
    f, power = welch(x, fs=PC.FS, nperseg=min(1024, len(x) // 2))
    inside = (f > BAND[0]) & (f < BAND[1])
    w = 2 * np.pi * f[inside]

    def residual(q):
        level, frequency, zeta = q
        wn = 2 * np.pi * frequency
        return np.log(level / ((wn ** 2 - w ** 2) ** 2 + (2 * zeta * wn * w) ** 2)) - np.log(power[inside] + 1e-12)

    fit = least_squares(residual, [power[inside].max() * (2 * np.pi * 0.8) ** 4 * 0.1, 0.8, 0.3], bounds=([0, 0.2, 0.02], [np.inf, 3, 2]))
    return fit.x[1], fit.x[2], float(np.sqrt(np.mean(PC.band(x, *PC.SWING) ** 2)))


def simulate(job):
    """One sim run: the resonance of its pitch from 1 s after liftoff, ("lost", when) if it was lost, or None without a liftoff."""
    flown = PC.fly(*job)
    if flown is None:
        return None
    pitch, _, lost = flown
    if lost is not None or len(pitch) < 800:
        return ("lost", lost)
    return resonance(pitch[100:])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--recordings", nargs="+", required=True, help="recording names (UTC starts)")
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--set", default="", help="settings to override, key=value,...")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--seeds", type=int, default=6)
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    args = parser.parse_args()
    overrides = {key: float(value) for key, value in (item.split("=") for item in args.set.split(",") if item)}
    hops = [(path, PC.real(path)) for path in map(recording, args.recordings)]
    for path, real in hops:
        frequency, zeta, swing = resonance(real["pitch"][100:])
        print(f"real {path.name[:19]}: {frequency:.2f} Hz, zeta {zeta:.2f}, swing band rms {swing:.1f}", flush=True)
    with Pool(args.workers) as pool:
        for model in args.models:
            airframe = json.loads(Path(model).read_text())
            jobs = [(recording_settings(path) | overrides, airframe, real["rest"], real["pattern"], real["tag"], args.seconds, seed) for path, real in hops for seed in range(args.seeds)]
            out = pool.map(simulate, jobs)
            flown = np.array([o for o in out if o and o[0] != "lost"])
            lost = sum(1 for o in out if o and o[0] == "lost")
            if not len(flown):
                print(f"{Path(model).stem} {args.set}: all lost ({lost}/{len(out)})", flush=True)
                continue
            print(f"{Path(model).stem} {args.set}: {np.median(flown[:, 0]):.2f} Hz, zeta median {np.median(flown[:, 1]):.2f} (25-75% {np.percentile(flown[:, 1], 25):.2f}-{np.percentile(flown[:, 1], 75):.2f}), swing band rms {np.median(flown[:, 2]):.1f} (p90 {np.percentile(flown[:, 2], 90):.1f}); lost {lost}/{len(out)}", flush=True)


if __name__ == "__main__":
    main()
