"""
The pitch response to the servos' common tilt in hover, real against a model: uv run -m sw.sim.pitch_gain [recordings...] [--airframe a.json ...]

For each recording's airborne hover (no flip, both motors over 300, tilt under 25 degrees): the pitch acceleration (the gyro's pitch rate differenced) against the servos' common physical tilt command (degrees, the trims taken off, the right servo mirrored), both band-passed to 1-3 Hz, as the cross-spectral gain |S_xy| / S_xx in deg/s² per degree of tilt. With --airframe the same from the plant driven by the logged commands one step ahead (sw/sim/flip_replay.py, the rates and attitude set back to the log every tick, its actuators carried on): the acceleration the model gives from the logged state, so a model's tilt authority and lag can be read against the flight's.
"""

import argparse

import numpy as np

from sw.sim import airframe as A
from sw.sim import flip_replay as F

BAND = (1.0, 3.0)


def gain(x, y, dt=A.DT, band=BAND):
    """|S_xy| / S_xx over the band, from the whole series (detrended)."""
    x, y = x - x.mean(), y - y.mean()
    f = np.fft.rfftfreq(len(x), dt)
    X, Y = np.fft.rfft(x * np.hanning(len(x))), np.fft.rfft(y * np.hanning(len(y)))
    m = (f >= band[0]) & (f <= band[1])
    return float(np.abs((np.conj(X[m]) * Y[m]).sum()) / (np.abs(X[m]) ** 2).sum())


def airborne(log):
    """The longest stretch of hover in a Log: no flip, both motors over 300."""
    ok = (log.phase == 0) & (log.throttle.min(1) > 300) & (np.abs(log.attitude).max(1) < np.radians(25))
    best, start = (0, 0), None
    for k, flag in enumerate(np.append(ok, False)):
        if flag and start is None:
            start = k
        elif not flag and start is not None:
            best = max(best, (k - start, start))
            start = None
    return best[1], best[1] + best[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--airframe", nargs="*", default=[])
    args = parser.parse_args()
    for name in args.recordings:
        full = F.Log(name, 0, 1e9)
        lo, hi = airborne(full)
        if hi - lo < 500:
            print(f"{name}: under 5 s of hover")
            continue
        log = F.Log(name, full.time[lo] + 1.0, full.time[hi - 1] - 0.5)
        tilt = np.degrees(log.tilt().mean(1))
        real = gain(tilt[1:], np.diff(np.degrees(log.gyro[:, 1])) / A.DT)
        line = f"{name}: {log.time[-1] - log.time[0]:.0f} s, real {real:.0f}"
        for airframe in args.airframe:
            result = F.replay(log, [F.plant_for(log, A.model(airframe))], every=A.DT)
            line += f", {airframe} {gain(tilt[1:], np.degrees(result['omega'][0, 1:, 1] - log.gyro[:-1, 1]) / A.DT):.0f}"
        print(line + " deg/s² per degree of common tilt", flush=True)


if __name__ == "__main__":
    main()
