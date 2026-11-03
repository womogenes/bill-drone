"""
The servos' steady tilt against the forward speed: uv run -m sw.sim.speed_hold <recordings or globs> [--sim]

Forward airspeed pitches this airframe nose-up (the rotors flapping back), and the pitch loop holds it with a steady forward thrust tilt, so below the pitch loop's bandwidth the mean servo tilt follows the speed: tilt = speed_pitch × pitch + speed_tilt × forward speed + offset (fw/drone/main/hover.h; fitted on 2026-10-31 and 11-01 at 5.0 degrees per m/s with speed_pitch -0.54). This fits it per flight on the tag-tracked hover, every signal smoothed over SMOOTH seconds. speed_tilt is the airframe's static speed moment over its held tilt authority, the balance outdoor flight runs on (a steady wind is a steady airspeed) and which the multi-step identification cannot see at hover speeds; the hover sim must give the same number. With --sim each recording's settings (and its test pattern) are flown on the sim plant (sweep.CURRENT with --airframe) and the same fit is made on the sim's own tilts, pitch and speed. A flight with uv run -m sw.excite speed --forward 0.5 --width 3 --rest 3 holds steady speeds long enough to pin it; ordinary hovers, at a few cm/s, do not (the sim gives 4-6 there whatever its static balance).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sw.fit import settings as recording_settings
from sw.flight_log import read_flight
from sw.sim import identify_airframe as N

SMOOTH = 2.0
# The balance's smoothing (s): the speed pattern's 3 s stretches need a shorter window than the hover's drift.
BALANCE_SMOOTH = 1.0
FS = 100


def smooth(x, seconds=SMOOTH):
    k = int(seconds * FS)
    return np.convolve(x, np.ones(k) / k, "same")


def fit(tilt, pitch, speed):
    """
    speed_tilt (degrees per m/s), speed_pitch, offset (degrees) and R² of the smoothed tilt on the smoothed pitch and speed, the ends (one window) left out; the speed's sd (m/s); and the balance: the tilt per m/s on the speed and its rate alone (degrees per m/s), the body's pitch left in.
    The balance is the static speed moment over the tilt's authority as the speed pattern measures it: in the sim it gives about 15 on airframe_2026-11-02_9in (static 15) and 6-8 on the calibrated airframe (static 8), where speed_tilt gives 2-6 on both. On ordinary hovers it scatters.
    """
    k = int(SMOOTH * FS)
    t, p, v = smooth(tilt)[k:-k], smooth(pitch)[k:-k], smooth(speed)[k:-k]
    X = np.column_stack([v, p, np.ones(len(t))])
    coef, *_ = np.linalg.lstsq(X, t, rcond=None)
    tb, vb = smooth(tilt, BALANCE_SMOOTH)[k:-k], smooth(speed, BALANCE_SMOOTH)[k:-k]
    B = np.column_stack([vb, np.gradient(vb, 1 / FS), np.ones(len(tb))])
    balance, *_ = np.linalg.lstsq(B, tb, rcond=None)
    return coef[0], coef[1], coef[2], 1 - np.var(t - X @ coef) / np.var(t), v.std(), balance[0]


def real(path):
    d, _ = read_flight(path)
    s = recording_settings(path)
    flags = d["flags"].to_numpy().astype(int)
    tracking = (flags >> 6 & 1) > 0
    good = np.flatnonzero(tracking & np.isfinite(d.hover_vx_m_s.to_numpy()))
    if len(good) < 6 * FS:
        return None
    span = slice(good[0] + FS, good[-1] - 2 * FS)
    tilt = ((d.left_servo_deg - s.get("left_trim", 0.0)) - (d.right_servo_deg - s.get("right_trim", 0.0))).to_numpy()[span] / 2
    yaw = np.radians(d.hover_yaw_deg.to_numpy()[span])
    forward = d.hover_vx_m_s.to_numpy()[span] * np.cos(yaw) + d.hover_vy_m_s.to_numpy()[span] * np.sin(yaw)
    fill = lambda v: np.interp(np.arange(len(v)), np.flatnonzero(np.isfinite(v)), v[np.isfinite(v)])
    return fit(fill(tilt), fill(d.pitch_deg.to_numpy()[span]), fill(forward))


def simulated(path, airframe, seconds, seed):
    from sw.sim.sweep import CURRENT, run
    s = recording_settings(path) | {"hover_distance": 2.0}
    plant = CURRENT | {"airframe": airframe | {f"{a}_push": 0.0 for a in ("roll", "pitch", "yaw")}}
    # The recording's test pattern (sw/excite.py speed: the hold point flown out and back at steady speeds), flown the same way.
    events = path.with_suffix(".jsonl")
    patterns = [e for e in map(json.loads, events.read_text().splitlines()) if e.get("type") == "pattern"] if events.exists() else []
    if patterns:
        plant["excite"] = patterns[-1]
    log = []
    result = run(s, plant, seconds=seconds, seed=seed, log=log, hover=True)
    if not result["stable"] or result.get("liftoff_s") is None:
        return None
    t = np.array([e[0] for e in log]) - result["liftoff_s"]
    keep = t > 4.0
    yaw = np.array([e[2][2] for e in log])
    position = np.array([e[1] for e in log])
    velocity = np.gradient(position[:, :2], 1 / FS, axis=0)
    forward = velocity[:, 0] * np.cos(yaw) + velocity[:, 1] * np.sin(yaw)
    tilt = np.degrees([(e[4][0] + e[4][1]) / 2 for e in log])
    return fit(tilt[keep], np.degrees([e[2][1] for e in log])[keep], forward[keep])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--airframe", type=Path, default=Path(__file__).with_name("models") / "airframe_2026-11-02_9in.json")
    parser.add_argument("--seconds", type=float, default=50.0)
    parser.add_argument("--seeds", type=int, default=3)
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text())
    for path in N.paths(args.recordings):
        r = real(path)
        if r is None:
            print(f"{path.stem[:19]}: too little tracked hover")
            continue
        line = f"{path.stem[:19]} real: speed_tilt {r[0]:5.2f}°/(m/s) speed_pitch {r[1]:+.2f} offset {r[2]:+.1f}° R² {r[3]:.2f} (speed sd {r[4]:.2f} m/s), balance {r[5]:5.2f}°/(m/s)"
        if args.sim:
            for seed in range(args.seeds):
                m = simulated(path, airframe, args.seconds, seed)
                line += "\n" + (f"    sim seed {seed}: unstable" if m is None else f"    sim seed {seed}: speed_tilt {m[0]:5.2f}°/(m/s) speed_pitch {m[1]:+.2f} offset {m[2]:+.1f}° R² {m[3]:.2f} (speed sd {m[4]:.2f}), balance {m[5]:5.2f}")
        print(line, flush=True)


if __name__ == "__main__":
    main()
