"""
The servos' steady tilt against the forward speed: uv run -m sw.sim.speed_hold <recordings or globs> [--sim] [--static AIRSPEEDS --airframe model.json]

Forward airspeed pitches this airframe nose-up (the rotors flapping back), and the pitch loop holds it with a steady forward thrust tilt, so below the pitch loop's bandwidth the mean servo tilt follows the speed: tilt = speed_pitch × pitch + speed_tilt × forward speed + offset (fw/drone/main/hover.h; fitted on 2026-10-31 and 11-01 at 5.0 degrees per m/s with speed_pitch -0.54). This fits it per flight on the tag-tracked hover, every signal smoothed over SMOOTH seconds. speed_tilt is the airframe's static speed moment over its held tilt authority, the balance outdoor flight runs on (a steady wind is a steady airspeed) and which the multi-step identification cannot see at hover speeds; the hover sim must give the same number. With --sim each recording's settings (and its test pattern) are flown on the sim plant (sweep.CURRENT with --airframe) and the same fit is made on the sim's own tilts, pitch and speed. With --static (added 2026-11-09) it flies each recording's settings on the GPS hold in a steady headwind at each airspeed instead and fits the steady servo tilt and body pitch on the airspeed, the number the 2026-11-09 outdoor flights measure against the GPS (sw/sim/gps_truth.py analysis 2: 7.8 degrees of tilt per m/s, the body about 4.5 degrees nose-up per m/s). A flight with uv run -m sw.excite speed --forward -0.5 --width 3 --rest 3 (away from the tag first) holds steady speeds long enough to pin it; ordinary hovers, at a few cm/s, do not (the sim gives 4-6 there whatever its static balance).
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.flight_log import read_flight, test_pattern
from sw.flight_log import settings as recording_settings
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
    played = test_pattern(path)
    if played:
        plant["excite"] = played
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


def static(job):
    """
    The static speed balance in the hover sim: the GPS hold (exact GPS, no turbulence, no drifting torques) taking off and holding still in a steady headwind of airspeed (m/s, negative a tailwind) for seconds, on a recording's settings. Returns the airspeed, the mean servo tilt and body pitch (deg), ground speed (m/s) over the last half, the distance blown (m) and the nose's angle off the wind (deg), or None when lost or down on the floor.
    """
    airframe, settings, airspeed, seconds, seed = job
    from sw.sim.outdoor import GPS
    from sw.sim.outdoor import plant as outdoor_plant
    from sw.sim.sweep import run
    p, _ = outdoor_plant(airframe, 0.0, seed, True, True, settings.get("hover_climb", 1.0))
    # Air blowing toward the tail (world -x, the nose at engagement facing it) is a forward airspeed.
    p |= {"wind_mean": (-airspeed, 0.0), "wind": 0.0, "wind_vertical": 0.0, "disturbances": [], "gps": GPS | {"velocity_noise": 0.0, "vertical_noise": 0.0, "position_noise": 0.0, "heading": 0.0}}
    # The hold steers by the GPS only once its heading is pinned (79fa5b2), which a drone held still in a steady wind may never do: the calibration pulses pin it.
    s = dict(settings) | {"hover_camera": 0, "hover_center": 0, "hover_gps_heading": 0.0, "hover_gps_calibrate": settings.get("hover_gps_calibrate") or 5.0}
    log = []
    result = run(s, p, seconds=seconds, seed=seed, log=log, hover=True)
    if not result["stable"]:
        return None
    t = np.array([e[0] for e in log])
    position = np.array([e[1] for e in log])
    late = t > seconds / 2
    if position[late, 2].min() < 0.3:
        return None
    tilt = np.degrees([(e[4][0] + e[4][1]) / 2 for e in log])[late]
    pitch = np.degrees([e[2][1] for e in log])[late]
    velocity = np.gradient(position[:, :2], 1 / FS, axis=0)[late]
    speed = np.hypot(*velocity.T)
    # The airspeed along the nose as the airframe feels it (the nose may turn off the wind).
    yaw = np.array([e[2][2] for e in log])[late]
    forward = (velocity[:, 0] + airspeed) * np.cos(yaw) + velocity[:, 1] * np.sin(yaw)
    return float(forward.mean()), float(tilt.mean()), float(pitch.mean()), float(speed.mean()), float(np.hypot(*(position[-1, :2] - position[0, :2]))), float(np.degrees(yaw.mean()))


def static_balance(airframe, settings, airspeeds, seconds=40.0, seed=0, workers=1):
    """static at each airspeed, and the slopes of tilt and pitch on airspeed (deg per m/s) over those held (within 5 m and under 0.3 m/s)."""
    from multiprocessing import Pool
    jobs = [(airframe, settings, v, seconds, seed) for v in airspeeds]
    if workers > 1:
        with Pool(min(workers, len(jobs))) as pool:
            rows = pool.map(static, jobs)
    else:
        rows = [static(job) for job in jobs]
    held = np.array([row for row in rows if row is not None and row[4] < 5.0 and row[3] < 0.3])
    slopes = (np.polyfit(held[:, 0], held[:, 1], 1)[0], np.polyfit(held[:, 0], held[:, 2], 1)[0]) if len(held) >= 2 else (np.nan, np.nan)
    return rows, slopes


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--airframe", type=Path, default=N.A.MODELS / "airframe_2026-11-02_9in.json")
    parser.add_argument("--seconds", type=float, default=50.0)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--static", type=lambda text: [float(v) for v in text.split(",")], help="airspeeds (m/s): the static balance in a steady headwind on each recording's settings instead (static)")
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    airframe = json.loads(args.airframe.read_text())
    if args.static:
        for path in N.paths(args.recordings):
            rows, slopes = static_balance(airframe, recording_settings(path), args.static, args.seconds, 0, args.workers)
            print(f"{path.stem[:19]} settings, {args.airframe.name}: tilt {slopes[0]:.2f}°/(m/s), pitch {slopes[1]:+.2f}°/(m/s) (2026-11-09 GPS truth: 7.8, about -4.5)")
            for v, row in zip(args.static, rows):
                print(f"    {v:+.1f} m/s: " + ("lost" if row is None else f"tilt {row[1]:+6.2f}°, pitch {row[2]:+6.2f}°, ground speed {row[3]:.2f} m/s, {row[4]:.1f} m from where it started, nose {row[5]:+.0f}° off the wind, airspeed along it {row[0]:+.2f} m/s"), flush=True)
        return
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
