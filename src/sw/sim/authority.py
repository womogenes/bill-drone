"""
The airframe's one model (settings.def roll_authority, pitch_authority, yaw_authority; indi.c ASSEMBLY_RATIO) fitted to the drone's flights the way the firmware uses it: uv run -m sw.sim.authority [--model ROLL PITCH SWING YAW]

INDI and the servos' dead-time predictor (fw/drone/main/indi.c) take the model's angular accelerations for the actuators as estimated from what was sent (the servos' dead time, loop and slew, the motors' dead time and lag), and the predictor adds what the drone has beyond the model, low-passed over PREDICT_TAU and held. So each tick k of every airborne, unclipped stretch of the new ESCs' flights since the pivots were raised (the hops left out: their feet push), the body rate's change over the next HORIZON ticks is foretold as the model's accelerations over them (the firmware's own estimate, flight.Incremental, read tick by tick) plus HORIZON ticks of the held residual, and the authorities are the least squares fit of that on the one plant's training flights (scorecard_flights.json training airframe_2026-11-15_plant); it is linear in them. Roll: per newton of the left motor's thrust over the right's. Pitch: per newton of forward thrust (thrust times the sine of the tilt, summed) and the assemblies' swing (their summed tilt acceleration, ASSEMBLY_RATIO). Yaw: per newton of the right's forward thrust over the left's. Held out, per kind of flight, the share of the rate's change explained (1 - residual variance over the change's); with --model, the same for a model as given.

2026-11-15, the first fit (servos 60 ms, wn 37.5, zeta 0.87; motors 40 ms and 35 ms): roll 8.0, pitch 8.68 with swing 0.106, yaw 11.35; held out, roll 0.66 of the hovers' change, 0.79 of the doublets' and bursts', pitch 0.79 / 0.89 / 0.88, yaw 0.59 / 0.41 / 0.66. The models it replaced on the same flights: the controller's (roll 14.5, pitch 8.68 with swing 0.18, yaw 16.1 with the props' reaction) roll 0.23 / 0.50 / 0.54, pitch 0.76 / 0.89 / 0.78, yaw 0.27 / 0.21 / 0.40; the predictor's (0.7 of the controller's pitch) 0.75 / 0.83 / 0.79; the burst's (13.5, 16.8, 11.34) roll 0.33 / 0.58 / 0.62, pitch 0.05 / 0.24 / 0.49, yaw 0.59 / 0.39 / 0.64. The old ESCs' flights fit apart (roll 14.6 with 0.16 explained, yaw 27): their table is not the thrust. Fits by frequency band instead come out low below 1.5 Hz and high above, the closed loop's doing (the hold moves the actuators against what moves the drone), which this criterion, the firmware's own, does not ask about. Dead times and lags fitted on the same criterion (models/NOTES.md "One model"): the servos' sit on a ridge where 5-7 ticks with wn 25-60 and zeta 0.5-1.1 all explain within 0.015 of pitch's best; the motors' 1 tick and 66 ms explained 0.71 of the held-out roll, 3-5 ticks with 20-50 ms 0.75-0.76; the props' reaction in yaw and a rate damping in any axis explained under 0.01 more and were left out.
"""

import argparse
import re

import numpy as np
from scipy.signal import butter, lfilter

from sw.sim import flight as F
from sw.sim import scorecard as S

INDI = (F.SOURCE.parent / "indi.c").read_text()
HORIZON = int(re.search(r"#define INDI_SERVO_DELAY (\d+)", (F.SOURCE.parent / "indi.h").read_text()).group(1)) - 1
FILTER_HZ = float(re.search(r"#define FILTER_HZ ([0-9.]+)f", INDI).group(1))
PREDICT_TAU = float(re.search(r"#define PREDICT_TAU ([0-9.]+)f", INDI).group(1))
DT = 0.01
CONFIGURATIONS = ("nobands_new_esc", "bands_new_esc", "boat")
TRAINING = "airframe_2026-11-15_plant"
# Regressor columns: roll's, pitch's two, yaw's.
AXES = {"roll": (0, [0]), "pitch": (1, [1, 2]), "yaw": (2, [3])}


def held(x):
    """indi.c's beyond: a first-order low-pass over PREDICT_TAU, each tick's value as it stands then."""
    out, acc, share = np.zeros_like(x), np.zeros(x.shape[1:]), DT / (PREDICT_TAU + DT)
    for k in range(len(x)):
        acc = acc + (x[k] - acc) * share
        out[k] = acc
    return out


def flight_data(fl):
    """The regressors (ticks, 4) and targets (ticks, 3) at each usable tick of a flight, and the raw change for the score."""
    indi = F.Incremental({})
    thrust = np.interp(fl.throttle, fl.table[:, 0], fl.table[:, 1]) * (fl.throttle > 0)
    n = len(fl.time)
    columns, before = np.zeros((n, 4)), np.zeros(2)
    for k in range(n):
        indi.update(fl.gyro[k], False, 0.0, 0.0, dt=DT)
        s = indi.state
        tilt, rate, motor = np.array(s.tilt[:]), np.array(s.tilt_rate[:]), np.array(s.thrust[:])
        vertical, forward = motor * np.cos(tilt), motor * np.sin(tilt)
        columns[k] = (vertical[0] - vertical[1], forward.sum(), -(rate - before).sum() / DT, forward[1] - forward[0])
        before = rate
        indi.sent(fl.tilt[k], thrust[k])
    ticks = np.flatnonzero(fl.airborne & ~fl.clipped)
    ticks = ticks[(ticks > 100) & (ticks + HORIZON + 1 < n)]
    ticks = ticks[~fl.clipped[ticks + HORIZON]]
    # The model's accelerations over the next HORIZON ticks (the ticks the predictor steps through), less the held residual's share of them; the residual's measured part goes to the target.
    total = np.vstack([np.zeros((1, 4)), np.cumsum(columns, 0)])
    ahead = (total[ticks + HORIZON + 1] - total[ticks + 1]) * DT
    smooth = butter(2, FILTER_HZ / 50)
    measured = lfilter(*smooth, np.vstack([np.zeros((1, 3)), np.diff(fl.gyro, axis=0) / DT]), axis=0)
    modeled = lfilter(*smooth, columns, axis=0)
    X = ahead - HORIZON * DT * held(np.vstack([modeled[:1], modeled[:-1]]))[ticks]
    change = fl.gyro[ticks + HORIZON] - fl.gyro[ticks]
    return X, change - HORIZON * DT * held(measured)[ticks], change


def explained(rows, axis, columns, coef):
    X = np.vstack([r[2][:, columns] for r in rows])
    y = np.concatenate([r[3][:, axis] for r in rows])
    raw = np.concatenate([r[4][:, axis] for r in rows])
    return 1 - np.var(y - X @ coef) / np.var(raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", nargs=4, type=float, metavar=("ROLL", "PITCH", "SWING", "YAW"), help="score this model too")
    args = parser.parse_args()
    listing = S.flight_list()
    training = set(listing["training"][TRAINING])
    rows = []
    for entry in S.entries(listing):
        if entry["configuration"] not in CONFIGURATIONS or entry["kind"] == "hop":
            continue
        fl = S.Flight(entry, listing["configurations"][entry["configuration"]])
        rows.append((entry["kind"], entry["name"] in training) + flight_data(fl))
    kinds = (("hover", "hover"), ("doublet", "doublet"), ("burst", "burst"), ("divergence", "divergence"))
    for name, (axis, columns) in AXES.items():
        train = [r for r in rows if r[1]]
        X = np.vstack([r[2][:, columns] for r in train])
        y = np.concatenate([r[3][:, axis] for r in train])
        coef = np.linalg.lstsq(X, y, rcond=None)[0]
        models = [("fit", coef)]
        if args.model:
            given = {"roll": [args.model[0]], "pitch": args.model[1:3], "yaw": [args.model[3]]}[name]
            models.append(("given", np.array(given)))
        for label, c in models:
            scores = [f"{kind} {explained(held_out, axis, columns, c):.2f} ({len(held_out)})" for kind, prefix in kinds if (held_out := [r for r in rows if not r[1] and r[0].startswith(prefix)])]
            print(f"{name:5s} {label:5s} {np.array2string(c, precision=3)}  held out: " + ", ".join(scores))


if __name__ == "__main__":
    main()
