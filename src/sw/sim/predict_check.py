"""
How well the servos' dead-time predictor (fw/drone/main/indi.c indi_predict) foresees the drone: uv run -m sw.sim.predict_check FLIGHT ...

Each recording's commands (servo angles, throttles on its thrust table) and gyro are run through the firmware's INDI estimate and predictor tick by tick on the recording's own model settings, as flight_rt.c does, and the predicted change in pitch rate over the predictor's horizon (INDI_SERVO_DELAY - 1 ticks) is set against the gyro's own change over the same ticks, in the hovering ticks, by band: the regression slope (1 is the right size) and the correlation. Over 2026-11-15's hovers (2026-11-12 01:07 and 2026-11-15 04:12 held out), with indi.c's ASSEMBLY_RATIO 0.18 and PREDICT_AUTHORITY 0.7: the 1.5-4 Hz band, where the hold shakes, slope 1.07-1.25 and correlation 0.88-0.96; below 1.5 Hz 0.95-1.20 and 0.85-0.98; above 4 Hz the prediction is as large as the drone's motion there but has no skill (correlation under 0.7).
"""

import argparse

import numpy as np
from scipy.signal import butter, filtfilt

from sw.flight_log import recording, settings
from sw.sim import flight as F
from sw.sim import scorecard as S

BANDS = ((0.3, 1.5), (1.5, 4.0), (4.0, 15.0))
HORIZON = 5


def check(name, listing):
    entry = next(e for e in S.entries(listing) if e["name"].startswith(name))
    fl = S.Flight(entry, listing["configurations"][entry["configuration"]])
    indi = F.Incremental(settings(recording(fl.name)))
    thrust = np.interp(fl.throttle, fl.table[:, 0], fl.table[:, 1]) * (fl.throttle > 0)
    predicted = np.zeros(len(fl.time))
    for k in range(len(fl.time)):
        indi.update(fl.gyro[k], True, 1.0, 900, dt=0.01)
        predicted[k] = indi.predicted[1]
        indi.sent(fl.tilt[k], thrust[k])
    ticks = np.flatnonzero(fl.hovering)
    ticks = ticks[ticks + HORIZON < len(fl.time)]
    out = []
    for lo, hi in BANDS:
        b, a = butter(2, [lo / 50, hi / 50], "band")
        p, g = filtfilt(b, a, predicted)[ticks], filtfilt(b, a, fl.gyro[:, 1])
        g = g[ticks + HORIZON] - g[ticks]
        out.append(f"{lo}-{hi} Hz: slope {np.dot(p, g) / np.dot(p, p):.2f} r {np.corrcoef(p, g)[0, 1]:.2f} ({np.degrees(p.std()):.1f} vs {np.degrees(g.std()):.1f} deg/s)")
    print(f"{entry['name']}  " + "  |  ".join(out))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("flights", nargs="+")
    args = parser.parse_args()
    listing = S.flight_list()
    for name in args.flights:
        check(name, listing)


if __name__ == "__main__":
    main()
