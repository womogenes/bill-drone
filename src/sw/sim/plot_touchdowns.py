"""
Raw data around touchdown for four landings of 2026-11-11/12 (UTC): uv run -m sw.sim.plot_touchdowns [--out sw/sim/models/landing_touchdowns_2026-11-12.png]

Columns: 22:59 indoors and 01:15 outdoors, which tipped over backward after the touchdown cut, 01:11 which stayed up but whose motors ran 1.9 s on the floor, and 01:09 whose motors ran 2.4 s on the floor. Rows: pitch (positive nose-down), pitch rate, the accelerometer's body-up specific force (clipped at 2 g), the servos' mean physical tilt (positive forward), the base throttle and the hover's height estimate. Time is from the contact (the first sample of the last run above 1.2 g up to the cut, or read off the log where no jolt marks it); the orange line is the cut.
"""

import argparse
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sw.flight_log import read_flight, recording

# Recording, label, contact (s from the recording's start; None: the jolt before the cut).
LANDINGS = [("2026-11-11T22-59", "22:59 indoors, tipped back", None), ("2026-11-12T01-15", "01:15 outdoors, tipped back", None),
            ("2026-11-12T01-11", "01:11 outdoors, late cut, upright", 39.50), ("2026-11-12T01-09", "01:09 outdoors, late cut", 56.30)]
INK, MUTED, GRID, BLUE, ORANGE = "#0b0b0b", "#52514e", "#e4e3df", "#2a78d6", "#eb6834"


def load(name, contact):
    data, _ = read_flight(recording(name))
    data = data.reset_index(drop=True)
    t = (data.esp_time_us.values - data.esp_time_us.values[0]) / 1e6
    armed = data["flags"].values.astype(int) & 1
    cut = t[len(armed) - 1 - np.argmax(armed[::-1])] + 0.01
    if contact is None:
        k = np.searchsorted(t, cut + 0.01)
        while k > 0 and data.az_g.values[k] <= 1.2:
            k -= 1
        while k > 0 and data.az_g.values[k - 1] > 1.2:
            k -= 1
        contact = t[k]
    return data, t - contact, cut - contact


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", default="sw/sim/models/landing_touchdowns_2026-11-12.png")
    args = parser.parse_args()
    rows = [("pitch (deg)", lambda d: d.pitch_deg), ("pitch rate (deg/s)", lambda d: d.gy_deg_s), ("body-up specific force (g)", lambda d: d.az_g),
            ("servo tilt, mean (deg fwd)", lambda d: (d.left_servo_deg - d.right_servo_deg) / 2), ("base throttle", lambda d: d.base_throttle), ("height estimate (m)", lambda d: d.hover_height_m)]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(len(rows), len(LANDINGS), figsize=(16, 13), sharex="col", sharey="row")
    for column, (name, label, contact) in enumerate(LANDINGS):
        data, t, cut = load(name, contact)
        shown = (t > -1.0) & (t < cut + 0.8)
        for row, (ylabel, value) in enumerate(rows):
            axis = axes[row, column]
            axis.grid(True, color=GRID, linewidth=0.6)
            axis.plot(t[shown], value(data)[shown], color=BLUE, linewidth=1.5)
            axis.axvline(0, color=MUTED, linestyle="--", linewidth=1, label="contact")
            axis.axvline(cut, color=ORANGE, linewidth=1.5, label="motor cut")
            if column == 0:
                axis.set_ylabel(ylabel)
        axes[0, column].set_title(label, color=INK)
        axes[-1, column].set_xlabel("s from contact")
    axes[0, 0].legend(loc="lower left", frameon=False)
    figure.tight_layout()
    figure.savefig(args.out, dpi=110)
    print(args.out)


if __name__ == "__main__":
    main()
