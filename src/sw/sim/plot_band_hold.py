"""
The 2026-11-11 tag holds with and without the rubber bands on the servos, as logged: uv run -m sw.sim.plot_band_hold [recordings ...] [--out models/band_hold_2026-11-11.png]

One column per recording, from the hold's first sighting airborne: the range to the tag against its target (the setpoint less the pilot's nudge), the pitch flown against the pitch the hold asked for, the logged carry (the trim less the push, what the drone logs as its trim) with the push and trim the flown hover.c had (sw/sim/hold_replay.py with DRAG_CAMERA_BIAS 0; not logged), and the servos' mean commanded forward tilt beyond their trims with the pitch plus it.
"""

import argparse
import os
from pathlib import Path

import numpy as np

# The push as the flown firmware learned it (before DRAG_CAMERA_BIAS).
os.environ["HOVER_DEFINES"] = "DRAG_CAMERA_BIAS=0"
from sw.flight_log import read_flight, recording, settings  # noqa: E402
from sw.sim.hold_replay import replay  # noqa: E402

RECORDINGS = ["2026-11-11T22-57", "2026-11-11T23-17", "2026-11-11T23-35"]
TITLES = {"2026-11-11T22-57": "22:57 UTC, no bands", "2026-11-11T23-17": "23:17 UTC, bands", "2026-11-11T23-35": "23:35 UTC, bands"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="*", default=RECORDINGS)
    parser.add_argument("--out", type=Path, default=Path(__file__).with_name("models") / "band_hold_2026-11-11.png")
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, len(args.recordings), figsize=(5.5 * len(args.recordings), 11), sharex="col", squeeze=False)
    for column, name in enumerate(args.recordings):
        path = recording(name)
        data, _ = read_flight(path)
        s = settings(path)
        # The replay runs over the hover's ticks only; the log is cut to the same ticks.
        hold = data[((data["flags"].to_numpy().astype(int) >> 1) & 3) != 0]
        r = replay(path)
        n = min(len(hold), len(r["t"]))
        hold, t = hold.iloc[:n], r["t"][:n]
        start = t[np.argmax((r["tracking"][:n] > 0) & (r["height"][:n] > 0.15))]
        t = t - start
        servo = ((hold.left_servo_deg - s["left_trim"]) - (hold.right_servo_deg - s["right_trim"])).to_numpy() / 2
        smooth = lambda x: np.convolve(x, np.ones(100) / 100, mode="same")
        ax = axes[0, column]
        ax.plot(t, np.hypot(hold.hover_x_m, hold.hover_y_m), label="range")
        ax.plot(t, hold.hover_range_target_m, label="target")
        ax.set_ylabel("m")
        ax.set_title(TITLES.get(name, name))
        ax = axes[1, column]
        ax.plot(t, hold.pitch_deg, lw=0.5, alpha=0.5, label="pitch")
        ax.plot(t, hold.target_pitch_deg, lw=0.8, label="pitch asked (target)")
        ax.plot(t, smooth(hold.target_pitch_deg.to_numpy()), lw=1.5, label="pitch asked, 1 s mean")
        ax.set_ylabel("degrees, + nose down")
        ax = axes[2, column]
        ax.plot(t, hold.hover_trim_x_m_s2, label="carry (logged)")
        ax.plot(t, r["push"][:n], label="push (replayed)")
        ax.plot(t, r["trim"][:n], label="trim (replayed)")
        ax.set_ylabel("m/s², + forward")
        ax = axes[3, column]
        ax.plot(t, smooth(servo), label="servo tilt beyond trim, 1 s mean")
        ax.plot(t, smooth(hold.pitch_deg.to_numpy() + servo), label="pitch + servo tilt, 1 s mean")
        ax.set_ylabel("degrees, + forward")
        ax.set_xlabel("s from first sighting airborne")
        for ax in axes[:, column]:
            ax.axhline(0, color="gray", lw=0.5)
            ax.set_xlim(-1, t[-1])
            ax.grid(alpha=0.3)
    for ax in axes[:, 0]:
        ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.out, dpi=90)
    print(args.out)


if __name__ == "__main__":
    main()
