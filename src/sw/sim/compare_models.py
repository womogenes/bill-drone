"""
Airframe models against the same real flights, open loop on the logged commands (sw/sim/flip_replay.py): uv run -m sw.sim.compare_models airframe_a.json airframe_b.json ...

Rate error RMS (roll/pitch/yaw, deg/s) with the rates and attitude set back to the log every 0.2 s and every 1 s, over: the first flip (2026-11-12 02:27:04, 9.85-11.0 s), the post-raise roll-step ladders (12 steps, 2.8 s each), and post-raise and pre-raise GPS-hold hovers. Each model flies at its own geometry.
"""

import argparse

import numpy as np

from sw.sim import airframe as A
from sw.sim import flip_replay as F

LADDERS = {"2026-11-12T01-18-33": (15.48, 18.88, 22.28, 25.68, 29.08, 32.48), "2026-11-12T01-55-51": (14.11, 17.51, 20.91, 24.31), "2026-11-12T01-59-36": (15.29, 18.69)}
HOVERS_RAISED = {"2026-11-12T02-27-04": (4.0, 9.8), "2026-11-12T02-21-10": (8.0, 20.0), "2026-11-12T02-14-11": (8.0, 25.0), "2026-11-12T01-54-10": (10.0, 30.0)}
HOVERS_BEFORE = {"2026-11-09T06-19-08": (20.0, 60.0), "2026-11-09T05-53-53": (20.0, 50.0)}


def score(airframe, windows, every):
    rows = []
    for name, start, end in windows:
        log = F.Log(name, start, end)
        result = F.replay(log, [F.plant_for(log, A.model(airframe))], every=every)
        rows.append(F.errors(log, result, 1e9)[0])
    return np.mean(rows, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("airframes", nargs="+")
    args = parser.parse_args()
    sets = {
        "flip 9.85-11.0 s": [("2026-11-12T02-27-04", 9.85, 11.0)],
        "ladders (post-raise)": [(name, t - 0.3, t + 2.5) for name, starts in LADDERS.items() for t in starts],
        "hovers (post-raise)": [(name, *window) for name, window in HOVERS_RAISED.items()],
        "hovers (2026-11-09, pre-raise)": [(name, *window) for name, window in HOVERS_BEFORE.items()],
    }
    for label, windows in sets.items():
        for every in (0.2, 1.0):
            line = f"{label}, set back every {every:g} s:"
            for airframe in args.airframes:
                line += f"  {airframe} {'/'.join(f'{v:.1f}' for v in score(airframe, windows, every))}"
            print(line, flush=True)


if __name__ == "__main__":
    main()
