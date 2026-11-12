"""
Which untested term decides a flip: fly a parameter set on drawn plants (sw/sim/tune_flip.py's draws, flip_firmware.uncertain) and set each draw's outcome against what was drawn: uv run -m sw.sim.flip_sensitivity params.json [--n 512] [--workers 40]

Prints, for each drawn term, the clean share and ground hits in the lower and upper third of its range, so a term whose thirds differ is one the flip's outcome hangs on (and a real measurement of it is worth most).
"""

import argparse
import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.flight_log import recording, settings as recording_settings
from sw.sim import flip_firmware as FF
from sw.sim import tune_flip as T


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", type=Path)
    parser.add_argument("--n", type=int, default=512)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=50000)
    args = parser.parse_args()
    settings = recording_settings(recording(T.SETTINGS)) | FF.FLIGHT | T.HIGH | T.FAST_CLIMB
    defaults = {name: getattr(FF.DEFAULTS, name) for name, *_ in FF.Params._fields_}
    params = defaults | json.loads(args.params.read_text())
    jobs = T.jobs(params, args.n, args.seed, settings)
    with Pool(args.workers) as pool:
        results = pool.map(FF.fly, jobs, chunksize=1)
    table = []
    for job, rows in zip(jobs, results):
        overrides, scale, gyro = FF.uncertain(job[2], job[3])
        for k, r in enumerate(rows):
            if r["started"] is None:
                continue
            drawn = overrides | {key: float(value[k]) for key, value in scale.items()} | {"band": job[1], "gyro_scale": float(gyro[k].mean())}
            table.append((drawn, r))
    print(f"{args.params}: {T.summary([r for _, r in table])}")
    for key in table[0][0]:
        values = np.array([d[key] for d, _ in table])
        clean = np.array([r["clean"] for _, r in table])
        touched = np.array([r["touched"] for _, r in table])
        low, high = np.percentile(values, [33.3, 66.7])
        parts = [(values <= low), (values > low) & (values <= high), (values > high)]
        text = "  ".join(f"{values[m].min():.3g}-{values[m].max():.3g}: clean {clean[m].mean() * 100:3.0f}% ground {touched[m].mean() * 100:3.0f}%" for m in parts if m.any())
        print(f"  {key:16} {text}")


if __name__ == "__main__":
    main()
