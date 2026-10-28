"""
Where the fall recovery (sw/sim/recover.py) works: safe landings by drop height and starting tilt, uv run python -m sw.sim.recover_map [--params params.json] [--envs 64] [--workers 6] [--out recover_map.png]

Each cell drops envs drones from HEIGHTS m, tilted TILTS degrees from upright (180 is inverted) about a random horizontal axis at a random heading, with plants randomized as recover.run does, for 20 s. The four panels start still or tumbling (RATE rad/s about a random axis) and at rest or moving (velocity N(0, 1) m/s per axis). Saves a heatmap of the safe share per cell and prints, per cell, the safe share and the median touchdown speed and tilt.
"""

import argparse
import itertools
import json
import math
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.sim import recover as C
from sw.sim import rigid as R

HEIGHTS = (2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
TILTS = (0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
RATE = 3.0
PANELS = [(tumbling, moving) for tumbling in (False, True) for moving in (False, True)]


def tilted(rng, tilt):
    """Quaternions tilted tilt radians [env] from upright about a random horizontal axis, at a random heading."""
    n = len(tilt)
    heading = rng.uniform(0, 2 * math.pi, n)
    yaw = np.column_stack([np.cos(heading / 2), np.zeros(n), np.zeros(n), np.sin(heading / 2)])
    axis = rng.uniform(0, 2 * math.pi, n)
    lean = np.column_stack([np.cos(tilt / 2), np.sin(tilt / 2) * np.cos(axis), np.sin(tilt / 2) * np.sin(axis), np.zeros(n)])
    return R.quat_multiply(lean, yaw)


def cell_runs(job):
    """One panel at one height, every tilt: (safe, touchdown speed, touchdown tilt) [tilt, env]."""
    params, height, tumbling, moving, envs, seed = job
    n = envs * len(TILTS)
    tilt = np.radians(np.repeat(TILTS, envs))

    def scenarios(count, s, height_range=None):
        rng = np.random.default_rng(s)
        position = np.column_stack([np.zeros(count), np.zeros(count), np.full(count, height)])
        velocity = rng.normal(0, 1.0, (count, 3)) if moving else np.zeros((count, 3))
        axis = rng.normal(0, 1, (count, 3))
        omega = axis / np.linalg.norm(axis, axis=1, keepdims=True) * RATE if tumbling else np.zeros((count, 3))
        return position, velocity, tilted(rng, tilt), omega

    C.scenarios = scenarios
    result = C.run(params, n=n, seed=seed, seconds=20.0, randomize=True)
    shape = (len(TILTS), envs)
    return result["safe"].reshape(shape), result["first_speed"].reshape(shape), result["first_tilt"].reshape(shape)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=Path(__file__).with_name("models") / "recover_params.json")
    parser.add_argument("--envs", type=int, default=64)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--out", type=Path, default=Path("recover_map.png"))
    args = parser.parse_args()
    params = json.loads(args.params.read_text())
    jobs = [(params, h, tumbling, moving, args.envs, 1000 * p + i) for p, (tumbling, moving) in enumerate(PANELS) for i, h in enumerate(HEIGHTS)]
    with Pool(args.workers) as pool:
        out = pool.map(cell_runs, jobs)
    safe = np.zeros((len(PANELS), len(HEIGHTS), len(TILTS)))
    speed, angle = np.zeros_like(safe), np.zeros_like(safe)
    for (params_, h, tumbling, moving, envs, seed), (s, v, t) in zip(jobs, out):
        p, i = PANELS.index((tumbling, moving)), HEIGHTS.index(h)
        safe[p, i] = s.mean(1)
        speed[p, i] = np.nanmedian(v, 1)
        angle[p, i] = np.nanmedian(t, 1)
    names = [f"{'tumbling ' + str(RATE) + ' rad/s' if tumbling else 'no rotation'}, {'moving N(0, 1) m/s' if moving else 'at rest'}" for tumbling, moving in PANELS]
    print(f"{args.params}: safe landings, {args.envs} drops per cell, plants randomized; overall {safe.mean() * 100:.0f}%")
    for p, name in enumerate(names):
        print(f"\n{name} (overall {safe[p].mean() * 100:.0f}%): safe % / median touchdown speed m/s / median touchdown tilt °")
        print("  height  " + "".join(f"{t:>16.0f}°" for t in TILTS))
        for i, h in enumerate(HEIGHTS):
            print(f"  {h:4.0f} m  " + "".join(f"{safe[p, i, j] * 100:5.0f}% {speed[p, i, j]:4.1f} {angle[p, i, j]:3.0f}°" for j in range(len(TILTS))))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 9), sharex=True, sharey=True, constrained_layout=True)
    for p, ax in enumerate(axes.flat):
        image = ax.imshow(safe[p] * 100, origin="lower", cmap="viridis", vmin=0, vmax=100, aspect="auto")
        for i, j in itertools.product(range(len(HEIGHTS)), range(len(TILTS))):
            ax.text(j, i, f"{safe[p, i, j] * 100:.0f}", ha="center", va="center", fontsize=8, color="white" if safe[p, i, j] < 0.6 else "black")
        ax.set_title(f"{names[p]} ({safe[p].mean() * 100:.0f}% overall)", fontsize=10)
        ax.set_xticks(range(len(TILTS)), [f"{t:.0f}" for t in TILTS])
        ax.set_yticks(range(len(HEIGHTS)), [f"{h:.0f}" for h in HEIGHTS])
    for ax in axes[1]:
        ax.set_xlabel("starting tilt from upright (°; 180 is inverted)")
    for ax in axes[:, 0]:
        ax.set_ylabel("drop height (m)")
    fig.colorbar(image, ax=axes, label="safe landings (%)", shrink=0.8)
    fig.suptitle(f"Fall recovery in the full-attitude sim: safe landings per {args.envs} drops, plants randomized ({args.params.name})")
    fig.savefig(args.out, dpi=120)
    print(f"\nsaved {args.out}")


if __name__ == "__main__":
    main()
