"""
A simulated flight as a visual-inertial dataset (sw/vio/dataset.py): the identified airframe (sw/sim/sweep.py IDENTIFIED_ESC) flown by the firmware's hover, in wind, its hold moved sideways partway, with the drone's IMU as it reads it and its FPV camera rendered in an outdoor scene (sw/vio/scene.py).

uv run --with mujoco -m sw.vio.simdata --out /tmp/vio/sim0 [--seconds 40] [--seed 0] [--wind 1.0] [--sync-ms 0] [--jitter-ms 1] [--drop 0.01] [--bytes 5000]
Needs MUJOCO_GL=egl (set here) for offscreen rendering.

The camera timing is the FPV stream's: a frame every PERIOD_S, read out over scene.READOUT_S, JPEG at about --bytes (5000 over UDP, 2600 through fw/fpv_rx), --drop of them lost. Each frame is stamped at the middle of its readout on the drone's clock plus --sync-ms (the error of mapping the camera's clock onto the drone's) and Gaussian --jitter-ms.
"""

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

from sw.sim import sweep  # noqa: E402
from sw.vio import dataset, scene  # noqa: E402

PERIOD_S = 0.026
RECORDING = Path(__file__).resolve().parents[1] / "recordings/2026-10-29T07-20-47-024Z_2551243463.jsonl"
GRAVITY = 9.80665


def fly(seconds, seed, wind, height):
    """The sim's log: per 10 ms tick, position, roll/pitch/yaw and the IMU as read (gyro rad/s, specific force g)."""
    settings = json.loads(RECORDING.read_text().splitlines()[0])["settings"] | {"hover_ceiling": 2.0}
    plant = sweep.IDENTIFIED_ESC | {"wind": wind, "wind_tau": 2.0, "nudge": (6.0, 0.0, 0.3, 5.0),
                                    "camera": sweep.IDENTIFIED_ESC["camera"] | {"tag": (2.0, 0.1, height)}}
    log = []
    result = sweep.run(settings, plant, seconds=seconds, seed=seed, log=log, hover=True)
    return log, result


def poses(log):
    """pose(t): the body's world position and body-to-world rotation at t, interpolated between ticks."""
    times = np.array([row[0] for row in log])
    positions = np.array([row[1] for row in log])
    angles = np.unwrap(np.array([row[2] for row in log]), axis=0)

    def pose(t):
        position = np.array([np.interp(t, times, positions[:, k]) for k in range(3)])
        roll, pitch, yaw = (np.interp(t, times, angles[:, k]) for k in range(3))
        return position, np.array(sweep.rotation(roll, pitch, yaw))

    return pose, times, positions, angles


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", required=True)
    parser.add_argument("--seconds", type=float, default=40)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--wind", type=float, default=1.0, help="m/s RMS per horizontal axis")
    parser.add_argument("--height", type=float, default=1.2, help="the hold's height (m)")
    parser.add_argument("--sync-ms", type=float, default=0.0)
    parser.add_argument("--jitter-ms", type=float, default=1.0)
    parser.add_argument("--drop", type=float, default=0.01)
    parser.add_argument("--bytes", type=int, default=5000)
    parser.add_argument("--still", type=float, default=3.0, help="seconds resting on the ground, motors off, before arming (an estimator's static initialization uses them)")
    args = parser.parse_args()
    log, result = fly(args.seconds, args.seed, args.wind, args.height)
    print(f"flight: stable {result['stable']}, {len(log)} ticks")
    # Resting before arming: the first tick's pose held, the IMU reading gravity with the MPU6050's own noise and no rotor vibration.
    rest = np.random.default_rng(args.seed + 3)
    first = log[0]
    up = np.array(sweep.rotation(*first[2])).T @ np.array([0.0, 0.0, 1.0])
    ticks = round(args.still / sweep.DT)
    still = [(first[0] - (ticks - k) * sweep.DT, first[1], first[2], *first[3:11], list(rest.normal(0, 0.002, 3)), list(up + rest.normal(0, 0.004, 3))) for k in range(ticks)]
    log = still + log
    # Times from 1 s, as a recording's clock would be (some estimators treat zero or negative times as unset).
    shift = 1.0 - log[0][0]
    log = [(row[0] + shift, *row[1:]) for row in log]
    pose, times, positions, angles = poses(log)
    out = dataset.Writer(args.out)
    for row in log:
        gyro, accel = row[11], row[12]
        out.inertial(row[0], gyro, [a * GRAVITY for a in accel])
    velocity = np.gradient(positions, times, axis=0)
    for i in range(len(log)):
        out.ground_truth(times[i], positions[i], np.array(sweep.rotation(*angles[i])), velocity[i])
    rng = np.random.default_rng(args.seed + 7)
    camera, jpeg = scene.Camera(Path(args.out) / "scene", args.seed), scene.Jpeg(args.bytes)
    sizes = []
    start = times[0]
    while start + scene.READOUT_S < times[-1]:
        grey = camera.frame(start, pose)
        data = jpeg(grey)
        sizes.append(len(data))
        if rng.random() >= args.drop:
            import cv2

            decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
            stamp = start + scene.READOUT_S / 2 + args.sync_ms / 1000 + rng.normal(0, args.jitter_ms / 1000)
            out.frame(stamp, cv2.imencode(".png", decoded)[1].tobytes())
        start += PERIOD_S
    camera.close()
    out.close()
    print(f"{len(out.frames)} frames, JPEG mean {np.mean(sizes):.0f} B, {len(out.imu)} IMU samples")


if __name__ == "__main__":
    main()
