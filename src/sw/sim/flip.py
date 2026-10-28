"""
A roll flip in the full-attitude simulation (sw/sim/rigid.py), flown by the recovery controller (sw/sim/recover.py) with a manoeuvre on top: uv run -m sw.sim.flip [--params flip_params.json] [--n 256]

From a hover at 3 m: hold, punch up at climb_thrust times the weight for climb_time s (level; the motors' 0.2 s response makes a speed loop too slow for this), so the drone is rising when it starts to turn and the turn's fall is spent climbing, then spin about the roll axis (the strong one: differential thrust) toward spin_rate rad/s with the collective at spin_thrust of the weight, counting the rotation from the gyro; with handoff rad of the turn left, the recovery controller takes over to brake, level and hold. The flip's parameters may also carry recovery-controller ones (any key of recover.DEFAULTS), which override the recovery parameters for the flip: the hold after it can be tuned apart from the fall recovery. A flip is clean when the drone never touches the ground, ends within 10° of upright and still, and loses little height.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sw.sim import recover as C
from sw.sim import rigid as R

DEFAULTS = {"climb_thrust": 2.0, "climb_time": 0.5, "spin_rate": 12.0, "k_spin": 25.0, "spin_thrust": 0.6, "handoff": 1.2}
HOLD, CLIMB, SPIN, RECOVER = 0, 1, 2, 3
START = 3.0
FLIP_AT = 1.5


class Flip:
    def __init__(self, control, params):
        self.c, self.p = control, DEFAULTS | (params or {})
        n = control.sim.n
        self.stage = np.full(n, HOLD)
        self.turned = np.zeros(n)
        control.override = self

    def __call__(self, alpha, collective, omega):
        p, c = self.p, self.c
        spinning = self.stage == SPIN
        weight = c.sim.mass * R.G
        spin_alpha = np.column_stack([p["k_spin"] * (p["spin_rate"] - omega[:, 0]), -p["k_spin"] * omega[:, 1], -p["k_spin"] * 0.3 * omega[:, 2]])
        alpha = np.where(spinning[:, None], spin_alpha, alpha)
        collective = np.where(spinning, p["spin_thrust"] * weight, np.where(self.stage == CLIMB, p["climb_thrust"] * weight, collective))
        return alpha, collective

    def advance(self, time, gyro):
        p, c = self.p, self.c
        self.stage = np.where((self.stage == HOLD) & (time >= FLIP_AT), CLIMB, self.stage)
        self.stage = np.where((self.stage == CLIMB) & (time >= FLIP_AT + p["climb_time"]), SPIN, self.stage)
        self.turned += np.where(self.stage == SPIN, gyro[:, 0] * R.DT, 0.0)
        done = (self.stage == SPIN) & (self.turned >= 2 * math.pi - p["handoff"])
        c.phase = np.where(done, C.RIGHT, c.phase)
        self.stage = np.where(done, RECOVER, self.stage)


def run(params, recovery, n=256, seed=0, seconds=10.0, randomize=True, record=False):
    recovery = (recovery or {}) | {key: value for key, value in (params or {}).items() if key in C.DEFAULTS}
    sim = R.Sim(n, seed=seed, scale=C.randomization(n, seed) if randomize else None)
    hover = sim.mass * R.G / 2
    obs = sim.reset(np.tile([0, 0, START], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)), motor=np.column_stack([np.full(n, hover), np.full(n, hover)]))
    control = C.Controller(sim, recovery | {"engage": 0.0, "descent": 0.0})
    control.reset(sim.quat.copy(), np.zeros((n, 3)), np.full(n, START))
    control.phase[:] = C.ARREST
    control.thrust[:] = hover
    control.asked[:] = hover
    flip = Flip(control, params)
    lowest = np.full(n, START)
    highest = np.full(n, START)
    trace = []
    for k in range(int(seconds / R.DT)):
        flip.advance(sim.time, obs["gyro"])
        throttle, tilt = control(obs, sim.time)
        obs = sim.step(throttle, tilt)
        lowest = np.minimum(lowest, sim.pos[:, 2])
        highest = np.maximum(highest, sim.pos[:, 2])
        if record:
            trace.append((sim.time, sim.pos.copy(), sim.quat.copy(), sim.motor.copy(), sim.tilt.copy(), np.where(flip.stage == SPIN, 0, 1)))
    up = sim.up()[:, 2]
    still = np.linalg.norm(sim.omega, axis=1) < 1.0
    clean = ~sim.touched & (up > math.cos(math.radians(10))) & still & (flip.turned >= 2 * math.pi - flip.p["handoff"] - 0.01)
    result = {"clean": clean, "lost": START - lowest, "climbed": highest - START, "touched": sim.touched, "up": up, "turned": flip.turned}
    return (result, trace) if record else result


def summary(r):
    return f"clean {r['clean'].mean() * 100:.1f}% of {len(r['clean'])} | height lost median {np.median(r['lost']):.2f} m (90th {np.percentile(r['lost'], 90):.2f}), climbed median {np.median(r['climbed']):.2f} m, hit the ground {r['touched'].mean() * 100:.0f}%, upright at the end {(r['up'] > math.cos(math.radians(10))).mean() * 100:.0f}%"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path)
    parser.add_argument("--recovery", type=Path, default=Path(__file__).with_name("models") / "recover_params.json")
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--nominal", action="store_true")
    args = parser.parse_args()
    params = json.loads(args.params.read_text()) if args.params else {}
    recovery = json.loads(args.recovery.read_text()) if args.recovery.exists() else {}
    print(summary(run(params, recovery, args.n, randomize=not args.nominal)))


if __name__ == "__main__":
    main()
