"""
Parity of sw/sim/rigid.py's C backend (sw/sim/rigid_step.c) with the numpy reference, and their speed: uv run python -m sw.sim.test_rigid [--ticks 300] [--bench]

Both backends start from identical random states (any orientation, fast rates and speeds, heights from below the floor to well above it, actuators mid-motion), take the same random commands and randomized plant scales, with the disturbances off and on (the same random stream), and must agree to TOLERANCE relative to each quantity's size. The two differ only by rounding (summation order, libm against numpy's sin and cos), but the ground contact is chaotic (stiff friction, thresholds), so those 1e-16 differences grow to O(1) within about 100 ticks of tumbling on the floor. So the test checks: the whole run with the C sim resynchronised to the numpy state before every tick (every tick's map, over all states visited), a free run of FREE ticks from the start, and a free run of the whole length well above the floor.
"""

import argparse
import time

import numpy as np

from sw.sim import rigid as R

TOLERANCE = 1e-9
FREE = 20
STATE = ("pos", "vel", "quat", "omega", "accel", "contact", "contact_now", "touched", "throttles", "commands")
ACTUATORS = ("rate", "att", "vel", "motor", "horn", "hvel", "tilt", "tvel")
FIELDS = ("pos", "vel", "quat", "omega", "accel", "contact", "contact_now", "motor", "horn", "hvel", "tilt", "tvel", "rate", "att", "vel_body", "gyro", "sensed_accel")


def start(n, seed, disturbances, backend, height=(-0.05, 0.8)):
    rng = np.random.default_rng(seed)
    scale = {key: rng.uniform(0.7, 1.3, n) for key in ("pitch_tilt", "roll_thrust", "inertia", "drag", "disturbance")}
    plant = R.load_plant() if disturbances else R.load_plant(disturbances=None)
    sim = R.Sim(n, plant=plant, seed=seed + 1, scale=scale, backend=backend)
    position = np.column_stack([rng.normal(0, 1, (n, 2)), rng.uniform(*height, n)])
    velocity = rng.normal(0, 2, (n, 3))
    omega = rng.normal(0, 6, (n, 3))
    motor = rng.uniform(0, 5, (n, 2)) * (rng.random((n, 1)) < 0.8)
    tilt = rng.uniform(-0.8, 0.8, (n, 2))
    obs = sim.reset(position, velocity, R.random_quat(rng, n), omega, motor=motor, tilt=tilt)
    # Actuators mid-motion rather than steady.
    sim.state.hvel[:] = rng.normal(0, 100, (n, 2))
    sim.state.tvel[:] = rng.normal(0, 100, (n, 2))
    sim.state.horn[:] += rng.normal(0, 5, (n, 2))
    return sim, obs


def snapshot(sim, obs):
    st = sim.state
    values = {"pos": sim.pos, "vel": sim.vel, "quat": sim.quat, "omega": sim.omega, "accel": sim.accel, "contact": sim.contact, "contact_now": sim.contact_now}
    values |= {"motor": st.motor, "horn": st.horn, "hvel": st.hvel, "tilt": st.tilt, "tvel": st.tvel, "rate": st.rate, "att": st.att, "vel_body": st.vel, "gyro": obs["gyro"], "sensed_accel": obs["accel"]}
    return {key: np.array(value, float) for key, value in values.items()}


def difference(a, b):
    """The largest difference of each field relative to that field's size (at least 1)."""
    return {key: float(np.max(np.abs(a[key] - b[key])) / max(1.0, float(np.max(np.abs(b[key]))))) for key in FIELDS}


def resync(target, source):
    """Copy source's state into target's arrays in place (the C backend keeps pointers to them)."""
    for name in STATE:
        getattr(target, name)[:] = getattr(source, name)
    for name in ACTUATORS:
        getattr(target.state, name)[:] = getattr(source.state, name)


def parity(n=256, ticks=300, seed=0, disturbances=True, synced=True, free=None, height=(-0.05, 0.8)):
    """The largest relative difference over the run; synced resynchronises before every tick after the first free ticks."""
    sims = [start(n, seed, disturbances, backend, height) for backend in ("numpy", "c")]
    rng = np.random.default_rng(seed + 2)
    worst, windows = {}, []
    free = ticks if free is None else free
    for tick in range(ticks):
        if synced and tick >= free:
            resync(sims[1][0], sims[0][0])
        # Random commands held for a few ticks, beyond the clip limits now and then.
        if tick % 5 == 0:
            throttle = rng.uniform(-100, 1400, (n, 2)) * (rng.random((n, 1)) < 0.9)
            tilt = rng.uniform(-1.3, 1.3, (n, 2))
        states = [snapshot(sim, sim.step(throttle, tilt)) for sim, obs in sims]
        error = difference(states[1], states[0])
        assert (sims[0][0].touched == sims[1][0].touched).all(), f"touch flags differ at tick {tick}"
        for key, value in error.items():
            worst[key] = max(worst.get(key, 0.0), value)
        if tick % 50 == 49 or tick == free - 1:
            windows.append((tick + 1, max(worst.values())))
    touched = sims[0][0].touched.mean()
    landed = (sims[0][0].contact_now > 0).mean()
    mode = f"free {free} ticks then resynchronised every tick" if synced and free < ticks else "free run"
    print(f"disturbances {'on' if disturbances else 'off'}, {mode}: {n} envs, {ticks} ticks, heights {height[0]}-{height[1]} m, {100 * touched:.0f}% touched the ground, {100 * landed:.0f}% in contact at the end")
    print("  largest relative difference so far, by tick: " + ", ".join(f"{tick}: {value:.1e}" for tick, value in windows))
    print("  worst field: " + max(worst, key=worst.get) + f" {max(worst.values()):.1e}")
    return worst


def bench(n, backend, seconds=1.0):
    sim, obs = start(n, 0, True, backend)
    throttle, tilt = np.full((n, 2), 600.0), np.zeros((n, 2))
    sim.step(throttle, tilt)
    count, began = 0, time.perf_counter()
    while time.perf_counter() - began < seconds:
        for _ in range(10):
            sim.step(throttle, tilt)
        count += 10
    return n * count / (time.perf_counter() - began)


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--ticks", type=int, default=300)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--bench", action="store_true", help="also measure env-steps per second")
    args = parser.parse_args()
    for disturbances in (False, True):
        for case in ({"free": FREE}, {"free": FREE, "synced": False, "ticks": FREE}, {"synced": False, "height": (40.0, 60.0)}):
            case = {"ticks": args.ticks} | case
            worst = parity(args.n, disturbances=disturbances, **case)
            assert max(worst.values()) < TOLERANCE, worst
    print(f"parity within {TOLERANCE:g}")
    if args.bench:
        for n in (64, 512, 4096):
            speeds = {backend: bench(n, backend) for backend in ("numpy", "c")}
            print(f"n={n}: numpy {speeds['numpy']:,.0f}, c {speeds['c']:,.0f} env-steps/s ({speeds['c'] / speeds['numpy']:.1f}x)")


if __name__ == "__main__":
    main()
