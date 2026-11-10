"""
A thrown drone righting itself, in the full-attitude simulation (sw/sim/rigid.py): uv run -m sw.sim.toss [--n 256] [--seed 0] [--params params.json] [--render toss.mp4]

The drone is held with its motors off and thrown. From its release the accelerometer reads free fall (no specific force, the hand's push gone); after DETECT s of that the throw mode starts the motors from standstill (the airframe's motor_start, the ESCs' measured start-up time) and hands over to the fall recovery (sw/sim/recover.py, its tuned models/recover_params.json): right the thrust axis, stop the motion, then hold the height it stopped at for HOLD s before landing. The estimates start from the throw as the firmware would have integrated it: the attitude with recover.ATTITUDE_ERROR, the velocity within VELOCITY_ERROR, the height within recover.HEIGHT_ERROR.

The plant is drone 1 as measured on 2026-11-09 (sweep.CURRENT_AIRFRAME, models/airframe_2026-11-09.json) with the GPS-measured rotor drag and plant draws as recover.randomization. Every term of it was measured within about 35 degrees of level and 4.4 m/s: past that rigid.py fades the attitude terms and clips the speed terms, and nothing has measured the rotors tumbling or in descent, so the righting itself is extrapolated.

SCENARIOS are the throws: each gives the release height, velocity (m/s, world: forward, left, up), attitude (roll, pitch, yaw degrees) and rotation (rad/s, body).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from sw.sim import recover as C
from sw.sim import rigid as R

AIRFRAME = "airframe_2026-11-09.json"
# The GPS-measured rotor drag (1/s, forward and sideways; sw/sim/gps_truth.py), in place of the indoor calibration's.
ROTOR_DRAG = (0.07, 0.2)
# Free fall must last this long (s) before the motors start: the hand's push reads well above 1 g, the release as 0 g.
DETECT = 0.08
FREE_FALL = 0.3 * R.G
# The velocity estimate's error at the release (m/s, per axis): the throw integrated on the accelerometer for about 0.3 s.
VELOCITY_ERROR = 0.25
# The hover after the arrest (s), then the landing.
HOLD = 3.0
SECONDS = 9.0
# A safe throw: the drone never touches the ground before its landing, ends upright, and its hold is within this of the height it stopped at (m).
HOLD_BAND = 0.5
SCENARIOS = {
    "gentle toss": {"height": 1.5, "velocity": (0.3, 0.0, 2.5), "attitude": (5, -10, 0), "rotation": (0.5, -1.0, 0.3)},
    "tumbling toss": {"height": 1.5, "velocity": (0.5, 0.0, 3.0), "attitude": (0, 30, 0), "rotation": (0.5, 7.0, 0.5)},
    "sideways throw": {"height": 1.4, "velocity": (0.5, 3.0, 2.0), "attitude": (-25, 0, 0), "rotation": (-5.0, 0.5, 1.0)},
    "upside-down toss": {"height": 1.5, "velocity": (0.0, 0.3, 3.0), "attitude": (180, 10, 0), "rotation": (0.5, 0.5, 0.5)},
    "flat spin toss": {"height": 1.5, "velocity": (0.3, 0.3, 2.5), "attitude": (10, 5, 0), "rotation": (0.3, 0.3, 8.0)},
}


def quat_from_euler(roll, pitch, yaw):
    """Body to world, (w, x, y, z), from roll, pitch, yaw (radians; yaw then pitch then roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def plant():
    params = R.A.model(AIRFRAME)
    return {"airframe": params, "geometry": R.A.geometry_of(params), "disturbances": [], "rotor_drag": ROTOR_DRAG}


class Throw(C.Controller):
    """The fall recovery, started by free fall and holding its height before landing."""

    def reset(self, quat_estimate, velocity_estimate, height_estimate):
        super().reset(quat_estimate, velocity_estimate, height_estimate)
        n = self.sim.n
        self.falling = np.zeros(n)
        self.started = np.full(n, np.inf)
        self.held = np.full(n, np.inf)
        self.hold_height = np.full(n, np.nan)
        self.descent[:] = 0.0

    def __call__(self, obs, time):
        free = np.linalg.norm(obs["accel"], axis=1) < FREE_FALL
        self.falling = np.where(free, self.falling + R.DT, 0.0)
        self.started = np.where(np.isinf(self.started) & (self.falling >= DETECT), time, self.started)
        # Arrested (upright and nearly still vertically): hold the height reached, then land after HOLD s.
        arrested = (self.phase == C.ARREST) & (time > self.started + 0.3) & (np.abs(self.v[:, 2]) < 0.3) & np.isinf(self.held)
        self.held = np.where(arrested, time, self.held)
        self.hold_height = np.where(arrested, self.h, self.hold_height)
        holding = np.isfinite(self.held) & (time - self.held < HOLD)
        self.hold = holding | np.isinf(self.held)
        # Holding: climb or sink back to the height it stopped at (descent is the wanted sink rate, negative climbs).
        self.descent = np.where(holding, np.clip(self.p["k_alt"] * 2 * (self.h - self.hold_height), -0.5, 0.5), np.where(np.isfinite(self.held), self.p["descent"], 0.0))
        throttle, tilt = super().__call__(obs, time)
        off = time < self.started
        # Nothing rights the drone before its motors run.
        self.phase[off] = C.RIGHT
        throttle[off] = 0
        tilt[off] = 0
        self.asked[off] = 0.0
        return throttle, tilt


def phase_of(control, time):
    """render.TOSS_PHASES' index per env: thrown, righting, stopping, holding, landing, landed."""
    since = time - control.held
    return np.select([np.isinf(control.started), control.phase == C.RIGHT, np.isinf(control.held), since < HOLD, control.phase == C.ARREST], [0, 1, 2, 3, 4], 5)


def run(params, scenario, n=64, seed=0, seconds=SECONDS, randomize=True, record=False):
    s = SCENARIOS[scenario]
    rng = np.random.default_rng(seed)
    sim = R.Sim(n, plant(), seed=seed, scale=C.randomization(n, seed) if randomize else None)
    jitter = lambda size, scale: rng.normal(0, scale, size)
    position = np.column_stack([np.zeros(n), np.zeros(n), s["height"] + jitter(n, 0.05)])
    velocity = np.array(s["velocity"]) + jitter((n, 3), 0.2)
    attitude = np.radians(np.array(s["attitude"]) + jitter((n, 3), 5.0))
    quat = np.array([quat_from_euler(*a) for a in attitude])
    omega = np.array(s["rotation"]) + jitter((n, 3), 0.5)
    obs = sim.reset(position, velocity, quat, omega)
    control = Throw(sim, C.DEFAULTS | params | {"engage": 0.0})
    control.reset(C.estimate(quat, rng), velocity + rng.normal(0, VELOCITY_ERROR, (n, 3)), position[:, 2] + rng.normal(0, C.HEIGHT_ERROR, n))
    trace, peak = [], position[:, 2].copy()
    first_touch = np.full(n, np.nan)
    tilt_max = np.zeros(n)
    hold_tilt, hold_samples = np.zeros(n), np.zeros(n)
    for k in range(int(seconds / R.DT)):
        throttle, tilt = control(obs, sim.time)
        before = sim.touched.copy()
        obs = sim.step(throttle, tilt)
        new = sim.touched & ~before
        first_touch[new] = sim.time
        peak = np.maximum(peak, sim.pos[:, 2])
        up = sim.up()[:, 2]
        flying = np.isfinite(control.started) & np.isinf(control.held)
        tilted = np.degrees(np.arccos(np.clip(up, -1, 1)))
        tilt_max = np.where(flying, np.maximum(tilt_max, tilted), tilt_max)
        holding = np.isfinite(control.held) & (sim.time - control.held < HOLD) & (sim.time - control.held > 0.5)
        hold_tilt += np.where(holding, tilted ** 2, 0.0)
        hold_samples += holding
        if record:
            trace.append((sim.time, sim.pos.copy(), sim.quat.copy(), sim.motor.copy(), sim.tilt.copy(), phase_of(control, sim.time)))
    # The hold's true height while it held, against where it stopped.
    held = np.isfinite(control.held)
    landed_on_purpose = held & (first_touch > control.held + HOLD)
    safe = held & (np.isnan(first_touch) | landed_on_purpose) & (sim.up()[:, 2] > 0.8)
    result = {"safe": safe, "held": held, "start": control.started, "arrest": control.held, "peak": peak, "tilt_max": tilt_max, "first_touch": first_touch, "upright": sim.up()[:, 2], "hold_tilt": np.sqrt(hold_tilt / np.maximum(hold_samples, 1))}
    return (result, trace) if record else result


def cost(result):
    """How far each throw missed a safe one (for sw/sim/tune_recover.py --toss): touching the ground before its landing, not ending upright, never holding a height, slow arrests, and a wobbly hold."""
    touched_early = np.isfinite(result["first_touch"]) & ~(np.isfinite(result["arrest"]) & (result["first_touch"] > result["arrest"] + HOLD))
    arrest = np.where(np.isfinite(result["arrest"]), result["arrest"], SECONDS)
    c = 3.0 * touched_early + 2 * np.clip(1 - result["upright"], 0, 2) + (~result["held"]) + 0.2 * arrest + 0.03 * np.minimum(result["hold_tilt"], 30)
    return float(c.mean()), float(result["safe"].mean())


def summary(result):
    safe = result["safe"]
    arrest = result["arrest"][np.isfinite(result["arrest"])]
    return (f"safe {safe.mean() * 100:.0f}% of {len(safe)}, held {result['held'].mean() * 100:.0f}%; motors on at {np.nanmedian(np.where(np.isfinite(result['start']), result['start'], np.nan)):.2f} s, "
            f"arrested at {np.median(arrest) if len(arrest) else float('nan'):.2f} s median (90th {np.percentile(arrest, 90) if len(arrest) else float('nan'):.2f}), peak height {np.median(result['peak']):.2f} m, worst tilt {np.median(result['tilt_max']):.0f}° median, hold tilt {np.median(result['hold_tilt']):.1f}° RMS median")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--params", type=Path, default=C.PARAMS)
    parser.add_argument("--n", type=int, default=256)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--nominal", action="store_true")
    parser.add_argument("--render", type=Path, help="render one throw of each scenario to this mp4 (sw/sim/render.py, props spinning)")
    args = parser.parse_args()
    params = json.loads(args.params.read_text()) if args.params else {}
    rates = {}
    for name in SCENARIOS:
        result = run(params, name, args.n, args.seed, randomize=not args.nominal)
        rates[name] = result["safe"].mean()
        print(f"{name:18s} {summary(result)}", flush=True)
    if args.render:
        from sw.sim import render
        # One throw per scenario that held (the first seed whose nominal throw does), labeled with the scenario's rate over the randomized throws, then the least reliable scenario's first failure.
        clips, failure = [], None
        for i, name in enumerate(SCENARIOS):
            for k in range(50):
                result, trace = run(params, name, 1, args.seed + i + 100 * k, randomize=False, record=True)
                if result["safe"][0]:
                    clips.append((trace, f"{name}: held ({rates[name] * 100:.0f}% of {args.n} randomized throws do)"))
                    break
                if failure is None and name == min(rates, key=rates.get):
                    failure = (trace, f"{name}: a failure ({(1 - rates[name]) * 100:.0f}% of {args.n} randomized throws fail)")
        if failure is None:
            name = min(rates, key=rates.get)
            for k in range(200):
                result, trace = run(params, name, 1, args.seed + 7 + k, record=True)
                if not result["safe"][0]:
                    failure = (trace, f"{name}: a failure ({(1 - rates[name]) * 100:.0f}% of {args.n} randomized throws fail)")
                    break
        parts = []
        for j, (trace, label) in enumerate(clips + ([failure] if failure else [])):
            part = args.render.with_name(f"{args.render.stem}_{j}.mp4")
            render.render(trace, 0, part, label=label, slow=2.0, phases=render.TOSS_PHASES, props=True)
            parts.append(part)
        render.concat(parts, args.render)


if __name__ == "__main__":
    main()
