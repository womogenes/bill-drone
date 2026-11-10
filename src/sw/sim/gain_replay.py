"""
Fly recordings' stick steps and rate kicks on airframe models with other attitude gains: uv run -m sw.sim.gain_replay <recordings> --models models/airframe_A.json [...] --sets "pitch_p=28,pitch_damping=6" "pitch_p=20,pitch_i=15" [...] [--kick 40]

For every model and gain set (settings the replayed firmware controller takes: pitch_p, pitch_i, pitch_damping, pitch_damping_tau, target_rate, integral_limit, roll_*, roll_target_rate), the controller flies the model closed loop from the recordings' logged state on their logged references:
- each stick step on --axis (pitch, or roll since 2026-11-08 for roll_target_rate; validate_model.steps: a target jump of over 8 degrees held 0.6 s or more), flown as a clean step: its references held at their values just after the jump for 3 s (the logged hover loop moved them as the real drone answered, which a model on other gains would not have done): overshoot (peak over the step), time to peak, settling time (the last time the attitude is more than 10% of the span from the held setpoint), the second swing (the largest return past the setpoint after the peak, over the span) and the RMS error from the setpoint over the last 1.5 s, which the forward speed's moment and the integrator set;
- rate kicks: from every half second of airborne flight, a pitch rate of --kick deg/s added at once (a gust's impulse) with the references held at their logged values: the peak pitch excursion from the start, the settling time (last time 2 degrees or more from the no-kick run) and the largest overshoot back past the start (degrees).
The set "logged" flies each recording's own gains. Steps and kicks are medians over the recordings. Added 2026-11-08 to choose the gains for the next outdoor flight on models that predict the first outdoor flights' steps (sw/sim/crossval_airframe.py), since the logged steps were flown at one gain set.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.flight_log import parse_settings
from sw.sim import airframe as A
from sw.sim import identify_airframe as I
from sw.sim import validate_model as V
from sw.sim.flight import DAMPING_TAU, ROLL_TARGET_RATE

DEG = np.pi / 180
LONGEST = 3.0
SETTLE = 0.1
KICK_EVERY = 50


def overrides(text):
    return {} if text == "logged" else parse_settings(text)


def fly(params, data, lanes, settings, kick=0.0, hold=False, horizon=int(LONGEST / I.DT)):
    """Closed-loop attitudes [step, lane, axis] from lanes (flight, start) with the settings overriding the logged gains and pitch rate kick (rad/s) added at the start; with hold, each lane's references held at their values just after its start."""
    F = np.array([f for f, _ in lanes])
    S0 = np.array([s for _, s in lanes])
    flights = np.unique(F)
    keep, push, trim = I.teacher(params, data, flights)
    J = np.searchsorted(flights, F)
    state = A.State(len(F))
    for key in keep:
        setattr(state, key, keep[key][J, S0].copy())
    state.rate, state.att = data.rate[F, S0].copy(), data.att[F, S0].copy()
    state.vel = np.where(data.vel[F, S0] != 0, data.vel[F, S0], state.vel)
    state.rate[:, 1] += kick
    throttle, command, voltage = data.throttle[F].copy(), data.command[F].copy(), data.voltage[F]
    states, commands = data.states[F, S0].copy(), data.commands[F].copy()
    for key, value in settings.items():
        if key == "pitch_damping_tau":
            commands[:, DAMPING_TAU] = value
        elif key == "roll_target_rate":
            commands[:, ROLL_TARGET_RATE] = value * DEG
        elif key in ("target_rate",):
            commands[:, I.INDEX[key]] = value * DEG
        else:
            commands[:, I.INDEX[key]] = value
    if kick:
        # The damping filter starts from the kicked rate, as if the gust had come over a tick.
        states[:, 17] = state.rate[:, 1]
    attitudes = np.zeros((horizon, len(F), 2))
    lane = np.arange(len(F))
    logged = data.targets
    if hold:
        data.targets = logged.copy()
        for f, s in lanes:
            data.targets[f, s : s + horizon] = logged[f, min(s + 1, data.size - 1)]
    lane_trim = (trim[0][J], trim[1][J])
    for j in range(horizon):
        k = np.minimum(S0 + j, data.size - 1)
        throttle[lane, k], command[lane, k] = data.control(states, commands, F, k, state.att, state.rate)
        state, _ = A.step(params, state, throttle, command, k, voltage[lane, k], push[J], lane_trim, geometry=I.GEOM, table=data.table)
        attitudes[j] = state.att
    data.targets = logged
    return attitudes


def step_metrics(values, goal):
    """Overshoot (peak over the span from the start to the held setpoint goal), time to peak, settling time (last time more than 10% of the span from the goal), second swing (the largest return past the goal after the peak, over the span) and the RMS error from the goal over the last half (degrees) of one step response (radians)."""
    span = goal - values[0]
    scaled = (values - values[0]) / span
    peak = int(np.argmax(scaled))
    outside = np.flatnonzero(np.abs(scaled - 1) > SETTLE)
    settle = (outside[-1] + 1) * I.DT if len(outside) else 0.0
    second = float(1 - scaled[peak:].min())
    late = np.sqrt(np.mean((values[len(values) // 2 :] - goal) ** 2)) / DEG
    return scaled[peak], peak * I.DT, settle, second, late


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--sets", nargs="+", default=["logged"])
    parser.add_argument("--kick", type=float, default=40.0, help="deg/s of pitch rate added at each kick start (0 = no kicks)")
    parser.add_argument("--geometry", help="default: each model's")
    parser.add_argument("--json", type=Path, help="write the medians as JSON")
    parser.add_argument("--axis", choices=("pitch", "roll"), default="pitch", help="the axis whose stick steps are flown (kicks stay pitch)")
    parser.add_argument("--velocity", choices=("logged", "tag"), default="tag", help="the logged hover velocity everywhere, or only where the tag measured it (identify_airframe.Data)")
    args = parser.parse_args()
    records = [r for r in (I.load(p, 0.5) for p in I.paths(args.recordings)) if r]
    print(f"{len(records)} flights: " + ", ".join(r["name"] for r in records), flush=True)
    data = I.Data(records, None, args.velocity)
    axis = 1 if args.axis == "pitch" else 0
    found = [(i, start, held, size) for i in range(len(records)) for step_axis, start, held, size in V.steps(i, data) if step_axis == axis]
    kicks = [(i, s) for i, r in enumerate(records) for s in range(r["lift"] + 50, r["end"] - int(LONGEST / I.DT), KICK_EVERY)]
    print(f"{len(found)} {args.axis} steps, {len(kicks)} kick starts", flush=True)
    results = []
    for model_path in args.models:
        loaded = json.loads(Path(model_path).read_text())
        params = A.complete(loaded)
        params["right_thrust_gain"] = I.thrust_split(records)
        I.GEOM = A.load_geometry(args.geometry or loaded.get("geometry") or A.GEOMETRY_FILE)
        print(f"\n{model_path}", flush=True)
        for text in args.sets:
            settings = overrides(text)
            row = {"model": model_path, "set": text}
            if found:
                attitudes = fly(params, data, [(i, s) for i, s, _, _ in found], settings, hold=True)
                metrics = []
                for lane, (i, start, held, size) in enumerate(found):
                    values = np.concatenate([[data.att[i, start, axis]], attitudes[:-1, lane, axis]])
                    goal = data.targets[i, min(start + 1, data.size - 1), 1 + axis]
                    if abs(goal - values[0]) > 5 * DEG:
                        metrics.append(step_metrics(values, goal))
                m = np.median(np.array(metrics), 0)
                row.update(overshoot=m[0], peak=m[1], settle=m[2], second=m[3], late=m[4])
            line = f"  {text:48s}"
            if found:
                line += f" steps: overshoot {row['overshoot']:.2f}, peak {row['peak']:.2f} s, settle {row['settle']:.2f} s, second swing {row['second']:.2f}, late error {row['late']:.1f}°"
            if args.kick and kicks:
                kicked = fly(params, data, kicks, settings, args.kick * DEG)[:, :, 1]
                calm = fly(params, data, kicks, settings)[:, :, 1]
                difference = (kicked - calm) / DEG
                excursion = np.abs(difference).max(0)
                late = [np.flatnonzero(np.abs(d) >= 2.0) for d in difference.T]
                settle = np.array([(l[-1] + 1) * I.DT if len(l) else 0.0 for l in late])
                back = np.maximum(-difference.min(0), 0)
                lost = np.mean(excursion > 30)
                row.update(kick_peak=float(np.median(excursion)), kick_settle=float(np.median(settle)), kick_settle90=float(np.percentile(settle, 90)), kick_back=float(np.median(back)), kick_lost=float(lost))
                line += f"; kick {args.kick:g} deg/s: peak {row['kick_peak']:.1f}°, back {row['kick_back']:.1f}°, settle {row['kick_settle']:.2f} s (90% {row['kick_settle90']:.2f}), past 30° {100 * lost:.0f}%"
            print(line, flush=True)
            results.append(row)
    if args.json:
        args.json.write_text(json.dumps(results, indent=1) + "\n")


if __name__ == "__main__":
    main()
