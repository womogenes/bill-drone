"""
Closed-loop validation of the full-attitude sim (sw/sim/rigid.py) against hover recordings: uv run python -m sw.sim.validate_rigid [recordings] [--stride 25]

From every stride-th airborne sample of each flight, the firmware controller (with its replayed state at that sample) flies the rigid-body sim on the logged references alone for up to 1 s, as sw/sim/identify_airframe.py scores the airframe model it was identified with. The sim starts at the logged attitude, rates and velocity, with the airframe's actuator states from the one-step-ahead teacher and the logged command history, the flight's own push, no random disturbances and no sensor noise, so what differs from the airframe's own prediction is only what rigid.py adds or changes: quaternion rotation with the gyroscopic coupling, the thrust-scaled ("wash") flying terms, translation and drag, and no battery sag (rigid.py flies at the thrust table's voltage). Printed per horizon for both models: R² of the rates and the RMS error of the attitude change (degrees), against the log.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import identify_airframe as N
from sw.sim import rigid as R

# The four hovers after the 2026-10-28 ESC recalibration that models/airframe_2026-10-28.json was identified from.
FLIGHTS = ["sw/recordings/2026-10-28T06-54-20-503Z_1581871433.bin", "sw/recordings/2026-10-28T06-55-20-397Z_757306949.bin",
           "sw/recordings/2026-10-28T06-59-38-797Z_3919631855.bin", "sw/recordings/2026-10-28T07-00-59-352Z_1068518903.bin"]
HORIZONS = (5, 10, 25, 50, 100)


def quat_from(att):
    """Quaternions [lane, 4] for roll and pitch [lane, 2] (rad) at zero yaw, in rigid.py's convention (its State.att inverts this)."""
    half = att / 2
    qx = np.column_stack([np.cos(half[:, 0]), np.sin(half[:, 0]), np.zeros(len(att)), np.zeros(len(att))])
    qy = np.column_stack([np.cos(half[:, 1]), np.zeros(len(att)), np.sin(half[:, 1]), np.zeros(len(att))])
    return R.quat_multiply(qy, qx)


def rigid_predict(p, data, flights, horizons, stride):
    """identify_airframe.predict's closed-loop free runs, flown on rigid.Sim instead of the airframe model."""
    keep, push = N.teacher(p, data, flights)
    F, S0, valid = data.lanes(flights, horizons, stride)
    J = np.searchsorted(flights, F)
    n = len(F)
    sim = R.Sim(n, plant=R.load_plant(disturbances=None), backend="numpy")
    att, rate = data.att[F, S0], data.rate[F, S0]
    quat = quat_from(att)
    velocity = np.einsum("nij,nj->ni", R.rotation(quat), np.column_stack([data.vel[F, S0], np.zeros(n)]))
    sim.reset(np.tile([0.0, 0.0, 1.0], (n, 1)), velocity, quat, rate)
    for key in ("motor", "horn", "hvel", "tilt", "tvel"):
        getattr(sim.state, key)[:] = keep[key][J, S0]
    back = np.clip(S0[:, None] - sim.history + np.arange(sim.history)[None], 0, None)
    sim.throttles[:] = data.throttle[F[:, None], back]
    sim.commands[:] = data.command[F[:, None], back]
    sim.push[:] = push[J]
    states, commands = data.states[F, S0].copy(), data.commands[F].copy()
    rates = np.zeros((max(horizons), n, 3))
    attitudes = np.zeros((max(horizons), n, 2))
    for j in range(max(horizons)):
        k = np.minimum(S0 + j, data.size - 1)
        up = R.rotation(sim.quat)[:, 2, :]
        now = np.column_stack([np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))])
        throttle, tilt = data.control(states, commands, F, k, now, sim.omega)
        sim.step(throttle, np.radians(tilt))
        up = R.rotation(sim.quat)[:, 2, :]
        rates[j] = sim.omega
        attitudes[j] = np.column_stack([np.arctan2(up[:, 1], up[:, 2]), np.arctan2(-up[:, 0], np.hypot(up[:, 1], up[:, 2]))])
    return F, S0, valid, rates, attitudes


def scores(prediction, data, horizons):
    F, S0, valid, rates, attitudes = prediction
    out = {}
    for j, h in enumerate(horizons):
        v = valid[j]
        f, s = F[v], S0[v]
        rate, real = rates[h - 1][v], data.rate[f, s + h]
        change, real_change = attitudes[h - 1][v] - data.att[f, s], data.att[f, s + h] - data.att[f, s]
        r2 = 1 - ((rate - real) ** 2).sum(0) / ((real - real.mean(0)) ** 2).sum(0)
        out[h] = (r2, np.degrees(np.sqrt(((change - real_change) ** 2).mean(0))), v.sum())
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="*", default=FLIGHTS)
    parser.add_argument("--airframe", type=Path, default=Path(__file__).with_name("models") / "airframe_2026-10-28.json")
    parser.add_argument("--stride", type=int, default=25)
    args = parser.parse_args()
    p = A.complete(json.loads(args.airframe.read_text()))
    records = [r for r in (N.load(path) for path in N.paths(args.recordings)) if r]
    data = N.Data(records)
    flights = np.arange(len(records))
    print(f"{len(records)} flights, {sum(r['end'] - r['lift'] for r in records) * N.DT:.0f} s airborne")
    model = scores(N.predict(p, data, flights, HORIZONS, args.stride), data, HORIZONS)
    rigid = scores(rigid_predict(p, data, flights, HORIZONS, args.stride), data, HORIZONS)
    print("horizon   rate R² roll/pitch/yaw: airframe | rigid      attitude error roll/pitch (°): airframe | rigid      starts")
    for h in HORIZONS:
        (ra, ea, count), (rr, er, _) = model[h], rigid[h]
        print(f"{h * 10:5d} ms  {ra[0]:5.2f} {ra[1]:5.2f} {ra[2]:5.2f} | {rr[0]:5.2f} {rr[1]:5.2f} {rr[2]:5.2f}      {ea[0]:5.2f} {ea[1]:5.2f} | {er[0]:5.2f} {er[1]:5.2f}      {count}")


if __name__ == "__main__":
    main()
