"""
Cross-validate airframe model structures by flight session: uv run --with scipy -m sw.sim.crossval_airframe <recordings or globs> --sessions sessions.json --structures structures.json --structure NAME --held-out SESSION|none [--start model.json] [--geometry geometry.json] [--evaluations N] [--instrument W] [--workers N] [--out result.json]

A session is a stretch of flights with one hardware state (props, bands, mass, battery holder), given in sessions.json as {"name": {"from": "2026-11-02T03-49", "until": "2026-11-02T05-35", "mass": 1.0, "weight": 1}, ...} (recording-name prefixes, until exclusive; mass in kg, default the geometry's; weight repeats the session's flights in the fit): the flights of different sessions share every fitted parameter, and only what the user changed (the mass, which sets each flight's thrust level) differs between them. A structure is a set of free parameters (structures.json: {"name": {"free": [...]}}); every other parameter stays at the start model's value.
For the structure and held-out session given, the free parameters are fitted (identify_airframe's closed-loop multi-step cost) on every other session's flights from the start model, not from a fit that saw the held-out flights, and the held-out session is then scored as validate_model scores a model: the closed-loop attitude error at 100 ms to 1 s and its stick steps' overshoot, time to peak and response error. --held-out none fits on every session and scores the fit on its own flights. Run every (structure, session) pair in parallel and compare their held-out scores and the spread of their fitted parameters (added 2026-11-08, after the 2026-11-08 model predicted the first outdoor flights worse than an unfitted one).
"""

import argparse
import json
import os
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import identify_airframe as I
from sw.sim import validate_model as V

DEG = np.pi / 180
HORIZONS = (10, 25, 50, 100)
# Each flight's mass (kg) by its recording name, for its thrust level.
MASS = {}


def trims(p, data, flights):
    """identify_airframe.trims with each flight's own mass (its session's)."""
    gains = A.motor_terms(p, len(data.records))[2]
    pairs = [A.trim_for(A.table_thrust(p, data.throttle[i, r["lift"]:r["end"]], data.voltage[i, r["lift"]:r["end"]], data.table, gains[i]), dict(I.GEOM, mass=MASS.get(r["name"], I.GEOM["mass"]))) for i, r in ((i, data.records[i]) for i in flights)]
    return np.array([level for level, _ in pairs]), np.array([mean for _, mean in pairs])


def initialise(mass, *args):
    MASS.update(mass)
    I.trims = trims
    I.initialise(*args)


def evaluate(params, data, flights):
    """The closed-loop attitude error (deg, roll and pitch) at HORIZONS and the stick steps of these flights flown by params."""
    parts = I.score(np.array([params[key] for key in I.FREE]), flights, True)
    errors = {}
    for h in HORIZONS:
        _, _, change, real = parts[h]
        errors[h * 10] = [round(float(e), 3) for e in np.sqrt(((change - real) ** 2).mean(0)) / DEG]
    horizon = int(V.LONGEST / I.DT)
    F, S0, valid, rates, attitudes = I.predict(params, data, np.array(flights), [1, horizon], 1, True)
    steps = []
    for i in flights:
        for axis, start, held, size in V.steps(i, data):
            lane = np.flatnonzero((F == i) & (S0 == start))
            if not len(lane):
                continue
            n = min(held, horizon)
            model = np.concatenate([[data.att[i, start, axis]], attitudes[: n - 1, lane[0], axis]])
            real = data.att[i, start : start + n, axis]
            steps.append({"flight": data.records[i]["name"], "axis": "roll" if axis == 0 else "pitch", "size": round(float(size / DEG), 2), "real": [round(float(v), 3) for v in V.response(real, size)], "model": [round(float(v), 3) for v in V.response(model, size)], "error": round(float(np.sqrt(np.mean((model - real) ** 2)) / DEG), 3)})
    return errors, steps


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("recordings", nargs="+")
    parser.add_argument("--sessions", type=Path, required=True)
    parser.add_argument("--structures", type=Path, required=True)
    parser.add_argument("--structure", required=True)
    parser.add_argument("--held-out", required=True)
    parser.add_argument("--start", type=Path, default=A.MODELS / "airframe_2026-10-29_10in_zerofit.json")
    parser.add_argument("--geometry", default="geometry_2026-10-29_10in.json")
    parser.add_argument("--evaluations", type=int, default=40)
    parser.add_argument("--instrument", type=float, default=1.0, help="weight of identify_airframe's instrumented one-step cost on the test-pattern flights")
    parser.add_argument("--shortest", type=float, default=0.5)
    parser.add_argument("--velocity", choices=("logged", "tag"), default="tag", help="the logged hover velocity everywhere, or only where the tag measured it (identify_airframe.Data)")
    parser.add_argument("--workers", type=int, default=os.cpu_count())
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    sessions = json.loads(args.sessions.read_text())
    free = json.loads(args.structures.read_text())[args.structure]["free"]
    geometry = A.load_geometry(args.geometry)
    session_of = lambda name: next((s for s, spec in sessions.items() if spec["from"] <= name < spec["until"]), None)
    records = [r for r in (I.load(p, args.shortest) for p in I.paths(args.recordings)) if r and session_of(r["name"])]
    # A session's "weight" (default 1) repeats its flights in the fit, e.g. to make a few outdoor flights count against many indoor ones.
    records = [r for r in records for _ in range(1 if session_of(r["name"]) == args.held_out else sessions[session_of(r["name"])].get("weight", 1))]
    mass = {r["name"]: sessions[session_of(r["name"])].get("mass", geometry["mass"]) for r in records}
    names = [session_of(r["name"]) for r in records]
    test = [i for i, s in enumerate(names) if s == args.held_out]
    train = [i for i, s in enumerate(names) if s != args.held_out]
    print(f"{args.structure}, held out {args.held_out}: {len(train)} flights to fit, {len(test)} held out; free: {', '.join(free) or 'nothing'}", flush=True)
    start = A.complete(json.loads(args.start.read_text()))
    data = I.Data(records, None, args.velocity)
    start["right_thrust_gain"] = I.thrust_split(records)
    for key in I.TIED:
        if key not in free:
            start[key] = I.TIED[key]
    for key, other in I.EQUAL.items():
        if key not in free:
            start[key] = start[other]
    base = {key: start[key] for key in A.KEYS}
    setup = (mass, data, free, base, None, 0.0, geometry, args.instrument)
    initialise(*setup)
    with Pool(max(1, min(args.workers, len(free))), initializer=initialise, initargs=setup) as pool:
        x = np.array([start[key] for key in free])
        cost = None
        if free:
            x, cost = I.fit(pool, x, train, args.evaluations)
    params = I.unpack(x)
    scored = test or train
    errors, steps = evaluate(params, data, scored)
    pitch_steps = [s for s in steps if s["axis"] == "pitch"]
    print(f"cost {cost}; held-out attitude error (roll, pitch deg) " + ", ".join(f"{h} ms {e[0]:.2f}/{e[1]:.2f}" for h, e in errors.items()), flush=True)
    if pitch_steps:
        real = np.array([s["real"] for s in pitch_steps])
        model = np.array([s["model"] for s in pitch_steps])
        print(f"{len(pitch_steps)} pitch steps: overshoot {np.nanmedian(real[:, 0]):.2f} -> {np.nanmedian(model[:, 0]):.2f}, time to peak {np.nanmedian(real[:, 1]):.2f} -> {np.nanmedian(model[:, 1]):.2f} s, response error median {np.median([s['error'] for s in pitch_steps]):.1f}°", flush=True)
    print("fitted: " + ", ".join(f"{key} {params[key]:.4g}" for key in free), flush=True)
    result = {"structure": args.structure, "held_out": args.held_out, "free": free, "cost": cost, "train": len(train), "test": len(test), "params": {key: round(float(params[key]), 6) for key in free}, "errors": errors, "steps": steps}
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=1) + "\n")


if __name__ == "__main__":
    main()
