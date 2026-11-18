"""
Fit drone 1's plant on the scorecard's own measure (sw/sim/scorecard.py): the named terms of an airframe moved by CMA-ES to lower the scorecard total (the mean over kinds of each flight's prediction error over the do-nothing prediction's; open loop, or with --firmware the firmware in the loop as scorecard.py --firmware) on a training set of flights, the rest held out: uv run -m sw.sim.fit_plant --base PLANT.json --fit name[=low:high],... [--train PREFIX ...] [--hold PREFIX ...] [--every 0.2] [--workers 32] [--generations 40] [--firmware] [--out fitted.json]

A name is an airframe term (airframe.PARAMS, bounds from there unless given), a geometry term the airframe then carries as measured ("geometry:pitch_inertia", "geometry:com_z"), a configuration's own term ("CONFIGURATION:term", e.g. "nobands_new_esc:motor_tau": the plant's "configurations"; several joined by + share one value), or the disturbance state's times ("observer:filter", "observer:correlation"). Side-paired servo and motor terms set alone set both sides (scorecard.plant_for). Each flight's own terms (its CoM forward and left and its yaw push, scorecard.flight_terms) are found on the best plant so far every --terms-every generations and held in between. Flights are read once per worker. --score only scores the base (training and held out) with the same machinery.

With --response W the cost adds W times each configuration's frequency-response misfit (sw/sim/plant_response.py: the plant linearized, servo command to pitch rate, against the response the configuration's training flights measure in hover, 0.45-3.6 Hz; the mean squared complex log ratio weighted by the coherence), which pins the pitch chain's gain and phase where the open-loop predictions only see them through the loop's noise. With --closed W and --closed-flights, it also adds W times the closed loop's misfit: each named flight's settings flown on the candidate by the firmware's C from the ground (scorecard_closed.rerun, one run), the squared log ratio of its pitch shake's frequency and of its RMS in 1.5-4 Hz to the log's, and 4 for a run that tips where the drone did not (or the reverse).
"""

import argparse
import json
import math
import time as clock
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from sw.sim import airframe as A
from sw.sim import plant_response as P
from sw.sim import scorecard as S

cache = {}
listing = None


def flight(entry):
    """A worker's Flight for an entry, read once."""
    if entry["name"] not in cache:
        cache[entry["name"]] = S.Flight(entry, listing["configurations"][entry["configuration"]])
    return cache[entry["name"]]


def initialize(path, firmware=False):
    global listing
    listing = S.flight_list(path)
    S.FIRMWARE = firmware


def apply(base, names, values):
    """The airframe parameters with the fitted names set to values."""
    params = json.loads(json.dumps(base))
    for name, value in zip(names, values):
        value = float(value)
        if name.startswith("geometry:"):
            params.setdefault("geometry_terms", {})[name.split(":", 1)[1]] = value
        elif name.startswith("observer:"):
            term = name.split(":", 1)[1]
            observer = params.setdefault("observer", {"correlation": 10.0, "sd": [1.0, 1.0, 1.0], "noise": [3.0, 5.0, 3.0]})
            # sd_roll, noise_pitch, ...: one axis's value; sd, noise: all three.
            if term.split("_")[0] in ("sd", "noise"):
                key, *axis = term.split("_")
                values = list(observer[key]) if isinstance(observer[key], list) else [observer[key]] * 3
                for i, a in enumerate(("roll", "pitch", "yaw")):
                    if not axis or axis[0] == a:
                        values[i] = value
                observer[key] = values
            else:
                observer[term] = value
        elif ":" in name:
            group, term = name.split(":", 1)
            for configuration in group.split("+"):
                params.setdefault("configurations", {}).setdefault(configuration, {})[term] = value
                for key, other in (("servo_delay", "right_servo_delay"), ("servo_wn", "right_servo_wn"), ("motor_tau", "right_motor_tau"), ("motor_delay", "right_motor_delay")):
                    if term == key and f"{group}:{other}" not in names:
                        params["configurations"][configuration][other] = value
        else:
            params[name] = value
            for key, other in (("servo_delay", "right_servo_delay"), ("servo_wn", "right_servo_wn"), ("motor_tau", "right_motor_tau"), ("motor_delay", "right_motor_delay")):
                if name == key and other not in names:
                    params[other] = value
    return params


def plant_from(entry, params):
    """scorecard.plant_for on parameters held in memory."""
    return S.plant_for(flight(entry), params, {})


def score(task):
    """One flight on one candidate: scorecard.score_flight's arrays."""
    entry, params, every, warmup, terms = task
    f = flight(entry)
    horizon = round(max(S.HORIZONS) / A.DT)
    index, phase = f.starts(every, warmup, horizon)
    ticks = np.array([round(h / A.DT) for h in S.HORIZONS])
    if len(index) == 0:
        return None
    plant = plant_from(entry, params)
    if terms is not None:
        plant = S.with_terms(plant, terms)
    with np.errstate(all="ignore"):
        predicted = S.predict(f, plant, index, round(warmup / A.DT), horizon)
    if not np.isfinite(predicted).any():
        predicted[:] = 1e3
    predicted = np.nan_to_num(predicted, nan=1e3, posinf=1e3, neginf=-1e3)
    target, start = f.targets(index, ticks)
    error = predicted[:, ticks - 1] - target
    naive = start[:, None] - target
    for c in (3, 4):
        error[..., c], naive[..., c] = S.wrap(error[..., c]), S.wrap(naive[..., c])
    return {"name": f.name, "kind": f.kind, "error": error, "naive": naive, "phase": phase, "time": f.time[index], "reach": np.zeros((len(index), 2)), "terms": terms}


def terms_task(task):
    entry, params, warmup = task
    f = flight(entry)
    return S.flight_terms(f, plant_from(entry, params), round(warmup / A.DT))


def find_terms(pool, chosen, params, warmup):
    """Each flight's own terms on these parameters (the nearest flight's in its configuration for one too short)."""
    return S.nearest_terms(chosen, list(pool.map(terms_task, [(e, params, warmup) for e in chosen])))


def response_task(task):
    """One configuration's frequency-response misfit for one candidate: (entry for the trim, measured frequencies, response, coherence, parameters)."""
    entry, frequency, response, coherence, params = task
    plant = plant_from(entry, params)
    try:
        with np.errstate(all="ignore"):
            model = P.computed(plant, frequency)
        error = np.log(model / response)
        value = float(np.sum(np.abs(error) ** 2 * coherence) / np.sum(coherence))
        return value if math.isfinite(value) else 10.0
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return 10.0


def closed_task(task):
    """One flight's closed-loop misfit for one candidate (scorecard_closed.rerun, one run): (name, the log's measures, parameters)."""
    name, log, params = task
    from sw.sim import scorecard_closed as C
    # The scorecard's predictions in this worker rest the hull on the flight's feet (scorecard.make_sim sets rigid.HULL); the closed loop flies from rigid.py's own.
    S.R.HULL = S.HULL.copy()
    try:
        with np.errstate(all="ignore"):
            run = C.rerun((name, params, 1, 0, {}))["runs"][0]
    except Exception:
        return 10.0
    tipped_log = log.get("tip_after_liftoff") is not None
    if run.get("tipped") != tipped_log:
        return 4.0
    if tipped_log:
        return 0.0 if run.get("tip_after_liftoff") is None else math.log(max(run["tip_after_liftoff"], 0.1) / log["tip_after_liftoff"]) ** 2
    if run.get("peak_hz") is None or run.get("band_rms") is None:
        return 4.0
    return math.log(run["peak_hz"] / log["peak_hz"]) ** 2 + math.log(max(run["band_rms"], 0.5) / log["band_rms"]) ** 2


def responses(chosen):
    """Each configuration's measured response over its training flights' hovers (configurations with under 60 s of them are left out): [(entry, frequency, response, coherence)]."""
    out = []
    by = {}
    for e in chosen:
        by.setdefault(e["configuration"], []).append(e)
    for configuration, group in by.items():
        flights = [flight(e) for e in group if e["kind"] not in ("hop", "burst", "divergence")]
        hovering = sum(((f.hovering & (f.time - f.first_on > 3) & (f.last_on - f.time > 2)).sum() for f in flights)) * A.DT
        measured = P.measured(flights) if hovering >= 60 else None
        if measured is None:
            continue
        frequency, response, coherence = measured
        band = (frequency >= 0.45) & (frequency <= 3.6)
        out.append(([e for e in group if e["kind"] not in ("hop", "burst", "divergence")][0], frequency[band], response[band], np.clip(coherence[band], 0, 1)))
    return out


def total(results):
    kept = [r for r in results if r is not None]
    return S.summarize(kept) if kept else None


def evaluate(pool, chosen, candidates, every, warmup, terms, measured=(), weight=0.0, closed=(), closed_weight=0.0):
    """Each candidate's summary over the chosen flights (one pool map over every candidate and flight), with "response" its frequency-response misfit (the mean over the configurations measured) and "cost" the total plus weight times it."""
    tasks = [(e, params, every, warmup, terms.get(e["name"])) for params in candidates for e in chosen]
    jobs = [(m[0], m[1], m[2], m[3], params) for params in candidates for m in measured] if weight else []
    results = list(pool.map(score, tasks, chunksize=1))
    misfits = list(pool.map(response_task, jobs)) if jobs else []
    loops = [(name, log, params) for params in candidates for name, log in closed] if closed_weight else []
    loop_misfits = list(pool.map(closed_task, loops, chunksize=1)) if loops else []
    out = []
    for i in range(len(candidates)):
        part = results[i * len(chosen): (i + 1) * len(chosen)]
        summary = total(part)
        if summary:
            summary["response"] = float(np.mean(misfits[i * len(measured): (i + 1) * len(measured)])) if jobs else 0.0
            summary["closed"] = float(np.mean(loop_misfits[i * len(closed): (i + 1) * len(closed)])) if loops else 0.0
            summary["cost"] = summary["total"] + weight * summary["response"] + closed_weight * summary["closed"]
        out.append(summary)
    return out


def bounds_of(name, given):
    if given:
        low, high = given.split(":")
        return float(low), float(high)
    term = name.split(":", 1)[1] if ":" in name else name
    if name.startswith("observer:"):
        return {"filter": (0.05, 3.0), "correlation": (0.3, 60.0)}.get(term, (0.05, 10.0) if term.startswith("sd") else (0.3, 30.0))
    if name.startswith("geometry:"):
        return {"pitch_inertia": (0.002, 0.008), "roll_inertia": (0.008, 0.018), "yaw_inertia": (0.007, 0.016), "com_z": (-0.03, 0.04)}[term]
    return A.PARAMS[term][:2]


def current(base, name):
    if name.startswith("geometry:"):
        term = name.split(":", 1)[1]
        geometry = A.geometry_of(base)
        return geometry["com"][2] if term == "com_z" else geometry[term]
    if name.startswith("observer:"):
        term = name.split(":", 1)[1]
        key, *axis = term.split("_")
        value = (base.get("observer") or {"correlation": 10.0, "sd": 1.0, "noise": 4.0}).get(key, 1.0)
        return value[("roll", "pitch", "yaw").index(axis[0]) if axis else 0] if isinstance(value, list) else value
    if ":" in name:
        group, term = name.split(":", 1)
        return base.get("configurations", {}).get(group.split("+")[0], {}).get(term, base.get(term, A.PARAMS[term][2]))
    return base.get(name, A.PARAMS[name][2] if name in A.PARAMS else 0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", required=True)
    parser.add_argument("--fit", default="")
    parser.add_argument("--train", nargs="*", default=[], help="training flights (recording-name prefixes; default every scored flight not held out)")
    parser.add_argument("--hold", nargs="*", default=[], help="held-out flights (prefixes)")
    parser.add_argument("--list", default=str(S.FLIGHTS))
    parser.add_argument("--every", type=float, default=0.2)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--population", type=int, default=0)
    parser.add_argument("--sigma", type=float, default=0.15)
    parser.add_argument("--terms-every", type=int, default=10)
    parser.add_argument("--out", default="fitted.json")
    parser.add_argument("--score", action="store_true")
    parser.add_argument("--firmware", action="store_true", help="score with the firmware in the loop (scorecard.py --firmware)")
    parser.add_argument("--response", type=float, default=0.0, help="weight of the frequency-response misfit in the cost")
    parser.add_argument("--closed", type=float, default=0.0, help="weight of the closed-loop misfit in the cost")
    parser.add_argument("--closed-flights", nargs="*", default=[], help="flights (full recording names) flown closed loop for it")
    args = parser.parse_args()
    import cma
    initialize(args.list, args.firmware)
    base = A.model(args.base)
    everything = S.entries(listing)
    held = [e for e in everything if any(e["name"].startswith(p) for p in args.hold)]
    train = [e for e in everything if e not in held and (not args.train or any(e["name"].startswith(p) for p in args.train))]
    print(f"training {len(train)} flights, held out {len(held)}", flush=True)
    specs = [item.split("=") for item in args.fit.split(",") if item]
    names = [s[0] for s in specs]
    bounds = np.array([bounds_of(s[0], s[1] if len(s) > 1 else None) for s in specs])
    began = clock.time()
    with ProcessPoolExecutor(args.workers, initializer=initialize, initargs=(args.list, args.firmware)) as pool:
        terms = find_terms(pool, train + held, base, args.warmup)
        initialize(args.list, args.firmware)
        measured = responses(train) if args.response else []
        if measured:
            print("frequency responses measured for " + ", ".join(m[0]["configuration"] for m in measured), flush=True)
        from sw.sim import scorecard_closed as C
        closed = [(name, C.log_measures(name)) for name in args.closed_flights] if args.closed else []
        for name, log in closed:
            print(f"closed loop {name}: {log}", flush=True)
        extra = {"closed": closed, "closed_weight": args.closed}
        if args.score or not names:
            for label, chosen in (("training", train), ("held out", held)):
                if chosen:
                    summary = evaluate(pool, chosen, [base], args.every, args.warmup, terms, measured, 1.0 if measured else 0.0, closed, 1.0 if closed else 0.0)[0]
                    print(f"{label}: total {summary['total']:.4f} response {summary['response']:.4f} closed {summary['closed']:.3f} " + ", ".join(f"{k} {v:.3f}" for k, v in summary["kinds"].items()), flush=True)
            return
        start = np.array([current(base, n) for n in names])
        x0 = np.clip((start - bounds[:, 0]) / (bounds[:, 1] - bounds[:, 0]), 0.02, 0.98)
        options = {"bounds": [0, 1], "seed": 1, "verbose": -9}
        if args.population:
            options["popsize"] = args.population
        es = cma.CMAEvolutionStrategy(x0, args.sigma, options)
        decode = lambda x: bounds[:, 0] + np.clip(x, 0, 1) * (bounds[:, 1] - bounds[:, 0])
        cost = lambda summary: summary["cost"] if summary and math.isfinite(summary["cost"]) else 1e3
        best = (cost(evaluate(pool, train, [base], args.every, args.warmup, terms, measured, args.response, **extra)[0]), start)
        print(f"start {best[0]:.4f} " + " ".join(f"{n}={v:.4g}" for n, v in zip(names, start)), flush=True)
        for generation in range(args.generations):
            xs = es.ask()
            candidates = [apply(base, names, decode(x)) for x in xs]
            summaries = evaluate(pool, train, candidates, args.every, args.warmup, terms, measured, args.response, **extra)
            costs = [cost(s) for s in summaries]
            es.tell(xs, costs)
            i = int(np.argmin(costs))
            if costs[i] < best[0]:
                best = (costs[i], decode(xs[i]))
            print(f"gen {generation} {clock.time() - began:.0f} s best {best[0]:.4f} gen {costs[i]:.4f} (total {summaries[i]['total'] if summaries[i] else math.nan:.4f}, response {summaries[i]['response'] if summaries[i] else math.nan:.4f}, closed {summaries[i]['closed'] if summaries[i] else math.nan:.3f}) " + " ".join(f"{n}={v:.4g}" for n, v in zip(names, best[1])), flush=True)
            if (generation + 1) % args.terms_every == 0 and generation + 1 < args.generations:
                terms = find_terms(pool, train + held, apply(base, names, best[1]), args.warmup)
                best = (cost(evaluate(pool, train, [apply(base, names, best[1])], args.every, args.warmup, terms, measured, args.response, **extra)[0]), best[1])
                print(f"  terms refound: best {best[0]:.4f}", flush=True)
        fitted = apply(base, names, best[1])
        terms = find_terms(pool, train + held, fitted, args.warmup)
        for label, chosen in (("training", train), ("held out", held)):
            if chosen:
                summary = evaluate(pool, chosen, [fitted], args.every, args.warmup, terms, measured, 1.0 if measured else 0.0, closed, 1.0 if closed else 0.0)[0]
                print(f"{label}: total {summary['total']:.4f} response {summary['response']:.4f} " + ", ".join(f"{k} {v:.3f}" for k, v in summary["kinds"].items()), flush=True)
                for u in summary["units"]:
                    print(f"   {u['name']:24} {u['kind']:14} {u['score']:.3f}", flush=True)
    fitted["fit"] = {"names": names, "values": [float(v) for v in best[1]], "train": [e["name"] for e in train], "held_out": [e["name"] for e in held], "every": args.every}
    Path(args.out).write_text(json.dumps(fitted, indent=1))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
