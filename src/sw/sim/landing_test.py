"""
Hover landings for the touchdown detector (fw/drone/main/hover.c), in the simulator and replayed from recordings.
uv run -m sw.sim.landing_test [--runs 200] [--source path/to/hover.c] [--defines "PROBE_RATE=1.0f ..."] flies randomized landings: each takes off on IDENTIFIED_ESC with the 2026-10-29 07:20 camera-hover settings, holds at a random tag height, then lands at a random land_rate (0.05-0.2 m/s) with the hover_mass setting off by up to ±20 % from what the plant needs, the plant's mass off by up to ±10 %, the pack at 16.7 V, 14.3 V or sagging from one to the other over the flight (through the firmware's voltage compensation against the airframe's fitted voltage power, which leaves up to 18 % more thrust per table newton on the low pack), a small level-reference error and accelerometer bias, and the camera's height drifting by up to ±0.3 m over the descent (the drone's attitude error at the tag's 2 m range). --no-camera lands without the tag, --gust adds rougher air. It reports any touchdown declared in the air, how long after the body first touched the floor the landing ended (by the detector, or by the tilt and shock cut), the contact speed and the tilt through the touchdown.
uv run -m sw.sim.landing_test --replay sw/recordings/<flight>.bin [--contact s] replays a recorded landing through hover.c with the recorded estimates, specific force and attitude forced in each tick. Before the contact time given (or throughout without one) the drone is taken to be in the air: thrust the new code flies below the recorded one shows in the specific force as that difference, lagged by the motors, over the weight. After it the floor holds the drone and the recorded specific force stands. It prints when probes start and what they find, and when touchdown comes.
--source flies another copy of hover.c (its own hover.h beside it gives the state layout), so versions can be compared without touching the one other simulations use.
"""

import argparse
import ctypes
import hashlib
import json
import math
import os
import random
import re
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

RECORDING = Path(__file__).resolve().parents[1] / "recordings/2026-10-29T07-20-47-024Z_2551243463.jsonl"
# The pack's resting extremes (V) and the voltage the thrust table was measured at (the recording's thrust_voltage).
PACKS = (16.7, 14.3)
# The hover_mass setting that matches IDENTIFIED_ESC's hold thrust at the thrust table's voltage (kg): its hold flies at about 9.5 N.
SIM_MASS = 0.97
# The motors' dead time and lag for the replay's in-air specific force (s; models/airframe_2026-10-29.json: 50 ms dead time, 43 ms lag).
MOTOR_LAG = 0.1


class Done(Exception):
    """Raised from the hover tick once the landing has finished, to end the flight early (the floor contacts make the rest slow to simulate)."""


# The camera's field of view (sweep.CAMERA's, degrees), but reaching 20 degrees above its axis: the drone saw the tag on the floor at 0.5-0.65 m above it from 2 m (2026-10-29 07:20 and 07:42), which the simulated camera's 12.4 degrees would lose, leaving its height to the IMU alone, which the drone did not do.
FOV = (18.0, 20.0, 15.0)
TYPES = {"bool": ctypes.c_bool, "float": ctypes.c_float, "int": ctypes.c_int, "int64_t": ctypes.c_int64, "uint32_t": ctypes.c_uint32}


def state_class(header):
    """
    A ctypes structure for HoverState as the header lays it out, so a copy of hover.c with other fields can be flown.
    """
    text = re.sub(r"//[^\n]*", "", header.read_text())
    constants = {name: int(value) for name, value in re.findall(r"#define\s+(\w+)\s+(\d+)", text)}
    body = re.search(r"typedef struct \{([^}]*)\} HoverState;", text).group(1)
    fields = []
    for declaration in body.split(";"):
        declaration = declaration.strip()
        if not declaration:
            continue
        kind, names = declaration.split(None, 1)
        for name in names.split(","):
            name = name.strip()
            dims = [int(constants.get(d, d)) for d in re.findall(r"\[(\w+)\]", name)]
            ctype = TYPES[kind]
            for size in reversed(dims):
                ctype = ctype * size
            fields.append((name.split("[")[0], ctype))
    return type("State", (ctypes.Structure,), {"_fields_": fields})


def install(source, defines):
    """
    Point sw.sim.hover at a build of source (with defines) and its header's state layout, and record each tick of the hover the simulation creates.
    """
    from sw.sim import hover

    if source:
        source = Path(source).resolve()
        tag = hashlib.sha1((source.read_text() + source.with_suffix(".h").read_text() + " ".join(defines)).encode()).hexdigest()[:12]
        library = source.with_name("libhover-test-" + tag + ".so")
        if not library.exists():
            hover.build(source, library, defines)
        hover.State = state_class(source.with_suffix(".h"))
        hover.library = hover.load(library, hover.State)
    elif defines:
        raise SystemExit("--defines needs --source (or set HOVER_DEFINES)")
    # The unwrapped class, so flying several cases in one process does not stack the wrappers.
    base = hover.__dict__.setdefault("Unrecorded", hover.Hover)

    class Recorded(base):
        ticks = []
        starts = []
        how = None
        height_error = 0.0

        def __init__(self, settings):
            super().__init__(settings)
            Recorded.ticks = []
            Recorded.starts = []
            Recorded.how = None
            self.probing = self.landed = False
            self.landing_us = None
            self.off = 0.0

        def update(self, mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, drag=None):
            # The camera's height error near the floor (the drone's attitude error times the 2 m range): from the landing's start the tag's apparent height drifts by height_error (m, positive reads the drone low) over the time the descent should take.
            if mode != hover.HOLD and self.landing_us is None:
                self.landing_us, self.descent = now_us, max(0.5, self.state.height / self.gains.land_rate)
            if pose and self.landing_us is not None:
                share = min(1.0, (now_us - self.landing_us) / 1e6 / self.descent)
                pose.tag[1] += Recorded.height_error * share
            out = super().update(mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, drag)
            state = self.state
            self.off = self.off + dt if state.landed and out.force <= 0 else 0
            if self.off > 1.0:
                raise Done
            probing = getattr(state, "probing", False)
            if probing and not self.probing:
                # What most likely started the probe: the stalled climb rate, the height estimate under the floor, the accelerometer's stopped descent, or else the landing's time.
                Recorded.starts.append("stall" if state.stall_time >= 0.2 else "low" if state.height < 0 else "impulse" if state.impulse > 0.08 else "time")
            if self.probing and not probing and not state.landed:
                Recorded.starts[-1] += " aborted"
            if state.landed and not self.landed:
                # Touchdown declared (thrust still on) or a contact cut (tilt or shock: thrust off at once).
                Recorded.how = "cut" if out.force <= 0 else "touchdown"
            self.landed = state.landed
            self.probing = probing
            Recorded.ticks.append((now_us / 1e6 - 1, mode, state.landed, out.force, vertical_g, state.height, probing))
            return out

    hover.Hover = Recorded
    return Recorded


def draw(seed):
    """
    One randomized landing.
    """
    rng = random.Random(seed)
    pack = rng.choice(("high", "low", "sag"))
    return {
        "seed": seed,
        "land_rate": rng.choice((0.05, 0.1, 0.15, 0.2)),
        "mass_error": rng.uniform(0.8, 1.2),
        "plant_mass": rng.uniform(0.9, 1.1),
        "pack": pack,
        "volts": {"high": (PACKS[0], PACKS[0]), "low": (PACKS[1], PACKS[1]), "sag": PACKS}[pack],
        "tag_height": rng.uniform(0.3, 0.65),
        "level_error": (rng.uniform(-1, 1), rng.uniform(-1.5, 1.5)),
        "accel_bias": (rng.uniform(-0.005, 0.005), rng.uniform(-0.005, 0.005)),
        "height_error": rng.uniform(-0.3, 0.3),
        "land_at": rng.uniform(5, 9),
    }


def fly(case, source=None, defines=()):
    """
    Fly one case; return its touchdown figures (times in s after liftoff).
    """
    from sw.sim import airframe, sweep

    recorded = install(source, list(defines))
    settings = json.loads(RECORDING.read_text().splitlines()[0])["settings"]
    thrust_voltage = settings["thrust_voltage"]
    recorded.height_error = case["height_error"]
    power = sweep.IDENTIFIED_ESC["airframe"]["voltage_power"]
    # The thrust the compensated motors make per table newton at the pack's starting voltage, so hover_mass is off by mass_error from what this plant needs.
    factor = (case["volts"][0] / thrust_voltage) ** (power - 2)
    settings |= {"hover_land_rate": case["land_rate"], "hover_mass": SIM_MASS * case["plant_mass"] / factor * case["mass_error"], "hover_ceiling": 1.2}
    if not case.get("camera", True):
        settings["hover_camera"] = 0
    settings |= case.get("settings", {})
    plant = sweep.IDENTIFIED_ESC | {"land_at": case["land_at"], "mass": sweep.IDENTIFIED_ESC["mass"] * case["plant_mass"], "level_error": case["level_error"], "accel_bias": case["accel_bias"],
                                    **({"gust": case["gust"], "roll_gust": case["gust"]} if case.get("gust") else {}),
                                    "camera": sweep.IDENTIFIED_ESC["camera"] | {"tag": (2.0, 0.1, case["tag_height"]), "fov": FOV}}
    expected = case["tag_height"] / case["land_rate"]
    seconds = 1.5 + case["land_at"] + 2 * expected + 6
    # The pack over the flight, and the firmware's compensation for it (flight_rt.c: thrust_scale times the battery ratio squared): the motors then make the table thrust times ratio^(voltage_power - 2), which the airframe's voltage input gives at this ratio.
    ticks = [0]
    step = airframe.Airframe.step

    def stepped(self, throttles, commands, voltage, *rest, **named):
        start, end = case["volts"]
        volts = start + (end - start) * min(1.0, ticks[0] * sweep.DT / seconds)
        ticks[0] += 1
        ratio = volts / thrust_voltage
        return step(self, throttles, commands, ratio ** ((power - 2) / power), *rest, **named)

    airframe.Airframe.step = stepped
    log = []
    try:
        result = sweep.run(settings, plant, seconds=seconds, seed=case["seed"], log=log, hover=True)
    except Done:
        result = {"stable": True}
    finally:
        airframe.Airframe.step = step
    out = dict(case, stable=result["stable"], fail_s=result.get("fail_s"))
    # The lowest body corner above the floor each tick, and liftoff: when it first left the floor, as sweep.run counts it.
    lowest = [row[1][2] + min(sweep.apply(sweep.rotation(*row[2]), foot)[2] for foot in sweep.BODY_FEET) for row in log]
    lifted = next((row[0] for row, height in zip(log, lowest) if height > 0), None)
    if lifted is None:
        return out | {"error": "no liftoff"}
    times = [row[0] - lifted for row in log]
    start = next((i for i, t in enumerate(times) if t >= case["land_at"]), None)
    if start is None:
        return out | {"error": "failed before landing"}
    # Contact: the lowest corner within 1 mm of the floor (resting on it at full weight sags it about 1 mm).
    contact = next((i for i in range(start, len(log)) if lowest[i] <= 0.001), None)
    # The hover's tick at each control step (it runs from arming on).
    by_step = {round(tick[0] / sweep.DT): tick for tick in recorded.ticks}
    fired = next((i for i in range(start, len(log)) if i in by_step and by_step[i][2]), None)
    off = next((i for i in range(fired, len(log)) if by_step.get(i, (0, 0, True, 0.0))[3] <= 0), None) if fired is not None else None
    probes = 0
    previous = False
    for tick in recorded.ticks:
        if tick[0] - lifted >= case["land_at"]:
            if tick[6] and not previous:
                probes += 1
            previous = tick[6]
    out["starts"] = recorded.starts
    out["how"] = recorded.how
    out |= {"land_height": lowest[start], "contact_s": times[contact] if contact is not None else None, "fired_s": times[fired] if fired is not None else None, "off_s": times[off] if off is not None else None, "probes": probes}
    if contact is not None:
        out["contact_speed"] = -(lowest[contact] - lowest[contact - 2]) / (2 * sweep.DT)
        last = max(contact, off) if off is not None else len(log) - 1
        out["contact_tilt"] = max(math.degrees(max(abs(row[2][0]), abs(row[2][1]))) for row in log[contact:last + 50])
    if fired is not None:
        out["fired_height"] = lowest[fired]
        out["estimate_at_fire"] = by_step[fired][5]
    return out


def summary(results, label):
    def percentile(values, q):
        values = sorted(values)
        return values[min(len(values) - 1, int(q * len(values)))] if values else float("nan")

    # Holds that never got the drone off the floor, came down before the landing or flew away (the simulated hold's own failures) say nothing about touchdown.
    failed = [r for r in results if "error" in r or not 0.1 <= r["land_height"] <= 1.5]
    results = [r for r in results if r not in failed]
    air = [r for r in results if r.get("fired_s") is not None and (r.get("contact_s") is None or r["fired_s"] < r["contact_s"] - 0.005)]
    declared = [r for r in air if r.get("how") != "cut"]
    never = [r for r in results if r.get("contact_s") is not None and r.get("fired_s") is None]
    unstable = [r for r in results if not r["stable"]]
    print("%s: %d landings (%d holds failed first), %d touchdowns declared in the air, %d tilt or shock cuts in the air, %d never ended after contact, %d fell over or crashed" % (
        label, len(results), len(failed), len(declared), len(air) - len(declared), len(never), len(unstable)))
    ended = [r for r in results if r not in air and r.get("fired_s") is not None and r.get("contact_s") is not None]
    for how, name in (("touchdown", "by the detector"), ("cut", "by the tilt or shock cut")):
        late = [r["fired_s"] - r["contact_s"] for r in ended if r.get("how") == how]
        if late:
            print("  ended %s: %d, after contact (s) median %.2f, 75%% %.2f, 90%% %.2f, max %.2f, within 0.5 s %d%%" % (
                name, len(late), percentile(late, 0.5), percentile(late, 0.75), percentile(late, 0.9), max(late), round(100 * sum(d <= 0.5 for d in late) / len(late))))
    for rate in sorted({r["land_rate"] for r in results}):
        group = [r for r in results if r["land_rate"] == rate]
        late = [r["fired_s"] - r["contact_s"] for r in ended if r["land_rate"] == rate]
        speeds = [r["contact_speed"] for r in group if "contact_speed" in r]
        print("  land_rate %.2f: %d landings, after contact median %.2f max %.2f s, contact speed median %.2f max %.2f m/s, probes %.1f a landing (%d%% found air)" % (
            rate, len(group), percentile(late, 0.5), max(late, default=float("nan")), percentile(speeds, 0.5), max(speeds, default=float("nan")),
            sum(r["probes"] for r in group) / len(group), round(100 * sum(s.endswith("aborted") for r in group for s in r.get("starts", [])) / max(1, sum(len(r.get("starts", [])) for r in group)))))
    speeds = [r["contact_speed"] for r in results if "contact_speed" in r]
    tilts = [r["contact_tilt"] for r in results if "contact_tilt" in r]
    if speeds:
        print("  contact speed (m/s): median %.2f, 90%% %.2f, max %.2f; tilt through touchdown (deg): median %.1f, 90%% %.1f, max %.1f" % (
            percentile(speeds, 0.5), percentile(speeds, 0.9), max(speeds), percentile(tilts, 0.5), percentile(tilts, 0.9), max(tilts)))
    for r in air:
        print("  in the air (%s): seed %d at %.3f m (estimate %.3f), contact %s" % (r.get("how"), r["seed"], r["fired_height"], r.get("estimate_at_fire", float("nan")), r["contact_s"]))
    for r in never + unstable:
        print("  problem: seed %d land_rate %.2f contact %s ended %s stable %s" % (r["seed"], r["land_rate"], r.get("contact_s"), r.get("fired_s"), r["stable"]))


def replay(recording, contact=None, source=None, defines=()):
    """
    Replay a recorded landing through hover.c (see the module notes); print what its touchdown logic does.
    """
    import numpy as np

    from sw.flight_log import read_flight
    from sw.sim import hover

    install(source, list(defines))
    recording = Path(recording)
    data, truncated = read_flight(recording.with_suffix(".bin"))
    start = json.loads(recording.with_suffix(".jsonl").read_text().splitlines()[0])
    settings, table = start["settings"], np.array(start["thrust_table"])
    t = (data.esp_time_us.values - data.esp_time_us.values[0]) / 1e6
    roll, pitch = np.radians(data.roll_deg.values), np.radians(data.pitch_deg.values)
    # The vertical specific force as flight_rt.c takes it (the IMU attitude is the level-referenced one turned back by pose_zero), and the collective thrust from the base throttle.
    level = pitch + math.radians(start["pose_zero_deg"][1])
    vertical = -data.ax_g.values * np.sin(level) + np.cos(level) * (data.ay_g.values * np.sin(roll) + data.az_g.values * np.cos(roll))
    force = np.where(data.base_throttle.values > 0, 2 * np.interp(data.base_throttle.values, table[:, 0], table[:, 1]) * math.cos(math.radians(settings["pitch_trim"])), 0.0)
    flags = data["flags"].values
    mode = (flags >> 1) & 3
    # The recording drops packets: each sample is held over the ticks it is missing, so the 100 Hz timing (landing time, filters) is kept.
    grid = np.arange(t[0], t[-1], 0.01)
    pick = np.searchsorted(t, grid, side="right") - 1
    t, roll, pitch, vertical, force, mode = grid, roll[pick], pitch[pick], vertical[pick], force[pick], mode[pick]
    height, climb = data.hover_height_m.values[pick], data.hover_climb_m_s.values[pick]
    gains = hover.gains(settings)
    state, out = hover.State(), hover.Output()
    difference, flown, first, engaged = 0.0, None, None, False
    print("%s: land_rate %.2f, hover_mass %.2f, contact %s" % (recording.name, gains.land_rate, gains.mass, "%.2f s" % contact if contact is not None else "none (in the air throughout)"))
    for i in range(len(t)):
        if mode[i] == 0:
            if not engaged:
                hover.library.hover_rest(ctypes.byref(state), float(vertical[i]), False, 0.01)
                continue
            break
        engaged = True
        air = contact is None or t[i] < contact
        if first is None and mode[i] != hover.HOLD:
            first = t[i]
            print("  %7.2f landing starts at an estimated %.2f m" % (t[i], height[i]))
        if state.engaged:
            # The recorded estimates, so the replay sees what the drone's estimator saw.
            state.height = float(height[i]) - state.climb_rate * 0.01
            state.climb_rate = float(climb[i])
        landing = mode[i] != hover.HOLD
        if landing and air and flown is not None:
            difference += ((flown - force[i]) - difference) * (1 - math.exp(-0.01 / MOTOR_LAG))
        else:
            difference = 0.0
        weight = state.hold_force if getattr(state, "hold_force", 0) > 0 else gains.mass * 9.81
        before = (state.landed, getattr(state, "probing", False))
        hover.library.hover_update(ctypes.byref(state), ctypes.byref(gains), int(mode[i]), None, int(t[i] * 1e6) + 1000000, 0.01, float(roll[i]), float(pitch[i]), 0.0,
                                   float(vertical[i]) + difference / weight, state.force if state.engaged else float(force[i]), None, ctypes.byref(out))
        if not landing:
            # The hold as flown: its thrust (and the integral that gives it, so the landing starts from it), which the new code's references learn from.
            lean = max(0.5, math.cos(roll[i]) * math.cos(pitch[i]))
            if hasattr(state, "hold_force") and height[i] >= 0.05:
                state.hold_force += (float(force[i]) - state.hold_force) * (1 - math.exp(-0.01 / 1.0))
            state.force = out.force = float(force[i])
            state.integral = float(force[i]) * lean / gains.mass - 9.81
        flown = out.force
        probing = getattr(state, "probing", False)
        if probing and not before[1]:
            print("  %7.2f probe starts (stall %.2f s, height estimate %+.2f m, impulse %+.3f m/s, %.1f s into the landing)" % (t[i], state.stall_time, state.height, state.impulse, state.land_time))
        if before[1] and not probing and not state.landed:
            print("  %7.2f probe finds air (accelerometer %+.3f m/s against %+.3f in the air)" % (t[i], state.probe_velocity, state.probe_expected))
        if state.landed and not before[0]:
            what = "tilt or shock cut" if out.force <= 0 else "touchdown"
            if hasattr(state, "probe_velocity") and what == "touchdown":
                what += " (accelerometer %+.3f m/s against %+.3f in the air)" % (state.probe_velocity, state.probe_expected)
            print("  %7.2f %s at %.2f of the hold's thrust" % (t[i], what, out.force / weight))
        if state.landed and out.force <= 0:
            print("  %7.2f thrust off" % t[i])
            break
    else:
        print("  %7.2f recording ends without touchdown" % t[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--runs", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("--source", help="hover.c to fly (default the firmware's)")
    parser.add_argument("--defines", default="", help="compile-time overrides for --source, space separated")
    parser.add_argument("--save", help="write every run's figures here (JSON)")
    parser.add_argument("--replay", help="a recording to replay instead of simulating")
    parser.add_argument("--contact", type=float, help="with --replay: when the drone touched the floor (s from the recording's start)")
    parser.add_argument("--no-camera", action="store_true", help="never see the tag (hover_camera 0): height from the IMU alone, as a landing outdoors after a lost link")
    parser.add_argument("--set", nargs="*", default=[], help="settings to override, name=value")
    parser.add_argument("--gust", type=float, default=0, help="random pitch and roll disturbance (rad/s² RMS) on top of the room's; indoors fitted 0.49 and 0.89")
    args = parser.parse_args()
    defines = args.defines.split()
    if args.replay:
        replay(args.replay, args.contact, args.source, defines)
        return
    overrides = {name: float(value) for name, value in (pair.split("=") for pair in args.set)}
    cases = [draw(args.seed + i) | {"camera": not args.no_camera, "gust": args.gust, "settings": overrides} for i in range(args.runs)]
    with ProcessPoolExecutor(args.workers) as pool:
        results = list(pool.map(fly, cases, [args.source] * len(cases), [defines] * len(cases)))
    if args.save:
        Path(args.save).write_text(json.dumps(results, indent=1))
    summary(results, args.source or "firmware hover.c")


if __name__ == "__main__":
    main()
