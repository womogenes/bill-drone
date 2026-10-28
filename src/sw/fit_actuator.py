"""
Pitch actuator system identification from airborne recordings: uv run -m sw.fit_actuator sw/recordings/<flight>.bin [...]
Each servo's logged command goes through a model of the real servo (dead time, first-order lag, slew limit, backlash, each side its own backlash) to a physical tilt; pitch acceleration is then regressed on the tilted thrust's torque, a constant push and the pitch rate (aerodynamic damping). The actuator parameters are searched on a grid and scored by leave-one-flight-out prediction, against the plain linear model of sw.fit_airborne.
"""

import itertools
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from sw.fit import acceleration, settings
from sw.flight_log import read_flight
from sw.sim.flight import thrust_table

DEG = np.pi / 180
DT = 0.01
# Pitch-rate change over this many samples is the second score: less noisy than a differentiated gyro.
HORIZON = 10


def window(path):
    """
    The airborne part of a flight: from liftoff (vertical specific force past 1.12 g once collective reaches 600) to an upset past 35°, an impact, motor stop or touchdown.
    """
    data, _ = read_flight(path)
    setting = settings(path)
    on = np.flatnonzero(data.base_throttle.values >= 600)
    if not len(on) or not (data.az_g.values[on[0]:] > 1.12).any():
        return None
    lift = on[0] + np.argmax(data.az_g.values[on[0]:] > 1.12) + 3
    shock = np.sqrt(data.ax_g**2 + data.ay_g**2 + data.az_g**2).values
    flags = data["flags"].values
    end = next((i for i in range(lift + 3, len(data)) if abs(data.pitch_deg[i]) > 35 or abs(data.roll_deg[i]) > 35 or shock[i] > 2.5 or data.base_throttle[i] == 0 or flags[i] & 16), len(data))
    if end - lift < 50:
        return None
    ratio = (data.battery_v.values / setting["thrust_voltage"]) ** 2 if setting.get("thrust_voltage") else 1.0
    table = np.array(thrust_table(path), float)
    left = np.interp(data.left_throttle.values, table[:, 0], table[:, 1]) * ratio
    right = np.interp(data.right_throttle.values, table[:, 0], table[:, 1]) * ratio
    # Commanded physical tilt, positive forward: the right servo is mirrored, and the trims are each servo's zero.
    commands = np.column_stack([data.left_servo_deg.values - setting["left_trim"], -(data.right_servo_deg.values - setting["right_trim"])])
    return {
        "name": path.name[11:19], "lift": lift, "end": end, "setting": setting, "thrust": np.column_stack([left, right]), "command": commands,
        "rate": data.gy_deg_s.values * DEG, "accel": acceleration(data.gy_deg_s.values, 5), "pitch": data.pitch_deg.values * DEG,
    }


def servo(command, delay, tau, slew, backlash):
    """
    Physical tilt (degrees) from a servo's command (degrees): dead time in samples, then a first-order lag (s) limited to slew (°/s), then backlash (total play, degrees) between the horn and the motor mount.
    """
    shifted = np.concatenate([np.full(delay, command[0]), command[:len(command) - delay]]) if delay else command
    keep = np.exp(-DT / tau) if tau > 0 else 0.0
    step = slew * DT if slew else np.inf
    half = backlash / 2
    horn = shifted[0]
    tilt = horn
    out = np.empty_like(command)
    for i, target in enumerate(shifted):
        move = (target - horn) * (1 - keep)
        horn += min(step, max(-step, move))
        if horn - tilt > half:
            tilt = horn - half
        elif tilt - horn > half:
            tilt = horn + half
        out[i] = tilt
    return out


def torque(flight, params):
    """
    Model pitch acceleration from the tilted thrust, in rad/s², as the controller's model would predict it (effectiveness 1).
    """
    delay, tau, slew, left_play, right_play = params
    left = servo(flight["command"][:, 0], delay, tau, slew, left_play) * DEG
    right = servo(flight["command"][:, 1], delay, tau, slew, right_play) * DEG
    thrust = flight["thrust"]
    setting = flight["setting"]
    return (thrust[:, 0] * np.sin(left) + thrust[:, 1] * np.sin(right)) * setting["pivot_height"] / setting["pitch_inertia"]


def design(flight, model, damping):
    span = slice(flight["lift"], flight["end"])
    columns = [model[span], np.ones(flight["end"] - flight["lift"])]
    if damping:
        columns.append(flight["rate"][span])
    return np.column_stack(columns), flight["accel"][span]


def horizon_change(flight, model, coefficients, damping):
    """
    Measured and predicted change of pitch rate over HORIZON samples, the prediction integrating the model acceleration from the measured rate.
    """
    span = np.arange(flight["lift"], flight["end"] - HORIZON)
    columns = [model, np.ones_like(model)] + ([flight["rate"]] if damping else [])
    predicted = np.column_stack(columns) @ coefficients
    total = np.concatenate([[0], np.cumsum(predicted * DT)])
    return flight["rate"][span + HORIZON] - flight["rate"][span], total[span + HORIZON] - total[span]


def r2(measured, predicted):
    return 1 - np.var(measured - predicted) / np.var(measured)


def evaluate(flights, params, damping=True):
    """
    Leave-one-flight-out: coefficients fitted on the other flights predict each flight in turn. Returns pooled held-out R² of acceleration and of HORIZON rate change, and the all-flight coefficients.
    """
    models = [torque(flight, params) for flight in flights]
    blocks = [design(flight, model, damping) for flight, model in zip(flights, models)]
    measured, predicted, change, change_predicted = [], [], [], []
    for held in range(len(flights)):
        x = np.concatenate([b[0] for i, b in enumerate(blocks) if i != held])
        y = np.concatenate([b[1] for i, b in enumerate(blocks) if i != held])
        coefficients, *_ = np.linalg.lstsq(x, y, rcond=None)
        measured.append(blocks[held][1])
        predicted.append(blocks[held][0] @ coefficients)
        a, b = horizon_change(flights[held], models[held], coefficients, damping)
        change.append(a)
        change_predicted.append(b)
    x = np.concatenate([b[0] for b in blocks])
    y = np.concatenate([b[1] for b in blocks])
    coefficients, *_ = np.linalg.lstsq(x, y, rcond=None)
    return r2(np.concatenate(measured), np.concatenate(predicted)), r2(np.concatenate(change), np.concatenate(change_predicted)), coefficients


def linear(flights):
    """
    The sw.fit_airborne model, scored the same way: logged commands shifted by the best whole-sample delay, no lag, play or damping.
    """
    return max((evaluate(flights, (delay, 0.0, 0, 0.0, 0.0), damping=False) + (delay,) for delay in range(21)), key=lambda result: result[0])


FLIGHTS = []


def score(params):
    return params, evaluate(FLIGHTS, params)


def paths(arguments):
    """
    Recordings named on the command line, as files or glob patterns.
    """
    return sorted(path for argument in arguments for path in (Path(argument).parent.glob(Path(argument).name) if "*" in argument else [Path(argument)]))


def main():
    flights = [flight for flight in (window(path) for path in paths(sys.argv[1:])) if flight]
    FLIGHTS.extend(flights)
    for flight in flights:
        print(f"{flight['name']} airborne {flight['lift'] / 100:5.2f}-{flight['end'] / 100:5.2f} s, pitch_damping {flight['setting']['pitch_damping']}")
    accel, change, (slope, push), delay = linear(flights)
    print(f"linear: delay {delay * 10} ms, effectiveness {slope:.2f} x model, push {push:+.2f} rad/s², held-out R² accel {accel:.3f}, {HORIZON * 10} ms rate change {change:.3f}")
    grid = list(itertools.product(range(3, 11), (0.0, 0.02, 0.04, 0.06, 0.08), (0, 300, 500), (0.0, 1.0, 2.0, 3.0, 4.0), (0.0, 1.0, 2.0, 3.0, 4.0)))
    with Pool() as pool:
        results = pool.map(score, grid, chunksize=8)
    results.sort(key=lambda result: -result[1][1])
    for params, (accel, change, coefficients) in results[:12]:
        delay, tau, slew, left_play, right_play = params
        print(f"delay {delay * 10:3d} ms, lag {tau * 1000:3.0f} ms, slew {slew or 'inf'}, play L {left_play:.1f}° R {right_play:.1f}°: effectiveness {coefficients[0]:.2f}, push {coefficients[1]:+.2f}, damping {coefficients[2]:+.2f}/s, held-out R² accel {accel:.3f}, rate change {change:.3f}")


if __name__ == "__main__":
    main()
