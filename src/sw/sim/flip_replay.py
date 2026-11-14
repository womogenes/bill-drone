"""
A logged flight replayed open loop in the full-attitude simulation (sw/sim/rigid.py): the plant is driven by the recording's own motor throttles and servo angles from the logged state at the start, and its body rates, specific force and height are compared with the log's: uv run -m sw.sim.flip_replay [--recording 2026-11-12T02-27-04] [--start 9.85] [--end 11.0] [--airframe airframe_2026-11-12_raised.json] [--plot out.png]

For the flip of 2026-11-12 02:27:04 UTC (drone 1's first, which failed): what the plant must have to make the logged rates from the logged commands. The attitude starts from the logged estimate (gyro-only through the flip, so as good as the gyro), the rates from the gyro, the climb rate from the hold's, the horizontal speed zero; the motors and servos start steady at the first logged commands. The thrust trim is the recording's own (its airborne median table thrusts carry the weight, as identify_airframe takes each flight's), so the start holds its height. The gyro is clipped at the recording's full scale as the drone's was.
"""

import argparse
import math
from pathlib import Path

import numpy as np

from sw.flight_log import metadata, read_flight, recording, settings
from sw.sim import airframe as A
from sw.sim import flight
from sw.sim import rigid as R

FLIP = "2026-11-12T02-27-04"
# The flip's phases in that recording (s from its first sample): the climb starts at 9.89, the spin at 9.99, the righting at 10.80, the arrest at 10.88; the gyro clips at 11.37.
START, END = 9.85, 11.0
# The bands' tilt behind its command (degrees) and the accelerometer's forward error (g) as sw/sim/flip.py has them.
BAND_TILT = -2.5


class Log:
    """The recording's samples over [start, end] (s from its first sample): time, gyro (rad/s), throttles, servo angles (degrees), roll and pitch (rad), specific force (g), the hold's height and climb rate, the barometer, the flip's phase."""

    def __init__(self, name=FLIP, start=START, end=END):
        path = recording(name)
        df = read_flight(path)[0]
        time = (df["esp_time_us"] - df["esp_time_us"].iloc[0]).to_numpy() / 1e6
        # The thrust trim from the hover before the window (both motors over 300, no flip), or from the whole recording's when the window starts with it.
        flying = (df["flip"].fillna(0) == 0) & (df["left_throttle"] > 300) & (df["right_throttle"] > 300)
        airborne = df[flying & (time < start)] if (flying & (time < start)).sum() > 100 else df[flying]
        self.table_thrust = np.column_stack([airborne[f"{side}_throttle"].map(flight.thrust) for side in ("left", "right")])
        self.settings = settings(path)
        self.gyro_range = 32768 / metadata(path).get("gyro_counts_per_deg_s", 131.0) * A.DEG
        rows = df[(time >= start - 1e-6) & (time <= end + 1e-6)]
        self.time = time[rows.index]
        self.gyro = rows[["gx_deg_s", "gy_deg_s", "gz_deg_s"]].to_numpy() * A.DEG
        self.throttle = rows[["left_throttle", "right_throttle"]].to_numpy(float)
        self.servo = rows[["left_servo_deg", "right_servo_deg"]].to_numpy(float)
        self.attitude = rows[["roll_deg", "pitch_deg"]].to_numpy() * A.DEG
        self.accel = rows[["ax_g", "ay_g", "az_g"]].to_numpy()
        self.height = rows["hover_height_m"].to_numpy()
        self.climb = rows["hover_climb_m_s"].to_numpy()
        self.baro = rows["baro_altitude_m"].to_numpy()
        self.phase = rows["flip"].fillna(0).to_numpy()

    def tilt(self):
        """The physical tilt commands (rad, positive forward) the servo angles stand for."""
        s = self.settings
        trims = np.array([s.get("left_trim", 0.0), s.get("right_trim", 0.0)])
        directions = np.array([flight.MODEL["left_servo_direction"], flight.MODEL["right_servo_direction"]])
        return np.radians(directions * (self.servo - trims))


def quaternion(roll, pitch):
    """Roll about forward, then pitch about left (flight_attitude's), the heading zero."""
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


def plant_for(log, airframe, geometry=None, band_tilt=BAND_TILT, **overrides):
    """A rigid.Sim plant: the airframe (a saved model's parameters, with overrides) at the geometry (default its own) with the recording's thrust trim."""
    # left_fault and right_fault are not plant terms: the share of its table thrust each ESC delivers from the flip's start on (a failing ESC or lead, the hypothesis the airframe must not absorb).
    fault = (overrides.pop("left_fault", 1.0), overrides.pop("right_fault", 1.0))
    params = dict(airframe) | overrides
    geometry = geometry or A.geometry_of(params)
    level, mean = A.trim_for(log.table_thrust * np.array([1.0, params.get("right_thrust_gain", 1.0)]), geometry)
    params |= {"thrust_level": float(level), "hover_thrust": [float(m) for m in mean]}
    return {"airframe": params, "geometry": geometry, "disturbances": [], "rotor_drag": (0.07, 0.2), "tilt_offset": band_tilt, "fault": fault}


def replay(log, plants, scale=None, every=None):
    """Every plant (a list of plant_for dicts with one airframe and geometry; one lane each, or one plant and a scale dict of per-lane arrays) driven by the log's commands. Returns time, rates (rad/s, gyro-clipped), specific force (g) and height change (m) [lane, sample, ...]. every (s): the body's rates and attitude are set back to the log's that often (multiple shooting: each stretch predicts from the logged state, the actuators carry on), so a long window does not compound one stretch's error into the next."""
    runs = [simulate(log, plant, None, every) for plant in plants] if scale is None else [simulate(log, plants[0], scale, every)]
    return {key: np.concatenate([r[key] for r in runs]) for key in runs[0]}


def simulate(log, plant, scale, every=None):
    n = 1 if scale is None else len(next(iter(scale.values())))
    sim = R.Sim(n, plant, seed=0, scale=scale)
    sim.gyro_bias[:] = 0.0
    tilt = log.tilt()
    p = sim.p
    gain = np.tile([1.0, p["right_thrust_gain"]], (n, 1))
    start_thrust = A.actual_thrust(p, A.table_thrust(p, np.tile(log.throttle[0], (n, 1)), np.ones(n), gain=gain), sim.trim)
    q = np.tile(quaternion(*log.attitude[0]), (n, 1))
    sim.reset(np.tile([0.0, 0.0, 10.0], (n, 1)), np.tile([0.0, 0.0, log.climb[0]], (n, 1)), q, np.tile(log.gyro[0], (n, 1)), motor=start_thrust, tilt=np.tile(tilt[0], (n, 1)) - sim.tilt_offset)
    # reset starts the throttle history at the table's throttle for that thrust; the log's own throttle is what was sent.
    sim.throttles[:] = log.throttle[0]
    # A failing ESC as the throttle that gives its share of the table thrust, from the flip's first sample on.
    throttle = log.throttle.copy()
    fault = plant.get("fault", (1.0, 1.0))
    flipping = log.phase > 0
    for side in range(2):
        if fault[side] != 1.0:
            throttle[flipping, side] = [flight.throttle(fault[side] * flight.thrust(c)) if c > 0 else c for c in throttle[flipping, side]]
    if sim.backend == "c":
        sim.bind()
    rates, forces, heights, ends = [sim.omega.copy()], [sim.accel / R.G], [np.zeros(n)], [sim.omega.copy()]
    stride = round(every / A.DT) if every else 0
    for k in range(1, len(log.time)):
        if stride and k % stride == 0:
            sim.omega[:] = log.gyro[k - 1]
            sim.quat[:] = quaternion(*log.attitude[k - 1])
        sim.step(np.tile(throttle[k], (n, 1)), np.tile(tilt[k], (n, 1)))
        rates.append(np.clip(sim.turned, -log.gyro_range, log.gyro_range))
        forces.append(np.clip(sim.accel / R.G, -2.0, 2.0))
        heights.append(sim.pos[:, 2] - 10.0)
        ends.append(sim.omega.copy())
    # rates are each tick's mean (as the gyro reads), omega the body rates at its end.
    return {"rates": np.stack(rates, 1), "accel": np.stack(forces, 1), "height": np.stack(heights, 1), "omega": np.stack(ends, 1)}


def errors(log, result, end=10.88, start=0.0):
    """RMS rate error (deg/s) per axis [lane, 3] over the log's samples from start to end."""
    upto = (log.time <= end) & (log.time >= start)
    difference = (result["rates"][:, upto] - log.gyro[upto]) / A.DEG
    return np.sqrt((difference ** 2).mean(1))


# What the flip fit may move, with bounds: the rotors' spin momentum, the thrust's fall when a motor is cut, the roll damping and the differential thrust's share (roll authority far from hover), the lumped yaw-from-roll coupling and the props' reaction torque (yaw).
FIT = {"rotor_momentum": (0.0, 0.06), "motor_tau_down": (0.02, 0.5), "inflow_damping": (0.0, 2.0), "differential_slope": (0.2, 1.5), "yaw_from_roll": (-0.05, 0.05), "prop_torque": (-0.04, 0.04),
       "motor_idle": (0.0, 150.0), "motor_floor": (0.0, 4.0), "motor_coast": (0.01, 1.0), "left_fault": (0.2, 1.2), "right_fault": (0.2, 1.2), "motor_start": (0.0, 0.4), "edgewise_damping": (0.0, 1.0), "wash": (0.0, 1.0)}
# Residual scales: deg/s of rate error worth one g of specific force error.
RATE_SCALE, ACCEL_SCALE = 50.0, 0.1


def residuals(log, airframe, geometry, names, x, end, every=None):
    """The fit's residuals: rate errors (deg/s over RATE_SCALE) on every axis and the specific force's error along the shafts (g over ACCEL_SCALE) up to end, samples where the gyro clipped left out."""
    plant = plant_for(log, airframe, geometry, **dict(zip(names, map(float, x))))
    result = replay(log, [plant], every=every)
    upto = (log.time <= end) & (np.abs(log.gyro).max(1) < 0.98 * log.gyro_range)
    rates = ((result["rates"][0, upto] - log.gyro[upto]) / A.DEG / RATE_SCALE).ravel()
    accel = (result["accel"][0, upto, 2] - log.accel[upto, 2]) / ACCEL_SCALE
    return np.concatenate([rates, accel])


def fit(log, airframe, geometry=None, names=tuple(FIT), end=10.88, start_values=None, every=None):
    """Least squares on the replay over the log up to end: the named airframe terms (FIT's bounds), from the airframe's values (or start_values) clipped into them."""
    from scipy.optimize import least_squares
    low = np.array([FIT[k][0] for k in names])
    high = np.array([FIT[k][1] for k in names])
    complete = A.complete(airframe) | {"left_fault": 1.0, "right_fault": 1.0}
    x0 = np.clip([(start_values or {}).get(k, complete[k] or (low[i] + high[i]) / 2) for i, k in enumerate(names)], low + 1e-6, high - 1e-6)
    result = least_squares(lambda x: residuals(log, airframe, geometry, names, x, end, every), x0, bounds=(low, high), diff_step=1e-3, x_scale="jac", max_nfev=400)
    # The parameters' standard errors from the Jacobian, with the residuals' own scatter (strongly correlated samples, so a lower bound).
    dof = max(1, len(result.fun) - len(names))
    sigma2 = (result.fun ** 2).sum() / dof
    try:
        covariance = np.linalg.inv(result.jac.T @ result.jac) * sigma2
        spread = np.sqrt(np.diag(covariance))
        correlation = covariance / np.outer(spread, spread)
    except np.linalg.LinAlgError:
        spread, correlation = np.full(len(names), np.nan), None
    return dict(zip(names, map(float, result.x))), dict(zip(names, map(float, spread))), correlation, float(np.sqrt((result.fun ** 2).mean()))


def closed_loop(airframe, settings=FLIP, seed=1000):
    """The firmware's flip (sw/sim/flip_firmware.py) flown once on the airframe, nominal, with the recording's settings and the bands: its trace (time from the flip's start, gyro deg/s, throttles, servo angles, phase, height)."""
    from sw.flight_log import settings as recording_settings
    from sw.sim import flip_firmware as FF
    FF.TRACE = True
    flown = recording_settings(recording(settings)) | FF.FLIGHT
    row = FF.fly((flown, BAND_TILT, seed, 1, 40.0, math.radians(2000), 2.0, 20.0, True, airframe, "none"))[0]
    return np.array(row["trace"]), row


def plot(path, log, before, after, airframes):
    """Real against the replay on the old and the fitted plant (left), and the firmware's flip flown closed loop on each (right), the flip's start at 0."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, old, new = "#0b0b0b", "#eb6834", "#2a78d6"
    start = log.time[np.argmax(log.phase > 0)]
    loops = [closed_loop(a) for a in airframes]
    figure, axes = plt.subplots(3, 2, figsize=(11, 7.5), sharex="col", sharey="row")
    for row, name in enumerate(("roll rate", "pitch rate", "yaw rate")):
        left, right = axes[row]
        left.plot(log.time - start, log.gyro[:, row] / A.DEG, color=ink, lw=2, label="real (2026-11-12 02:27:04)")
        left.plot(log.time - start, before["rates"][0, :, row] / A.DEG, color=old, lw=1.5, label=f"replayed on {airframes[0]}")
        left.plot(log.time - start, after["rates"][0, :, row] / A.DEG, color=new, lw=1.5, label=f"replayed on {airframes[1]}")
        right.plot(log.time - start, log.gyro[:, row] / A.DEG, color=ink, lw=2)
        for (trace, result), color in zip(loops, (old, new)):
            right.plot(trace[:, 0], trace[:, 1 + row], color=color, lw=1.5)
        left.set_ylabel(f"{name} (deg/s)")
        for axis in (left, right):
            axis.grid(color="#e6e5e1", lw=0.6)
            axis.spines[["top", "right"]].set_visible(False)
            for edge, label, height in ((0.10, "spin", 0.98), (0.91, "righting", 0.98), (0.99, "arrest", 0.88)):
                axis.axvline(edge, color="#c3c2b7", lw=0.8)
                if row == 0:
                    axis.text(edge + 0.01, height, label, transform=axis.get_xaxis_transform(), va="top", fontsize=8, color="#52514e")
    axes[0, 0].set_title("Open loop: the plant driven by the logged throttles and servos", fontsize=10, loc="left")
    axes[0, 1].set_title("Closed loop: the firmware's flip flown on each plant", fontsize=10, loc="left")
    for axis in axes[-1]:
        axis.set_xlabel("s from the flip's start")
    axes[-1, 1].set_xlim(-0.05, 1.7)
    figure.legend(*axes[0, 0].get_legend_handles_labels(), loc="lower center", ncol=3, frameon=False, fontsize=9)
    figure.suptitle("Drone 1's first flip: the 11-09 hover model rolls it 3x too fast and flips clean; the fitted plant fails as the drone did", fontsize=11, x=0.01, ha="left")
    figure.tight_layout(rect=(0, 0.04, 1, 0.97))
    figure.savefig(path, dpi=110)
    print(f"saved {path}; closed loop: " + ", ".join(f"{a} clean {r.get('clean')} lost {r.get('lost', float('nan')):.2f} m" for a, (t, r) in zip(airframes, loops)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", default=FLIP)
    parser.add_argument("--start", type=float, default=START)
    parser.add_argument("--end", type=float, default=END)
    parser.add_argument("--airframe", default="airframe_2026-11-12_raised.json", help="the plant's airframe in sw/sim/models (NOTES.md's first replay: airframe_2026-11-09.json)")
    parser.add_argument("--geometry", help="a geometry file in sw/sim/models instead of the airframe's own")
    parser.add_argument("--plot", type=Path, help="save real against the replay on --airframe and on --fitted, open and closed loop, to this file")
    parser.add_argument("--fitted", default="airframe_2026-11-12_flip.json")
    args = parser.parse_args()
    log = Log(args.recording, args.start, args.end)
    airframe = A.model(args.airframe)
    plant = plant_for(log, airframe, A.load_geometry(args.geometry) if args.geometry else None)
    result = replay(log, [plant])
    rms = errors(log, result)[0]
    print(f"{args.recording} {args.start:g}-{args.end:g} s on {args.airframe}: rate error RMS to 10.88 s roll {rms[0]:.0f}, pitch {rms[1]:.0f}, yaw {rms[2]:.0f} deg/s")
    for k in range(0, len(log.time), 5):
        real, sim = log.gyro[k] / A.DEG, result["rates"][0, k] / A.DEG
        print(f"  {log.time[k]:.2f} {log.phase[k]:.0f}  real {real[0]:6.0f} {real[1]:6.0f} {real[2]:6.0f}  sim {sim[0]:6.0f} {sim[1]:6.0f} {sim[2]:6.0f}  height real {log.height[k] - log.height[0]:+.2f} sim {result['height'][0, k]:+.2f}")
    if args.plot:
        fitted = replay(log, [plant_for(log, A.model(args.fitted))])
        rms = errors(log, fitted)[0]
        print(f"on {args.fitted}: rate error RMS to 10.88 s roll {rms[0]:.0f}, pitch {rms[1]:.0f}, yaw {rms[2]:.0f} deg/s")
        plot(args.plot, log, result, fitted, (args.airframe, args.fitted))


if __name__ == "__main__":
    main()
