"""
The airframe's attitude dynamics as identified from flight logs (sw/sim/identify_airframe.py), shared by the identification (vectorised over many prediction lanes) and the hover simulation (sw/sim/sweep.py, one lane through Airframe).

The geometry is an explicit input (models/geometry_<day>.json; metres from the tube midpoint, body x forward, y left, z up; kg and kg·m²): mass, com, arm (the motor axes' |y|), pivot_z (the tilt axes), rotor_z (the prop plane at zero tilt), the body's roll, pitch and yaw inertia about the CoM, and each motor assembly's mass, CoM distance above its pivot and inertia about its tilt axis. Every body term is a torque from a fitted coefficient and the geometry, divided by the geometry's inertia, so a chassis change (battery, CoM, arm, pivot height, motor mass, legs) reaches the dynamics through the geometry alone: derive_geometry builds one from a parts list, cad_geometry from bicopter.xml's parts, and shifted applies CAD differences to a reference. The fitted coefficients are meant not to change with the geometry:
- motors: dead time, then a first-order lag on the thrust (the thrust table, times battery voltage over thrust_voltage to voltage_power; the right motor can differ in dead time, lag and gain). The "right" motor terms belong to the ESC that sat on the right until the 2026-10-29 06:45 UTC swap: lanes with p["swapped"] set (flights after it) give them to the left motor. A motor started from standstill makes no thrust for motor_start after its throttle turns positive (the ESC's start-up; the takeoffs show it, the airborne fit cannot). The level is pinned to the weight (each flight's trim: its airborne median table thrust times thrust level carries mass × g), and differential_slope is the share of a differential throttle step's table thrust the motors deliver (roll authority; the common part follows the table; from 2026-10-29b it is held at 1, which the ±1.5-3° PRBS flights measure, instead of fitted);
- servos: dead time, then a second-order position loop (servo_wn, servo_zeta) whose horn speed is limited to servo_slew; the horn drives the motor assembly through a link with play, as torques on the assembly's effective inertia (its CAD inertia about the tilt axis, its mass × CoM offset × pivot height, and servo_inertia, the servo's reflected rotor): link_k (N·m/rad), link_c (N·m·s/rad), a forward band preload (N·m) less band_k per radian of tilt. What the link does to the assembly it does the other way to the body;
- props: prop_torque (m: reaction torque per newton along each tilted shaft, opposite spins, so it acts in yaw and, tilted, in roll), prop_momentum (N·m·s per √N, for the gyroscopic roll torque of the props as they tilt);
- rotors: inflow_damping (s/m: roll damping as thrust lost per newton per m/s of axial inflow at the arm), edgewise_damping (s/m: the H-force per newton per m/s, for pitch at the rotors' height above the CoM and yaw at the arm);
- the slipstream: a share wash of the tilted thrust's horizontal force is cancelled by its slipstream on the airframe, acting at wash_z above the tube, so yaw tilt authority is 1 - wash of the geometric, the pitch tilt torque is (pivot height - wash × (wash_z - CoM height)) × T sin(tilt), and a tilt pushes the drone forward tilt_push as hard as the same body pitch (1 - wash in models saved without it; the camera measures about 0.66); a share pitch_wake of that torque builds up with lag wake_tau (the tilt is less effective held than swung);
- lumped terms, as torques so they scale with the geometry: roll_pendulum and pitch_pendulum (m: the attitude stiffness as an effective length times the thrust; free flight has no attitude spring, so this stands for something unmodelled), drag_z_x and drag_z_y (m above the tube: where the horizontal drag acts, so the speed moments follow the CoM height; from 2026-10-29b roll_pendulum is held at 0, and drag_z_y is set by the hovers' closed-loop statistics, where it damps the hover loop's sideways swing: the camera sees no sideways drag force, so its roll moment is better read as the downwind rotor losing thrust in the upwind one's wake), roll_from_yaw and yaw_from_roll (N·m per rad/s), and a constant push per flight;
- com_x is geometry the hovers calibrate (the hang test is its prior): the pushes absorb any steady torque, so only the thrust's variation sees it.
Angles are radians; servo commands, horn and tilt are degrees, positive forward (the right servo mirrored, as in the logs); velocity is forward and left in m/s.
"""

import json
import math
from pathlib import Path

import numpy as np

from sw.sim.flight import THRUST_TABLE

DT = 0.01
DEG = math.pi / 180
G = 9.81
SUB = 4
TABLE = np.array(THRUST_TABLE, float)
MODELS = Path(__file__).with_name("models")
GEOMETRY_FILE = "geometry_2026-10-28.json"

# Every parameter with its search bounds and starting value.
PARAMS = {
    "motor_delay": (0.0, 0.12, 0.07), "right_motor_delay": (0.0, 0.12, 0.07), "motor_tau": (0.002, 0.15, 0.005), "right_motor_tau": (0.002, 0.15, 0.005),
    "right_thrust_gain": (0.5, 1.5, 1.0), "voltage_power": (0.0, 3.0, 0.5), "differential_slope": (0.05, 3.0, 1.0), "motor_start": (0.0, 0.5, 0.0),
    "servo_delay": (0.0, 0.1, 0.02), "right_servo_delay": (0.0, 0.1, 0.02), "servo_wn": (8.0, 150.0, 31.0), "right_servo_wn": (8.0, 150.0, 31.0), "servo_zeta": (0.1, 2.0, 0.85), "servo_slew": (100.0, 3000.0, 300.0),
    "link_k": (0.01, 50.0, 2.0), "link_c": (0.0, 0.5, 0.03), "servo_inertia": (0.0, 0.005, 0.0004),
    "left_play": (0.0, 3.0, 0.6), "right_play": (0.0, 3.0, 0.6), "left_band": (-0.3, 0.3, 0.015), "right_band": (-0.3, 0.3, 0.015), "band_k": (-0.05, 0.05, 0.0), "right_gain": (0.6, 1.4, 1.0),
    "prop_torque": (-0.05, 0.05, -0.015), "prop_momentum": (-0.05, 0.05, 0.003),
    "inflow_damping": (-2.0, 5.0, 0.2), "edgewise_damping": (-2.0, 5.0, 0.05),
    "wash": (-1.0, 1.5, 0.0), "wash_z": (-0.2, 0.3, 0.02), "pitch_wake": (-1.5, 1.5, 0.0), "wake_tau": (0.01, 2.0, 0.15), "pitch_ground": (-1, 3, 0.0), "ground_scale": (0.01, 2.0, 0.1),
    "roll_pendulum": (-0.2, 0.2, 0.0), "pitch_pendulum": (-0.2, 0.2, 0.0), "drag_z_x": (-0.5, 0.5, 0.017), "drag_z_y": (-0.5, 0.5, 0.06),
    "roll_from_yaw": (-0.5, 0.5, 0.0), "yaw_from_roll": (-0.5, 0.5, 0.0), "com_x": (-0.01, 0.01, -0.0021),
    # Horizontal speed (only for prediction from logs; the simulation moves the vehicle itself): forward from pitch and the motors' mean tilt (1 - wash as strong), left from roll, each with linear drag.
    "vx_pitch": (-40, 40, 7.7), "vx_drag": (0, 5, 0.1), "vy_roll": (-20, 20, -9.0), "vy_drag": (0, 5, 0.2), "tilt_push": (0.0, 1.5, 0.66),
}
KEYS = list(PARAMS)
# Terms a parameter set may leave out (they then stay off).
OPTIONAL = ("pitch_ground", "pitch_wake", "motor_start")
# The running time (s) of a motor that has long been spinning.
RUNNING = 10.0
# The ESCs were swapped between the motors at this time (UTC, as recordings are named): the ESC on the right before it, whose terms are the "right" motor terms, drives the left motor after it.
ESC_SWAP = "2026-10-29T06-45"


def swapped(name):
    """Whether a recording (its path or name, starting with its UTC time) was flown with the ESCs swapped (see ESC_SWAP)."""
    return Path(name).name[:16] >= ESC_SWAP


def motor_terms(p, n):
    """Each lane's motor dead times, lags and static thrust gains [lane, side], the right ESC's terms on the left motor where p["swapped"] (per lane, default none)."""
    delay = np.array([p["motor_delay"], p["right_motor_delay"]], float)
    tau = np.array([p["motor_tau"], p["right_motor_tau"]], float)
    gain = np.array([1.0, p["right_thrust_gain"]])
    flip = np.broadcast_to(np.asarray(p.get("swapped", 0.0), float), (n,))[:, None] > 0
    return tuple(np.where(flip, v[::-1], v) for v in (np.tile(delay, (n, 1)), np.tile(tau, (n, 1)), np.tile(gain, (n, 1))))


def load_geometry(name=GEOMETRY_FILE):
    geometry = json.loads((MODELS / name).read_text())
    geometry["com"] = tuple(geometry["com"])
    return geometry


GEOMETRY = load_geometry()


def derive_geometry(parts, assembly, arm, pivot_z, rotor_z):
    """
    A geometry from parts (each {"mass", "pos": (x, y, z), optional "inertia": (Ixx, Iyy, Izz) about its own centre}) rigidly fixed to the body, both motor assemblies at zero tilt among them, and one assembly's {"mass", "com" (above its pivot), "inertia" (about its tilt axis)}.
    """
    mass = sum(p["mass"] for p in parts)
    com = np.sum([p["mass"] * np.asarray(p["pos"], float) for p in parts], 0) / mass
    inertia = np.zeros(3)
    for p in parts:
        d = np.asarray(p["pos"], float) - com
        inertia += np.asarray(p.get("inertia", (0.0, 0.0, 0.0)), float) + p["mass"] * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    return {"mass": float(mass), "com": tuple(map(float, com)), "arm": arm, "pivot_z": pivot_z, "rotor_z": rotor_z, "roll_inertia": float(inertia[0]), "pitch_inertia": float(inertia[1]), "yaw_inertia": float(inertia[2]),
            "assembly_mass": assembly["mass"], "assembly_com": assembly["com"], "assembly_inertia": assembly["inertia"]}


def cad_geometry(without=(), masses=None):
    """
    The geometry of bicopter.xml's parts (their assigned masses, CAD mesh inertia) without its fitted inertial, leaving out the named geoms (e.g. ("front_legs", "rear_legs")) and with masses {geom: kg} overriding; needs mujoco. Compare two of these, and apply the difference to a measured geometry with shifted.
    """
    import re

    import mujoco
    xml = MODELS / "bicopter.xml"
    text = re.sub(r"<inertial pos=[^>]*/>", "", xml.read_text())
    text = text.replace('file="meshes/', f'file="{xml.parent.resolve()}/meshes/').replace('file="../../../', f'file="{xml.parent.resolve().parents[2]}/')
    for name, mass in {**{name: 0.0 for name in without}, **(masses or {})}.items():
        text = re.sub(rf'(<geom name="{name}"[^>]*?) mass="[0-9.]+"', rf'\1 mass="{mass}"', text)
    model = mujoco.MjModel.from_xml_string(text)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drone = model.body("drone").id
    bodies = [i for i in range(1, model.nbody) if model.body_rootid[i] == drone]
    origin = data.xpos[drone]
    parts = []
    for i in bodies:
        rotation = data.ximat[i].reshape(3, 3)
        inertia = rotation @ np.diag(model.body_inertia[i]) @ rotation.T
        parts.append({"mass": float(model.body_mass[i]), "pos": data.xipos[i] - origin, "inertia": np.diag(inertia)})
    motor = model.body("left_motor").id
    pivot = data.xpos[motor]
    offset = data.xipos[motor] - pivot
    rotation = data.ximat[motor].reshape(3, 3)
    own = (rotation @ np.diag(model.body_inertia[motor]) @ rotation.T)[1, 1]
    assembly = {"mass": float(model.body_mass[motor]), "com": float(offset[2]), "inertia": float(own + model.body_mass[motor] * (offset[0] ** 2 + offset[2] ** 2))}
    return derive_geometry(parts, assembly, float(pivot[1] - origin[1]), float(pivot[2] - origin[2]), float(data.site("left_thrust").xpos[2] - origin[2]))


def shifted(reference, before, after):
    """reference (a measured geometry) changed by what changes from geometry before to after (e.g. two cad_geometry results): mass, CoM and inertias add the difference, the rest take after's values."""
    out = dict(reference)
    for key in after:
        if key == "com":
            out["com"] = tuple(r + a - b for r, a, b in zip(reference["com"], after["com"], before["com"]))
        elif key in ("mass", "roll_inertia", "pitch_inertia", "yaw_inertia", "com_x_shift"):
            out[key] = reference.get(key, 0.0) + after[key] - before.get(key, 0.0)
        elif after[key] != before.get(key):
            out[key] = after[key]
    # The fore/aft CoM is calibrated by the fit (com_x): a change carries over as a shift on the fitted value.
    out["com_x_shift"] = reference.get("com_x_shift", 0.0) + after["com"][0] - before["com"][0]
    return out


def resolve(p, geometry=GEOMETRY):
    """The constants a step needs from the coefficients and the geometry."""
    cz = geometry["com"][2]
    dz = geometry["pivot_z"] - cz
    return {
        "Ixx": geometry["roll_inertia"], "Iyy": geometry["pitch_inertia"], "Izz": geometry["yaw_inertia"], "arm": geometry["arm"], "cx": p["com_x"] + geometry.get("com_x_shift", 0.0), "cy": geometry["com"][1],
        "tilt_arm": dz - p["wash"] * (p["wash_z"] - cz), "hr": geometry["rotor_z"] - cz, "hx": p["drag_z_x"] - cz, "hy": p["drag_z_y"] - cz, "mass": geometry["mass"],
        "J": geometry["assembly_inertia"] + geometry["assembly_mass"] * geometry["assembly_com"] * dz + p["servo_inertia"],
    }


def thrust(throttle, table=TABLE):
    return np.interp(throttle, table[:, 0], table[:, 1])


def table_thrust(p, throttle, voltage, table=TABLE, gain=None):
    """The table's thrust for throttles [..., side] at battery ratios [...], with the right motor's static gain (or gain [..., side], from motor_terms) and the voltage power."""
    return thrust(throttle, table) * np.asarray(voltage, float)[..., None] ** p["voltage_power"] * (np.array([1.0, p["right_thrust_gain"]]) if gain is None else gain)


def trim_for(table_thrusts, geometry=GEOMETRY):
    """A flight's thrust trim from its airborne table thrusts [sample, side]: the level that makes their median carry the weight, and each side's median."""
    level = geometry["mass"] * G / np.median(table_thrusts.sum(-1))
    return level, np.median(table_thrusts, 0)


def actual_thrust(p, table_thrusts, trim):
    """
    The thrust [lane, side] the motors make for table thrusts [lane, side] with each lane's trim (level [lane], mean [lane, side]): the level times the mean plus the common departure and differential_slope of the differential one.
    """
    level, mean = trim
    departure = table_thrusts - mean
    common = departure.mean(1)
    differential = (departure[:, 0] - departure[:, 1]) / 2 * p["differential_slope"]
    return np.asarray(level)[:, None] * (mean + np.column_stack([common + differential, common - differential]))


def table_for(p, thrusts, trim):
    """The table thrusts [lane, side] that make these motor thrusts [lane, side] steadily (actual_thrust inverted)."""
    level, mean = trim
    departure = thrusts / np.asarray(level)[:, None] - mean
    common = departure.mean(1)
    differential = (departure[:, 0] - departure[:, 1]) / 2 / p["differential_slope"]
    return mean + np.column_stack([common + differential, common - differential])


class State:
    """Every lane's state: body rates, roll and pitch, horizontal velocity, then each side's motor thrust, horn angle and speed, and assembly tilt and speed, the lagged tilted thrust (N, summed over the sides) for pitch_wake, and each motor's running time since its throttle last turned positive (s, for motor_start; starts as long running)."""

    def __init__(self, n):
        for name, width in (("rate", 3), ("att", 2), ("vel", 2), ("motor", 2), ("horn", 2), ("hvel", 2), ("tilt", 2), ("tvel", 2), ("wake", 1)):
            setattr(self, name, np.zeros((n, width)))
        self.spin = np.full((n, 2), RUNNING)


def delayed(buffer, k, delay):
    """Each lane's buffer [lane, sample, side] at sample k minus the delay (one, one per side, or [lane, side]), interpolating between samples and never past k."""
    delay = np.broadcast_to(np.asarray(delay, float), (len(k), 2))
    x = k[:, None] - delay / DT
    low = np.floor(x).astype(int)
    fraction = x - low
    size = buffer.shape[1]
    first = np.clip(low, 0, size - 1)
    second = np.clip(np.minimum(low + 1, k[:, None]), 0, size - 1)
    lane = np.arange(len(k))[:, None]
    side = np.array([[0, 1]])
    return buffer[lane, first, side] * (1 - fraction) + buffer[lane, second, side] * fraction


def step(p, state, throttles, commands, k, voltage, push, trim, airborne=None, near=None, geometry=GEOMETRY, table=TABLE):
    """
    Advance every lane one control tick from sample k: throttles and commands are [lane, sample, side] buffers holding each lane's inputs up to k, voltage [lane] the battery over thrust_voltage, push [lane, axis] the constant angular accelerations (rad/s²), trim each lane's thrust trim (see actual_thrust). Lanes with airborne below 1 (default all airborne) scale the attitude and speed terms and the push by it. near [lane] is how far each lane is from the floor (default far): the tilt's pitch torque grows by 1 + pitch_ground × exp(-near / ground_scale). p may hold "tilt_gain" (per lane, default 1) scaling the tilt's pitch torque, and differential_slope per lane, to randomize the plant.
    Returns the state and the tick's mean angular accelerations [lane, axis].
    """
    c = resolve(p, geometry)
    n = len(k)
    flying = np.ones(n) if airborne is None else np.asarray(airborne, float)
    push = push * flying[:, None]
    gain = np.array([1.0, p["right_gain"]])
    motor_delay, motor_tau, thrust_gain = motor_terms(p, n)
    commanded = delayed(throttles, k, motor_delay)
    state.spin = np.where(commanded > 0, np.minimum(state.spin + DT, RUNNING), 0.0)
    target = actual_thrust(p, table_thrust(p, commanded, voltage, table, thrust_gain), trim) * (state.spin > p.get("motor_start", 0.0))
    state.motor = target + (state.motor - target) * np.exp(-DT / motor_tau)
    goal = delayed(commands, k, (p["servo_delay"], p["right_servo_delay"]))
    left, right = state.motor[:, 0], state.motor[:, 1]
    tilt = state.tilt * gain * DEG
    cl, cr, sl, sr = np.cos(tilt[:, 0]), np.cos(tilt[:, 1]), np.sin(tilt[:, 0]), np.sin(tilt[:, 1])
    rate, att, vel = state.rate, state.att, state.vel
    a, cy = c["arm"], c["cy"]
    total = left + right
    root = np.sqrt(np.maximum(state.motor, 0.0))
    # Roll: the thrusts at the arms (about the CoM's sideways offset), the props' tilted reaction torque, their gyroscopic torque as they tilt, inflow damping at the arm, the attitude term, the sideways drag at its height, cross-coupling.
    roll_torque = (
        (a - cy) * left * cl - (a + cy) * right * cr
        - p["prop_torque"] * (left * sl - right * sr)
        + p["prop_momentum"] * (root[:, 0] * state.tvel[:, 0] - root[:, 1] * state.tvel[:, 1]) * DEG
        - p["inflow_damping"] * a * a * total * rate[:, 0]
        - flying * p["roll_pendulum"] * total * att[:, 0]
        + flying * c["hy"] * p["vy_drag"] * c["mass"] * vel[:, 1]
        + p["roll_from_yaw"] * rate[:, 2]
    )
    roll = roll_torque / c["Ixx"] + push[:, 0]
    tilted = left * sl + right * sr
    state.wake = tilted[:, None] + (state.wake - tilted[:, None]) * math.exp(-DT / p["wake_tau"])
    ground = 1.0 if near is None else 1 + p["pitch_ground"] * np.exp(-np.asarray(near, float) / p["ground_scale"])
    arm_tilt = c["tilt_arm"] * p.get("tilt_gain", 1.0)
    # Pitch: the tilted thrusts at their effective height above the CoM and its lagged share, the CoM's fore/aft offset under the thrust, edgewise damping at the rotors' height, the attitude term, the forward drag at its height.
    pitch_torque = (
        arm_tilt * ground * tilted
        + p["pitch_wake"] * arm_tilt * state.wake[:, 0]
        + c["cx"] * (left * cl + right * cr)
        - p["edgewise_damping"] * c["hr"] ** 2 * total * rate[:, 1]
        - flying * p["pitch_pendulum"] * total * att[:, 1]
        - flying * c["hx"] * p["vx_drag"] * c["mass"] * vel[:, 0]
    )
    pitch_other = pitch_torque / c["Iyy"] + push[:, 1]
    # Yaw: the tilted thrusts at the arms (1 - wash of it), the props' reaction torque, edgewise damping at the arm, cross-coupling.
    yaw_torque = (
        (1 - p["wash"]) * ((a + cy) * right * sr - (a - cy) * left * sl)
        + p["prop_torque"] * (right * cr - left * cl)
        - p["edgewise_damping"] * a * a * total * rate[:, 2]
        + p["yaw_from_roll"] * rate[:, 0]
    )
    yaw = yaw_torque / c["Izz"] + push[:, 2]
    h = DT / SUB
    wn = np.array([p["servo_wn"], p["right_servo_wn"]])
    w2, z2 = wn ** 2, 2 * p["servo_zeta"] * wn
    J = c["J"]
    c2, cz = p["link_k"] / J, p["link_c"] / J
    half = np.array([p["left_play"], p["right_play"]]) / 2
    band = np.array([p["left_band"], p["right_band"]]) / J / DEG
    band_k = p["band_k"] / J
    ratio = J / c["Iyy"]
    q = rate[:, 1].copy()
    pitch = att[:, 1].copy()
    pitch_sum = np.zeros(n)
    for _ in range(SUB):
        state.hvel = np.clip(state.hvel + (w2 * (goal - state.horn) - z2 * state.hvel) * h, -p["servo_slew"], p["servo_slew"])
        state.horn = state.horn + state.hvel * h
        gap = state.horn - state.tilt
        gap = np.sign(gap) * np.maximum(np.abs(gap) - half, 0)
        link = c2 * gap + cz * (state.hvel - state.tvel) * (gap != 0) + band - band_k * state.tilt
        body = pitch_other - ratio * (link * DEG).sum(1)
        state.tvel = state.tvel + (link - body[:, None] / DEG) * h
        state.tilt = state.tilt + state.tvel * h
        q = q + body * h
        pitch = pitch + q * h
        pitch_sum += body
    accel = np.column_stack([roll, pitch_sum / SUB, yaw])
    state.rate = np.column_stack([rate[:, 0] + roll * DT, q, rate[:, 2] + yaw * DT])
    state.att = np.column_stack([att[:, 0] + state.rate[:, 0] * DT, pitch])
    mean_tilt = (state.tilt * gain).mean(1) * DEG
    state.vel = vel + np.column_stack([p["vx_pitch"] * (att[:, 1] + p["tilt_push"] * mean_tilt) - p["vx_drag"] * vel[:, 0], p["vy_roll"] * att[:, 0] - p["vy_drag"] * vel[:, 1]]) * DT
    return state, accel


def complete(params):
    """A full parameter set from a saved one: missing optional terms off, tilt_push in models saved before it at 1 - wash (as they flew), anything else missing at its starting value."""
    out = {key: params.get(key, 0.0 if key in OPTIONAL else PARAMS[key][2]) for key in KEYS}
    if "tilt_push" not in params:
        out["tilt_push"] = 1 - out["wash"]
    return out


def geometry_of(params):
    """The geometry a saved parameter set was fitted at (its "geometry" file, default GEOMETRY)."""
    return load_geometry(params["geometry"]) if params.get("geometry") else GEOMETRY


def hover_trim(params, geometry=GEOMETRY):
    """The thrust trim for the simulation: the saved fit's typical level and hover table thrusts, or the table as is at the weight."""
    if "thrust_level" in params:
        return params["thrust_level"], np.array(params["hover_thrust"], float)
    return 1.0, np.full(2, geometry["mass"] * G / 2)


class Airframe:
    """One lane for the simulation: keeps its own command history, and takes the body state from the simulation each tick."""

    def __init__(self, params, history=64, geometry=None):
        # tilt_gain (default 1) scales the tilt's pitch torque, for robustness studies; swapped (default off) flies the ESCs as after the 2026-10-29 swap.
        self.p = complete(params) | {"tilt_gain": params.get("tilt_gain", 1.0), "swapped": float(params.get("swapped", 0.0))}
        self.geometry = geometry or geometry_of(params)
        level, mean = hover_trim(params, self.geometry)
        self.trim = (np.array([level]), mean[None])
        self.push = np.array([[params.get("roll_push", 0.0), params.get("pitch_push", 0.0), params.get("yaw_push", 0.0)]])
        self.history = history
        self.state = State(1)
        self.throttles = np.zeros((1, history, 2))
        self.commands = np.zeros((1, history, 2))

    def reset(self, throttle=0.0, tilt=(0.0, 0.0), voltage=1.0):
        """Steady actuators at these commands (throttle, physical tilt command in degrees, battery voltage ratio)."""
        p = self.p
        self.throttles[:] = throttle
        self.commands[0] = tilt
        self.state.motor[0] = actual_thrust(p, table_thrust(p, np.full((1, 2), float(throttle)), np.array([voltage]), gain=motor_terms(p, 1)[2]), self.trim)[0]
        self.state.horn[0] = self.state.tilt[0] = tilt
        self.state.hvel[0] = self.state.tvel[0] = 0.0
        self.state.spin[0] = RUNNING if throttle > 0 else 0.0
        self.state.wake[0] = (self.state.motor[0] * np.sin(np.asarray(tilt) * np.array([1.0, p["right_gain"]]) * DEG)).sum()

    @property
    def motor(self):
        return tuple(self.state.motor[0])

    def step(self, throttles, commands, voltage, rate, angles, velocity, airborne=True, near=None):
        """
        One control tick: the motors' throttle commands, each servo's commanded physical tilt (degrees, positive forward), battery over thrust_voltage, the body's rates (rad/s), roll and pitch (rad), forward/left speed (m/s) and how far from the floor (see step). Returns the tick's mean angular accelerations (rad/s², roll/pitch/yaw), each motor's thrust (N) and each assembly's tilt (rad, positive forward).
        """
        self.throttles[0, :-1] = self.throttles[0, 1:]
        self.commands[0, :-1] = self.commands[0, 1:]
        self.throttles[0, -1] = throttles
        self.commands[0, -1] = commands
        self.state.rate[0] = rate
        self.state.att[0] = angles
        self.state.vel[0] = velocity
        self.state, accel = step(self.p, self.state, self.throttles, self.commands, np.array([self.history - 1]), np.array([voltage]), self.push, self.trim, np.array([float(airborne)]), None if near is None else np.array([near]), self.geometry)
        tilt = self.state.tilt[0] * np.array([1.0, self.p["right_gain"]]) * DEG
        return tuple(accel[0]), self.motor, tuple(tilt)
