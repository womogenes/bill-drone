"""
The airframe's attitude dynamics as identified from flight logs (sw/sim/identify_airframe.py), shared by the identification (vectorised over many prediction lanes) and the hover simulation (sw/sim/sweep.py, one lane through Airframe).

Motors: dead time, then a first-order lag on the thrust (the thrust table, times battery voltage over thrust_voltage to voltage_power; the right motor can differ in dead time, lag and gain).
Servos: dead time, then a second-order position loop (servo_wn, servo_zeta) whose horn speed is limited to servo_slew. The horn drives the motor assembly through a link (link_wn, link_zeta) with play; the rubber bands pull each assembly forward with a preload (left_band, right_band) less band_stiffness per degree of tilt. The assembly's inertia about the servo axis is inertia_ratio of the body's pitch inertia, so what the link does to the assembly it does the other way to the body.
Body: each axis's angular acceleration is a sum of physical terms (thrust torques at the controller model's geometry times an effectiveness, in pitch with a share pitch_wake of the tilted thrust's torque that builds up with lag wake_tau (fitted negative at about 0.7 s: the tilt is less effective held than swung), the props' gyroscopic and tilted drag torques, rate damping, attitude stiffness, moments from horizontal speed, cross-coupling) plus a constant push.
Angles are radians; servo commands, horn and tilt are degrees, positive forward (the right servo mirrored, as in the logs); velocity is forward and left in m/s.
"""

import math

import numpy as np

from sw.sim.flight import MODEL, THRUST_TABLE

DT = 0.01
DEG = math.pi / 180
SUB = 4
TABLE = np.array(THRUST_TABLE, float)

# Every parameter with its search bounds and starting value.
PARAMS = {
    "motor_delay": (0.0, 0.12, 0.07), "right_motor_delay": (0.0, 0.12, 0.07), "motor_tau": (0.002, 0.15, 0.005), "right_motor_tau": (0.002, 0.15, 0.005),
    "right_thrust_gain": (0.5, 1.5, 1.0), "voltage_power": (0.0, 3.0, 0.5),
    "servo_delay": (0.0, 0.1, 0.02), "right_servo_delay": (0.0, 0.1, 0.02), "servo_wn": (8.0, 150.0, 31.0), "right_servo_wn": (8.0, 150.0, 31.0), "servo_zeta": (0.1, 2.0, 0.85), "servo_slew": (100.0, 3000.0, 300.0),
    "link_wn": (10.0, 300.0, 80.0), "link_zeta": (0.05, 2.0, 0.5), "inertia_ratio": (0.0, 3.0, 0.3),
    "left_play": (0.0, 3.0, 0.6), "right_play": (0.0, 3.0, 0.6), "left_band": (-20000.0, 20000.0, 2000.0), "right_band": (-20000.0, 20000.0, 2000.0), "band_stiffness": (-2000.0, 2000.0, 0.0), "right_gain": (0.6, 1.4, 1.0),
    "roll_thrust": (-3, 3, 1.0), "roll_gyro": (-50, 50, 0.0), "roll_lift": (-3, 3, 0.0), "roll_damping": (-50, 50, 0.0), "roll_stiffness": (-100, 100, 0.0), "roll_velocity": (-100, 100, 0.0), "roll_from_yaw": (-30, 30, 0.0), "roll_prop_drag": (-3, 3, 0.0),
    "pitch_tilt": (-3, 3, 1.0), "pitch_wake": (-3, 3, 0.0), "wake_tau": (0.01, 1.0, 0.15), "pitch_ground": (-1, 3, 0.0), "ground_scale": (0.01, 2.0, 0.1), "pitch_lift": (-0.05, 0.05, 0.0), "pitch_damping": (-50, 50, 0.0), "pitch_stiffness": (-100, 100, 0.0), "pitch_velocity": (-100, 100, 0.0),
    "yaw_tilt": (-3, 3, 1.0), "yaw_drag": (-0.2, 0.2, 0.0), "yaw_damping": (-50, 50, 0.0), "yaw_from_roll": (-30, 30, 0.0),
    # Horizontal speed (only for prediction from logs; the simulation moves the vehicle itself): forward from pitch and the motors' mean tilt, left from roll, each with linear drag.
    "vx_pitch": (-40, 40, 7.7), "vx_tilt": (-40, 40, 0.0), "vx_drag": (0, 5, 0.1), "vy_roll": (-20, 20, -9.0), "vy_drag": (0, 5, 0.2),
}
KEYS = list(PARAMS)
# Zeroed on the ground, where the feet carry them (as is the constant push): the collective roll term the roll push cancels, and the moments from attitude and speed.
GROUND = ("roll_lift", "roll_stiffness", "pitch_stiffness", "roll_velocity", "pitch_velocity")
# Terms a parameter set may leave out (they then stay off).
OPTIONAL = ("roll_lift", "band_stiffness", "roll_prop_drag", "pitch_ground", "pitch_wake")


def thrust(throttle):
    return np.interp(throttle, TABLE[:, 0], TABLE[:, 1])


class State:
    """Every lane's state: body rates, roll and pitch, horizontal velocity, then each side's motor thrust, horn angle and speed, and assembly tilt and speed, and the lagged tilted thrust (N, summed over the sides) for pitch_wake."""

    def __init__(self, n):
        for name, width in (("rate", 3), ("att", 2), ("vel", 2), ("motor", 2), ("horn", 2), ("hvel", 2), ("tilt", 2), ("tvel", 2), ("wake", 1)):
            setattr(self, name, np.zeros((n, width)))


def delayed(buffer, k, delay):
    """Each lane's buffer [lane, sample, side] at sample k minus the delay (one, or one per side), interpolating between samples and never past k."""
    delay = np.broadcast_to(np.asarray(delay, float), (2,))
    x = k[:, None] - delay[None, :] / DT
    low = np.floor(x).astype(int)
    fraction = x - low
    size = buffer.shape[1]
    first = np.clip(low, 0, size - 1)
    second = np.clip(np.minimum(low + 1, k[:, None]), 0, size - 1)
    lane = np.arange(len(k))[:, None]
    side = np.array([[0, 1]])
    return buffer[lane, first, side] * (1 - fraction) + buffer[lane, second, side] * fraction


def step(p, state, throttles, commands, k, voltage, push, airborne=None, near=None, model=MODEL):
    """
    Advance every lane one control tick from sample k: throttles and commands are [lane, sample, side] buffers holding each lane's inputs up to k, voltage [lane] the battery over thrust_voltage, push [lane, axis] the constant angular accelerations (rad/s²). Lanes with airborne False (default all airborne) drop the GROUND terms and the push. near [lane] is how far each lane is from the floor (default far): the tilt's pitch effectiveness is pitch_tilt × (1 + pitch_ground × exp(−near / ground_scale)).
    Returns the state and the tick's mean angular accelerations [lane, axis].
    """
    m = model
    n = len(k)
    flying = np.ones(n) if airborne is None else np.asarray(airborne, float)
    term = {key: p[key] * flying for key in GROUND}
    push = push * flying[:, None]
    gain = np.array([1.0, p["right_gain"]])
    target = thrust(delayed(throttles, k, (p["motor_delay"], p["right_motor_delay"]))) * voltage[:, None] ** p["voltage_power"] * np.array([1.0, p["right_thrust_gain"]])
    state.motor = target + (state.motor - target) * np.exp(-DT / np.array([p["motor_tau"], p["right_motor_tau"]]))
    goal = delayed(commands, k, (p["servo_delay"], p["right_servo_delay"]))
    left, right = state.motor[:, 0], state.motor[:, 1]
    tilt = state.tilt * gain * DEG
    cl, cr, sl, sr = np.cos(tilt[:, 0]), np.cos(tilt[:, 1]), np.sin(tilt[:, 0]), np.sin(tilt[:, 1])
    rate, att, vel = state.rate, state.att, state.vel
    roll = (
        p["roll_thrust"] * m["arm"] / m["roll_inertia"] * (left * cl - right * cr)
        + term["roll_lift"] * m["arm"] / m["roll_inertia"] * (left * cl + right * cr)
        + p["roll_prop_drag"] * m["reaction_torque_ratio"] / m["roll_inertia"] * (left * sl - right * sr)
        + p["roll_damping"] * rate[:, 0] + term["roll_stiffness"] * att[:, 0] + term["roll_velocity"] * vel[:, 1] + p["roll_from_yaw"] * rate[:, 2]
        + p["roll_gyro"] * (state.tvel[:, 0] - state.tvel[:, 1]) * DEG
        + push[:, 0]
    )
    tilted = left * sl + right * sr
    state.wake = tilted[:, None] + (state.wake - tilted[:, None]) * math.exp(-DT / p["wake_tau"])
    pitch_other = (
        p["pitch_tilt"] * (1 + (0.0 if near is None else p["pitch_ground"] * np.exp(-np.asarray(near, float) / p["ground_scale"]))) * m["pivot_height"] / m["pitch_inertia"] * tilted
        + p["pitch_wake"] * m["pivot_height"] / m["pitch_inertia"] * state.wake[:, 0]
        + p["pitch_lift"] * (left * cl + right * cr) / m["pitch_inertia"]
        + p["pitch_damping"] * rate[:, 1] + term["pitch_stiffness"] * att[:, 1] + term["pitch_velocity"] * vel[:, 0]
        + push[:, 1]
    )
    yaw = (
        p["yaw_tilt"] * m["arm"] / m["yaw_inertia"] * (right * sr - left * sl)
        + p["yaw_drag"] * (right * cr - left * cl) / m["yaw_inertia"]
        + p["yaw_damping"] * rate[:, 2] + p["yaw_from_roll"] * rate[:, 0]
        + push[:, 2]
    )
    h = DT / SUB
    wn = np.array([p["servo_wn"], p["right_servo_wn"]])
    w2, z2 = wn ** 2, 2 * p["servo_zeta"] * wn
    c2, cz = p["link_wn"] ** 2, 2 * p["link_zeta"] * p["link_wn"]
    half = np.array([p["left_play"], p["right_play"]]) / 2
    band = np.array([p["left_band"], p["right_band"]])
    q = rate[:, 1].copy()
    pitch = att[:, 1].copy()
    pitch_sum = np.zeros(n)
    for _ in range(SUB):
        state.hvel = np.clip(state.hvel + (w2 * (goal - state.horn) - z2 * state.hvel) * h, -p["servo_slew"], p["servo_slew"])
        state.horn = state.horn + state.hvel * h
        gap = state.horn - state.tilt
        gap = np.sign(gap) * np.maximum(np.abs(gap) - half, 0)
        link = c2 * gap + cz * (state.hvel - state.tvel) * (gap != 0) + band - p["band_stiffness"] * state.tilt
        body = pitch_other - p["inertia_ratio"] * (link * DEG).sum(1)
        state.tvel = state.tvel + (link - body[:, None] / DEG) * h
        state.tilt = state.tilt + state.tvel * h
        q = q + body * h
        pitch = pitch + q * h
        pitch_sum += body
    accel = np.column_stack([roll, pitch_sum / SUB, yaw])
    state.rate = np.column_stack([rate[:, 0] + roll * DT, q, rate[:, 2] + yaw * DT])
    state.att = np.column_stack([att[:, 0] + state.rate[:, 0] * DT, pitch])
    mean_tilt = (state.tilt * gain).mean(1) * DEG
    state.vel = vel + np.column_stack([p["vx_pitch"] * att[:, 1] + p["vx_tilt"] * mean_tilt - p["vx_drag"] * vel[:, 0], p["vy_roll"] * att[:, 0] - p["vy_drag"] * vel[:, 1]]) * DT
    return state, accel


def complete(params):
    """A full parameter set from a saved one: missing optional terms off, anything else missing at its starting value."""
    return {key: params.get(key, 0.0 if key in OPTIONAL else PARAMS[key][2]) for key in KEYS}


class Airframe:
    """One lane for the simulation: keeps its own command history, and takes the body state from the simulation each tick."""

    def __init__(self, params, history=64):
        self.p = complete(params)
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
        self.state.motor[0] = thrust(throttle) * voltage ** p["voltage_power"] * np.array([1.0, p["right_thrust_gain"]])
        self.state.horn[0] = self.state.tilt[0] = tilt
        self.state.hvel[0] = self.state.tvel[0] = 0.0
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
        self.state, accel = step(self.p, self.state, self.throttles, self.commands, np.array([self.history - 1]), np.array([voltage]), self.push, np.array([airborne]), None if near is None else np.array([near]))
        tilt = self.state.tilt[0] * np.array([1.0, self.p["right_gain"]]) * DEG
        return tuple(accel[0]), self.motor, tuple(tilt)
