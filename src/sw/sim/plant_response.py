"""
The pitch frequency response, servo command to pitch rate, measured from the logs and computed from a plant: uv run -m sw.sim.plant_response [--plant PLANT.json ...] [--flights PREFIX ...] [--plot FIGURE.png]

Measured: over the hovering spans (3 s after the motors start to 2 s before they stop, flips left out) of the chosen flights, the cross-spectrum of the common servo command (the two servos' mean physical tilt, degrees) with the gyro's pitch rate (deg/s) over the command's spectrum (Welch, 10.24 s segments), with the coherence. In the hold's own pitch shake (2.3-2.4 Hz in every configuration) the coherence is 0.97-1.00, so there the estimate is the plant's whatever the loop; down to 0.5 Hz the pilots' targets used as the instrument (target to rate over target to command) agree with it where they carry power.

Computed: the plant (rigid.Sim's numpy step, play and the servo's deadband set to zero, which have no slope where the horn sits inside it) linearized about a hover at the flight's thrust trim by central differences of one control tick in its full state (position, velocity, attitude as a rotation vector, body rates, the motors' thrusts, each servo's horn and assembly angle and speed, the lagged tilted thrust), the input a common step of the servos' goal; the servo dead time as a phase, the gyro as the tick's mean rate and the command acting from the tick after it was logged (scorecard.predict), so the two are the same quantity.
"""

import argparse
import math

import numpy as np
from scipy.signal import csd, welch

from sw.sim import airframe as A
from sw.sim import rigid as R

DT = A.DT
FREQUENCIES = np.array([0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.34, 2.75, 3.1, 3.5])


def measured(flights, segment=1024):
    """The command-to-pitch-rate response of these scorecard.Flight objects: frequencies (Hz), the complex response ((deg/s)/deg) and the coherence, over their hovering spans."""
    total = {"uq": 0.0, "uu": 0.0, "qq": 0.0}
    frequency = None
    for f in flights:
        chosen = f.hovering & (f.time - f.first_on > 3) & (f.last_on - f.time > 2)
        index = np.flatnonzero(chosen)
        for span in np.split(index, np.flatnonzero(np.diff(index) > 1) + 1):
            if len(span) < segment:
                continue
            u = np.degrees(f.tilt[span].mean(1))
            q = np.degrees(f.gyro[span, 1])
            for name, (a, b) in {"uq": (u, q), "uu": (u, u), "qq": (q, q)}.items():
                frequency, spectrum = csd(a - a.mean(), b - b.mean(), fs=1 / DT, nperseg=segment)
                total[name] = total[name] + spectrum * len(span)
    if frequency is None:
        return None
    response = total["uq"] / total["uu"]
    coherence = (np.abs(total["uq"]) ** 2 / (total["uu"] * total["qq"])).real
    return frequency, response, coherence


def linearize(plant, epsilon=1e-4):
    """A and B of one control tick about a level hover at the plant's thrust trim (x_{k+1} = A x_k + B u_k, u the common servo goal in degrees), and the index of the pitch rate in the state."""
    params = dict(plant["airframe"]) | {"left_play": 0.0, "right_play": 0.0, "servo_deadband": 0.0}
    plant = dict(plant) | {"airframe": params, "disturbances": []}
    # Position, velocity, attitude, body rates (3 each), each side's motor thrust, horn angle and speed, tilt and tilt speed, the lagged tilted thrust.
    size = 23
    n = 2 * (size + 1)
    sim = R.Sim(n, plant, seed=0, backend="numpy")
    sim.gyro_bias[:] = 0.0
    level, mean = sim.trim
    motor = np.tile(level[0] * mean[0], (n, 1))
    # The tilt that holds the pitch still against the CoM's torque is the start (any steady trim; the loop holds it in flight).
    sim.reset(np.tile([0, 0, 10.0], (n, 1)), np.zeros((n, 3)), np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros((n, 3)), motor=motor, tilt=np.zeros((n, 2)))
    signs = np.repeat([1.0, -1.0], size + 1)
    which = np.tile(np.arange(size + 1), 2)
    st = sim.state
    rotation = np.zeros((n, 3))
    for lane in range(n):
        i, s = which[lane], signs[lane] * epsilon
        if i == size:
            continue
        if i < 3:
            sim.pos[lane, i] += s
        elif i < 6:
            sim.vel[lane, i - 3] += s
        elif i < 9:
            rotation[lane, i - 6] = s
        elif i < 12:
            sim.omega[lane, i - 9] += s
        elif i < 14:
            st.motor[lane, i - 12] += s
        elif i < 16:
            st.horn[lane, i - 14] += s
        elif i < 18:
            st.hvel[lane, i - 16] += s
        elif i < 20:
            st.tilt[lane, i - 18] += s
        elif i < 22:
            st.tvel[lane, i - 20] += s
        else:
            st.wake[lane, 0] += s
    sim.quat = R.integrate_quat(sim.quat, rotation, 1.0)
    command = np.zeros((n, 2))
    command[which == size] = signs[which == size, None] * epsilon
    # The goal the servos chase after their dead time: the whole history at the input, so the delayed sample is it.
    sim.commands[:] = np.degrees(command)[:, None, :]
    sim.throttles[:] = sim.throttles[:, -1:, :]
    throttle = sim.throttles[:, -1, :].copy()
    sim.step(throttle, command)
    after = np.column_stack([sim.pos, sim.vel, np.zeros((n, 3)), sim.omega, st.motor, st.horn, st.hvel, st.tilt, st.tvel, st.wake])
    # The attitude's change as a rotation vector from the unperturbed lane's (the input lanes' attitude starts level).
    nominal = sim.quat[which == size][0]
    turned = R.rotation_vector(R.quat_multiply(np.tile(R.quat_conjugate(nominal[None]), (n, 1)), sim.quat))
    after[:, 6:9] = turned
    plus, minus = after[:size + 1], after[size + 1:]
    J = (plus - minus) / (2 * epsilon)
    A_matrix = J[:size].T
    B = J[size][:, None]
    return A_matrix, B, 9 + 1, plant


def computed(plant, frequencies=FREQUENCIES):
    """The plant's command-to-pitch-rate response ((deg/s)/deg, complex) at these frequencies, as the logs measure it."""
    A_matrix, B, pitch, plant = linearize(plant)
    p = A.complete(plant["airframe"])
    delay = (p["servo_delay"] + p["right_servo_delay"]) / 2
    out = []
    eye = np.eye(len(A_matrix))
    for f in frequencies:
        z = np.exp(1j * 2 * math.pi * f * DT)
        x = np.linalg.solve(z * eye - A_matrix, B)[:, 0]
        # x is the state after the command's tick; the gyro reads the tick's mean rate, and the dead time delays the goal.
        rate = x[pitch] * (1 + 1 / z) / 2
        out.append(rate * np.exp(-1j * 2 * math.pi * f * delay))
    return np.array(out)


def main():
    from sw.sim import scorecard as S
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plant", nargs="*", default=list(S.PLANTS))
    parser.add_argument("--flights", nargs="+", default=["2026-11-15T06-32", "2026-11-15T06-46", "2026-11-15T06-47", "2026-11-15T06-48", "2026-11-15T06-49", "2026-11-15T06-50", "2026-11-15T06-58", "2026-11-15T07-12"])
    parser.add_argument("--plot", help="the gain and phase against frequency, the log's with its coherence and each plant's, here")
    args = parser.parse_args()
    listing = S.flight_list()
    entries = [e for e in S.entries(listing) if any(e["name"].startswith(p) for p in args.flights)]
    flights = [S.Flight(e, listing["configurations"][e["configuration"]]) for e in entries]
    frequency, response, coherence = measured(flights)
    pick = [int(np.argmin(np.abs(frequency - f))) for f in FREQUENCIES]
    print("frequency (Hz) " + " ".join(f"{frequency[k]:>12.2f}" for k in pick))
    print("log            " + " ".join(f"{abs(response[k]):6.1f}{math.degrees(np.angle(response[k])):6.0f}" for k in pick))
    print("coherence      " + " ".join(f"{coherence[k]:>12.2f}" for k in pick))
    curves = {}
    for name in args.plant:
        plant = S.plant_for(flights[0], name, {})
        h = computed(plant, frequency[pick])
        print(f"{S.label_of(name)[:14]:14} " + " ".join(f"{abs(v):6.1f}{math.degrees(np.angle(v)):6.0f}" for v in h))
        band = (frequency >= 0.3) & (frequency <= 4.0)
        curves[S.label_of(name)] = (frequency[band], computed(plant, frequency[band]))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        band = (frequency >= 0.3) & (frequency <= 4.0)
        fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        axes[0].plot(frequency[band], np.abs(response[band]), "k.-", lw=1.2, label="log (hovers' cross-spectra)")
        axes[1].plot(frequency[band], np.degrees(np.unwrap(np.angle(response[band]))), "k.-", lw=1.2)
        axes[2].plot(frequency[band], coherence[band], "k.-", lw=1.0)
        for (label, (f, h)), color in zip(curves.items(), ("#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e")):
            axes[0].plot(f, np.abs(h), color=color, lw=1.0, label=label)
            axes[1].plot(f, np.degrees(np.unwrap(np.angle(h))), color=color, lw=1.0)
        axes[0].set_ylabel("|pitch rate / servo command|\n((deg/s)/deg)", fontsize=8)
        axes[1].set_ylabel("phase (deg)", fontsize=8)
        axes[2].set_ylabel("coherence", fontsize=8)
        axes[2].set_xlabel("Hz", fontsize=8)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=7)
        axes[0].legend(fontsize=7)
        fig.suptitle("Servo command to pitch rate: " + ", ".join(e["name"] for e in entries)[:180], fontsize=7)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)


if __name__ == "__main__":
    main()
