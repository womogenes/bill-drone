"""
Cross-check of the full-attitude sim's ground contact (sw/sim/rigid.py: a spring-damper at each HULL point, friction viscous below SLIP) against MuJoCo's own contact physics: uv run --with mujoco python -m sw.sim.validate_contact [--seconds 2]

The same body touches down in both, motors off: rigid.py's mass and inertia (or, with --cad, the CAD inertia of models/bicopter.xml), contact at rigid.py's HULL points (MuJoCo: 1 mm spheres there, with friction FRICTION, elliptic cones and its default soft contact), dropped onto a plane from 1 mm above it over a grid of vertical speeds, horizontal speeds, tilts and directions: tilted in roll or pitch, the horizontal velocity toward the low side ("downhill") or away from it. For each case: whether it ends upright (body z within 37° of up after --seconds) and the peak contact force (the sum over points of each point's force magnitude). Printed: the agreement, the tip-over boundary from each (the lowest horizontal speed that tips it, by tilt and vertical speed), and the peak forces.
"""

import argparse
import itertools
import math

import mujoco
import numpy as np

from sw.sim import rigid as R

VERTICAL = (0.3, 0.7, 1.0, 1.5, 2.0)
HORIZONTAL = (0.0, 0.5, 1.0, 1.5, 2.0)
TILT = (0.0, 10.0, 20.0, 30.0, 40.0)
AXES = ("roll", "pitch")
DIRECTIONS = ("downhill", "uphill")
# The CAD inertia about the body axes (models/bicopter.xml's fullinertia diagonal), kg m².
CAD_INERTIA = (0.0057936, 0.0009466, 0.0054250)
UPRIGHT = 0.8
HULL = R.HULL


def cases():
    return list(itertools.product(VERTICAL, HORIZONTAL, TILT, AXES, DIRECTIONS))


def initial(case):
    """Position, velocity and quaternion for a case: lowest hull point 1 mm above the floor."""
    vz, vh, tilt, axis, direction = case
    a = math.radians(tilt) / 2
    quat = np.array([math.cos(a), math.sin(a), 0, 0] if axis == "roll" else [math.cos(a), 0, math.sin(a), 0])
    rot = R.rotation(quat[None])[0]
    lowest = (HULL @ rot.T)[:, 2].min()
    # Roll positive lifts the left side (the low side is -y, right); pitch positive is nose-down (the low side is +x, forward). Downhill is toward the low side.
    low = np.array([0.0, -1.0, 0.0]) if axis == "roll" else np.array([1.0, 0.0, 0.0])
    side = 1.0 if direction == "downhill" else -1.0
    return np.array([0.0, 0.0, 0.001 - lowest]), side * vh * low + np.array([0, 0, -vz]), quat


def rigid_run(seconds, inertia=None):
    all_cases = cases()
    n = len(all_cases)
    sim = R.Sim(n, plant=R.load_plant(disturbances=None), backend="numpy")
    if inertia is not None:
        sim.inertia[:] = inertia
    starts = [initial(c) for c in all_cases]
    sim.reset(np.array([s[0] for s in starts]), np.array([s[1] for s in starts]), np.array([s[2] for s in starts]), np.zeros((n, 3)))
    zero = np.zeros((n, 2))
    for _ in range(round(seconds / R.DT)):
        sim.step(zero, zero)
    return sim.up()[:, 2] > UPRIGHT, sim.contact.copy()


def mujoco_model(inertia):
    spheres = "\n".join(f'      <geom type="sphere" size="0.001" pos="{x} {y} {z}"/>' for x, y, z in HULL)
    xml = f"""
<mujoco>
  <option timestep="0.0005" cone="elliptic" gravity="0 0 -{R.G}"/>
  <default><geom friction="{R.FRICTION} 0.005 0.0001" condim="3"/></default>
  <worldbody>
    <geom type="plane" size="20 20 0.1"/>
    <body name="drone">
      <freejoint/>
      <inertial pos="0 0 0" mass="{R.A.GEOMETRY['mass']}" diaginertia="{inertia[0]} {inertia[1]} {inertia[2]}"/>
{spheres}
    </body>
  </worldbody>
</mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def mujoco_run(seconds, inertia):
    model = mujoco_model(inertia)
    data = mujoco.MjData(model)
    force = np.zeros(6)
    upright, peaks = [], []
    for case in cases():
        mujoco.mj_resetData(model, data)
        pos, vel, quat = initial(case)
        data.qpos[:3], data.qpos[3:7], data.qvel[:3] = pos, quat, vel
        peak = 0.0
        for _ in range(round(seconds / model.opt.timestep)):
            mujoco.mj_step(model, data)
            total = 0.0
            for i in range(data.ncon):
                mujoco.mj_contactForce(model, data, i, force)
                total += np.linalg.norm(force[:3])
            peak = max(peak, total)
        q = data.qpos[3:7]
        upright.append(1 - 2 * (q[1] ** 2 + q[2] ** 2) > UPRIGHT)
        peaks.append(peak)
    return np.array(upright), np.array(peaks)


def boundary(upright):
    """The lowest horizontal speed that tips it over [axis, direction][tilt, vertical], inf if none on the grid."""
    out = {}
    index = {c: i for i, c in enumerate(cases())}
    for axis, direction in itertools.product(AXES, DIRECTIONS):
        table = np.full((len(TILT), len(VERTICAL)), math.inf)
        for (t, tilt), (v, vz) in itertools.product(enumerate(TILT), enumerate(VERTICAL)):
            tipped = [vh for vh in HORIZONTAL if not upright[index[(vz, vh, tilt, axis, direction)]]]
            table[t, v] = min(tipped, default=math.inf)
        out[axis, direction] = table
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()
    model_inertia = tuple(R.A.GEOMETRY[key] for key in ("roll_inertia", "pitch_inertia", "yaw_inertia"))
    rigid_up, rigid_peak = rigid_run(args.seconds)
    mj_up, mj_peak = mujoco_run(args.seconds, model_inertia)
    cad_up = mujoco_run(args.seconds, CAD_INERTIA)[0]
    rigid_cad_up = rigid_run(args.seconds, np.array(CAD_INERTIA))[0]
    n = len(rigid_up)
    print(f"{n} touchdowns (vertical {VERTICAL} m/s, horizontal {HORIZONTAL} m/s, tilt {TILT}°, roll/pitch, downhill/uphill), motors off")
    print(f"ends upright: rigid {rigid_up.mean() * 100:.0f}%, MuJoCo {mj_up.mean() * 100:.0f}%; agree on {(rigid_up == mj_up).mean() * 100:.0f}% (rigid upright where MuJoCo tips: {(rigid_up & ~mj_up).sum()}, the reverse: {(~rigid_up & mj_up).sum()})")
    print(f"with the CAD inertia: rigid {rigid_cad_up.mean() * 100:.0f}%, MuJoCo {cad_up.mean() * 100:.0f}%; agree on {(rigid_cad_up == cad_up).mean() * 100:.0f}%")
    ratio = rigid_peak / np.maximum(mj_peak, 1e-6)
    print(f"peak contact force: rigid median {np.median(rigid_peak):.0f} N, MuJoCo {np.median(mj_peak):.0f} N; rigid/MuJoCo median {np.median(ratio):.2f} (10th {np.percentile(ratio, 10):.2f}, 90th {np.percentile(ratio, 90):.2f})")
    index = {c: i for i, c in enumerate(cases())}
    print("peak force (N) level, no slide, rigid | MuJoCo: " + ", ".join(f"{vz} m/s {rigid_peak[index[(vz, 0.0, 0.0, 'roll', 'downhill')]]:.0f} | {mj_peak[index[(vz, 0.0, 0.0, 'roll', 'downhill')]]:.0f}" for vz in VERTICAL))
    rb, mb = boundary(rigid_up), boundary(mj_up)
    print("lowest horizontal speed that tips it (m/s; - = none up to 2), rows tilt, columns vertical speed " + " ".join(f"{v}" for v in VERTICAL) + ": rigid | MuJoCo")
    fmt = lambda x: "  -" if math.isinf(x) else f"{x:3.1f}"
    for key in rb:
        print(f"  {key[0]} {key[1]}")
        for t, tilt in enumerate(TILT):
            print(f"    {tilt:4.0f}°  " + " ".join(fmt(x) for x in rb[key][t]) + "  |  " + " ".join(fmt(x) for x in mb[key][t]))


if __name__ == "__main__":
    main()
