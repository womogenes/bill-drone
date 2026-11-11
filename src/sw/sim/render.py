"""
Render full-attitude simulation runs (sw/sim/rigid.py traces) as video with MuJoCo, on the drone's own model (models/bicopter.xml): the body's pose and each motor's tilt are played back kinematically, the camera following the drone.

uv run -m sw.sim.render --out falls.mp4 [--params recover_params.json] [--episodes 4] [--seed 3]
Needs ffmpeg; renders offscreen (MUJOCO_GL=egl, or osmesa without a GPU).
"""

import argparse
import json
import math
import os
import subprocess
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

MODEL = Path(__file__).with_name("models") / "bicopter.xml"
PHASES = ("righting", "arresting / descending", "landed, motors off")
# sw/sim/toss.py's phases (toss.phase_of).
TOSS_PHASES = ("thrown, motors off", "righting", "stopping", "holding height", "landing", "landed")
# The 10-inch props (drone 1 from 2026-11-07): blade tip radius (m).
PROP_RADIUS = 0.127
# The props' drawn spin (revolutions per second of video at a motor's share of the hover thrust): the real ~100 rev/s would alias at any frame rate, so they turn this fast, scaling with the square root of the thrust as the speed does.
SPIN_REV_S = 7.0
# The model's measured range (degrees from upright): beyond it the airframe terms are extrapolated (rigid.ATTITUDE_RANGE fades them out).
MEASURED_TILT = 35.0


def big_floor(xml):
    """The model with a floor large enough for falls from several meters, and a sky; without its landing legs, as the drone now flies."""
    xml = xml.replace('size="3 3 0.1" rgba="0.85 0.85 0.85 1"', 'size="30 30 0.1" material="grid" rgba="1 1 1 1"')
    for legs in ("front_legs", "rear_legs"):
        xml = xml.replace(f'<geom name="{legs}" class="cad"', f'<geom name="{legs}" class="cad" rgba="0 0 0 0"')
    xml = xml.replace('<geom type="sphere" pos=', '<geom type="sphere" rgba="0 0 0 0" pos=')
    return xml.replace("<asset>", '<asset>\n    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.91" rgb2="0.80 0.82 0.84" width="512" height="512"/>\n    <material name="grid" texture="grid" texrepeat="40 40"/>\n    <texture type="skybox" builtin="gradient" rgb1="0.75 0.85 0.95" rgb2="0.95 0.97 1.0" width="256" height="256"/>', 1)


def spinning_props(xml):
    """Each prop on its own spinning hub (joints left_spin, right_spin), 10-inch blades, and a translucent disc for the blur (geoms left_disc, right_disc, faded in with the thrust)."""
    for side in ("left", "right"):
        old = f'<geom name="{side}_prop" type="ellipsoid" pos="0 0 0.0568" size="0.012 0.1143 0.0015" mass="0.012" rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>'
        assert old in xml, f"{side} prop not found in {MODEL.name}"
        new = (f'<body name="{side}_hub" pos="0 0 0.0568"><joint name="{side}_spin" axis="0 0 1"/>'
               f'<geom name="{side}_prop" type="ellipsoid" size="0.011 {PROP_RADIUS} 0.0015" mass="0.012" rgba="0.12 0.12 0.12 1" contype="0" conaffinity="0"/>'
               f'<geom name="{side}_disc" type="cylinder" size="{PROP_RADIUS} 0.0004" mass="0.0001" rgba="0.3 0.3 0.32 0" contype="0" conaffinity="0"/></body>')
        xml = xml.replace(old, new)
    return xml


def render(trace, env, path, width=960, height=720, fps=50, label="", slow=1.0, phases=PHASES, props=False):
    """One env of a recover.run(..., record=True) trace to an mp4 at path; slow > 1 plays it in slow motion. With props the props spin with their thrust (spinning_props)."""
    xml = big_floor(MODEL.read_text())
    if props:
        xml = spinning_props(xml)
    cwd = os.getcwd()
    os.chdir(MODEL.parent)
    try:
        model = mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(cwd)
    model.vis.global_.offwidth, model.vis.global_.offheight = width, height
    model.vis.headlight.ambient[:] = 0.45
    model.vis.headlight.diffuse[:] = 0.55
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height, width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance, camera.elevation, camera.azimuth = (1.5, -15.0, 135.0) if props else (2.2, -12.0, 135.0)
    left = model.joint("left_tilt").qposadr[0]
    right = model.joint("right_tilt").qposadr[0]
    step = max(1, round(1 / (fps * 0.01) / slow))
    if props:
        spins = [model.joint(f"{side}_spin").qposadr[0] for side in ("left", "right")]
        discs = [model.geom(f"{side}_disc").id for side in ("left", "right")]
        hover = None
        angle = [0.0, 0.0]
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", "-crf", "20", str(path)]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as ffmpeg:
        for t, pos, quat, motor, tilt, phase in trace[::step]:
            data.qpos[:3] = pos[env]
            data.qpos[3:7] = quat[env]
            data.qpos[left] = tilt[env, 0]
            data.qpos[right] = -tilt[env, 1]
            if props:
                # The props counter-rotate (left counterclockwise seen from above); speed as the square root of the thrust over a motor's hover share.
                hover = hover or 9.81 * 1.05 / 2
                for side, direction in ((0, 1.0), (1, -1.0)):
                    share = math.sqrt(max(motor[env, side], 0.0) / hover)
                    angle[side] += direction * 2 * math.pi * SPIN_REV_S * share / fps
                    data.qpos[spins[side]] = angle[side]
                    model.geom_rgba[discs[side], 3] = 0.28 * min(share, 1.2)
            mujoco.mj_forward(model, data)
            camera.lookat[:] = [pos[env, 0], pos[env, 1], max(pos[env, 2] - 0.2, 0.15)]
            renderer.update_scene(data, camera)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            up = 2 * (quat[env, 0] ** 2 + quat[env, 3] ** 2) - 1
            tilted = math.degrees(math.acos(max(-1, min(1, up))))
            text = f"{label}  t {t:4.2f} s   height {pos[env, 2]:4.2f} m   tilt from upright {tilted:5.1f}°{'  (beyond measured: extrapolated)' if props and tilted > MEASURED_TILT else ''}\nthrust L {motor[env, 0]:4.1f} N  R {motor[env, 1]:4.1f} N   motor tilt L {math.degrees(tilt[env, 0]):+5.1f}° R {math.degrees(tilt[env, 1]):+5.1f}°   {phases[int(phase[env])]}"
            draw.rectangle([0, 0, width, 52], fill=(255, 255, 255))
            draw.multiline_text((12, 6), text, fill=(20, 20, 20))
            ffmpeg.stdin.write(np.asarray(frame).tobytes())
    renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("falls.mp4"))
    parser.add_argument("--params", type=Path)
    parser.add_argument("--episodes", type=int, default=4)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--heights", type=float, nargs=2, default=(5.0, 9.0))
    parser.add_argument("--slow", type=float, default=1.0)
    parser.add_argument("--flip", action="store_true", help="render roll flips (sw/sim/flip.py) instead of falls")
    args = parser.parse_args()
    if args.flip:
        from sw.sim import flip as F
        recovery = json.loads(F.C.PARAMS.read_text())
        result, trace = F.run(json.loads(args.params.read_text()) if args.params else {}, recovery, n=args.episodes, seed=args.seed, record=True)
        parts = []
        for env in range(args.episodes):
            part = args.out.with_name(f"{args.out.stem}_{env}.mp4")
            render(trace, env, part, label=f"flip {env + 1}: {'CLEAN' if result['clean'][env] else 'FAILED'}", slow=args.slow, phases=("spinning", "flying"))
            parts.append(part)
            print(f"flip {env + 1}: {'clean' if result['clean'][env] else 'failed'}, height lost {result['lost'][env]:.2f} m", flush=True)
        concat(parts, args.out)
        return
    from sw.sim import recover as C
    C.scenarios.__defaults__ = (tuple(args.heights),)
    params = json.loads(args.params.read_text()) if args.params else {}
    result, trace = C.run(params, n=args.episodes, seed=args.seed, seconds=12.0, record=True)
    parts = []
    for env in range(args.episodes):
        verdict = "SAFE" if result["safe"][env] else "FAILED"
        part = args.out.with_name(f"{args.out.stem}_{env}.mp4")
        render(trace, env, part, label=f"fall {env + 1}: {verdict}", slow=args.slow)
        parts.append(part)
        print(f"fall {env + 1}: {verdict}, tilt at touch {result['first_tilt'][env]:.0f}°, speed {result['first_speed'][env]:.2f} m/s, contact {result['contact'][env]:.0f} N", flush=True)
    concat(parts, args.out)


def concat(parts, out):
    listing = out.with_suffix(".txt")
    listing.write_text("".join(f"file '{p.resolve()}'\n" for p in parts))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(listing), "-c", "copy", str(out)], check=True)
    for p in parts + [listing]:
        p.unlink()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
