"""
Side-by-side liftoff video of sw.sim.sweep runs, drawn with the MuJoCo CAD model: uv run -m sw.sim.slop.video
Writes sw/sim/slop/liftoff.mp4: real time, then the first 1.5 s at quarter speed, with a pitch strip chart.
"""

import math
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sw.sim.slop.liftoff import BASE, LIFT
from sw.sim.sweep import run

ROOT = Path(__file__).resolve().parent
COM_HEIGHT = 0.0172  # CoM above the tube midpoint, the MuJoCo body origin
WIDTH, HEIGHT, CHART = 480, 360, 220
FPS = 30
CASES = [
    ("Recommended: pitch 15/30/7, bias -7", {"pitch_p": 15, "pitch_i": 30, "pitch_damping": 7, "pitch_bias": -7}),
    ("No bias: pitch 10/10/5, bias 0", {"pitch_p": 10, "pitch_i": 10, "pitch_damping": 5, "pitch_bias": 0}),
]
PLANT = LIFT | {"disturbance": 10}
SECONDS = 6.0


def quaternion(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def pose(model, data, row):
    t, position, angles, estimate, servo, motor = row
    matrix = np.zeros(9)
    quat = np.array(quaternion(*angles))
    mujoco.mju_quat2Mat(matrix, quat)
    data.qpos[:3] = np.array(position) - matrix.reshape(3, 3) @ [0, 0, COM_HEIGHT]
    data.qpos[3:7] = quat
    data.qpos[model.joint("left_tilt").qposadr[0]] = servo[0]
    data.qpos[model.joint("right_tilt").qposadr[0]] = -servo[1]
    mujoco.mj_forward(model, data)


def chart(logs):
    figure, axis = plt.subplots(figsize=(2 * WIDTH / 100, CHART / 100), dpi=100)
    for (label, gains), log, color in zip(CASES, logs, ("#1f77b4", "#d62728")):
        axis.plot([r[0] for r in log], [math.degrees(r[2][1]) for r in log], color=color, label=label.split(":")[0])
    axis.axhline(45, color="0.5", linestyle=":", linewidth=1)
    axis.text(SECONDS, 45, "tip limit 45°", ha="right", va="bottom", fontsize=8, color="0.4")
    axis.set_xlim(0, SECONDS)
    axis.set_ylabel("pitch (°, + nose down)")
    axis.set_xlabel("time (s)")
    axis.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1, 0.9))
    axis.grid(alpha=0.3)
    figure.tight_layout()
    figure.canvas.draw()
    image = Image.fromarray(np.asarray(figure.canvas.buffer_rgba())[..., :3].copy())
    # Pixel x of t = 0 and t = SECONDS, for the cursor.
    left, right = (axis.transData.transform((x, 0))[0] for x in (0, SECONDS))
    plt.close(figure)
    return image, left, right


def main():
    # Sky and a 0.5 m checker floor large enough for the drift, so motion reads against the background.
    spec = mujoco.MjSpec.from_file(str(ROOT / "../models/bicopter.xml"))
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.55, 0.7, 0.9], rgb2=[0.95, 0.97, 1], width=256, height=256)
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.82, 0.82, 0.8], rgb2=[0.62, 0.64, 0.62], width=256, height=256)
    material = spec.add_material(name="grid", texrepeat=[80, 80])
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    floor = spec.geom("floor")
    floor.size = [40, 40, 0.1]
    floor.material = "grid"
    floor.rgba = [1, 1, 1, 1]
    model = spec.compile()
    model.vis.headlight.ambient[:] = 0.6
    model.vis.headlight.diffuse[:] = 0.8
    data = mujoco.MjData(model)
    logs = []
    for label, gains in CASES:
        log = []
        result = run(BASE | gains, PLANT, seconds=SECONDS, log=log)
        print(label, {k: round(v, 2) if isinstance(v, float) else v for k, v in result.items()})
        logs.append(log)
    background, left, right = chart(logs)
    font = ImageFont.load_default(size=15)
    small = ImageFont.load_default(size=13)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = model.body("drone").id
    # Side view along the tube, so pitch shows as rotation in the image plane.
    camera.distance = 1.1
    camera.azimuth = 90
    camera.elevation = -10
    # Real time, then the liftoff again at quarter speed.
    times = [(n / FPS, "") for n in range(round(SECONDS * FPS))] + [(n / FPS / 4, "1/4 speed") for n in range(round(1.5 * FPS * 4))]
    frames = ROOT / "frames"
    frames.mkdir(exist_ok=True)
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for index, (t, note) in enumerate(times):
            canvas = Image.new("RGB", (2 * WIDTH, HEIGHT + CHART), "white")
            for side, ((label, gains), log) in enumerate(zip(CASES, logs)):
                row = log[min(len(log) - 1, round(t / 0.01))]
                pose(model, data, row)
                renderer.update_scene(data, camera=camera)
                view = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(view)
                draw.rectangle((0, 0, WIDTH, 50), fill=(0, 0, 0))
                draw.text((10, 6), label, fill="white", font=font)
                draw.text((10, 28), f"t {t:4.2f} s   pitch {math.degrees(row[2][1]):+5.1f}°   height {row[1][2] - 0.1042:4.2f} m   {note}", fill="white", font=small)
                canvas.paste(view, (side * WIDTH, 0))
            ImageDraw.Draw(canvas).line([(side * WIDTH, 0), (side * WIDTH, HEIGHT)], fill="white", width=2)
            strip = background.copy()
            x = left + (right - left) * t / SECONDS
            ImageDraw.Draw(strip).line([(x, 5), (x, CHART - 30)], fill="black", width=2)
            canvas.paste(strip, (0, HEIGHT))
            canvas.save(frames / f"{index:05d}.png")
    video = ROOT / "liftoff.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(frames / "%05d.png"), "-pix_fmt", "yuv420p", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", str(video)], check=True)
    for frame in frames.iterdir():
        frame.unlink()
    frames.rmdir()
    print(video)


if __name__ == "__main__":
    main()
