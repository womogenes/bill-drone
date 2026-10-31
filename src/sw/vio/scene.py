"""
An outdoor scene and the drone's FPV camera in it, for visual-inertial odometry on simulated flights (sw/vio/simdata.py).

The scene is a 60 m field of grass and earth with buildings, trees, crates and fences at 3-30 m, every surface textured with fractal noise so a feature tracker has natural, non-repeating detail. The camera is the ESP32-CAM's FPV mode (fw/esp32cam/src/config.h): 480x320 over the OV3660's whole array, 46.9 x 32.2 degrees, looking along the body's forward axis from 40 mm ahead of and 33 mm above the body origin (models/bicopter.xml camera_board). A frame is read out row by row over READOUT_S (rolling shutter: the image is stitched from bands rendered at their rows' times), with sensor noise, then JPEG-compressed with its quality stepped toward a size as the camera does.
"""

import math
from pathlib import Path

import cv2
import numpy as np

WIDTH, HEIGHT = 480, 320
FOV_Y = 32.2
# Focal length in pixels and principal point, for the estimators' calibration.
FOCAL = HEIGHT / 2 / math.tan(math.radians(FOV_Y) / 2)
CENTRE = (WIDTH / 2, HEIGHT / 2)
# Camera in the body frame (x forward, y left, z up): position (m), and the camera-to-body rotation for an OpenCV camera (x right, y down, z forward).
CAMERA_POSITION = (0.040, 0.0, 0.033)
CAMERA_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# Rolling shutter: rows read top to bottom over this long (s), rendered in this many bands.
READOUT_S = 0.025
BANDS = 10
NOISE = 2.5


def fractal(size, octaves, seed, base=4):
    """Fractal noise in [0, 1], size x size, from bilinearly upsampled random grids."""
    rng = np.random.default_rng(seed)
    out = np.zeros((size, size), np.float32)
    weight = 1.0
    for octave in range(octaves):
        cells = base * 2 ** octave
        grid = rng.random((cells + 1, cells + 1)).astype(np.float32)
        out += weight * cv2.resize(grid, (size, size), interpolation=cv2.INTER_CUBIC)
        weight *= 0.55
    out -= out.min()
    return out / out.max()


def texture(path, size, colours, octaves, seed, grain=0.0):
    """A noise texture blended between two RGB colours, with optional fine grain, written as PNG."""
    mix = fractal(size, octaves, seed)[..., None]
    low, high = np.array(colours[0], np.float32), np.array(colours[1], np.float32)
    image = low * (1 - mix) + high * mix
    if grain:
        image += np.random.default_rng(seed + 1).normal(0, grain, image.shape)
    cv2.imwrite(str(path), cv2.cvtColor(np.clip(image, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR))


def build(folder, seed=0):
    """The scene's MJCF (with an 'eye' camera on a mocap body), its textures written into folder."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    textures = {
        "ground": ((60, 85, 35), (125, 115, 70), 9, 6.0),
        "wall": ((110, 100, 95), (185, 170, 150), 7, 8.0),
        "bark": ((60, 45, 30), (110, 90, 60), 6, 6.0),
        "leaves": ((25, 60, 20), (90, 140, 50), 8, 10.0),
        "wood": ((120, 85, 50), (190, 150, 100), 6, 5.0),
    }
    for index, (name, (low, high, octaves, grain)) in enumerate(textures.items()):
        texture(folder / f"{name}.png", 2048 if name == "ground" else 512, (low, high), octaves, seed * 10 + index, grain)
    rng = np.random.default_rng(seed)
    bodies = []

    def place(near, far):
        radius, angle = rng.uniform(near, far), rng.uniform(0, 2 * math.pi)
        return radius * math.cos(angle), radius * math.sin(angle), rng.uniform(0, math.pi)

    for _ in range(10):
        x, y, yaw = place(8, 30)
        w, d, h = rng.uniform(3, 8), rng.uniform(3, 8), rng.uniform(3, 9)
        bodies.append(f'<geom type="box" pos="{x:.2f} {y:.2f} {h / 2:.2f}" euler="0 0 {yaw:.2f}" size="{w / 2:.2f} {d / 2:.2f} {h / 2:.2f}" material="wall"/>')
    for _ in range(25):
        x, y, _ = place(4, 30)
        h, r = rng.uniform(2, 5), rng.uniform(0.1, 0.25)
        bodies.append(f'<geom type="cylinder" pos="{x:.2f} {y:.2f} {h / 2:.2f}" size="{r:.2f} {h / 2:.2f}" material="bark"/>')
        bodies.append(f'<geom type="ellipsoid" pos="{x:.2f} {y:.2f} {h:.2f}" size="{rng.uniform(1, 2):.2f} {rng.uniform(1, 2):.2f} {rng.uniform(1, 2.5):.2f}" material="leaves"/>')
    for _ in range(30):
        x, y, yaw = place(3, 15)
        s = rng.uniform(0.2, 0.6)
        bodies.append(f'<geom type="box" pos="{x:.2f} {y:.2f} {s:.2f}" euler="0 0 {yaw:.2f}" size="{s:.2f} {s:.2f} {s:.2f}" material="wood"/>')
    for _ in range(6):
        x, y, yaw = place(5, 20)
        bodies.append(f'<geom type="box" pos="{x:.2f} {y:.2f} 0.5" euler="0 0 {yaw:.2f}" size="{rng.uniform(2, 5):.2f} 0.04 0.5" material="wood"/>')
    assets = "".join(f'<texture name="{name}" type="2d" file="{folder / name}.png"/><material name="{name}" texture="{name}" texrepeat="{30 if name == "ground" else 1} {30 if name == "ground" else 1}" texuniform="{"false" if name == "ground" else "true"}"/>' for name in textures)
    return f"""<mujoco>
  <visual><global offwidth="{WIDTH}" offheight="{HEIGHT}"/><quality shadowsize="4096"/><headlight ambient="0.35 0.35 0.35" diffuse="0.3 0.3 0.3"/></visual>
  <asset>{assets}<texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.7 0.9" rgb2="0.9 0.93 0.97" width="256" height="256"/></asset>
  <worldbody>
    <light directional="true" pos="0 0 30" dir="-0.4 0.3 -1" diffuse="0.8 0.78 0.72" castshadow="true"/>
    <geom name="ground" type="plane" size="60 60 0.1" material="ground"/>
    {"".join(bodies)}
    <body name="eye" mocap="true"><camera name="eye" fovy="{FOV_Y}"/></body>
  </worldbody>
</mujoco>"""


# MuJoCo cameras look along their -z with +y up; turned into an OpenCV camera (z forward, y down).
OPENCV_TO_MUJOCO = np.diag([1.0, -1.0, -1.0])


class Camera:
    """Renders the drone's FPV frames. pose(t) gives the body's world position and body-to-world rotation at time t (s)."""

    def __init__(self, folder, seed=0):
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_string(build(folder, seed))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, HEIGHT, WIDTH)
        self.rng = np.random.default_rng(seed + 100)

    def close(self):
        self.renderer.close()

    def render_at(self, position, rotation):
        """One rendered RGB image with the body at position, rotation (body to world)."""
        world = rotation @ CAMERA_TO_BODY @ OPENCV_TO_MUJOCO
        self.data.mocap_pos[0] = np.asarray(position) + rotation @ np.array(CAMERA_POSITION)
        quaternion = np.zeros(4)
        self.mujoco.mju_mat2Quat(quaternion, world.flatten())
        self.data.mocap_quat[0] = quaternion
        # Bodies, then the cameras on them.
        self.mujoco.mj_kinematics(self.model, self.data)
        self.mujoco.mj_camlight(self.model, self.data)
        self.renderer.update_scene(self.data, camera="eye")
        return self.renderer.render()

    def frame(self, start, pose):
        """The grey frame whose first row starts reading out at start (s): each band at its own time, then sensor noise."""
        rows = np.empty((HEIGHT, WIDTH), np.float32)
        for band in range(BANDS):
            top, bottom = band * HEIGHT // BANDS, (band + 1) * HEIGHT // BANDS
            image = self.render_at(*pose(start + (band + 0.5) / BANDS * READOUT_S))
            rows[top:bottom] = cv2.cvtColor(image[top:bottom], cv2.COLOR_RGB2GRAY)
        rows += self.rng.normal(0, NOISE, rows.shape)
        return np.clip(rows, 0, 255).astype(np.uint8)


class Jpeg:
    """JPEG compression with the quality stepped one per frame toward target bytes, as fw/esp32cam/src/fpv.h does (OpenCV's quality runs the other way to the sensor's: higher is finer)."""

    def __init__(self, target, quality=60):
        self.target, self.quality = target, quality

    def __call__(self, grey):
        data = cv2.imencode(".jpg", grey, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1].tobytes()
        if len(data) > self.target:
            self.quality = max(5, self.quality - 1)
        elif len(data) < self.target * 3 // 4:
            self.quality = min(95, self.quality + 1)
        return data
