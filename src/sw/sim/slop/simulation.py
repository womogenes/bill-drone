"""
Run a controller against the MuJoCo drone with actuator delay and sensor noise.
See ../models/NOTES.md for dimensions, calibration sources, and approximations.
"""

import os
from collections import deque
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw

# These same files are copied to the ESP by the normal firmware upload.
from fw.drivers.calibration import THRUSTS, thrust, throttle, GyroCalibration
from fw.drivers.flight import Attitude, Controller, DEFAULT_TUNING

ROOT = Path(__file__).resolve().parent
model = mujoco.MjModel.from_xml_path(str(ROOT / "../models/bicopter.xml"))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
mass = model.body_mass.sum()

model.actuator_ctrlrange[:2] = [0, max(THRUSTS)]


def run(controller, seconds=60, seed=7):
    """
    Call controller(t, accel_g, gyro_rad_s) at 100 Hz and save a 10 FPS GIF.
    Return the GIF path and a log of simulated time, XYZ position, XYZ angular
    velocity, roll, and pitch, in seconds, metres, rad/s, and radians.
    """
    thrust_delay = 0.020  # Wait 20 ms before starting the modeled motor response.
    thrust_time_constant = 0.050
    servo_slack = np.deg2rad(0.5)  # ±0.5° play; reversal takes up 1°.
    gyro_noise = np.deg2rad(0.005 * np.sqrt(42))
    accel_noise = 0.0004 * np.sqrt(44)
    gyro_bias = np.deg2rad([0.02, 1.10, -1.95])

    rng = np.random.default_rng(seed)
    model.actuator_dynprm[:2, 0] = thrust_time_constant

    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.087
    mujoco.mj_forward(model, data)

    calibration = GyroCalibration()
    calibration.start()

    filtered_accel = data.sensor("accel").data.copy() / 9.81
    filtered_gyro = data.sensor("gyro").data.copy()

    thrust_queue = deque(
        np.zeros(2)
        for step in range(round(thrust_delay / model.opt.timestep))
    )
    requested = np.zeros(4)
    servo_target = np.zeros(2)
    frames, readings = [], []

    # Camera lighting follows the drone as it moves away from the fixed world light.
    model.vis.headlight.ambient[:] = 0.6
    model.vis.headlight.diffuse[:] = 0.8
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = model.body("drone").id
    camera.distance = 1.0
    camera.azimuth = 135
    camera.elevation = -20

    with mujoco.Renderer(model, height=300, width=400) as renderer:
        for step in range(round(seconds / model.opt.timestep)):
            # Filter the simulated sensor readings before adding measurement noise.
            alpha = model.opt.timestep / (0.005 + model.opt.timestep)
            filtered_accel += alpha * (
                data.sensor("accel").data / 9.81 - filtered_accel
            )
            filtered_gyro += alpha * (
                data.sensor("gyro").data - filtered_gyro
            )

            if step % 5 == 0:  # MPU and controller at 100 Hz.
                accel = filtered_accel + rng.normal(0, accel_noise, 3)
                gyro = np.rad2deg(
                    filtered_gyro + gyro_bias + rng.normal(0, gyro_noise, 3)
                )

                accel = np.clip(np.round(accel * 16384), -32768, 32767) / 16384
                gyro = np.deg2rad(
                    np.clip(np.round(gyro * 131), -32768, 32767) / 131
                )
                gyro = calibration.update(gyro)

                requested[:] = controller(data.time, accel, gyro)
                requested[:2] = [thrust(value) for value in requested[:2]]

                # Hold the servo target until the requested angle takes up the slack.
                servo_target = np.clip(
                    servo_target,
                    requested[2:] - servo_slack,
                    requested[2:] + servo_slack,
                )

            thrust_queue.append(requested[:2].copy())
            data.ctrl[:2] = thrust_queue.popleft()
            data.ctrl[2:] = servo_target
            mujoco.mj_step(model, data)

            if step % 50 == 0:  # Ten frames per second.
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)

                frame = Image.fromarray(renderer.render())
                ImageDraw.Draw(frame).text(
                    (10, 10),
                    f"{data.time:.1f} s",
                    fill="white",
                )
                frames.append(frame)

                # Log true attitude for evaluation; the controller receives accel and gyro.
                rotation = data.body("drone").xmat.reshape(3, 3)
                true_angles = [
                    np.arctan2(rotation[2, 1], rotation[2, 2]),
                    np.arcsin(-rotation[2, 0]),
                ]
                readings.append([
                    data.time,
                    *data.xpos[1],
                    *data.sensor("gyro").data.copy(),
                    *true_angles,
                ])

    gif = ROOT / "hover.gif"
    frames[0].save(
        gif,
        save_all=True,
        append_images=frames[1:],
        duration=100,
        loop=0,
    )

    return gif, np.array(readings)
