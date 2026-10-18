"""
Estimate attitude and calculate motor commands from accelerometer and gyro readings.
Firmware and simulation use this module. Internal angles are in radians and rates
are in rad/s; dashboard setpoints use degrees and degrees/s. Geometry and inertia
come from the 900 g MuJoCo model.
"""

import math
from .calibration import thrust, throttle

ARM = 0.172
PIVOT_HEIGHT = 0.0253692458911615
COM_X = -0.00202026450036472
INERTIA = (0.0118483264434811, 0.0023030647846826, 0.0106709253540914)
DEFAULT_TUNING = {
    "roll_p": 20.0,
    "roll_i": 3.0,
    "roll_damping": 6.0,
    "pitch_p": 20.0,
    "pitch_i": 3.0,
    "pitch_damping": 10.0,
    "yaw_damping": 4.0,
    "integral_limit": 3.0,
    "pitch_trim": -3.6,
    "left_trim": 0.9,
    "right_trim": -0.9,
}


def clamp(value, low, high):
    return max(low, min(high, value))


class Attitude:
    def __init__(self, accel=(0.0, 0.0, 1.0)):
        self.roll = math.atan2(accel[1], accel[2])
        self.pitch = math.atan2(-accel[0], math.sqrt(accel[1] ** 2 + accel[2] ** 2))

    def update(self, accel, gyro, dt, powered):
        roll, pitch = self.roll, self.pitch
        gx, gy, gz = gyro
        roll_rate = gx + math.tan(pitch) * (gy * math.sin(roll) + gz * math.cos(roll))
        pitch_rate = gy * math.cos(roll) - gz * math.sin(roll)
        gravity_roll = math.atan2(accel[1], accel[2])
        gravity_pitch = math.atan2(-accel[0], math.sqrt(accel[1] ** 2 + accel[2] ** 2))

        # Correct toward gravity slowly while powered, since acceleration also tilts its reading.
        weight = 1 - (0.9998 if powered else 0.99) ** (dt / 0.01)
        self.roll += roll_rate * dt
        self.pitch += pitch_rate * dt
        self.roll += weight * (gravity_roll - self.roll)
        self.pitch += weight * (gravity_pitch - self.pitch)
        return self.roll, self.pitch


class Controller:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = [0.0, 0.0]

    def update(self, attitude, gyro, dt, settings):
        base = settings["base_throttle"]
        if not settings["enabled"] or base == 0:
            self.reset()
            return [0.0, 0.0, 0.0, 0.0]

        roll, pitch = attitude
        trim = math.radians(settings["pitch_trim"])
        errors = [
            math.radians(settings["roll"]) - roll,
            math.radians(settings["pitch"]) + trim - pitch,
        ]
        torque = []
        for index, axis in enumerate(("roll", "pitch")):
            self.integral[index] = clamp(
                self.integral[index] + settings[axis + "_i"] * errors[index] * dt,
                -settings["integral_limit"], settings["integral_limit"],
            )
            torque.append(INERTIA[index] * (
                settings[axis + "_p"] * errors[index] + self.integral[index]
                - settings[axis + "_damping"] * gyro[index]
            ))
        torque.append(INERTIA[2] * settings["yaw_damping"] * (
            math.radians(settings["yaw_rate"]) - gyro[2]
        ))

        force = 2 * thrust(base) * math.cos(trim)
        vertical = [
            max(0, (force + torque[0] / ARM) / 2),
            max(0, (force - torque[0] / ARM) / 2),
        ]
        forward = (torque[1] - COM_X * force) / PIVOT_HEIGHT
        yaw_torque = torque[2] - 0.015 * (vertical[1] - vertical[0])
        horizontal = [
            (forward - yaw_torque / ARM) / 2,
            (forward + yaw_torque / ARM) / 2,
        ]
        angles = [
            clamp(
                -math.atan2(horizontal[0], vertical[0]) + math.radians(settings["left_trim"]),
                -math.pi / 3, math.pi / 3,
            ),
            clamp(
                math.atan2(horizontal[1], vertical[1]) + math.radians(settings["right_trim"]),
                -math.pi / 3, math.pi / 3,
            ),
        ]
        motors = [round(throttle(math.sqrt(x * x + z * z))) for x, z in zip(horizontal, vertical)]
        return motors + angles
