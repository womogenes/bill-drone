"""
The drone's hover outer loop (fw/drone/main/hover.c) for the host simulation, through ctypes on the firmware's own C, compiled here on first use. Structures mirror hover.h; gains are SI units and radians.
"""

import ctypes
import math
import os
import subprocess
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/hover.c"
HEADER = SOURCE.with_suffix(".h")
# Compile-time overrides of hover.c's constants for experiments, e.g. HOVER_DEFINES="LOST_EASE_S=0.3f"; each set gets its own library.
DEFINES = os.environ.get("HOVER_DEFINES", "").split()
LIBRARY = Path(__file__).with_name("libhover" + "".join("-" + define.replace("=", "_") for define in DEFINES) + ".so")
if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(SOURCE.stat().st_mtime, HEADER.stat().st_mtime):
    scratch = LIBRARY.with_suffix(".%d.tmp" % os.getpid())
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", *("-D" + define for define in DEFINES), str(SOURCE), "-o", str(scratch), "-lm"], check=True)
    scratch.replace(LIBRARY)
library = ctypes.CDLL(str(LIBRARY))

HISTORY = 24
HOLD, LAND, LAND_FAST = 1, 2, 3
GAIN_KEYS = ("kp", "kd", "tilt", "kside", "kz", "kdz", "kiz", "mass", "climb", "climb_rate", "land_rate", "kyaw", "yaw_rate", "center", "distance", "approach", "ceiling", "cam_pitch", "pitch_trim")
# Settings stored in degrees (hover_tilt, hover_yaw_rate, hover_cam_pitch) and the attitude controller's pitch_trim.
ANGULAR_KEYS = ("tilt", "yaw_rate", "cam_pitch", "pitch_trim")


class Gains(ctypes.Structure):
    _fields_ = [(key, ctypes.c_float) for key in GAIN_KEYS]


class Pose(ctypes.Structure):
    _fields_ = [("sequence", ctypes.c_uint32), ("capture_us", ctypes.c_int64), ("tag", ctypes.c_float * 3), ("normal", ctypes.c_float * 3)]


class State(ctypes.Structure):
    _fields_ = [(name, ctypes.c_bool) for name in ("engaged", "landed", "have_pose", "have_setpoint", "have_normal", "found", "tracking")] + [
        (name, ctypes.c_float) for name in ("bias", "height", "climb_rate", "target", "command", "integral", "force", "low_time", "yaw")
    ] + [
        ("history_us", ctypes.c_int64 * HISTORY), ("history", (ctypes.c_float * 5) * HISTORY), ("next", ctypes.c_int),
        ("sequence", ctypes.c_uint32), ("capture_us", ctypes.c_int64), ("seen_us", ctypes.c_int64),
        ("position", ctypes.c_float * 3), ("velocity", ctypes.c_float * 3),
    ] + [(name, ctypes.c_float) for name in ("tag_height", "setpoint", "bearing", "arc", "normal")] + [("trim", ctypes.c_float * 2)]


class Output(ctypes.Structure):
    _fields_ = [("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw_rate", ctypes.c_float), ("force", ctypes.c_float), ("tracking", ctypes.c_bool), ("velocity", ctypes.c_float * 2)]


library.hover_rest.argtypes = [ctypes.POINTER(State), ctypes.c_float, ctypes.c_bool, ctypes.c_float]
library.hover_rest.restype = None
library.hover_update.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Gains), ctypes.c_int, ctypes.POINTER(Pose), ctypes.c_int64, ctypes.c_float,
                                 ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(Output)]
library.hover_update.restype = None


def gains(settings):
    """
    Hover gains from dashboard settings (hover_* keys plus pitch_trim), degrees where the dashboard uses them.
    """
    values = {}
    for key in GAIN_KEYS:
        value = settings.get("pitch_trim" if key == "pitch_trim" else "hover_" + key, 0.0)
        values[key] = math.radians(value) if key in ANGULAR_KEYS else value
    return Gains(**values)


class Hover:
    def __init__(self, settings):
        self.gains = gains(settings)
        self.state = State()
        self.output = Output()

    def rest(self, vertical_g, spinning, dt):
        library.hover_rest(ctypes.byref(self.state), vertical_g, spinning, dt)

    def update(self, mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force):
        library.hover_update(ctypes.byref(self.state), ctypes.byref(self.gains), mode, ctypes.byref(pose) if pose else None, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, ctypes.byref(self.output))
        return self.output
