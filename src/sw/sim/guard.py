"""
The drone's fly-away failsafe and test-pattern limits (fw/drone/main/guard.c) for the host simulation, through ctypes on the firmware's own C, compiled here on first use. Structures mirror guard.h; settings are the failsafe_* and pattern_* entries of settings.def, with its defaults for recordings made before them.
"""

import ctypes
import math
from pathlib import Path

from sw.sim.hover import build

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/guard.c"
HEADERS = [SOURCE.with_suffix(".h"), SOURCE.with_name("hover.h")]
LIBRARY = Path(__file__).with_name("libguard.so")
OFF, BRAKE, HOLD, LAND = range(4)
PHASES = ("off", "brake", "hold", "land")
STOPS = ("playing", "distance", "tag lost", "tilt", "stick")
# settings.def: name, GuardGains member, default, whether stored in degrees.
SETTINGS = (("failsafe_speed", "speed", 2.0, False), ("failsafe_distance", "distance", 10.0, False), ("failsafe_time", "time", 1.5, False), ("failsafe_land", "land", 5.0, False),
            ("failsafe_tilt", "tilt", 15.0, True), ("pattern_distance", "pattern_distance", 0.7, False), ("pattern_tilt", "pattern_tilt", 30.0, True))


class Gains(ctypes.Structure):
    _fields_ = [(member, ctypes.c_float) for _, member, _, _ in SETTINGS]


class State(ctypes.Structure):
    _fields_ = [("failsafe", ctypes.c_int), ("far", ctypes.c_bool), ("phase_time", ctypes.c_float), ("fast", ctypes.c_float), ("quiet", ctypes.c_float), ("anchor", ctypes.c_float * 2),
                ("started", ctypes.c_bool), ("on_tag", ctypes.c_bool), ("located", ctypes.c_bool), ("stopped", ctypes.c_int), ("origin", ctypes.c_float * 3), ("nudge", ctypes.c_float * 2)]


if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(path.stat().st_mtime for path in [SOURCE, *HEADERS]):
    build(SOURCE, LIBRARY, [])
library = ctypes.CDLL(str(LIBRARY))
library.guard_hover.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Gains), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int64, ctypes.c_float]
library.guard_hover.restype = ctypes.c_int
library.guard_pattern.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Gains), ctypes.c_void_p, ctypes.c_int64, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float * 2, ctypes.c_float * 2]
library.guard_pattern.restype = ctypes.c_int
library.guard_pattern_reset.argtypes = [ctypes.POINTER(State)]
library.guard_pattern_reset.restype = None


def gains(settings):
    """The guards' settings from dashboard settings (degrees where the dashboard has them), settings.def's defaults where a recording predates them."""
    return Gains(**{member: math.radians(settings.get(name, default)) if angular else settings.get(name, default) for name, member, default, angular in SETTINGS})


class Guard:
    def __init__(self, settings):
        self.gains = gains(settings)
        self.state = State()

    def hover(self, hover_state, hover_gains, mode, now_us, dt):
        """The failsafe before a hover tick: the mode to fly; hover_gains (a copy of the hover's gains for this tick) and hover_state's braking may change, as on the drone."""
        return library.guard_hover(ctypes.byref(self.state), ctypes.byref(self.gains), ctypes.addressof(hover_state), ctypes.addressof(hover_gains), mode, now_us, dt)

    def pattern(self, hover_state, now_us, roll, pitch, stick_roll, stick_pitch, nudge, shift):
        """While a test pattern plays: 0 while it may go on, else why it stops (STOPS). hover_state is None outside the hover; nudge is the laptop's hold-point nudge and shift the pattern's own (m; forward, left)."""
        return library.guard_pattern(ctypes.byref(self.state), ctypes.byref(self.gains), ctypes.addressof(hover_state) if hover_state is not None else None, now_us, roll, pitch, stick_roll, stick_pitch, (ctypes.c_float * 2)(*nudge), (ctypes.c_float * 2)(*shift))
