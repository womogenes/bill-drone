"""
The drone's hover outer loop (fw/drone/main/hover.c) for the host simulation, through ctypes on the firmware's own C, compiled here on first use. Structures mirror hover.h; gains are SI units and radians.
"""

import ctypes
import math
import os
import subprocess
from pathlib import Path

from sw.sim import flight

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/hover.c"
HEADER = SOURCE.with_suffix(".h")
# Another hover.c to fly instead, built against this tree's hover.h (whose additions are all at the structures' ends), for comparing versions: HOVER_SOURCE=/path/old_hover.c.
if os.environ.get("HOVER_SOURCE"):
    SOURCE = Path(os.environ["HOVER_SOURCE"]).resolve()
# Compile-time overrides of hover.c's constants for experiments, e.g. HOVER_DEFINES="LOST_EASE_S=0.3f"; each set gets its own library.
DEFINES = os.environ.get("HOVER_DEFINES", "").split()
LIBRARY = Path(__file__).with_name("libhover" + ("-" + SOURCE.stem if os.environ.get("HOVER_SOURCE") else "") + "".join("-" + define.replace("=", "_") for define in DEFINES) + ".so")
HISTORY = 24
HOLD, LAND, LAND_FAST = 1, 2, 3
# What flight_rt.c assumes of its sensors (GPS_LATENCY_US, GPS_MAX_SPEED_ACCURACY, GPS_MAX_ACCURACY, BARO_LATENCY_US): a GPS solution is this old when it arrives (s) and counts only inside these speed and position accuracies (m/s, m); a barometer reading is this old (s).
GPS_LATENCY = 0.06
GPS_MAX_SPEED_ACCURACY = 0.6
GPS_MAX_ACCURACY = 8.0
BARO_LATENCY = 0.075
GAIN_KEYS = ("kp", "kd", "tilt", "kside", "kz", "kdz", "kiz", "mass", "climb", "climb_rate", "land_rate", "kyaw", "yaw_rate", "center", "distance", "approach", "ceiling", "square", "cam_pitch", "cam_roll", "drag", "drag_tilt", "drag_tau", "trim_tau", "trim_max", "thrust_tilt", "thrust_tilt_tau", "trim_start_x", "trim_start_y", "flow_depth", "flow_tau", "speed_tilt", "speed_pitch", "speed_takeoff", "speed_tau", "nudge_speed", "view", "gps_tau", "gps_place_tau", "gps_height_tau", "gps_heading", "baro_tau", "nudge_forward", "nudge_left", "nudge_up", "pitch_trim", "stick_pitch", "stick_roll", "thrust_direction", "gps_calibrate", "ekf", "return_speed")
# flight.py counts these to find the Command fields after them: a gain added here and not there shifts every one of them (the pitch damping's filter, the guards' gains) without an error.
assert len(GAIN_KEYS) == flight.HOVER_GAINS, "sw/sim/flight.py HOVER_GAINS must be the number of hover.h HoverGains members"
# Settings stored in degrees (hover_tilt, hover_yaw_rate, hover_cam_pitch, hover_cam_roll, hover_drag_tilt) and the attitude controller's pitch_trim.
ANGULAR_KEYS = ("tilt", "yaw_rate", "cam_pitch", "cam_roll", "drag_tilt", "pitch_trim", "speed_tilt", "speed_takeoff", "view", "gps_heading", "gps_calibrate")


class Gains(ctypes.Structure):
    _fields_ = [(key, ctypes.c_float) for key in GAIN_KEYS]


class Pose(ctypes.Structure):
    _fields_ = [("sequence", ctypes.c_uint32), ("capture_us", ctypes.c_int64), ("tag", ctypes.c_float * 3), ("normal", ctypes.c_float * 3)]


class Drag(ctypes.Structure):
    _fields_ = [("specific", ctypes.c_float * 3), ("tilt", ctypes.c_float)]


class Flow(ctypes.Structure):
    _fields_ = [("capture_us", ctypes.c_int64), ("rate", ctypes.c_float * 2)]


class Gps(ctypes.Structure):
    _fields_ = [("capture_us", ctypes.c_int64), ("velocity", ctypes.c_float * 3), ("position", ctypes.c_float * 3)]


class Baro(ctypes.Structure):
    _fields_ = [("capture_us", ctypes.c_int64), ("altitude", ctypes.c_float)]


class Estimate(ctypes.Structure):
    _fields_ = [("yaw", ctypes.c_float), ("velocity", ctypes.c_float * 3), ("position", ctypes.c_float * 2), ("height", ctypes.c_float), ("horizontal", ctypes.c_bool), ("vertical", ctypes.c_bool)]


class State(ctypes.Structure):
    _fields_ = [(name, ctypes.c_bool) for name in ("engaged", "landed", "have_pose", "have_setpoint", "have_normal", "found", "tracking")] + [
        (name, ctypes.c_float) for name in ("bias", "height", "climb_rate", "target", "command", "integral", "force", "yaw")
    ] + [
        ("history_us", ctypes.c_int64 * HISTORY), ("history", (ctypes.c_float * 8) * HISTORY), ("next", ctypes.c_int),
        ("sequence", ctypes.c_uint32), ("capture_us", ctypes.c_int64), ("seen_us", ctypes.c_int64),
        ("position", ctypes.c_float * 3), ("velocity", ctypes.c_float * 3),
    ] + [(name, ctypes.c_float) for name in ("tag_height", "setpoint", "bearing", "arc", "normal")] + [
        ("trim", ctypes.c_float * 2), ("drag_offset", ctypes.c_float * 2), ("drag_bias", ctypes.c_float * 2), ("place", ctypes.c_float * 2),
        ("drag_velocity", ctypes.c_float * 2), ("dragging", ctypes.c_bool), ("thrust_tilt", ctypes.c_float)] + [
        (name, ctypes.c_float) for name in ("hold_force", "hold_g", "recent_g", "land_time", "land_height", "stall_time", "impulse")] + [("probing", ctypes.c_bool)] + [
        (name, ctypes.c_float) for name in ("ceiling", "probe_start", "probe_g", "probe_force", "probe_velocity", "probe_expected", "probe_wait", "sink")] + [("nudge", ctypes.c_float * 3), ("nudge_rate", ctypes.c_float * 3),
        ("push", ctypes.c_float * 2), ("carry", ctypes.c_float * 2), ("flow_bias", ctypes.c_float), ("flow_us", ctypes.c_int64), ("braking", ctypes.c_float)] + [
        (name, ctypes.c_float) for name in ("speed_raw", "speed_offset", "speed_sum", "speed_time", "speed")] + [("speed_ready", ctypes.c_bool)] + [("reacquired_us", ctypes.c_int64)] + [
        ("inertial", ctypes.c_float * 2), ("track", ctypes.c_float * 2), ("gps_us", ctypes.c_int64), ("heading", ctypes.c_float), ("turn", ctypes.c_float * 2),
        ("turn_weight", ctypes.c_float), ("gps_velocity", ctypes.c_float * 2), ("gps_inertial", ctypes.c_float * 2), ("origin", ctypes.c_float * 3),
        ("rest", ctypes.c_float * 2), ("lifted", ctypes.c_float), ("shocked", ctypes.c_bool),
        ("baro_ground", ctypes.c_float), ("baro_grounded", ctypes.c_bool), ("baro_us", ctypes.c_int64), ("baro_wait", ctypes.c_float), ("baro_used", ctypes.c_bool), ("settle", ctypes.c_float), ("land_wait", ctypes.c_float), ("flown_pitch", ctypes.c_float), ("direction", ctypes.c_float), ("descended", ctypes.c_bool),
        ("heading_sum", ctypes.c_float * 4), ("heading_count", ctypes.c_int), ("heading_start_us", ctypes.c_int64), ("heading_block_us", ctypes.c_int64), ("turn_square", ctypes.c_float * 2), ("heading_sigma", ctypes.c_float), ("aligned", ctypes.c_bool), ("calibrate_time", ctypes.c_float), ("anchor", ctypes.c_float * 2), ("heading_inertial", ctypes.c_float),
        ("estimate", Estimate), ("estimated", ctypes.c_bool), ("flown", ctypes.c_bool), ("flown_tied", ctypes.c_bool), ("flown_position", ctypes.c_float * 2), ("flown_anchor", ctypes.c_float * 2), ("heading_steady", ctypes.c_float * 4), ("heading_steady_weight", ctypes.c_float), ("gps_fix", ctypes.c_float * 4), ("searching", ctypes.c_bool)]


class Output(ctypes.Structure):
    _fields_ = [("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw_rate", ctypes.c_float), ("force", ctypes.c_float), ("tracking", ctypes.c_bool), ("velocity", ctypes.c_float * 2)]


def build(source, library, defines):
    """Compile source into library with these -D defines, under a temporary name moved into place so parallel workers never load a half-written one."""
    scratch = library.with_suffix(".%d.tmp" % os.getpid())
    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-iquote", str(HEADER.parent), *("-D" + define for define in defines), str(source), "-o", str(scratch), "-lm"], check=True)
    scratch.replace(library)


def load(library, state):
    """The compiled hover.c at library, its functions declared for the state structure (State, or another copy's layout)."""
    library = ctypes.CDLL(str(library))
    library.hover_rest.argtypes = [ctypes.POINTER(state), ctypes.c_float, ctypes.c_bool, ctypes.c_float, ctypes.POINTER(Baro)]
    library.hover_rest.restype = None
    library.hover_update.argtypes = [ctypes.POINTER(state), ctypes.POINTER(Gains), ctypes.c_int, ctypes.POINTER(Pose), ctypes.c_int64, ctypes.c_float,
                                     ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(Drag), ctypes.POINTER(Flow),
                                     ctypes.POINTER(Gps), ctypes.POINTER(Baro), ctypes.POINTER(Output)]
    library.hover_body_to_camera.argtypes = [ctypes.POINTER(Gains), ctypes.c_float * 3, ctypes.c_float * 3]
    library.hover_body_to_camera.restype = None
    library.hover_update.restype = None
    # Copies older than the Kalman filter's estimate (HOVER_SOURCE) have no hover_estimate.
    if hasattr(library, "hover_estimate"):
        library.hover_estimate.argtypes = [ctypes.POINTER(state), ctypes.POINTER(Estimate)]
        library.hover_estimate.restype = None
    return library


if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(SOURCE.stat().st_mtime, HEADER.stat().st_mtime):
    build(SOURCE, LIBRARY, DEFINES)
library = load(LIBRARY, State)


def gains(settings):
    """
    Hover gains from dashboard settings (hover_* keys plus pitch_trim), degrees where the dashboard uses them.
    """
    values = {}
    for key in GAIN_KEYS:
        # Settings saved before an entry existed take its settings.def default (hover_square on, the trim's former constants), the rest zero.
        value = settings.get("pitch_trim" if key == "pitch_trim" else "hover_" + key, {"square": 1.0, "trim_tau": 4.0, "trim_max": 0.5, "thrust_tilt_tau": 1.0, "flow_tau": 0.3}.get(key, 0.0))
        values[key] = math.radians(value) if key in ANGULAR_KEYS else value
    return Gains(**values)


class Hover:
    def __init__(self, settings):
        self.gains = gains(settings)
        self.state = State()
        self.output = Output()

    def rest(self, vertical_g, spinning, dt, baro=None):
        """baro is (capture time in us, pressure altitude in m) or None, as in update."""
        reading = Baro(capture_us=baro[0], altitude=baro[1]) if baro else None
        library.hover_rest(ctypes.byref(self.state), vertical_g, spinning, dt, ctypes.byref(reading) if reading else None)

    def body_to_camera(self, body):
        """A body vector (forward, left, up) in the camera's axes (right, down, forward), as the firmware turns the gyro for the flow."""
        out = (ctypes.c_float * 3)()
        library.hover_body_to_camera(ctypes.byref(self.gains), (ctypes.c_float * 3)(*body), out)
        return list(out)

    def estimate(self, estimate):
        """The Kalman filter's estimate (an Estimate, from sw.sim.ekf) for the next update."""
        if hasattr(library, "hover_estimate"):
            library.hover_estimate(ctypes.byref(self.state), ctypes.byref(estimate))

    def update(self, mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, drag=None, flow=None, gps=None, baro=None, gains=None):
        """
        drag is (specific force in the level reference's body axes in g, commanded thrust tilt in radians) or None; flow is (capture time in us, translational rate right and down in rad/s) or None; gps is (capture time in us, velocity north, east, up in m/s, position north, east, up in m) or None; baro is (capture time in us, pressure altitude in m) or None; gains, when given, are flown this tick instead of self.gains (the runtime's per-tick copy, as the failsafe changes it).
        """
        inputs = Drag(specific=tuple(drag[0]), tilt=drag[1]) if drag else None
        measured = Flow(capture_us=flow[0], rate=tuple(flow[1])) if flow else None
        located = Gps(capture_us=gps[0], velocity=tuple(gps[1]), position=tuple(gps[2])) if gps else None
        reading = Baro(capture_us=baro[0], altitude=baro[1]) if baro else None
        library.hover_update(ctypes.byref(self.state), ctypes.byref(gains if gains is not None else self.gains), mode, ctypes.byref(pose) if pose else None, now_us, dt, roll, pitch, yaw_rate, vertical_g, force,
                             ctypes.byref(inputs) if inputs else None, ctypes.byref(measured) if measured else None, ctypes.byref(located) if located else None,
                             ctypes.byref(reading) if reading else None, ctypes.byref(self.output))
        return self.output
