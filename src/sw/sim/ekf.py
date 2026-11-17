"""
The drone's Kalman filter (fw/drone/main/ekf.c: attitude, velocity, position, IMU biases and barometer offset from the IMU, GPS and barometer) for the host, through ctypes on the firmware's own C, compiled here on first use. Sample and Output mirror ekf.h; the filter's own state is opaque (ekf_size bytes).
"""

import ctypes
from pathlib import Path

from sw.sim.hover import Baro, Estimate, Gps, build

SOURCE = Path(__file__).resolve().parents[2] / "fw/drone/main/ekf.c"
HEADERS = [Path(__file__).resolve().parents[2] / "fw/drone/main" / name for name in ("ekf.h", "hover.h")]
LIBRARY = Path(__file__).with_name("libekf.so")
G = 9.81


class Sample(ctypes.Structure):
    _fields_ = [("time_us", ctypes.c_int64), ("gyro", ctypes.c_float * 3), ("accel", ctypes.c_float * 3), ("dt", ctypes.c_float), ("spinning", ctypes.c_bool)]


class Output(ctypes.Structure):
    _fields_ = [("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float), ("velocity", ctypes.c_float * 3), ("position", ctypes.c_float * 3), ("height", ctypes.c_float),
                ("gyro_bias", ctypes.c_float * 3), ("accel_bias", ctypes.c_float * 3), ("baro_offset", ctypes.c_float), ("baro_transient", ctypes.c_float),
                ("tilt_sd", ctypes.c_float), ("yaw_sd", ctypes.c_float), ("velocity_sd", ctypes.c_float), ("position_sd", ctypes.c_float), ("height_sd", ctypes.c_float),
                ("aligned", ctypes.c_bool), ("still", ctypes.c_bool), ("heading_sd", ctypes.c_float), ("gps_innovation", ctypes.c_float * 3), ("baro_innovation", ctypes.c_float), ("rejected", ctypes.c_int)]


def load():
    """The library, built when missing or older than the source."""
    if not LIBRARY.exists() or LIBRARY.stat().st_mtime < max(path.stat().st_mtime for path in [SOURCE, *HEADERS]):
        build(SOURCE, LIBRARY, [])
    library = ctypes.CDLL(str(LIBRARY))
    library.ekf_size.restype = ctypes.c_int
    library.ekf_reset.argtypes = [ctypes.c_void_p]
    library.ekf_gps.argtypes = [ctypes.c_void_p, ctypes.POINTER(Gps)]
    library.ekf_baro.argtypes = [ctypes.c_void_p, ctypes.POINTER(Baro)]
    library.ekf_update.argtypes = [ctypes.c_void_p, ctypes.POINTER(Sample), ctypes.POINTER(Output)]
    library.ekf_estimate.argtypes = [ctypes.POINTER(Output), ctypes.POINTER(Estimate)]
    return library


library = load()


class Ekf:
    def __init__(self, library=library):
        self.library = library
        self.state = ctypes.create_string_buffer(library.ekf_size())
        self.output = Output()
        library.ekf_reset(self.state)

    def gps(self, capture_us, velocity, position):
        """A GPS solution: velocity north, east, up (m/s) and position north, east (m from any fixed point)."""
        self.library.ekf_gps(self.state, ctypes.byref(Gps(capture_us=capture_us, velocity=tuple(velocity), position=(position[0], position[1], 0.0))))

    def baro(self, capture_us, altitude):
        self.library.ekf_baro(self.state, ctypes.byref(Baro(capture_us=capture_us, altitude=altitude)))

    def update(self, time_us, gyro, accel, dt, spinning):
        """One tick: gyro (rad/s) and specific force (m/s²) in the IMU's body axes; returns the Output (reused between calls)."""
        self.library.ekf_update(self.state, ctypes.byref(Sample(time_us=time_us, gyro=tuple(gyro), accel=tuple(accel), dt=dt, spinning=spinning)), ctypes.byref(self.output))
        return self.output

    def estimate(self):
        """The last update's estimate as the hold flies it (sw.sim.hover Hover.estimate)."""
        estimate = Estimate()
        self.library.ekf_estimate(ctypes.byref(self.output), ctypes.byref(estimate))
        return estimate

