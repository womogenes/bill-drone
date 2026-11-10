"""
The drone's Kalman filter (fw/drone/main/ekf.c: attitude, velocity, position, IMU biases and barometer offset from the IMU, GPS and barometer) for the host, through ctypes on the firmware's own C, compiled here on first use. Sample and Output mirror ekf.h; the filter's own state is opaque (ekf_size bytes).

replay(f) runs it over a whole recording (sw/sim/gps_truth.py Flight) as flight_rt.c would feed it, after a few seconds of rest: every 100 Hz tick's gyro and calibrated accelerometer in the IMU's body axes, each GPS solution inside the firmware's accuracy masks at its arrival, dated GPS_LATENCY earlier, and each barometer reading dated BARO_LATENCY earlier.

Where the radio dropped a batch of records (Flight.gap) the recording has no gyro, and the Flight's is a straight line between the records either side, which integrates to the wrong attitude (2026-11-09 06:19 UTC at 165-167 s: 10 degrees over two gaps). On the drone the filter has every sample, so there replay turns it as the logged attitude turned across the gap (gap_gyro).
"""

import ctypes
import os
from pathlib import Path

import numpy as np

from sw.sim.hover import BARO_LATENCY, GPS_LATENCY, Baro, Estimate, Gps, build

SOURCE = Path(os.environ.get("EKF_SOURCE", Path(__file__).resolve().parents[2] / "fw/drone/main/ekf.c"))
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


def load(defines=()):
    """The library, built with -D defines (name=value strings) into its own file when there are any."""
    target = LIBRARY if not defines else LIBRARY.with_name("libekf_" + "_".join(d.replace("=", "-") for d in defines) + ".so")
    if not target.exists() or target.stat().st_mtime < max(path.stat().st_mtime for path in [SOURCE, *HEADERS]):
        build(SOURCE, target, list(defines))
    library = ctypes.CDLL(str(target))
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


def gap_gyro(f):
    """f.gyro with the samples in dropped batches replaced by the body rates of the logged roll and pitch (and the interpolated gyro's own heading rate)."""
    gyro = f.gyro.copy()
    dt = float(f.grid[1] - f.grid[0])
    roll_rate, pitch_rate, yaw_rate = (np.gradient(angle, dt) for angle in (f.roll, f.pitch, f.yaw))
    sr, cr, sp, cp = np.sin(f.roll), np.cos(f.roll), np.sin(f.pitch), np.cos(f.pitch)
    rates = np.column_stack([roll_rate - yaw_rate * sp, pitch_rate * cr + yaw_rate * cp * sr, -pitch_rate * sr + yaw_rate * cp * cr])
    gyro[f.gap] = rates[f.gap]
    return gyro


FIELDS = ("roll", "pitch", "yaw", "height", "baro_offset", "baro_transient", "tilt_sd", "yaw_sd", "velocity_sd", "position_sd", "height_sd", "aligned", "still", "heading_sd", "baro_innovation", "rejected")
VECTORS = ("velocity", "position", "gyro_bias", "accel_bias", "gps_innovation")


def replay(f, library=library, withhold=(), baro=True, rest=5.0, mend=True):
    """The filter over the whole recording; returns a dict of arrays on f.grid, one per Output member. GPS solutions arriving inside any (start, end) of withhold (s on the grid) are kept from it, and the barometer with baro False. A recording starts 1-4 s before its motors, and the drone has by then sat still since boot with the filter running, so the filter first gets rest s of that: the recording's own ticks before the motors start, played back and forth. mend False leaves the gyro in dropped batches as the Flight interpolated it: a test of how the filter comes back from a wrong attitude."""
    ekf = Ekf(library)
    fix_at = f.fixes(withhold)
    spinning = (f.left > 0) | (f.right > 0)
    accel = f.accel * G
    gyro = gap_gyro(f) if mend else f.gyro
    out = {name: np.zeros(len(f.grid)) for name in FIELDS} | {name: np.zeros((len(f.grid), 3)) for name in VECTORS}
    dt = float(f.grid[1] - f.grid[0])
    quiet = max(int(np.argmax(spinning)) - 10 if spinning.any() else len(f.grid), 2)
    for back in range(int(rest / dt), 0, -1):
        i = back % (2 * quiet)
        i = i if i < quiet else 2 * quiet - 1 - i
        now_us = int(round((f.grid[0] - back * dt) * 1e6))
        if baro and back % 2 == 0 and np.isfinite(f.baro[i]):
            ekf.baro(now_us - int(BARO_LATENCY * 1e6), float(f.baro[i]))
        ekf.update(now_us, gyro[i], accel[i], dt, False)
    for i in range(len(f.grid)):
        now_us = int(round(f.grid[i] * 1e6))
        if baro and i % 2 == 0 and np.isfinite(f.baro[i]):
            ekf.baro(now_us - int(BARO_LATENCY * 1e6), float(f.baro[i]))
        k = fix_at.get(i)
        if k is not None:
            u, p = f.gps_u[k], f.gps_position[k]
            ekf.gps(int(round((f.gps_t[k] - GPS_LATENCY) * 1e6)), (u.real, -u.imag, f.gps_up[k]), (p.real, -p.imag))
        o = ekf.update(now_us, gyro[i], accel[i], dt, bool(spinning[i]))
        for name in FIELDS:
            out[name][i] = getattr(o, name)
        for name in VECTORS:
            out[name][i] = getattr(o, name)[:]
    return out
