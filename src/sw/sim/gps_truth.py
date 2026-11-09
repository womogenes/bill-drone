"""
Drone 1's outdoor flights against the GPS and the barometer as independent truth: uv run -m sw.sim.gps_truth [recordings or globs] [--verbose] [--save DIR]

Written 2026-11-09 for drone 1's outdoor flights of that night, 05:38-06:53 UTC (sw/recordings/2026-11-09T05-*.bin, 06-*.bin; the default), to give numbers for the airframe refit (sw/sim/airframe.py) and the outer-loop design. Every recording is put on a 100 Hz grid of the drone's session clock; the GPS solutions (fix 3) on the same clock from drone_ms, which the bridge already dated to the solution's arrival on the drone (fw/controller/bridge.py subtracts age_ms; subtracting it again would move them about 40 ms early). The airborne span runs from 1.35 s after the throttle starts (liftoff 0.35 s, then the liftoff kick) to 0.3 s before the cut, a landing's touchdown flag or a tip past 40 degrees. It prints:
- Alignment: the GPS velocity's latency behind its arrival and the nose's heading (the gyro's yaw plus a constant and a drift), from the GPS velocity's changes over 0.5 s against the level specific force integrated. Pooled 70 ms (flights 50-120 ms); gain 0.81-1.01; drift under 10 degrees per minute except 06:10 (19). 06:25 (a still hover) moved too little to fit a heading and is left out of the horizontal analyses.
- 1. The thrust direction against the GPS acceleration (0.5 s windows): g tan(pitch), -g tan(roll) alone explain R² 0.31 (forward 0.06); g tan(pitch + k tilt) with the thrust-weighted servo tilt (60 ms servo delay), drag and a per-flight constant 0.69, at k 0.80 (0.74-0.94 per flight, 0.52 at 06:51), drag 0.13-0.19/s forward, 0.12-0.15 sideways; the roll axis wants gain 1.00 on -g tan(roll). 1b fits the same from the accelerometer in body axes alone (no attitude estimate, no GPS acceleration): k 0.77 (0.75 above 1.5 m/s), forward drag 0.04-0.08/s, sideways 0.20-0.22/s, constant offsets of 0.1-0.5 m/s² per flight. The per-flight constants of 1 are mostly attitude-reference error, not wind: on the flights that turned through 230-430 degrees the accelerometer sees under 0.1 m/s² of world-fixed force.
- 2. The speed moment: the 1 s mean servo tilt against the body's forward speed. On ground speed 7.8 degrees per m/s (R² 0.84; 7.5-7.7 slow or fast alone), linear from -1.3 to +4.4 m/s (binned: -13.4, -3.7, +0.4, +8.5, +17.7, +24.1, +29.0 degrees at -1.27, -0.28, +0.28, +1.30, +2.48, +3.45, +4.35 m/s), the body pitching nose-up 4-5 degrees per m/s alongside as the hold's thrust_tilt compensation takes the tilt off the pitch it asks for. On airspeed with the winds of 1 (2-7 m/s) only 1.3 (R² 0.28): those winds are wrong. Read as an anemometer at 7.8 degrees per m/s, the tilt's once-per-turn part on the flights that turned through 230-430 degrees gives 0.04-0.35 m/s of wind from the south-west to west, the direction 1 also finds. So the night's wind at 1-2 m was light and airspeed is ground speed to about 0.4 m/s. The fly-aways (05:53:53 end, 06:38:51, 06:49:38 end) are this balance at 3-5.5 m/s: the servos at 25-37 degrees forward hold a nose-up body (-17 to -25 degrees) whose thrust then points 0-8 degrees forward, so the hold cannot brake with hover_tilt 10-15.
- 3. The hover's velocity (hold frame) against the GPS: with hover_gps_tau 0.5 gain 0.69-0.98 and lag 0-140 ms, except 06:33-06:38 (lag 240-320 ms, gain 0.69-1.03); the hold frame's heading is off by -4 to -61 degrees, steady through a flight (it does not converge where it starts wrong: 05:45 -43, 06:33-06:38 -58 to -61). With hover_gps_tau 0 (06:49-06:53) gain 0.42-0.64, heading +10 to +26 degrees and biases to 0.84 m/s.
- 4. Vertical: the barometer (about 50 Hz) scatters 0.04-0.075 m on the floor and 0.06-0.09 m above 1 Hz in flight against the accelerometer, the same with foam (06:25 on) as without; over 4 s windows it tracks the accelerometer's double integral to 0.07-0.13 m RMS. With the motors spinning on the floor it reads -0.13..+0.14 m (no steady wash offset), in the first second after liftoff it dips 0.3-2.9 m low (the kick), and in the last 0.3 s before a landing's cut it reads 0.25-0.45 m (median 0.36) above the floor read 0.5 s after the cut, with the drone 0-0.15 m up; the floor itself moves -0.46 to +0.60 m between takeoff and landing (weather, wind pressure, where it landed). The vertical bob (6: the smoothed height about its 10 s mean) is 0.10-0.24 m RMS per flight (0.38 at 06:33, which climbed to 2 m), mostly below 0.5 Hz; the accelerometer's own displacement spectrum (4) rises to its 0.15 Hz edge, noise there, and gives 1-3 cm at 0.5-1.5 Hz. The GPS climb rate lags 40-180 ms, gain 0.84-1.67, and at the GPS-held liftoffs checked (05:45, 05:50, 06:19, 06:25) it read 0.5-1.2 m/s DOWN for 0.3-0.5 s while the drone climbed at up to 1 m/s.
- 5. Hover thrust (1 s blocks above 0.5 m): the body-axis specific force times 1.05 kg over the table thrust along the body axis is 1.351 x (battery / 16.1 V)^1.42 (14.3-16.1 V, 845 blocks): the firmware's thrust_voltage compensation squares the ratio, so on a sagging pack it asks for up to 6% more than it thinks. Left runs 48 counts (sd 10) above right, 1.08-1.13 x the right's table thrust (airframe right_thrust_gain 1.086); it follows the yaw servo differential (18 counts per degree) and barely height, speed or battery.
- 6. A smoother (Kalman forward, RTS back) per flight: vertical [height, climb, accelerometer bias, barometer offset] from the level vertical specific force, the barometer (0.12 m, 0.6 m near the floor, its offset a 0.03 m/√s random walk) and the floor before takeoff; horizontal [north, east, velocities, forward and left accelerometer bias] from the level specific force turned by the fitted heading, the GPS velocity (0.1 m/s, at the fitted latency) and loosely the GPS position (0.7 m). GPS altitude is not used. Residuals: GPS velocity 0.12-0.33 m/s RMS, GPS position 0.16-0.96 m, barometer 0.06-0.10 m clear of the floor. With --save DIR each flight's trajectory goes to DIR/<flight>.npz on the grid up to the end of flight.
- 7. The takeoff climb: 7 of the 12 GPS-held takeoffs climb to 1.15-1.62 m at 1.8-2.6 s while the target is 0.53-0.77 m (05:45, 05:46, 06:10, 06:19, 06:25, 06:33, 06:38; 06:19 and 06:25 worst), none of the 7 without the GPS hold overshoots early (they peak at or near their goal at 3-4 s). The hover's climb estimate there drops to about zero at liftoff as the GPS's false descent pulls it (take_gps at gps_tau 0.5), and the barometer only joins 1 s after liftoff (BARO_SETTLE_S), so the climb loop drives 0.07-0.14 g of excess thrust for the first second and the IMU height sits near 0 until the barometer snaps it up.
"""

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from sw.flight_log import FIRST_DRONE, drone, read_flight

RECORDINGS = Path(__file__).parents[1] / "recordings"
DEFAULT = ("2026-11-09T05-*.bin", "2026-11-09T06-*.bin")
# The analysed evening, 05:38-06:53 UTC; recordings from 06:54 on lack the drone's metadata.
FIRST, LAST = "2026-11-09T05-38", "2026-11-09T06-54"
# Foam over the barometer from this flight on.
FOAM_FROM = "2026-11-09T06-25"
G = 9.81
FS = 100.0
DT = 1 / FS
DEG = math.pi / 180
# Takeoff: ESC start-up and the climb off the floor (models/NOTES.md: 0.34 s from throttle onset to liftoff), then the liftoff kick, left out of the airborne span.
LIFTOFF = 0.35
SETTLE = 1.0
# The end of the airborne span: this long before the landing's touchdown flag, the cut or a tip past TIP.
BEFORE_END = 0.3
TIP = 40 * DEG
# The servo's dead time plus its lag (airframe_2026-11-08b: 40 ms dead time, wn 35-40 rad/s): the commanded tilt reaches the thrust this much later.
TILT_DELAY = 0.06
# GPS velocity differences over this span (s) measure the acceleration: long enough to beat the 0.1-0.2 m/s velocity noise, short enough that the accelerometer's slow bias and the gyro heading's drift stay small.
SPAN = 0.5
# The latency search (s) and the heading's evidence windows (s).
LATENCIES = np.arange(0.0, 0.301, 0.01)
HEADING_WINDOW = 30.0
# The tilt share search for the thrust direction fit.
SHARES = np.arange(0.0, 1.501, 0.02)
# Quasi-steady blocks for the speed moment (s), and the most a block's pitch and tilt may wander (degrees RMS) to count as steady.
BLOCK = 1.0
STEADY = 3.0
# Heights below this (m, the hover's estimate) are near-ground for thrust and noise statistics.
CLEAR = 0.5
MASS = 1.05


def events(path):
    return [json.loads(line) for line in path.with_suffix(".jsonl").read_text().splitlines()]


def stamp(text):
    return datetime.fromisoformat(text.replace("Z", "+00:00")).timestamp()


def smooth(x, seconds):
    """A centered moving average over seconds (edges averaged over what there is)."""
    k = min(len(x), max(1, int(round(seconds * FS))))
    kernel = np.ones(k)
    return np.convolve(x, kernel, "same") / np.convolve(np.ones(len(x)), kernel, "same")


def lstsq(X, y):
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    return coef


def r2(y, residual):
    """The share of y's variance (about its mean) that a model leaving residual explains."""
    y = np.asarray(y)
    return 1 - np.sum(np.abs(residual) ** 2) / np.sum(np.abs(y - y.mean()) ** 2)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compass(u):
    """A world vector as north + i west (this script's complex convention) to (speed, direction it blows or moves toward in degrees clockwise from north)."""
    return abs(u), math.degrees(math.atan2(-u.imag, u.real)) % 360


class Flight:
    """
    One recording on a uniform 100 Hz grid of the drone's session clock, with its GPS solutions on the same clock.
    Horizontal vectors are complex: in the world north + i west (counterclockwise from above, like the body's forward + i left), so a body vector b turns into the world as b e^(i theta) with theta the nose's direction counterclockwise from north.
    """

    def __init__(self, path):
        self.path = path
        self.name = path.name[:19]
        self.time = path.name[11:19].replace("-", ":")
        self.events = events(path)
        meta = {}
        for event in self.events:
            if event["type"] in ("start", "metadata"):
                meta = event | meta
        self.meta = meta
        self.settings = {}
        for event in self.events:
            self.settings |= event.get("settings", {})
        s = self.settings
        self.table = np.array(meta["thrust_table"], float)
        self.start_wall = stamp(next(event for event in self.events if event["type"] == "start")["received_at"])
        self.stop = next((event["reason"] for event in self.events if event["type"] == "stop"), "")
        data, self.truncated = read_flight(path)
        data = data.sort_values("esp_time_us")
        t = data.esp_time_us.to_numpy() / 1e6
        self.grid = grid = np.arange(math.ceil(t[0] * FS) / FS, t[-1], DT)
        after = np.clip(np.searchsorted(t, grid), 0, len(t) - 1)
        before = np.clip(after - 1, 0, len(t) - 1)
        # Samples more than 30 ms from a received record sit in a dropped batch: interpolated over, and kept out of the airborne span.
        self.gap = np.minimum(np.abs(t[after] - grid), np.abs(t[before] - grid)) > 0.03
        column = lambda name: np.interp(grid, t, data[name].to_numpy().astype(float))
        flags = data["flags"].to_numpy().astype(int)[after]
        self.armed = (flags & 1) > 0
        self.mode = (flags >> 1) & 3
        self.landed = ((flags >> 4) & 1) > 0
        self.roll = column("roll_deg") * DEG
        self.pitch = column("pitch_deg") * DEG
        self.left = column("left_throttle")
        self.right = column("right_throttle")
        # Each motor's physical forward tilt from its trim (degrees; the right servo is mirrored).
        self.left_tilt = column("left_servo_deg") - s.get("left_trim", 0.0)
        self.right_tilt = -(column("right_servo_deg") - s.get("right_trim", 0.0))
        self.battery = column("battery_v")
        self.hover_v = column("hover_vx_m_s") + 1j * column("hover_vy_m_s")
        self.hover_yaw = column("hover_yaw_deg") * DEG
        self.hover_height = column("hover_height_m")
        self.hover_height_target = column("hover_height_target_m")
        self.hover_climb = column("hover_climb_m_s")
        baro = data["baro_altitude_m"].to_numpy()
        valid = np.isfinite(baro)
        self.baro = np.interp(grid, t[valid], baro[valid]) if valid.sum() > 1 else np.full(len(grid), np.nan)
        self.gyro = np.column_stack([column(f"g{axis}_deg_s") for axis in "xyz"]) * DEG
        # The accelerometer as the firmware calibrates it (flight_rt.c), turned by pose_zero into the level reference's body axes (drag_inputs).
        raw = np.column_stack([column(f"a{axis}_g") for axis in "xyz"])
        offset = np.array([s.get(f"accel_offset_{axis}", 0.0) for axis in "xyz"])
        gain = np.array([s.get(f"accel_gain_{axis}", 1.0) for axis in "xyz"])
        a = (raw - offset) * gain
        zr, zp = (value * DEG for value in meta["pose_zero_deg"])
        y = a[:, 1] * math.cos(zr) - a[:, 2] * math.sin(zr)
        z = a[:, 1] * math.sin(zr) + a[:, 2] * math.cos(zr)
        f = np.column_stack([a[:, 0] * math.cos(zp) + z * math.sin(zp), y, -a[:, 0] * math.sin(zp) + z * math.cos(zp)])
        self.specific = f
        cr, sr, cp, sp = np.cos(self.roll), np.sin(self.roll), np.cos(self.pitch), np.sin(self.pitch)
        # The specific force turned level (m/s²) as hover.c turns it: forward + i left in the heading frame, and up.
        self.level = G * (cp * f[:, 0] + sp * (sr * f[:, 1] + cr * f[:, 2])) + 1j * G * (cr * f[:, 1] - sr * f[:, 2])
        self.up = G * (-sp * f[:, 0] + cp * (sr * f[:, 1] + cr * f[:, 2]))
        # Heading from the gyro (rad, counterclockwise), as flight_rt.c's heading rate integrated over the recording.
        self.yaw = np.cumsum((self.gyro[:, 1] * sr + self.gyro[:, 2] * cr) / cp) * DT
        thrust = lambda throttle: np.interp(throttle, self.table[:, 0], self.table[:, 1])
        self.left_n, self.right_n = thrust(self.left), thrust(self.right)
        lt, rt = self.left_tilt * DEG, self.right_tilt * DEG
        # The thrust-weighted mean tilt (rad), as drag_inputs takes the thrust's forward tilt, delayed by the servo.
        tilt = np.arctan2(self.left_n * np.sin(lt) + self.right_n * np.sin(rt), np.maximum(self.left_n * np.cos(lt) + self.right_n * np.cos(rt), 1e-6))
        self.tilt = np.interp(grid - TILT_DELAY, grid, tilt)
        self.thrust_z = self.left_n * np.cos(lt) + self.right_n * np.cos(rt)
        self.load_gps()
        self.load_sticks()
        self.find_airborne()

    def load_gps(self):
        started = self.meta["started_ticks_us"] / 1e6
        rows, seen = [], set()
        for event in self.events:
            if event["type"] != "gps" or event.get("fix") != 3 or not event.get("ok") or event["itow_ms"] in seen:
                continue
            seen.add(event["itow_ms"])
            # drone_ms is already the solution's arrival on the drone's clock (the bridge subtracts age_ms: fw/controller/bridge.py), the clock started_ticks_us is on.
            rows.append([event["drone_ms"] / 1000 - started, *event["velocity"], event["latitude"], event["longitude"], event["horizontal_accuracy"], event["speed_accuracy"]])
        rows = np.array(rows, float).reshape(-1, 8)
        rows = rows[np.argsort(rows[:, 0])]
        self.gps_t = rows[:, 0]
        # Velocity as north + i west, and up (m/s).
        self.gps_u = rows[:, 1] - 1j * rows[:, 2]
        self.gps_up = -rows[:, 3]
        lat0 = rows[0, 4] if len(rows) else 0.0
        self.gps_position = (rows[:, 4] - lat0) * 111320.0 - 1j * (rows[:, 5] - (rows[0, 5] if len(rows) else 0.0)) * 111320.0 * math.cos(lat0 * DEG)
        self.gps_hacc = rows[:, 6]
        self.gps_sacc = rows[:, 7]

    def load_sticks(self):
        """The laptop's stick commands as a mask on the session clock (the start record's arrival taken as time 0; good to about 0.1 s), held 1 s past release."""
        self.stick = np.zeros(len(self.grid), bool)
        moves = [(stamp(event["received_at"]) - self.start_wall, abs(event.get("pitch", 0.0)) > 0 or abs(event.get("roll", 0.0)) > 0) for event in self.events if event["type"] == "command"]
        for (time, held), (after, _) in zip(moves, moves[1:] + [(self.grid[-1], False)]):
            if held:
                self.stick[(self.grid >= time) & (self.grid < after + 1.0)] = True

    def find_airborne(self):
        """Throttle onset, liftoff, the end of flight (touchdown flag, cut or tip-over) and the airborne mask between them (less the liftoff kick and the last BEFORE_END)."""
        on = np.flatnonzero((self.left > 0) & self.armed)
        self.flown = len(on) > 2 * FS
        self.air = np.zeros(len(self.grid), bool)
        if not self.flown:
            return
        self.onset = self.grid[on[0]]
        self.liftoff = self.onset + LIFTOFF
        ends = [self.grid[on[-1]]]
        later = self.grid > self.liftoff
        for mask in (self.landed & later, (np.maximum(np.abs(self.roll), np.abs(self.pitch)) > TIP) & later):
            if mask.any():
                ends.append(self.grid[np.argmax(mask)])
        self.end = min(ends)
        self.air = (self.grid > self.liftoff + SETTLE) & (self.grid < self.end - BEFORE_END) & ~self.gap
        self.flown = self.air.sum() > 3 * FS

    def has_gps(self):
        return self.flown and np.count_nonzero((self.gps_t > self.liftoff) & (self.gps_t < self.end)) > 50


def flights(patterns):
    paths = []
    for pattern in patterns:
        path = Path(pattern)
        paths += [path] if path.exists() else sorted(RECORDINGS.glob(pattern))
    paths = sorted({path for path in paths if path.suffix == ".bin"})
    return [path for path in paths if FIRST <= path.name[:16] < LAST and drone(path) == FIRST_DRONE]


def integral(x):
    """The running integral of a 100 Hz series (starts at 0)."""
    return np.concatenate([[0], np.cumsum(x)[:-1]]) * DT


def at(f, series, times):
    """A 100 Hz series (real or complex) at times on the drone clock."""
    if np.iscomplexobj(series):
        return np.interp(times, f.grid, series.real) + 1j * np.interp(times, f.grid, series.imag)
    return np.interp(times, f.grid, series)


def pairs(f, latency, span=SPAN, mask=None):
    """
    GPS samples k paired with the sample j about span earlier, both on the drone clock less latency, with the whole window inside mask (default the airborne span): (k, j, start, end).
    """
    mask = f.air if mask is None else mask
    times = f.gps_t - latency
    j = np.clip(np.searchsorted(times, times - span), 0, len(times) - 1)
    k = np.arange(len(times))
    # Where the earlier pick overshoots (missing solutions), its predecessor may be closer.
    j = np.where((j > 0) & (np.abs(times[k] - times[np.maximum(j - 1, 0)] - span) < np.abs(times[k] - times[j] - span)), j - 1, j)
    good = (j < k) & (np.abs(times[k] - times[j] - span) < 0.06)
    outside = np.concatenate([[0], np.cumsum(~mask)])
    lo = np.clip(np.searchsorted(f.grid, times[j]), 0, len(f.grid))
    hi = np.clip(np.searchsorted(f.grid, times[k]), 0, len(f.grid))
    good &= (times[j] > f.grid[0]) & (times[k] < f.grid[-1]) & (outside[hi] - outside[lo] == 0) & (hi > lo)
    return k[good], j[good], times[j[good]], times[k[good]]


class Alignment:
    """
    The GPS velocity's latency behind its arrival and the nose's heading, from the GPS velocity's changes over SPAN against the accelerometer's (the level specific force turned by the gyro heading and integrated):
    u_gps(k) - u_gps(j) = gain e^(i theta0) (V(t_k - latency) - V(t_j - latency)), V the integral of level e^(i yaw). The heading's drift is the slope of theta0 over HEADING_WINDOW windows.
    """

    def __init__(self, f):
        integrated = integral(f.level * np.exp(1j * f.yaw))
        rows = []
        for latency in LATENCIES:
            k, j, start, end = pairs(f, latency)
            a = at(f, integrated, end) - at(f, integrated, start)
            b = f.gps_u[k] - f.gps_u[j]
            c = np.vdot(a, b) / np.vdot(a, a)
            rows.append((np.sum(np.abs(b - c * a) ** 2) / np.sum(np.abs(b) ** 2), latency, c, len(k)))
        self.curve = np.array([row[0] for row in rows])
        self.residual, self.latency, c, self.count = min(rows, key=lambda row: row[0])
        self.gain, self.theta0 = abs(c), np.angle(c)
        # The heading per window, weighted by the inertial change it saw.
        k, j, start, end = pairs(f, self.latency)
        a = at(f, integrated, end) - at(f, integrated, start)
        b = f.gps_u[k] - f.gps_u[j]
        self.windows = []
        for low in np.arange(start.min() if len(start) else 0, end.max() if len(end) else 0, HEADING_WINDOW):
            pick = (start >= low) & (start < low + HEADING_WINDOW)
            if pick.sum() > 30:
                c = np.vdot(a[pick], b[pick])
                self.windows.append((low + HEADING_WINDOW / 2, wrap(np.angle(c) - self.theta0), np.sum(np.abs(a[pick]) ** 2)))
        self.drift = 0.0
        if len(self.windows) >= 2:
            w = np.array(self.windows)
            X = np.column_stack([np.ones(len(w)), w[:, 0] - w[:, 0].mean()]) * np.sqrt(w[:, 2])[:, None]
            self.drift = lstsq(X, w[:, 1] * np.sqrt(w[:, 2]))[1]
            mid = w[:, 0].mean()
            self.theta0 -= self.drift * mid
        self.good = self.residual < 0.7 and 0.7 < self.gain < 1.3
        self.theta = self.theta0 + f.yaw + self.drift * f.grid
        # The GPS velocity at the time it measured (m/s, north + i west), on the 100 Hz grid.
        self.velocity = np.interp(f.grid, f.gps_t - self.latency, f.gps_u.real) + 1j * np.interp(f.grid, f.gps_t - self.latency, f.gps_u.imag)


# Analysis 1: the thrust direction against the GPS's horizontal acceleration.


class ThrustData:
    """The GPS acceleration windows of one flight, and the integrals the thrust direction models are made of."""

    def __init__(self, f, align):
        self.f, self.align = f, align
        self.k, self.j, self.start, self.end = pairs(f, align.latency)
        self.span = self.end - self.start
        self.measured = (f.gps_u[self.k] - f.gps_u[self.j]) / self.span
        self.turn = np.exp(1j * align.theta)
        self.middle = at(f, self.turn, (self.start + self.end) / 2)
        self.middle /= np.abs(self.middle)
        self.speed = np.abs(at(f, align.velocity, (self.start + self.end) / 2))
        self.flyaway = self.speed > 2.0

    def mean(self, series):
        """series' mean over each window."""
        integrated = integral(series)
        return (at(self.f, integrated, self.end) - at(self.f, integrated, self.start)) / self.span

    def attitude(self, share, roll_gain=1.0):
        """The thrust direction's horizontal acceleration (world): g tan(pitch + share tilt) forward, -g tan(roll) left, turned by the heading."""
        f = self.f
        return self.mean(self.turn * (G * np.tan(f.pitch + share * f.tilt) - 1j * roll_gain * G * np.tan(f.roll)))

    def drag(self, wind):
        """The drag regressors at a wind (world, m/s): the forward and left airspeed (body) turned back into the world, negated."""
        air = np.conj(self.turn) * (self.align.velocity - wind)
        return -self.mean(self.turn * air.real), -self.mean(self.turn * 1j * air.imag)

    def wind_terms(self, cx, cy):
        """The acceleration a unit wind north and west makes through drag cx, cy (body)."""
        north = np.conj(self.turn)
        west = np.conj(self.turn) * 1j
        return self.mean(self.turn * (cx * north.real + 1j * cy * north.imag)), self.mean(self.turn * (cx * west.real + 1j * cy * west.imag))


def complex_lstsq(columns, target):
    """Real coefficients for complex regressors."""
    X = np.vstack([np.column_stack([c.real for c in columns]), np.column_stack([c.imag for c in columns])])
    return lstsq(X, np.concatenate([target.real, target.imag]))


def fit_thrust(data, share, isotropic=True, bias=False, roll_gain=1.0, iterations=12):
    """
    One thrust direction model at a tilt share over every flight's windows: drag cx, cy (1/s; one c when isotropic) on the airspeed, each flight's constant wind and (with bias) a constant body-frame acceleration (the attitude reference's error).
    Returns (cx, cy, winds, biases, residuals).
    """
    targets = [d.measured - d.attitude(share, roll_gain) for d in data]
    n = len(data)
    if isotropic:
        # Isotropic drag is linear: c times minus the ground speed, plus c times the wind as a constant per flight.
        columns, rows = [], []
        for i, d in enumerate(data):
            columns.append(-d.mean(d.align.velocity))
        X = [np.concatenate(columns)]
        for i, d in enumerate(data):
            for unit in (1, 1j):
                X.append(np.concatenate([np.full(len(e.measured), unit, complex) if e is d else np.zeros(len(e.measured), complex) for e in data]))
            if bias:
                for unit in (1, 1j):
                    X.append(np.concatenate([d.mean(d.turn * unit) if e is d else np.zeros(len(e.measured), complex) for e in data]))
        coef = complex_lstsq(X, np.concatenate(targets))
        c = coef[0]
        per = 4 if bias else 2
        winds = [complex(coef[1 + per * i], coef[2 + per * i]) / c for i in range(n)]
        biases = [complex(coef[3 + per * i], coef[4 + per * i]) if bias else 0j for i in range(n)]
        residual = np.concatenate(targets) - sum(x * value for x, value in zip(X, coef))
        return c, c, winds, biases, np.split(residual, np.cumsum([len(d.measured) for d in data])[:-1])
    cx, cy, winds, biases, _ = fit_thrust(data, share, True, bias, roll_gain)
    for _ in range(iterations):
        # Drag given the winds (and the biases as extra columns).
        X = [np.concatenate([d.drag(w)[0] for d, w in zip(data, winds)]), np.concatenate([d.drag(w)[1] for d, w in zip(data, winds)])]
        if bias:
            for d in data:
                for unit in (1, 1j):
                    X.append(np.concatenate([d.mean(d.turn * unit) if e is d else np.zeros(len(e.measured), complex) for e in data]))
        coef = complex_lstsq(X, np.concatenate(targets))
        cx, cy = coef[:2]
        biases = [complex(coef[2 + 2 * i], coef[3 + 2 * i]) for i in range(n)] if bias else [0j] * n
        # The winds given the drag, flight by flight.
        winds = []
        for d, b in zip(data, biases):
            ground = d.mean(d.turn * (cx * (np.conj(d.turn) * d.align.velocity).real + 1j * cy * (np.conj(d.turn) * d.align.velocity).imag))
            target = d.measured - d.attitude(share, roll_gain) + ground - d.mean(d.turn * b)
            w = complex_lstsq(list(d.wind_terms(cx, cy)), target)
            winds.append(complex(w[0], w[1]))
    residuals = []
    for d, w, b in zip(data, winds, biases):
        dx, dy = d.drag(w)
        residuals.append(d.measured - d.attitude(share, roll_gain) - cx * dx - cy * dy - d.mean(d.turn * b))
    return cx, cy, winds, biases, residuals


def explained(data, residuals, pick=None):
    """R² of the measured acceleration, overall and along the body's forward and left axes, over the windows pick selects (default all)."""
    measured = np.concatenate([d.measured for d in data])
    residual = np.concatenate(residuals)
    turn = np.concatenate([d.middle for d in data])
    pick = np.ones(len(measured), bool) if pick is None else pick
    body, rest = measured[pick] * np.conj(turn[pick]), residual[pick] * np.conj(turn[pick])
    return r2(measured[pick], residual[pick]), r2(body.real, rest.real), r2(body.imag, rest.imag)


def thrust_direction(fs, aligns):
    data = [ThrustData(f, a) for f, a in zip(fs, aligns)]
    data = [d for d in data if len(d.measured) > 30]
    flyaway = np.concatenate([d.flyaway for d in data])
    rows = {}
    rows["(a) g tan(pitch), -g tan(roll) alone"] = (None, None, None, [d.measured - d.attitude(0.0) for d in data])
    best = None
    curve = []
    for share in SHARES:
        cost = sum(np.sum(np.abs(r) ** 2) for r in fit_thrust(data, share)[4])
        curve.append((share, cost))
        if best is None or cost < best[0]:
            best = (cost, share)
    share = best[1]
    attitude_only = fit_thrust(data, 0.0)
    rows["(a+) attitude + isotropic drag + per-flight lean"] = (0.0, *attitude_only[:2], attitude_only[4])
    model = fit_thrust(data, share)
    rows["(b) + tilt share k"] = (share, *model[:2], model[4])
    aniso_best = None
    for value in np.arange(max(0.0, share - 0.3), share + 0.31, 0.05):
        result = fit_thrust(data, value, isotropic=False)
        cost = sum(np.sum(np.abs(r) ** 2) for r in result[4])
        if aniso_best is None or cost < aniso_best[0]:
            aniso_best = (cost, value, result)
    aniso = aniso_best[2]
    rows["(b') k, drag forward/sideways"] = (aniso_best[1], *aniso[:2], aniso[4])
    biased_best = None
    for value in np.arange(max(0.0, share - 0.3), share + 0.31, 0.05):
        result = fit_thrust(data, value, bias=True)
        cost = sum(np.sum(np.abs(r) ** 2) for r in result[4])
        if biased_best is None or cost < biased_best[0]:
            biased_best = (cost, value, result)
    biased = biased_best[2]
    rows["(b'') k, isotropic drag, + body-frame bias"] = (biased_best[1], *biased[:2], biased[4])
    # The roll axis' gain on -g tan(roll), at the best model.
    roll_rows = []
    for gain in np.arange(0.6, 1.41, 0.05):
        res = fit_thrust(data, share, roll_gain=gain)[4]
        roll_rows.append((sum(np.sum(np.abs(r) ** 2) for r in res), gain))
    roll_gain = min(roll_rows)[1]
    # Each flight's own tilt share (drag held at the pooled value), to see whether k holds across flights.
    per_flight = []
    for d, wind in zip(data, model[2]):
        costs = []
        for value in SHARES:
            target = d.measured - d.attitude(value)
            dx, dy = d.drag(wind)
            costs.append(np.sum(np.abs(target - model[0] * dx - model[1] * dy) ** 2))
        per_flight.append(SHARES[int(np.argmin(costs))])
    return {"data": data, "rows": rows, "share": share, "model": model, "aniso": aniso, "aniso_share": aniso_best[1], "biased": biased, "biased_share": biased_best[1], "flyaway": flyaway, "curve": curve, "roll_gain": roll_gain, "per_flight": per_flight}


def body_blocks(fs, aligns, winds=None):
    """
    1 s blocks (half overlapping) of the airborne span: the body-frame specific force over its body-z part times g (m/s²; forward, left: the attitude estimate does not enter), tan of the thrust-weighted tilt, the body-frame ground velocity (m/s), the airspeed at a wind, the heading, the mean tilt and pitch (deg), the GPS acceleration along the body axes and the pitch's wander.
    """
    blocks = []
    n = int(BLOCK * FS)
    for index, (f, a) in enumerate(zip(fs, aligns)):
        turn = np.exp(-1j * a.theta)
        ground = turn * a.velocity
        air = turn * (a.velocity - (winds[index] if winds else 0j))
        acceleration = turn * (np.gradient(smooth(a.velocity.real, 0.5), DT) + 1j * np.gradient(smooth(a.velocity.imag, 0.5), DT))
        for start in range(0, len(f.grid) - n, n // 2):
            span = slice(start, start + n)
            if not f.air[span].all():
                continue
            fx, fy, fz = f.specific[span].mean(0)
            blocks.append({
                "flight": index, "name": f.name, "fx": G * fx / fz, "fy": G * fy / fz, "tan": np.tan(f.tilt[span]).mean(), "ground": ground[span].mean(), "air": air[span].mean(),
                "theta": np.angle(np.exp(1j * a.theta[span]).mean()), "tilt": np.degrees(f.tilt[span]).mean(), "pitch": np.degrees(f.pitch[span]).mean(),
                "pitch_sd": np.degrees(f.pitch[span]).std(), "acceleration": acceleration[span].mean(), "speed": np.abs(a.velocity[span]).mean(), "stick": f.stick[span].any(),
            })
    return blocks


def body_drag(blocks, count):
    """
    The thrust direction and drag from the accelerometer alone, in body axes: fx = offset + k g tan(tilt) - cx (ground forward - wind forward), fy = offset - cy (ground left - wind left), with a constant offset and wind per flight (the wind as cos/sin terms of the heading, whose amplitudes are cx|w| and cy|w|).
    Returns (k, cx, cy, R² x, R² y, per flight [(offset x, offset y, cx w, cy w)]).
    """
    flight = np.array([b["flight"] for b in blocks])
    cos, sin = np.cos([b["theta"] for b in blocks]), np.sin([b["theta"] for b in blocks])
    fx, fy = np.array([b["fx"] for b in blocks]), np.array([b["fy"] for b in blocks])
    ground = np.array([b["ground"] for b in blocks])
    # The wind's body components: forward wN cos + wW sin, left -wN sin + wW cos (north + i west, as everywhere here).
    X, Y = [G * np.array([b["tan"] for b in blocks]), -ground.real], [-ground.imag]
    for i in range(count):
        m = (flight == i).astype(float)
        X += [m, m * cos, m * sin]
        Y += [m, -m * sin, m * cos]
    X, Y = np.column_stack(X), np.column_stack(Y)
    cx_coef, cy_coef = lstsq(X, fx), lstsq(Y, fy)
    k, cx, cy = cx_coef[0], cx_coef[1], cy_coef[0]
    per = []
    for i in range(count):
        if (flight == i).sum() < 5:
            per.append(None)
            continue
        per.append((cx_coef[2 + 3 * i], cy_coef[1 + 3 * i], complex(cx_coef[3 + 3 * i], cx_coef[4 + 3 * i]), complex(cy_coef[2 + 3 * i], cy_coef[3 + 3 * i])))
    return k, cx, cy, r2(fx, fx - X @ cx_coef), r2(fy, fy - Y @ cy_coef), per


# Analysis 2: the forward speed's nose-up moment as the servo tilt that holds it.


def moment_fit(blocks, pick, speed="ground"):
    """The mean tilt (deg) on the forward speed (ground, or air at the analysis 1 winds) over blocks pick selects, with and without the pitch and the GPS acceleration as further regressors."""
    rows = [b for b in blocks if pick(b)]
    if len(rows) < 8:
        return None
    tilt = np.array([b["tilt"] for b in rows])
    forward = np.array([b[speed].real for b in rows])
    pitch = np.array([b["pitch"] for b in rows])
    accel = np.array([b["acceleration"].real for b in rows])
    ones = np.ones(len(rows))
    X = np.column_stack([ones, forward])
    coef = lstsq(X, tilt)
    residual = tilt - X @ coef
    # Blocks overlap by half and the swing correlates neighbours: the standard error is optimistic by about 2x.
    se = math.sqrt((np.linalg.inv(X.T @ X) * residual.var())[1, 1])
    full = np.column_stack([ones, forward, pitch, accel])
    rich = lstsq(full, tilt)
    bins = []
    for low in np.arange(-3.0, 6.0, 1.0):
        inside = (forward >= low) & (forward < low + 1.0)
        if inside.sum() >= 3:
            bins.append((inside.sum(), forward[inside].mean(), tilt[inside].mean(), pitch[inside].mean()))
    return {"n": len(rows), "coef": coef, "se": se, "r2": r2(tilt, residual), "rich": rich, "rich_r2": r2(tilt, tilt - full @ rich), "bins": bins, "range": (forward.min(), forward.max())}


def heading_harmonic(blocks, index):
    """The tilt's and the thrust lean's (pitch + 0.8 tilt) once-per-revolution part over a flight's slow hover blocks against the heading: amplitude (deg) and the heading (deg counterclockwise from north) where each peaks."""
    rows = [b for b in blocks if b["flight"] == index and b["speed"] < 0.7 and not b["stick"]]
    if len(rows) < 10:
        return None
    theta = np.array([b["theta"] for b in rows])
    X = np.column_stack([np.ones(len(rows)), np.cos(theta), np.sin(theta)])
    out = []
    for values in (np.array([b["tilt"] for b in rows]), np.array([b["pitch"] + 0.8 * b["tilt"] for b in rows])):
        c = lstsq(X, values)
        out.append((math.hypot(c[1], c[2]), math.degrees(math.atan2(c[2], c[1])) % 360))
    return len(rows), out


# Analysis 3: the hover's own velocity estimate against the GPS.


def estimator(f, a):
    """The hover's velocity (hold frame) against the GPS (world) while the hover is engaged: lag, gain, the frame's heading against the true one, bias, and how the heading error moves over the flight."""
    engaged = f.air & (f.mode > 0)
    if engaged.sum() < 5 * FS:
        return None
    truth = a.velocity
    # The hold frame's x axis is the nose when hover_yaw is 0: in the world at theta - hover_yaw.
    frame = np.angle(np.sum(np.exp(1j * (a.theta - np.unwrap(f.hover_yaw)))[engaged]))
    best = None
    for lag in np.arange(-0.3, 0.81, 0.02):
        shifted = at(f, truth, f.grid - lag)
        x, y = shifted[engaged], f.hover_v[engaged]
        X = np.column_stack([x, np.ones(len(x))])
        coef = np.linalg.lstsq(X, y, rcond=None)[0]
        cost = np.sum(np.abs(y - X @ coef) ** 2)
        if best is None or cost < best[0]:
            best = (cost, lag, coef, x, y)
    cost, lag, coef, x, y = best
    # The firmware turns the world into the hold frame by e^(-i frame) when right: the gain's angle less that is its heading error (positive: its frame is turned counterclockwise of the truth, so it reads velocities turned clockwise).
    error = wrap(np.angle(coef[0]) + frame)
    windows = []
    times = f.grid[engaged]
    shifted = at(f, truth, f.grid - lag)[engaged]
    for low in np.arange(times.min(), times.max(), 10.0):
        pick = (times >= low) & (times < low + 10.0) & (np.abs(shifted) > 0.0)
        if pick.sum() > 3 * FS and np.sum(np.abs(shifted[pick] - shifted[pick].mean()) ** 2) > 0.5:
            c = np.vdot(shifted[pick] - shifted[pick].mean(), f.hover_v[engaged][pick] - f.hover_v[engaged][pick].mean())
            windows.append((low - f.liftoff, math.degrees(wrap(np.angle(c) + frame))))
    raw_rms = np.sqrt(np.mean(np.abs(y - x * np.exp(-1j * frame)) ** 2))
    return {"lag": lag, "gain": abs(coef[0]), "error": math.degrees(error), "bias": coef[1], "rms": math.sqrt(cost / len(y)), "raw_rms": raw_rms, "r2": r2(y, y - np.column_stack([x, np.ones(len(x))]) @ coef), "windows": windows, "seconds": engaged.sum() / FS}


# Analysis 4: the vertical, barometer against accelerometer.


def detrend(x, order=2):
    t = np.linspace(-1, 1, len(x))
    return x - np.polyval(np.polyfit(t, x, order), t)


def vertical(f):
    """The barometer's noise on the ground and in flight, against the accelerometer's double integral over short windows; the takeoff and landing offsets; the vertical bob's spectrum."""
    out = {}
    baro, up = f.baro, f.up - G
    on = np.searchsorted(f.grid, f.onset)
    # On the ground before the motors start: the reading's scatter about a 1 s average.
    if on > FS:
        ground = baro[:on]
        out["ground_noise"] = np.std(ground - smooth(ground, 1.0))
        out["ground_level"] = ground[-int(FS):].mean()
    # In flight: windows of 4 s where the height is clear of the floor; the accelerometer's double integral and the barometer, each less a quadratic (the accelerometer's bias and the start velocity), compared.
    clear = f.air & (f.hover_height > CLEAR)
    n = int(4 * FS)
    rms, gains, noise, corr = [], [], [], []
    for start in range(0, len(f.grid) - n, n // 2):
        span = slice(start, start + n)
        if not clear[span].all():
            continue
        h = detrend(np.cumsum(np.cumsum(up[span]) * DT) * DT)
        b = detrend(baro[span])
        rms.append(np.sqrt(np.mean((b - h) ** 2)))
        gains.append(np.dot(h, b) / np.dot(h, h))
        corr.append(np.corrcoef(h, b)[0, 1])
        # Above 1 Hz the accelerometer's height is truth to under a millimetre of bias: the barometer's scatter about it there is its noise.
        noise.append(np.std((b - h) - smooth(b - h, 1.0)))
    if rms:
        out.update(window_rms=np.median(rms), window_gain=np.median(gains), window_corr=np.median(corr), flight_noise=np.median(noise), windows=len(rms), motion=np.median([np.std(detrend(np.cumsum(np.cumsum(up[s:s + n]) * DT) * DT)) for s in range(0, len(f.grid) - n, n // 2) if clear[s:s + n].all()]))
    # Takeoff: motors spinning on the floor (onset to liftoff) against the motors off before, and the first second after liftoff.
    if "ground_level" in out:
        spin = (f.grid >= f.onset + 0.1) & (f.grid < f.liftoff)
        out["spinup_offset"] = baro[spin].mean() - out["ground_level"]
        lift = (f.grid >= f.liftoff) & (f.grid < f.liftoff + 1.0)
        out["liftoff_min"] = baro[lift].min() - out["ground_level"]
    # Landing: the last 0.3 s before the cut (the touchdown flag is often missed between logged samples; a landing's cut is its touchdown to 0.1 s) against the floor from 0.5 s after the motors stopped.
    if f.stop == "Landed":
        touchdown = f.end
        off = np.flatnonzero((f.grid > touchdown) & (f.left == 0))
        rest = baro[(f.grid > f.grid[off[0]] + 0.5)] if len(off) else np.array([])
        # A tipped-over drone's barometer sometimes reads a glitch (05:41: -22 m): the median, and nothing when the rest scatters.
        if len(off) and f.grid[-1] - f.grid[off[0]] > 0.8 and np.ptp(rest) < 1.0:
            before = (f.grid >= touchdown - 0.3) & (f.grid < touchdown)
            out["landing_offset"] = baro[before].mean() - np.median(rest)
            out["landing_hover_height"] = f.hover_height[before].mean()
            out["drift"] = np.median(rest) - out.get("ground_level", np.nan)
            out["moved"] = abs(f.gps_position[-1] - f.gps_position[np.searchsorted(f.gps_t, f.onset)]) if len(f.gps_t) else np.nan
    # The vertical bob: displacement spectrum from the accelerometer over the clear span (Welch, 10.24 s Hann segments).
    index = np.flatnonzero(clear)
    if len(index) > 12 * FS:
        segments = []
        m = 1024
        runs = np.split(index, np.flatnonzero(np.diff(index) > 1) + 1)
        for run in runs:
            for s in range(0, len(run) - m + 1, m // 2):
                x = up[run[s:s + m]]
                x = (x - x.mean()) * np.hanning(m)
                segments.append(np.abs(np.fft.rfft(x)) ** 2 / (FS * np.sum(np.hanning(m) ** 2)) * 2)
        if segments:
            psd = np.mean(segments, 0)
            freq = np.fft.rfftfreq(m, DT)
            w = 2 * math.pi * np.maximum(freq, 1e-9)
            displacement = psd / w ** 4
            band = lambda low, high: math.sqrt(np.sum(displacement[(freq >= low) & (freq < high)]) * (freq[1] - freq[0]))
            useful = (freq >= 0.15) & (freq < 5)
            out["bob_peak"] = freq[useful][np.argmax(displacement[useful])]
            out["bob_accel_peak"] = freq[(freq >= 0.3) & (freq < 5)][np.argmax(psd[(freq >= 0.3) & (freq < 5)])]
            out["bob"] = (band(0.15, 0.5), band(0.5, 1.5), band(1.5, 5.0))
            out["bob_segments"] = len(segments)
    return out


def vertical_latency(f):
    """The GPS climb rate's latency and gain against the accelerometer's, as the horizontal alignment (differences over SPAN)."""
    integrated = integral(f.up - G)
    rows = []
    for latency in np.arange(0.0, 0.61, 0.02):
        k, j, start, end = pairs(f, latency)
        if len(k) < 30:
            return None
        a = at(f, integrated, end) - at(f, integrated, start)
        b = f.gps_up[k] - f.gps_up[j]
        c = np.dot(a, b) / np.dot(a, a)
        rows.append((np.sum((b - c * a) ** 2) / np.sum(b ** 2), latency, c))
    return min(rows)


# Analysis 5: hover thrust.


def hover_thrust(fs, aligns, winds):
    """1 s blocks clear of the floor: the table thrust along the body axis (N, at the throttles), the accelerometer's specific force along it, battery voltage, and the left-right throttle difference with what might explain it."""
    rows = []
    for f, a, wind in zip(fs, aligns, winds):
        clear = f.air & (f.hover_height > CLEAR)
        air = np.conj(np.exp(1j * a.theta)) * (a.velocity - wind) if a is not None else np.zeros(len(f.grid), complex)
        n = int(BLOCK * FS)
        for start in range(0, len(f.grid) - n, n):
            span = slice(start, start + n)
            if not clear[span].all():
                continue
            rows.append({
                "flight": f.name, "table": f.thrust_z[span].mean(), "specific": f.specific[span, 2].mean(), "battery": f.battery[span].mean(), "ratio": f.battery[span].mean() / f.settings.get("thrust_voltage", 16.1),
                "difference": (f.left - f.right)[span].mean(), "throttle": ((f.left + f.right) / 2)[span].mean(), "height": f.hover_height[span].mean(),
                "forward": air[span].real.mean(), "left": air[span].imag.mean(), "yaw_tilt": (f.left_tilt - f.right_tilt)[span].mean() / 2, "roll": np.degrees(f.roll[span]).mean(),
                "split": (f.left_n / np.maximum(f.right_n, 1e-6))[span].mean(), "gps": a is not None,
            })
    return rows


# The smoother: IMU, barometer and GPS velocity (and, loosely, GPS position) fused forward and back.

# Process noise: the level specific force's per-sample scatter (m/s², vibration), the accelerometer bias' random walk (m/s² per √s) and the barometer offset's (m per √s; weather and wind pressure moved the ground reading 0.4 m over 80 s at 06:10).
ACCEL_NOISE = 1.0
BIAS_WALK = 0.01
BARO_WALK = 0.03
# Measurement noise: GPS velocity (m/s; the fits' residuals), GPS position (m; it wanders), the barometer clear of the floor and near it (m; the wash reads low there and spikes at liftoff).
GPS_VELOCITY_SD = 0.1
GPS_POSITION_SD = 0.7
BARO_SD = 0.12
BARO_NEAR_SD = 0.6
# Near the floor for the barometer: the first BARO_AFTER_LIFTOFF s after liftoff, the barometer itself under BARO_NEAR m above the takeoff floor (the hover's own height read 1-2 m low in some landings), the last BARO_BEFORE_END s.
BARO_AFTER_LIFTOFF = 1.5
BARO_NEAR = 0.5
BARO_BEFORE_END = 0.3


def kalman(F, u, Q, measurements, x0, P0):
    """
    A linear Kalman filter and RTS smoother over steps: F [n, d, d], u [n, d] (the input's effect), Q [d, d] or [n, d, d], measurements a list per step of (H [m, d], z [m], R [m, m]) or None; returns the smoothed states [n, d] and covariances [n, d, d].
    """
    n, d = u.shape
    xs, Ps, xp, Pp = np.zeros((n, d)), np.zeros((n, d, d)), np.zeros((n, d)), np.zeros((n, d, d))
    x, P = x0.copy(), P0.copy()
    for i in range(n):
        if i:
            x = F[i] @ x + u[i]
            P = F[i] @ P @ F[i].T + (Q[i] if Q.ndim == 3 else Q)
        xp[i], Pp[i] = x, P
        if measurements[i] is not None:
            H, z, R = measurements[i]
            S = H @ P @ H.T + R
            K = np.linalg.solve(S, H @ P).T
            x = x + K @ (z - H @ x)
            P = P - K @ H @ P
        xs[i], Ps[i] = x, P
    for i in range(n - 2, -1, -1):
        C = np.linalg.solve(Pp[i + 1].T, F[i + 1] @ Ps[i].T).T
        xs[i] = xs[i] + C @ (xs[i + 1] - xp[i + 1])
        Ps[i] = Ps[i] + C @ (Ps[i + 1] - Pp[i + 1]) @ C.T
    return xs, Ps


def smoothed(f, a=None):
    """
    The flight's best trajectory: vertical [height above the takeoff floor, climb rate, accelerometer bias, barometer offset] from the level vertical specific force, the barometer and the floor at rest before the motors start; horizontal [north, east, their velocities, the forward and left accelerometer bias] from the level specific force turned by the fitted heading, the latency-aligned GPS velocity and loosely the GPS position, when the heading is known (a). Returns a dict of arrays on the grid up to the end of flight (the cut, touchdown or tip-over: the impacts after it are no motion the IMU integrates well).
    """
    last = min(len(f.grid), np.searchsorted(f.grid, f.end))
    grid = f.grid[:last]
    n = last
    onset = np.searchsorted(grid, f.onset)
    landed = False
    # Vertical.
    a_up = (f.up - G)[:n]
    F = np.tile(np.array([[1, DT, -DT * DT / 2, 0], [0, 1, -DT, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]), (n, 1, 1))
    u = np.column_stack([a_up * DT * DT / 2, a_up * DT, np.zeros(n), np.zeros(n)])
    g = np.array([DT * DT / 2, DT, 0, 0])
    Q = np.outer(g, g) * ACCEL_NOISE ** 2 + np.diag([0, 0, BIAS_WALK ** 2 * DT, BARO_WALK ** 2 * DT])
    baro = f.baro[:n]
    floor = np.nanmean(baro[max(onset - 100, 0):max(onset, 1)]) if onset > 10 else np.nanmean(baro[:50])
    near = (grid < f.liftoff + BARO_AFTER_LIFTOFF) | (baro - floor < BARO_NEAR) | (grid > f.end - BARO_BEFORE_END)
    measurements = []
    for i in range(n):
        rows = []
        if i < onset or (landed and grid[i] > f.end + 0.3):
            # At rest on the floor: still, and (before takeoff) at height 0.
            rows += [([0, 1, 0, 0], 0.0, 0.01)] + ([([1, 0, 0, 0], 0.0, 0.01)] if i < onset else [])
        if i % 2 == 0 and np.isfinite(baro[i]) and grid[i] <= f.end:
            rows.append(([1, 0, 0, 1], baro[i], BARO_NEAR_SD if near[i] and i >= onset else BARO_SD))
        measurements.append((np.array([r[0] for r in rows], float), np.array([r[1] for r in rows]), np.diag([r[2] ** 2 for r in rows])) if rows else None)
    ground = baro[:max(onset, 1)].mean() if onset > 10 else np.nanmean(baro[:50])
    rest = a_up[:onset].mean() if onset > 10 else 0.0
    xs, Ps = kalman(F, u, Q, measurements, np.array([0, 0, rest, ground]), np.diag([0.01, 0.01, 0.3, 0.05]) ** 2)
    out = {"t": grid, "up": xs[:, 0], "climb": xs[:, 1], "bias_up": xs[:, 2], "baro_offset": xs[:, 3], "up_sd": np.sqrt(Ps[:, 0, 0]), "near_floor": near, "air": f.air[:n],
           "baro": baro, "roll": f.roll[:n], "pitch": f.pitch[:n], "tilt": f.tilt[:n], "left_tilt": f.left_tilt[:n], "right_tilt": f.right_tilt[:n], "left_throttle": f.left[:n], "right_throttle": f.right[:n], "battery": f.battery[:n]}
    if a is None:
        return out
    # Horizontal: state north, east, their velocities, the bias forward and left; the bias turns into the world with the heading.
    theta = a.theta[:n]
    c, s = np.cos(theta), np.sin(theta)
    # Body (forward, left) to (north, east): north = fx cos - fy sin, east = -(fx sin + fy cos).
    R = np.stack([np.stack([c, -s], -1), np.stack([-s, -c], -1)], -2)
    level = np.stack([f.level.real[:n], f.level.imag[:n]], -1)
    accel = np.einsum("nij,nj->ni", R, level)
    F = np.tile(np.eye(6), (n, 1, 1))
    F[:, 0, 2] = F[:, 1, 3] = DT
    F[:, 0:2, 4:6] = -R * DT * DT / 2
    F[:, 2:4, 4:6] = -R * DT
    u = np.concatenate([accel * DT * DT / 2, accel * DT, np.zeros((n, 2))], 1)
    Q = np.zeros((6, 6))
    for axis in range(2):
        g = np.zeros(6)
        g[axis], g[2 + axis] = DT * DT / 2, DT
        Q += np.outer(g, g) * ACCEL_NOISE ** 2
    Q[4, 4] = Q[5, 5] = BIAS_WALK ** 2 * DT
    velocity_at = {}
    for k in range(len(f.gps_t)):
        i = int(round((f.gps_t[k] - a.latency - grid[0]) / DT))
        if 0 <= i < n and grid[i] <= f.end:
            velocity_at[i] = k
    north0, east0 = f.gps_position[0].real, -f.gps_position[0].imag
    measurements = []
    for i in range(n):
        H, z, Rm = [], [], []
        if i < onset or (landed and grid[i] > f.end + 0.3):
            H += [[0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0]]
            z += [0.0, 0.0]
            Rm += [0.01, 0.01]
        if i in velocity_at:
            k = velocity_at[i]
            H += [[0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]]
            # The position is measured when the velocity is: the same latency.
            z += [f.gps_u[k].real, -f.gps_u[k].imag, f.gps_position[k].real - north0, -f.gps_position[k].imag - east0]
            Rm += [GPS_VELOCITY_SD, GPS_VELOCITY_SD, GPS_POSITION_SD, GPS_POSITION_SD]
        measurements.append((np.array(H, float), np.array(z), np.diag(np.square(Rm))) if H else None)
    xs, Ps = kalman(F, u, Q, measurements, np.zeros(6), np.diag([1.0, 1.0, 0.05, 0.05, 0.3, 0.3]) ** 2)
    out.update(north=xs[:, 0], east=xs[:, 1], v_north=xs[:, 2], v_east=xs[:, 3], bias_forward=xs[:, 4], bias_left=xs[:, 5], horizontal_sd=np.sqrt(Ps[:, 0, 0] + Ps[:, 1, 1]), velocity_sd=np.sqrt(Ps[:, 2, 2] + Ps[:, 3, 3]),
               heading=theta, latency=np.array(a.latency), gps_index=np.array(sorted(velocity_at)), gps_record=np.array([velocity_at[i] for i in sorted(velocity_at)]))
    return out


def smoother_residuals(f, s):
    """Each sensor against the smoothed trajectory: GPS velocity and position (RMS, m/s and m), the barometer clear of the floor (RMS) and near it (mean and the worst 0.3 s mean before touchdown, m), the GPS climb rate (RMS at the best lag, ms)."""
    out = {}
    t = s["t"]
    baro_error = s["baro"] - s["baro_offset"] - s["up"]
    flying = (t > f.liftoff) & (t < f.end)
    clear = flying & ~s["near_floor"]
    out["baro_rms"] = np.sqrt(np.nanmean(baro_error[clear] ** 2)) if clear.any() else np.nan
    near = flying & s["near_floor"] & (s["up"] < 0.5)
    out["baro_near"] = np.nanmean(baro_error[near]) if near.sum() > 20 else np.nan
    if f.stop == "Landed":
        before = (t > f.end - 0.3) & (t < f.end)
        out["baro_touchdown"] = np.nanmean(baro_error[before])
        out["height_touchdown"] = s["up"][before].mean()
    lift = (t > f.liftoff) & (t < f.liftoff + 1.0)
    out["baro_liftoff"] = np.nanmin(baro_error[lift]) if lift.any() else np.nan
    out["bias_up"] = (s["bias_up"][flying].mean(), np.ptp(s["bias_up"][flying])) if flying.any() else (np.nan, np.nan)
    out["baro_drift"] = s["baro_offset"][-1] - s["baro_offset"][0]
    # The vertical bob from the smoothed height (the accelerometer's own spectrum is noise below 0.5 Hz): its scatter about a 10 s mean, and its fastest swing's period band, clear of the floor.
    if clear.sum() > 10 * FS:
        out["bob"] = np.std((s["up"] - smooth(s["up"], 10.0))[clear])
    if "north" in s:
        i, k = s["gps_index"], s["gps_record"]
        keep = flying[i]
        dv = (s["v_north"][i] - f.gps_u[k].real) + 1j * (s["v_east"][i] + f.gps_u[k].imag)
        out["gps_velocity_rms"] = np.sqrt(np.mean(np.abs(dv[keep]) ** 2))
        north0, east0 = f.gps_position[0].real, -f.gps_position[0].imag
        dp = (s["north"][i] - (f.gps_position[k].real - north0)) + 1j * (s["east"][i] - (-f.gps_position[k].imag - east0))
        out["gps_position_rms"] = np.sqrt(np.mean(np.abs(dp[keep]) ** 2))
        out["bias_horizontal"] = np.hypot(s["bias_forward"][flying], s["bias_left"][flying]).mean()
        best = None
        for lag in np.arange(0.0, 0.61, 0.02):
            pick = (f.gps_t - lag > f.liftoff + 0.5) & (f.gps_t - lag < f.end - 0.3)
            if pick.sum() < 20:
                break
            e = np.sqrt(np.mean((np.interp(f.gps_t[pick] - lag, t, s["climb"]) - f.gps_up[pick]) ** 2))
            if best is None or e < best[0]:
                best = (e, lag)
        if best:
            out["gps_climb_rms"], out["gps_climb_lag"] = best
    return out


def takeoff(f, s):
    """The takeoff climb against its target: the peak height in the first 4 s after the motors start (smoothed), when, the hover's target then and its own height estimate there, the excess vertical specific force over the first second airborne, the liftoff's worst tilt and the battery."""
    t = s["t"]
    window = (t >= f.onset) & (t < f.onset + 4.0)
    if window.sum() < 300 or f.mode[np.searchsorted(f.grid, f.onset + 0.1)] == 0:
        return None
    i = np.flatnonzero(window)[np.argmax(s["up"][window])]
    early = (f.grid >= f.liftoff) & (f.grid < f.liftoff + 1.0)
    settle = (t >= f.onset + 4.0) & (t < f.onset + 8.0)
    return {
        "peak": s["up"][i], "time": t[i] - f.onset, "target": f.hover_height_target[i], "hover": f.hover_height[i], "baro_peak": np.nanmax(s["baro"][window] - s["baro_offset"][window]),
        "then": s["up"][settle].min() if settle.any() else np.nan, "goal": f.settings.get("hover_climb", np.nan),
        "excess": f.up[early].mean() / G - 1, "tilt": math.degrees(np.max(np.maximum(np.abs(f.roll[early]), np.abs(f.pitch[early])))), "battery": f.battery[early].mean(),
        "climb_peak": np.max(s["climb"][window]), "hover_climb_peak": np.max(f.hover_climb[np.flatnonzero(window)]),
    }


def table(header, rows):
    widths = [max(len(str(x)) for x in column) for column in zip(header, *rows)]
    print("  ".join(str(h).ljust(w) for h, w in zip(header, widths)))
    for row in rows:
        print("  ".join(str(x).ljust(w) for x, w in zip(row, widths)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recordings", nargs="*", default=list(DEFAULT))
    parser.add_argument("--verbose", action="store_true", help="per-window heading detail")
    parser.add_argument("--save", type=Path, help="write each flight's smoothed trajectory to DIR/<flight>.npz")
    args = parser.parse_args()
    fs = [Flight(path) for path in flights(args.recordings)]
    print(f"{len(fs)} recordings of drone 1: " + ", ".join(f"{f.time} {f.air.sum() / FS:.0f} s" for f in fs))

    print("\n== Alignment: GPS velocity latency (behind its arrival on the drone) and heading, from velocity changes over", SPAN, "s ==")
    aligns, rows = [], []
    for f in fs:
        a = Alignment(f) if f.has_gps() else None
        aligns.append(a)
        if a is None:
            rows.append([f.time, f"{f.air.sum() / FS:.0f}", "no fix" if f.flown else "no flight", "", "", "", "", "", f"{f.settings.get('hover_gps_tau', 0):g}"])
            continue
        rows.append([f.time, f"{f.air.sum() / FS:.0f}", f"{a.latency * 1000:.0f}", f"{a.gain:.2f}", f"{1 - a.residual:.2f}", f"{math.degrees(a.theta0) % 360:.0f}", f"{math.degrees(a.drift) * 60:+.1f}", "yes" if a.good else "NO", f"{f.settings.get('hover_gps_tau', 0):g}"])
    table(["flight", "air s", "latency ms", "gain", "R²", "nose ccw of N at yaw 0", "drift deg/min", "usable", "gps_tau"], rows)
    good = [(f, a) for f, a in zip(fs, aligns) if a is not None and a.good]
    pooled = sum(a.curve * a.count for f, a in good)
    pooled_latency = LATENCIES[np.argmin(pooled)]
    weights = np.array([a.count for f, a in good], float)
    latencies = np.array([a.latency for f, a in good])
    print(f"Pooled latency {pooled_latency * 1000:.0f} ms (flights {np.percentile(latencies, 25) * 1000:.0f}-{np.percentile(latencies, 75) * 1000:.0f} ms interquartile, weighted mean {np.average(latencies, weights=weights) * 1000:.0f} ms)")
    if args.verbose:
        for f, a in good:
            print(f"  {f.time} heading windows (mid s, deg off the fit, evidence):", ", ".join(f"{t:.0f}:{math.degrees(x):+.0f}/{w:.1f}" for t, x, w in a.windows))
    gf = [f for f, a in good]
    ga = [a for f, a in good]

    print("\n== 1. Thrust direction against the GPS acceleration (", len(gf), "flights,", "airborne windows of", SPAN, "s) ==")
    result = thrust_direction(gf, ga)
    data = result["data"]
    print(f"Windows: {sum(len(d.measured) for d in data)} ({int(result['flyaway'].sum())} above 2 m/s ground speed); flights: {', '.join(d.f.time for d in data)}")
    rows = []
    for name, (share, cx, cy, residuals) in result["rows"].items():
        total, forward, left = explained(data, residuals)
        fast = explained(data, residuals, result["flyaway"])[0] if result["flyaway"].sum() > 20 else float("nan")
        slow = explained(data, residuals, ~result["flyaway"])[0]
        rows.append([name, "" if share is None else f"{share:.2f}", "" if cx is None else f"{cx:.3f}", "" if cy is None else f"{cy:.3f}", f"{total:.3f}", f"{forward:.3f}", f"{left:.3f}", f"{slow:.3f}", f"{fast:.3f}"])
    table(["model", "k", "c fwd /s", "c side /s", "R² all", "R² fwd", "R² left", "R² <2 m/s", "R² >2 m/s"], rows)
    print("Cost against k (isotropic drag + leans): " + ", ".join(f"{k:.1f}:{c / result['curve'][0][1]:.3f}" for k, c in result["curve"][::5]))
    print(f"Roll axis: the best gain on -g tan(roll) at k {result['share']:.2f} is {result['roll_gain']:.2f}")
    print("Per-flight k (drag and winds held at the pooled fit): " + ", ".join(f"{d.f.time} {k:.2f}" for d, k in zip(data, result["per_flight"])))
    print("(The per-flight constants below are the world-fixed lean over c; with c this small they are not winds: see the accelerometer-only fit.)")
    rows = []
    for d, w_iso, w_bias, b in zip(data, result["model"][2], result["biased"][2], result["biased"][3]):
        rows.append([d.f.time, "%.1f m/s from %03.0f" % compass(-w_iso), "%.1f m/s from %03.0f" % compass(-w_bias), f"{abs(b):.2f}", f"{math.degrees(np.ptp(d.align.theta[d.f.air])):.0f}"])
    table(["flight", "lean/c (iso)", "lean/c (with body bias)", "body bias m/s²", "yaw range deg"], rows)

    print("\n== 1b. The same from the accelerometer alone in body axes (no attitude estimate, no GPS acceleration): fx = off + k g tan(tilt) - cx v_fwd, fy = off - cy v_left, a constant offset and wind per flight ==")
    blocks = body_blocks(gf, ga)
    k, cx, cy, rx, ry, per = body_drag(blocks, len(gf))
    print(f"k {k:.2f}, cx {cx:.3f} /s, cy {cy:.3f} /s (on ground speed; R² forward {rx:.3f}, left {ry:.3f}; {len(blocks)} blocks of {BLOCK} s)")
    fast = [b for b in blocks if b["speed"] > 1.5]
    if len(fast) > 20:
        kf, cxf, cyf, *_ = body_drag(fast, len(gf))
        print(f"  blocks above 1.5 m/s only ({len(fast)}): k {kf:.2f}, cx {cxf:.3f}, cy {cyf:.3f}")
    rows = []
    for f, a, p in zip(gf, ga, per):
        if p is None:
            continue
        ox, oy, xw, yw = p
        rows.append([f.time, f"{ox:+.2f}", f"{oy:+.2f}", f"{abs(xw):.2f}", f"{abs(yw):.2f}", f"{abs(xw) / max(cx, 1e-3):.1f}", f"{abs(yw) / max(cy, 1e-3):.1f}", f"{math.degrees(np.ptp(a.theta[f.air])):.0f}"])
    table(["flight", "offset fx m/s²", "offset fy m/s²", "cx|w| m/s²", "cy|w| m/s²", "|w| from fx", "|w| from fy", "yaw range deg"], rows)

    print("\n== 2. Speed moment: the mean thrust-weighted tilt (deg, forward) against the body's forward speed (m/s), 1 s blocks ==")
    winds = {d.f.name: w for d, w in zip(data, result["model"][2])}
    flight_winds = [winds.get(f.name, 0j) for f in gf]
    blocks = body_blocks(gf, ga, flight_winds)
    groups = {
        "all": lambda b: True,
        "slow (ground < 0.7 m/s)": lambda b: b["speed"] < 0.7,
        "fast (ground > 1.5 m/s)": lambda b: b["speed"] > 1.5,
        "pitch steady (sd < 3 deg)": lambda b: b["pitch_sd"] < STEADY,
    }
    rows = []
    for name, pick in groups.items():
        for speed in ("ground", "air"):
            fit = moment_fit(blocks, pick, speed)
            if fit is None:
                continue
            rows.append([name, "ground" if speed == "ground" else "air (1 winds)", fit["n"], f"{fit['coef'][1]:.2f} ± {fit['se']:.2f}", f"{fit['coef'][0]:+.1f}", f"{fit['r2']:.2f}", f"{fit['rich'][1]:.2f} / {fit['rich'][2]:+.2f} / {fit['rich'][3]:+.2f}", f"{fit['rich_r2']:.2f}", f"{fit['range'][0]:.1f}..{fit['range'][1]:.1f}"])
    table(["blocks", "speed", "n", "deg per m/s", "offset deg", "R²", "with pitch, accel: deg/(m/s) / deg/deg / deg/(m/s²)", "R²", "speed range"], rows)
    fit = moment_fit(blocks, lambda b: True, "ground")
    print("  binned by ground forward speed (n, mean speed, tilt, pitch): " + "; ".join(f"{n}: {v:+.2f} m/s {t:+.1f} deg (pitch {p:+.1f})" for n, v, t, p in fit["bins"]))
    rows = []
    for index, f in enumerate(gf):
        h = heading_harmonic(blocks, index)
        if h is None:
            continue
        n, ((tilt_amp, tilt_at), (lean_amp, lean_at)) = h
        rows.append([f.time, n, f"{math.degrees(np.ptp(ga[index].theta[f.air])):.0f}", f"{tilt_amp:.1f} at {tilt_at:.0f}", f"{lean_amp:.1f} at {lean_at:.0f}", f"{tilt_amp / fit['coef'][1]:.2f}" if np.ptp(ga[index].theta[f.air]) > math.pi else "(yaw < 180)"])
    table(["flight", "slow blocks", "yaw range", "tilt once-per-turn deg at nose ccw", "lean (pitch+0.8 tilt) deg at", "wind implied by tilt m/s"], rows)

    print("\n== 3. The hover's velocity estimate (hold frame) against the GPS (latency-corrected) while engaged ==")
    rows = []
    for f, a in good:
        e = estimator(f, a)
        if e is None:
            continue
        rows.append([f.time, f"{f.settings.get('hover_gps_tau', 0):g}", f"{e['seconds']:.0f}", f"{e['lag'] * 1000:+.0f}", f"{e['gain']:.2f}", f"{e['error']:+.0f}", f"{e['bias'].real:+.2f},{e['bias'].imag:+.2f}", f"{e['rms']:.2f}", f"{e['raw_rms']:.2f}", f"{e['r2']:.2f}", " ".join(f"{t:.0f}s:{x:+.0f}" for t, x in e["windows"])])
    table(["flight", "gps_tau", "engaged s", "lag ms", "gain", "heading err deg", "bias x,y m/s", "fit rms", "rms as flown", "R²", "heading error by 10 s window (from liftoff)"], rows)

    print("\n== 4. Vertical: barometer against the accelerometer ==")
    rows = []
    for f in fs:
        if not f.flown:
            continue
        v = vertical(f)
        lat = vertical_latency(f) if f.has_gps() else None
        foam = "foam" if f.name >= FOAM_FROM else "bare"
        fmt = lambda key, form="{:.2f}": form.format(v[key]) if key in v and np.isfinite(v[key]) else ""
        rows.append([f.time, foam, f"{f.settings.get('hover_baro_tau', 0):g}", fmt("ground_noise", "{:.3f}"), fmt("flight_noise", "{:.3f}"), fmt("window_rms", "{:.3f}"), fmt("window_gain"), fmt("window_corr"), fmt("motion", "{:.3f}"),
                     fmt("spinup_offset"), fmt("liftoff_min"), fmt("landing_offset"), fmt("landing_hover_height"), fmt("drift"), fmt("moved", "{:.1f}"),
                     f"{v['bob_peak']:.2f}/{v['bob_accel_peak']:.2f}" if "bob" in v else "", " ".join(f"{x * 100:.1f}" for x in v["bob"]) if "bob" in v else "",
                     f"{lat[1] * 1000:.0f}/{lat[2]:.2f}" if lat else ""])
    table(["flight", "baro", "baro_tau", "ground noise m", "flight noise >1Hz m", "4 s window rms m", "baro/accel gain", "corr", "accel motion sd m", "spin-up offset", "liftoff min", "landing offset", "hover height then", "ground drift", "moved m", "bob peak Hz disp/accel", "bob cm 0.15-0.5/0.5-1.5/1.5-5 Hz", "GPS climb latency ms/gain"], rows)

    print("\n== 5. Hover thrust: 1 s blocks clear of the floor ==")
    alignment = {f.name: a for f, a in zip(fs, aligns)}
    thrust_rows = hover_thrust([f for f in fs if f.flown], [alignment[f.name] if alignment[f.name] is not None and alignment[f.name].good else None for f in fs if f.flown], [winds.get(f.name, 0j) for f in fs if f.flown])
    level = np.array([MASS * G * r["specific"] / r["table"] for r in thrust_rows])
    ratio = np.array([r["ratio"] for r in thrust_rows])
    X = np.column_stack([np.ones(len(level)), np.log(ratio)])
    coef = lstsq(X, np.log(level))
    print(f"Thrust level (body-axis specific force x {MASS} kg over the table thrust along the body axis): {np.median(level):.3f} median; fitted level x ratio^p: level at thrust_voltage {math.exp(coef[0]):.3f}, voltage power p {coef[1]:.2f} (battery {ratio.min() * 16.1:.2f}-{ratio.max() * 16.1:.2f} V, {len(level)} blocks)")
    print("(airframe_2026-11-08b: thrust_level 1.2991 at voltage_power 1.6 for 1.05 kg; the firmware flies voltage power 2 at left/right_thrust_scale 1.3)")
    rows = []
    for name in sorted({r["flight"] for r in thrust_rows}):
        rs = [r for r in thrust_rows if r["flight"] == name]
        lv = np.array([MASS * G * r["specific"] / r["table"] for r in rs])
        d = np.array([r["difference"] for r in rs])
        rows.append([name[11:19], len(rs), f"{np.mean([r['battery'] for r in rs]):.2f}", f"{np.mean([r['throttle'] for r in rs]):.0f}", f"{np.median(lv):.3f}", f"{MASS * 1.2991 / np.median(lv):.3f}", f"{d.mean():.0f} ± {d.std():.0f}", f"{np.mean([r['split'] for r in rs]):.3f}"])
    table(["flight", "blocks", "battery V", "mean throttle", "thrust level", "mass at level 1.2991 kg", "left-right throttle", "left/right table thrust"], rows)
    d = np.array([r["difference"] for r in thrust_rows])
    names = ["const", "height m", "fwd airspeed", "left airspeed", "battery V", "mean throttle", "yaw tilt deg", "roll deg"]
    gps_rows = [r for r in thrust_rows if r["gps"]]
    X = np.column_stack([np.ones(len(gps_rows)), *[np.array([r[key] for r in gps_rows]) for key in ("height", "forward", "left", "battery", "throttle", "yaw_tilt", "roll")]])
    y = np.array([r["difference"] for r in gps_rows])
    coef = lstsq(X, y)
    sd = X[:, 1:].std(0)
    print(f"Left-right throttle {d.mean():.0f} counts mean (sd {d.std():.0f}) over {len(d)} blocks; regressed on (GPS flights, {len(y)} blocks, R² {r2(y, y - X @ coef):.2f}): " + ", ".join(f"{n} {c:+.1f} (x sd {c * s:+.1f})" for n, c, s in zip(names[1:], coef[1:], sd)))


    print("\n== 6. Smoother (forward Kalman + RTS): residuals of each sensor against the reconstructed trajectory ==")
    rows, climbs = [], []
    for f, a in zip(fs, aligns):
        if not f.flown:
            continue
        s = smoothed(f, a if a is not None and a.good else None)
        if args.save:
            args.save.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(args.save / f"{f.name}.npz", **s)
        r = smoother_residuals(f, s)
        fmt = lambda key, form="{:.2f}": form.format(r[key]) if key in r and np.isfinite(r[key]) else ""
        rows.append([f.time, "yes" if "north" in s else "vertical only", fmt("gps_velocity_rms", "{:.3f}"), fmt("gps_position_rms"), fmt("bias_horizontal"), fmt("baro_rms", "{:.3f}"), fmt("baro_near"), fmt("baro_liftoff"), fmt("baro_touchdown"), fmt("height_touchdown"),
                     f"{r['bias_up'][0]:+.2f} ({r['bias_up'][1]:.2f})", fmt("baro_drift"), fmt("bob"), f"{r['gps_climb_rms']:.2f} at {r['gps_climb_lag'] * 1000:.0f} ms" if "gps_climb_rms" in r else ""])
        climbs.append((f, takeoff(f, s)))
    table(["flight", "horizontal", "GPS vel rms m/s", "GPS pos rms m", "accel bias fwd/left m/s²", "baro rms clear m", "baro near floor mean", "baro min 1st s", "baro last 0.3 s", "height then", "up bias (range) m/s²", "baro offset drift m", "height bob sd m", "GPS climb rate rms (lag)"], rows)
    if args.save:
        print(f"Saved to {args.save}/<flight>.npz: t (drone session clock, s), up, climb, bias_up, baro_offset, up_sd, near_floor, air, baro; with the heading known north, east (m from the first fix), v_north, v_east, bias_forward, bias_left, horizontal_sd, velocity_sd, heading (rad ccw from north), latency; and the attitude, tilts, throttles and battery on the same grid.")

    print("\n== 7. Takeoff climb against its target (smoothed height) ==")
    rows = []
    for f, c in climbs:
        if c is None:
            continue
        rows.append([f.time, f"{c['goal']:.1f}", f"{c['peak']:.2f}", f"{c['time']:.1f}", f"{c['target']:.2f}", f"{c['hover']:.2f}", f"{c['baro_peak']:.2f}", f"{c['then']:.2f}", f"{c['climb_peak']:.2f}", f"{c['hover_climb_peak']:.2f}", f"{c['excess']:+.2f}", f"{c['tilt']:.0f}", f"{c['battery']:.2f}"])
    table(["flight", "climb goal m", "peak m", "at s from onset", "hover target then", "hover height then", "baro peak", "lowest 4-8 s", "peak climb m/s", "hover's climb peak", "1st s up force -1 (g)", "liftoff tilt deg", "battery V"], rows)

if __name__ == "__main__":
    main()
