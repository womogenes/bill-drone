"""
Binary flight-log layout from the drone (drone/main/flight_rt.c Record and radio.c) and the test pattern's channels, shared by the laptop service, sw/flight_log.py and sw/excite.py, and the one decoder of the recordings' records (decode_records).
"""

import struct

# The 48-byte record of older logs, then the hover loop's state (flight_rt.c Record): tag position (mm), sighting age (ms, 65535 none), velocity (mm/s), height (mm), climb rate (mm/s), height and range targets (mm), arc and yaw (centidegrees), trim (mm/s²) and drag-model velocity (mm/s); then the barometer's pressure altitude in the standard atmosphere (mm, BARO_NONE without a reading in the last 100 ms); then the guards (drone/main/guard.h): bits 0-1 the fly-away failsafe's phase (FAILSAFE_PHASES), bit 2 set off by distance, bits 3-5 why the test pattern stopped (PATTERN_STOPS); then the Kalman filter's estimate (drone/main/ekf.h), which the hold flies only where hover_ekf says: roll, pitch and yaw (centidegrees; yaw from north once its heading is known), velocity north, west and up (mm/s), position north and west of where it last rested (cm), height (mm), the standard deviations of its horizontal velocity (cm/s) and height (cm), and its flags (EKF_FLAGS from bit 0, bits 8-15 the GPS solutions refused in a row); then the camera's newest dated frame: its sequence's low 16 bits and how long before the sample its center row was read (10 us, 65535 none), which ties the camera's black-box frames to the IMU.
BASE_FORMAT = "<II7h2h2H2h3hHHHH"
HOVER_FORMAT = BASE_FORMAT + "3hH12h"
BARO_RECORD_FORMAT = HOVER_FORMAT + "i"
GUARD_RECORD_FORMAT = BARO_RECORD_FORMAT + "H"
EKF_RECORD_FORMAT = GUARD_RECORD_FORMAT + "9h2BH"
CAMERA_RECORD_FORMAT = EKF_RECORD_FORMAT + "2H"
# Then the newest camera frame's camera -> tag rotation as a unit quaternion (w, x, y, z, 32767 = 1; all zero without a decided, trusted sighting in that frame).
SAMPLE_FORMAT = CAMERA_RECORD_FORMAT + "4h"
HEADER_FORMAT = "<4sIHH"
BASE_SIZE = struct.calcsize(BASE_FORMAT)  # 48 bytes
HOVER_SIZE = struct.calcsize(HOVER_FORMAT)  # 80 bytes
BARO_RECORD_SIZE = struct.calcsize(BARO_RECORD_FORMAT)  # 84 bytes
GUARD_RECORD_SIZE = struct.calcsize(GUARD_RECORD_FORMAT)  # 86 bytes
EKF_RECORD_SIZE = struct.calcsize(EKF_RECORD_FORMAT)  # 108 bytes
CAMERA_RECORD_SIZE = struct.calcsize(CAMERA_RECORD_FORMAT)  # 112 bytes
RECORD_SIZE = struct.calcsize(SAMPLE_FORMAT)  # 120 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 12 bytes
BARO_NONE = -(2**31)
# Each FLG1 header's record size picks the layout, so older recordings still decode.
FORMATS = {BASE_SIZE: BASE_FORMAT, HOVER_SIZE: HOVER_FORMAT, BARO_RECORD_SIZE: BARO_RECORD_FORMAT, GUARD_RECORD_SIZE: GUARD_RECORD_FORMAT, EKF_RECORD_SIZE: EKF_RECORD_FORMAT, CAMERA_RECORD_SIZE: CAMERA_RECORD_FORMAT, RECORD_SIZE: SAMPLE_FORMAT}
MAX_BATCH = 8  # Records per FLG1 batch at most (radio.c LOG_MAX: 3 since the 120-byte record, 4 since the 108-byte one, 5 since the 80-byte one, 8 before).


def batches(payload):
    """
    The FLG1 batches in one flight-log message as (offset of the first record, session, record count, record size): one per message from older drones, up to radio.c LOG_BATCHES laid end to end (as the recording holds them) while a newer drone catches up a backlog. Raises ValueError for anything else.
    """
    found, offset = [], 0
    while offset < len(payload):
        if len(payload) - offset < HEADER_SIZE:
            raise ValueError("Truncated FLG1 header")
        magic, session, count, size = struct.unpack_from(HEADER_FORMAT, payload, offset)
        offset += HEADER_SIZE
        if magic != b"FLG1" or size not in FORMATS or not 1 <= count <= MAX_BATCH or len(payload) - offset < count * size:
            raise ValueError("Invalid FLG1 batch")
        found.append((offset, session, count, size))
        offset += count * size
    return found

STOP_REASONS = (
    "", "Stopped", "Controller heartbeat lost", "IMU read failed",
    "Tilt limit exceeded", "Control sample delayed", "IMU calibration", "Landed", "Landed after link loss", "Landed by the fly-away failsafe",
    "Recovery failed",
)
# The guards' states (drone/main/guard.h FAILSAFE_* and PATTERN_*), the roll flip's phase (drone/main/flip.h flip_phase: climbing, spinning or righting, arresting) and a recovery's stage and cause (drone/main/recover.h RECOVER_*: righting or leveling; past tip_limit, a failing flip, the gyro at full scale, the pilot's level-and-hold), as the record's guard field holds them; the status carries the first eight bits in its flags' bits 8-15 and the recovery in its reason (decode_reason). The flip's phases and the recovery's causes are worded as the dashboard shows them (the status, the replay's events).
FAILSAFE_PHASES = (None, "brake", "hold", "land")
PATTERN_STOPS = (None, "distance", "tag lost", "tilt", "stick", "landing", "recovery", None)
FLIP_PHASES = (None, "climbs", "turns", "arrests")
RECOVERY_STAGES = (None, "righting", "leveling", None)
RECOVERY_CAUSES = (None, "tipped", "flip failed", "gyro saturated", "level", None, None, None)
# The Kalman filter's flags from bit 0 (flight_rt.c Record ekf_flags): its heading is known, it is resting still, it is good enough for the hold to fly horizontally and vertically, and the hold flew on it this tick.
EKF_FLAGS = ("aligned", "still", "horizontal", "vertical", "flown")


def guard_state(bits):
    """The guards from their bits: the failsafe's phase (None when off), whether distance set it off, why the test pattern stopped (None when it did not), the flip's phase (None when none runs), and a recovery's stage and cause (None when none runs)."""
    return {"failsafe": FAILSAFE_PHASES[bits & 3], "failsafe_far": bool(bits & 4), "pattern_stop": PATTERN_STOPS[bits >> 3 & 7], "flip": FLIP_PHASES[bits >> 6 & 3],
            "recovery": RECOVERY_STAGES[bits >> 8 & 3], "recovery_cause": RECOVERY_CAUSES[bits >> 10 & 7]}


def decode_reason(reason):
    """The status's reason byte (drone/main/runtime.h Status) as the stop reason's index and the recovery's bits as the guard field holds them (bits 8-9 the stage, 10-12 the cause): the stage in bits 6-7, the cause less one in bits 4-5 while one runs."""
    stage = reason >> 6
    return reason & 15, (stage << 8 | ((reason >> 4 & 3) + 1) << 10) if stage else 0


# The test pattern's channels in the drone's order (drone/main/flight_rt.c excite), and their units: roll and pitch targets and servo tilts in degrees (servos positive forward), the yaw-rate target in degrees/s, collective and motors in throttle, the hover hold point's speed toward the tag and to the drone's left in m/s.
PATTERN_CHANNELS = ("roll", "pitch", "yaw", "base", "left_servo", "right_servo", "left_motor", "right_motor", "forward", "left")
# Each channel's largest value either way, as the drone refuses past it (flight_rt.c excite_limit; test_bridge.py checks they agree). The roll and pitch targets went from 10 to 60 degrees on 2026-11-12 for the roll-step ladder.
PATTERN_LIMITS = (60, 60, 60, 150, 8, 8, 150, 150, 1, 1)


def pattern_refusal(pattern):
    """Why the drone would refuse this pattern's values ("A value is past its channel's limit"), or None."""
    for segment in pattern.get("segments", []):
        for name, limit in zip(PATTERN_CHANNELS, PATTERN_LIMITS):
            if abs(round(segment.get(name, 0) * 100) / 100) > limit:
                return f"{name} {segment[name]} is past its limit {limit}"
    return None


# The hover loop's state (fw/drone/main/flight_rt.c Record), NaN in recordings from before it was logged. In the yaw-turned level frame (x forward, y left, z up at hover_yaw_deg 0): the tag position the loop last took and that sighting's age, its velocity estimate; then height above the engage point and climb rate, the height, range and arc targets, the gyro heading, the trim and the drag model's velocity in body axes (x forward, y left). Tag and age are NaN without a sighting, range and arc targets before they are set (flags bit 7), and the drag velocity while the drag model is off (flags bit 5).
HOVER_COLUMNS = [
    "hover_x_m", "hover_y_m", "hover_z_m", "hover_tag_age_ms", "hover_vx_m_s", "hover_vy_m_s",
    "hover_height_m", "hover_climb_m_s", "hover_height_target_m", "hover_range_target_m", "hover_arc_deg",
    "hover_yaw_deg", "hover_trim_x_m_s2", "hover_trim_y_m_s2", "hover_drag_vx_m_s", "hover_drag_vy_m_s",
]
# The barometer's pressure altitude in the standard atmosphere (m; only its changes are height) and whether there was a reading in the last 100 ms (the altitude is NaN without one, and in recordings from before the barometer).
BARO_COLUMNS = ["baro_altitude_m", "baro_valid"]
# The guards (fw/drone/main/guard.h): the fly-away failsafe's phase (0 off, 1 brake, 2 hold, 3 land; FAILSAFE_PHASES), whether distance set it off, and why the test pattern stopped (0 it did not, 1 distance, 2 tag lost, 3 tilt, 4 stick, 5 landing, 6 recovery; PATTERN_STOPS), the roll flip's phase (0 none, 1 climbing, 2 spinning or righting, 3 arresting; FLIP_PHASES; 0 in recordings from before the flip), and a recovery's stage (0 none, 1 righting, 2 leveling; RECOVERY_STAGES) and cause (0 none, 1 past tip_limit, 2 a failing flip, 3 the gyro at full scale, 4 level-and-hold; RECOVERY_CAUSES; both 0 in recordings from before recover.c); NaN in recordings from before them.
GUARD_COLUMNS = ["failsafe", "failsafe_far", "pattern_stop", "flip", "recovery", "recovery_cause"]
# The Kalman filter's estimate (fw/drone/main/ekf.h), logged whether or not the hold flew it; NaN in recordings from before it. Roll and pitch as roll_deg and pitch_deg; yaw counterclockwise from north once ekf_aligned (before that from where the nose pointed when it started); velocity north, west and up; position north and west of where it last rested and height above it; the standard deviations of its horizontal velocity and height (2.55 at most); whether its heading is known, it is resting still, it is good enough for the hold to fly horizontally and vertically, and the hold flew on it that tick (hover_ekf); and the GPS solutions it has refused in a row.
EKF_COLUMNS = [
    "ekf_roll_deg", "ekf_pitch_deg", "ekf_yaw_deg", "ekf_north_m_s", "ekf_west_m_s", "ekf_up_m_s", "ekf_north_m", "ekf_west_m", "ekf_height_m",
    "ekf_velocity_sd_m_s", "ekf_height_sd_m", *("ekf_" + name for name in EKF_FLAGS), "ekf_rejected",
]
# The camera's newest dated frame: its sequence's low 16 bits, and how long before esp_time_us its center row was read (us; the drone truncates to 10 us, so the middle of that step).
CAMERA_COLUMNS = ["camera_sequence", "camera_age_us"]
# That frame's camera -> tag rotation as a unit quaternion (w, x, y, z; camera axes right, down, forward), NaN when the frame had no decided, trusted sighting and in recordings from before it was logged: the attitude's ground truth under a floor tag.
TAG_ROTATION_COLUMNS = ["tag_qw", "tag_qx", "tag_qy", "tag_qz"]
NAN = float("nan")
COLUMNS = [
    "session", "sequence", "esp_time_us", "ax_g", "ay_g", "az_g", "gx_deg_s", "gy_deg_s", "gz_deg_s", "temperature_c",
    "roll_deg", "pitch_deg", "left_throttle", "right_throttle", "left_servo_deg", "right_servo_deg",
    "target_roll_deg", "target_pitch_deg", "target_yaw_deg_s", "base_throttle", "control_us", "flags", "battery_v",
    *HOVER_COLUMNS, *BARO_COLUMNS, *GUARD_COLUMNS, *EKF_COLUMNS, *CAMERA_COLUMNS, *TAG_ROTATION_COLUMNS,
]


def hover_values(flags, values):
    """
    The hover columns from a record's raw hover fields, or all NaN for an older record without them.
    """
    if not values:
        return [NAN] * len(HOVER_COLUMNS)
    x, y, z, age, vx, vy, height, climb, height_target, range_target, arc, yaw, trim_x, trim_y, drag_x, drag_y = values
    seen, targeted, dragging = age != 65535, flags >> 7 & 1, flags >> 5 & 1
    return [
        *(value / 1000 if seen else NAN for value in (x, y, z)),
        age if seen else NAN,
        vx / 1000,
        vy / 1000,
        height / 1000,
        climb / 1000,
        height_target / 1000,
        range_target / 1000 if targeted else NAN,
        arc / 100 if targeted else NAN,
        yaw / 100,
        trim_x / 1000,
        trim_y / 1000,
        *(value / 1000 if dragging else NAN for value in (drag_x, drag_y)),
    ]


def ekf_values(values):
    """The Kalman filter's columns from a record's raw fields after the guards, or all NaN for an older record without them."""
    if not values:
        return [NAN] * len(EKF_COLUMNS)
    flags = values[11]
    return [
        *(value / 100 for value in values[:3]), *(value / 1000 for value in values[3:6]), *(value / 100 for value in values[6:8]), values[8] / 1000,
        *(value / 100 for value in values[9:11]), *(flags >> bit & 1 for bit in range(len(EKF_FLAGS))), flags >> 8,
    ]


def decode_records(data, session, signs, bias, counts, accel_counts):
    """
    A recording's records (its .bin, FLG1 batches laid end to end) as rows of COLUMNS in the order they were written, resent ones included: signs the sensor-to-body axes, bias the gyro's (deg/s), counts its counts per deg/s and accel_counts the accelerometer's per g, from the recording's metadata. Also whether it ends mid-batch. Raises ValueError for a batch that is not this session's or not FLG1.
    """
    rows, truncated, offset = [], False, 0
    while offset < len(data):
        if len(data) - offset < HEADER_SIZE:
            truncated = True
            break
        magic, batch_session, count, size = struct.unpack_from(HEADER_FORMAT, data, offset)
        if magic != b"FLG1" or batch_session != session or size not in FORMATS or not 1 <= count <= MAX_BATCH:
            raise ValueError("Invalid flight-log packet")
        offset += HEADER_SIZE
        end = min(offset + count * size, len(data))
        for start in range(offset, end - size + 1, size):
            fields = struct.unpack_from(FORMATS[size], data, start)
            stamp, seq, ax, ay, az, temperature, gx, gy, gz, roll, pitch = fields[:11]
            left, right, left_servo, right_servo, target_roll, target_pitch, target_yaw, base, control_us, flags, voltage_mv = fields[11:22]
            rows.append([
                session, seq, stamp,
                *(value * sign / accel_counts for value, sign in zip((ax, ay, az), signs)),
                *(value * sign / counts - zero for value, sign, zero in zip((gx, gy, gz), signs, bias)),
                temperature / 340 + 36.53,
                *(value / 100 for value in (roll, pitch)),
                left, right,
                *(value / 100 for value in (left_servo, right_servo, target_roll, target_pitch, target_yaw)),
                base, control_us, flags, voltage_mv / 1000,
                *hover_values(flags, fields[22:38]),
                *((fields[38] / 1000, True) if len(fields) > 38 and fields[38] != BARO_NONE else (NAN, False)),
                *((fields[39] & 3, fields[39] >> 2 & 1, fields[39] >> 3 & 7, fields[39] >> 6 & 3, fields[39] >> 8 & 3, fields[39] >> 10 & 7) if len(fields) > 39 else (NAN,) * 6),
                *ekf_values(fields[40:52]),
                *((fields[52], NAN if fields[53] == 65535 else fields[53] * 10 + 5) if len(fields) > 52 else (NAN, NAN)),
                *((value / 32767 for value in fields[54:58]) if len(fields) > 54 and any(fields[54:58]) else (NAN,) * 4),
            ])
        if end - offset != count * size:
            truncated = True
            break
        offset = end
    return rows, truncated
