"""
Decode a saved flight recording: uv run -m sw.flight_log sw/recordings/<flight>.bin
Writes a CSV beside the binary file and reports gaps or an incomplete final packet.
"""

import argparse
import json
import math
import struct
from pathlib import Path

import pandas as pd
from fw.controller.log_format import BARO_NONE, EKF_FLAGS, HEADER_FORMAT, HEADER_SIZE, FORMATS, MAX_BATCH

# The hover loop's state (fw/drone/main/flight_rt.c Record), NaN in recordings from before it was logged. In the yaw-turned level frame (x forward, y left, z up at hover_yaw_deg 0): the tag position the loop last took and that sighting's age, its velocity estimate; then height above the engage point and climb rate, the height, range and arc targets, the gyro heading, the trim and the drag model's velocity in body axes (x forward, y left). Tag and age are NaN without a sighting, range and arc targets before they are set (flags bit 7), and the drag velocity while the drag model is off (flags bit 5).
HOVER_COLUMNS = [
    "hover_x_m", "hover_y_m", "hover_z_m", "hover_tag_age_ms", "hover_vx_m_s", "hover_vy_m_s",
    "hover_height_m", "hover_climb_m_s", "hover_height_target_m", "hover_range_target_m", "hover_arc_deg",
    "hover_yaw_deg", "hover_trim_x_m_s2", "hover_trim_y_m_s2", "hover_drag_vx_m_s", "hover_drag_vy_m_s",
]
# The barometer's pressure altitude in the standard atmosphere (m; only its changes are height) and whether there was a reading in the last 100 ms (the altitude is NaN without one, and in recordings from before the barometer).
BARO_COLUMNS = ["baro_altitude_m", "baro_valid"]
# The guards (fw/drone/main/guard.h): the fly-away failsafe's phase (0 off, 1 brake, 2 hold, 3 land; log_format FAILSAFE_PHASES), whether distance set it off, and why the test pattern stopped (0 it did not, 1 distance, 2 tag lost, 3 tilt, 4 stick; PATTERN_STOPS); NaN in recordings from before them.
GUARD_COLUMNS = ["failsafe", "failsafe_far", "pattern_stop"]
# The Kalman filter's estimate (fw/drone/main/ekf.h), logged whether or not the hold flew it; NaN in recordings from before it. Roll and pitch as roll_deg and pitch_deg; yaw counterclockwise from north once ekf_aligned (before that from where the nose pointed when it started); velocity north, west and up; position north and west of where it last rested and height above it; the standard deviations of its horizontal velocity and height (2.55 at most); whether its heading is known, it is resting still, it is good enough for the hold to fly horizontally and vertically, and the hold flew on it that tick (hover_ekf); and the GPS solutions it has refused in a row.
EKF_COLUMNS = [
    "ekf_roll_deg", "ekf_pitch_deg", "ekf_yaw_deg", "ekf_north_m_s", "ekf_west_m_s", "ekf_up_m_s", "ekf_north_m", "ekf_west_m", "ekf_height_m",
    "ekf_velocity_sd_m_s", "ekf_height_sd_m", *("ekf_" + name for name in EKF_FLAGS), "ekf_rejected",
]
NAN = float("nan")


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


# The first drone's ID (its station MAC, fw/pairs.h): recordings made before the start records carried one (UTC names before IDS_FROM) are all from it.
FIRST_DRONE = "20:50:0d:8b:7e:68"
IDS_FROM = "2026-11-04"
RECORDINGS = Path(__file__).with_name("recordings")


def recording(name):
    """The recording in sw/recordings whose file name starts with name (its UTC time, e.g. 2026-11-09T06-38-51)."""
    paths = sorted(RECORDINGS.glob(name + "*.bin"))
    if not paths:
        raise FileNotFoundError(f"no recording {name}*.bin in {RECORDINGS}")
    return paths[0]


def events(path):
    """A recording's events (the .jsonl beside its .bin): the session's start, the drone's metadata, settings changes, commands, GPS solutions, test patterns and the stop."""
    return [json.loads(line) for line in Path(path).with_suffix(".jsonl").read_text().splitlines()]


def metadata(path):
    """The start event, plus the drone's metadata when it arrived after the session started (as from 2026-11-09)."""
    merged = {}
    for event in events(path):
        if event["type"] in ("start", "metadata"):
            merged = event | merged
    return merged


def settings(path):
    """The settings a recording was flown on: the start's, then every change during it."""
    merged = {}
    for event in events(path):
        merged |= event.get("settings", {})
    return merged


def parse_settings(text):
    """Settings given on a command line as name=value,name=value."""
    return {name: float(value) for name, value in (item.split("=") for item in text.split(",") if item)}


def test_pattern(path):
    """The last test pattern a recording played (sw/excite.py), or None."""
    patterns = [event for event in events(path) if event.get("type") == "pattern"]
    return patterns[-1] if patterns else None


def drone(path):
    """The physical drone a recording was flown by (its station MAC), from its start record; None when a later recording does not say."""
    path = Path(path)
    for event in events(path):
        if event["type"] == "start":
            return event.get("drone") or (FIRST_DRONE if path.name < IDS_FROM else None)
    return FIRST_DRONE if path.name < IDS_FROM else None


def ekf_values(values):
    """The Kalman filter's columns from a record's raw fields after the guards, or all NaN for an older record without them."""
    if not values:
        return [NAN] * len(EKF_COLUMNS)
    flags = values[11]
    return [
        *(value / 100 for value in values[:3]), *(value / 1000 for value in values[3:6]), *(value / 100 for value in values[6:8]), values[8] / 1000,
        *(value / 100 for value in values[9:11]), *(flags >> bit & 1 for bit in range(len(EKF_FLAGS))), flags >> 8,
    ]


def read_flight(path):
    meta = metadata(path)
    bias = [math.degrees(value) for value in meta["gyro_bias_rad_s"]]
    signs = meta["sensor_to_body"]
    rows = []
    truncated = False
    with path.open("rb") as file:
        while header := file.read(HEADER_SIZE):
            if len(header) != HEADER_SIZE:
                truncated = True
                break
            magic, session, count, size = struct.unpack(HEADER_FORMAT, header)
            if magic != b"FLG1" or session != meta["session"] or size not in FORMATS or not 1 <= count <= MAX_BATCH:
                raise ValueError("Invalid flight-log packet")
            payload = file.read(count * size)
            for offset in range(0, len(payload) - size + 1, size):
                fields = struct.unpack_from(FORMATS[size], payload, offset)
                stamp, seq, ax, ay, az, temperature, gx, gy, gz, roll, pitch = fields[:11]
                left, right, left_servo, right_servo, target_roll, target_pitch, target_yaw, base, control_us, flags, voltage_mv = fields[11:22]
                rows.append([
                    session, seq, stamp,
                    *(value * sign / 16384 for value, sign in zip((ax, ay, az), signs)),
                    *(value * sign / 131 - zero for value, sign, zero in zip((gx, gy, gz), signs, bias)),
                    temperature / 340 + 36.53,
                    *(value / 100 for value in (roll, pitch)),
                    left, right,
                    *(value / 100 for value in (left_servo, right_servo, target_roll, target_pitch, target_yaw)),
                    base, control_us, flags, voltage_mv / 1000,
                    *hover_values(flags, fields[22:38]),
                    *((fields[38] / 1000, True) if len(fields) > 38 and fields[38] != BARO_NONE else (NAN, False)),
                    *((fields[39] & 3, fields[39] >> 2 & 1, fields[39] >> 3 & 7) if len(fields) > 39 else (NAN, NAN, NAN)),
                    *ekf_values(fields[40:]),
                ])
            if len(payload) != count * size:
                truncated = True
                break

    columns = [
        "session", "sequence", "esp_time_us", "ax_g", "ay_g", "az_g", "gx_deg_s", "gy_deg_s", "gz_deg_s", "temperature_c",
        "roll_deg", "pitch_deg", "left_throttle", "right_throttle", "left_servo_deg", "right_servo_deg",
        "target_roll_deg", "target_pitch_deg", "target_yaw_deg_s", "base_throttle", "control_us", "flags", "battery_v",
        *HOVER_COLUMNS, *BARO_COLUMNS, *GUARD_COLUMNS, *EKF_COLUMNS,
    ]
    data = pd.DataFrame(rows, columns=columns).drop_duplicates(["session", "sequence"])
    return data, truncated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    args = parser.parse_args()
    data, truncated = read_flight(args.file)
    output = args.file.with_suffix(".csv")
    data.to_csv(output, index=False)
    gaps = int(data.sequence.diff().sub(1).clip(lower=0).sum())
    print(f"Saved {output}: {len(data)} samples, {gaps} missing between samples, truncated tail: {truncated}")


if __name__ == "__main__":
    main()
