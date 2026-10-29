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
from fw.controller.log_format import HEADER_FORMAT, HEADER_SIZE, FORMATS, MAX_BATCH

# The hover loop's state (fw/drone/main/flight_rt.c Record), NaN in recordings from before it was logged. In the yaw-turned level frame (x forward, y left, z up at hover_yaw_deg 0): the tag position the loop last took and that sighting's age, its velocity estimate; then height above the engage point and climb rate, the height, range and arc targets, the gyro heading, the trim and the drag model's velocity in body axes (x forward, y left). Tag and age are NaN without a sighting, range and arc targets before they are set (flags bit 7), and the drag velocity while the drag model is off (flags bit 5).
HOVER_COLUMNS = [
    "hover_x_m", "hover_y_m", "hover_z_m", "hover_tag_age_ms", "hover_vx_m_s", "hover_vy_m_s",
    "hover_height_m", "hover_climb_m_s", "hover_height_target_m", "hover_range_target_m", "hover_arc_deg",
    "hover_yaw_deg", "hover_trim_x_m_s2", "hover_trim_y_m_s2", "hover_drag_vx_m_s", "hover_drag_vy_m_s",
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


def read_flight(path):
    # The start line, plus the drone's metadata when it arrived after the session started.
    metadata = {}
    for line in path.with_suffix(".jsonl").read_text().splitlines():
        event = json.loads(line)
        if event["type"] in ("start", "metadata"):
            metadata = event | metadata
    bias = [math.degrees(value) for value in metadata["gyro_bias_rad_s"]]
    signs = metadata["sensor_to_body"]
    rows = []
    truncated = False
    with path.open("rb") as file:
        while header := file.read(HEADER_SIZE):
            if len(header) != HEADER_SIZE:
                truncated = True
                break
            magic, session, count, size = struct.unpack(HEADER_FORMAT, header)
            if magic != b"FLG1" or session != metadata["session"] or size not in FORMATS or not 1 <= count <= MAX_BATCH:
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
                    *hover_values(flags, fields[22:]),
                ])
            if len(payload) != count * size:
                truncated = True
                break

    columns = [
        "session", "sequence", "esp_time_us", "ax_g", "ay_g", "az_g", "gx_deg_s", "gy_deg_s", "gz_deg_s", "temperature_c",
        "roll_deg", "pitch_deg", "left_throttle", "right_throttle", "left_servo_deg", "right_servo_deg",
        "target_roll_deg", "target_pitch_deg", "target_yaw_deg_s", "base_throttle", "control_us", "flags", "battery_v",
        *HOVER_COLUMNS,
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
