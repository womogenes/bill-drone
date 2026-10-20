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
from fw.controller.log_format import HEADER_FORMAT, HEADER_SIZE, SAMPLE_FORMAT, RECORD_SIZE, MAX_BATCH


def read_flight(path):
    metadata = json.loads(path.with_suffix(".jsonl").read_text().splitlines()[0])
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
            if (
                magic != b"FLG1"
                or session != metadata["session"]
                or size != RECORD_SIZE
                or not 1 <= count <= MAX_BATCH
            ):
                raise ValueError("Invalid flight-log packet")
            payload = file.read(count * size)
            for offset in range(0, len(payload) - size + 1, size):
                (
                    stamp,
                    seq,
                    ax,
                    ay,
                    az,
                    temperature,
                    gx,
                    gy,
                    gz,
                    roll,
                    pitch,
                    left,
                    right,
                    left_servo,
                    right_servo,
                    target_roll,
                    target_pitch,
                    target_yaw,
                    base,
                    control_us,
                    flags,
                    voltage_mv,
                ) = struct.unpack_from(SAMPLE_FORMAT, payload, offset)
                accel = [
                    value * sign / 16384 for value, sign in zip((ax, ay, az), signs)
                ]
                gyro = [
                    value * sign / 131 - zero
                    for value, sign, zero in zip((gx, gy, gz), signs, bias)
                ]
                rows.append(
                    [
                        session,
                        seq,
                        stamp,
                        *accel,
                        *gyro,
                        temperature / 340 + 36.53,
                        roll / 100,
                        pitch / 100,
                        left,
                        right,
                        left_servo / 100,
                        right_servo / 100,
                        target_roll / 100,
                        target_pitch / 100,
                        target_yaw / 100,
                        base,
                        control_us,
                        flags,
                        voltage_mv / 1000,
                    ]
                )
            if len(payload) != count * size:
                truncated = True
                break

    columns = [
        "session",
        "sequence",
        "esp_time_us",
        "ax_g",
        "ay_g",
        "az_g",
        "gx_deg_s",
        "gy_deg_s",
        "gz_deg_s",
        "temperature_c",
        "roll_deg",
        "pitch_deg",
        "left_throttle",
        "right_throttle",
        "left_servo_deg",
        "right_servo_deg",
        "target_roll_deg",
        "target_pitch_deg",
        "target_yaw_deg_s",
        "base_throttle",
        "control_us",
        "flags",
        "battery_v",
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
    print(
        f"Saved {output}: {len(data)} samples, {gaps} missing between samples, truncated tail: {truncated}"
    )


if __name__ == "__main__":
    main()
