"""
Decode a saved flight recording: uv run -m sw.flight_log sw/recordings/<flight>.bin
Writes a CSV beside the binary file and reports gaps or an incomplete final packet.
"""

import argparse
import json
import math
from pathlib import Path

import pandas as pd
from fw.controller.log_format import COLUMNS, decode_records

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


def read_flight(path):
    meta = metadata(path)
    bias = [math.degrees(value) for value in meta["gyro_bias_rad_s"]]
    signs = meta["sensor_to_body"]
    # The gyro's counts per deg/s: 131 (±250 deg/s) in recordings from before the metadata said.
    counts = meta.get("gyro_counts_per_deg_s", 131)
    rows, truncated = decode_records(path.read_bytes(), meta["session"], signs, bias, counts)
    data = pd.DataFrame(rows, columns=COLUMNS).drop_duplicates(["session", "sequence"])
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
