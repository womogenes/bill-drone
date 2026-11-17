"""
Decode a saved flight recording: uv run -m sw.flight_log sw/recordings/<flight>.bin
Writes a CSV beside the binary file and reports gaps or an incomplete final packet.
"""

import argparse
import json
import math
import re
from pathlib import Path

import pandas as pd
from fw.controller.log_format import COLUMNS, decode_records

# The first drone's ID (its station MAC, fw/pairs.h): recordings made before the start records carried one (UTC names before IDS_FROM) are all from it.
FIRST_DRONE = "20:50:0d:8b:7e:68"
IDS_FROM = "2026-11-04"
RECORDINGS = Path(__file__).with_name("recordings")
SETTINGS_DEF = (Path(__file__).resolve().parents[1] / "fw/drone/main/settings.def").read_text()
# The airframe's authorities (settings.def roll_authority, pitch_authority, yaw_authority) and their defaults; the model recordings before them were flown on, with its defaults; the gains in rad/s² each authority scales (settings.c legacy_scale).
AUTHORITIES = {name: float(re.search(rf"TUNE\({name}, [^)]*, ([0-9.]+)\)", SETTINGS_DEF).group(1)) for name in ("roll_authority", "pitch_authority", "yaw_authority")}
LEGACY_MODEL = {"arm": 0.172, "pivot_height": 0.0253692458911615, "roll_inertia": 0.0118483264434811, "pitch_inertia": 0.0023030647846826, "yaw_inertia": 0.0106709253540914}
LEGACY_GAINS = {"roll_authority": ("roll_p", "roll_i", "roll_damping", "roll_bias"), "pitch_authority": ("pitch_p", "pitch_i", "pitch_damping", "pitch_bias", "pitch_speed"), "yaw_authority": ("yaw_damping", "yaw_i", "yaw_bias")}


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
    """The settings a recording was flown on: the start's, then every change during it, as the firmware now reads them (migrated)."""
    merged = {}
    for event in events(path):
        merged |= event.get("settings", {})
    return migrated(merged)


def migrated(settings):
    """Settings as the firmware reads a blob from before the airframe's authorities (settings.c legacy_scale): the authorities' defaults, and each gain scaled by its authority over what the older model gave the controller (arm / roll_inertia, pivot_height / pitch_inertia, arm / yaw_inertia), so the controller asks the motors and servos for what it did. Settings without the older model, or with an authority, are as given."""
    if not any(key in settings for key in LEGACY_MODEL) or any(key in settings for key in AUTHORITIES):
        return settings
    m = LEGACY_MODEL | {key: settings[key] for key in LEGACY_MODEL if key in settings}
    before = {"roll_authority": m["arm"] / m["roll_inertia"], "pitch_authority": m["pivot_height"] / m["pitch_inertia"], "yaw_authority": m["arm"] / m["yaw_inertia"]}
    out = {key: value for key, value in settings.items() if key not in LEGACY_MODEL and key not in ("com_x", "reaction_torque_ratio")} | AUTHORITIES
    for authority, names in LEGACY_GAINS.items():
        for name in names:
            if name in out:
                out[name] *= AUTHORITIES[authority] / before[authority]
    return out


def parse_settings(text):
    """Settings given on a command line as name=value,name=value."""
    return {name: float(value) for name, value in (item.split("=") for item in text.split(",") if item)}


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
    # The accelerometer's counts per g: 16384 (±2 g) in recordings from before the metadata said.
    rows, truncated = decode_records(path.read_bytes(), meta["session"], signs, bias, counts, meta.get("accel_counts_per_g", 16384))
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
