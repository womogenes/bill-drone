"""
Antenna comparison with fw/antenna_test on an ESP32 board behind a DevKit's USB-serial: uv run -m sw.antenna_test LABEL [--seconds 30] [--port /dev/ttyUSB0]

Reads the board's scans for --seconds and prints each network's median signal strength (dBm) and spread, then appends them under LABEL to sw/recordings/antenna/results.jsonl, so runs with different antennas at the same spot compare directly. The port opens with RTS asserted, which holds the DevKit's own ESP32 in reset so only the tested board talks on the line.
"""

import argparse
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import serial

RESULTS = Path(__file__).resolve().parent / "recordings" / "antenna" / "results.jsonl"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("label", help="what is being tested, e.g. onboard, ufl-dipole-vertical")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, 115200, 0.2
    port.dtr, port.rts = False, True
    port.open()
    time.sleep(0.5)
    port.reset_input_buffer()
    readings, resets, buffer = {}, [], b""
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        buffer += port.read(4096)
        *lines, buffer = buffer.split(b"\n")
        for line in lines:
            text = line.decode(errors="replace").strip()
            if match := re.fullmatch(r"S \d+ (.+) (\d+) (-\d+)", text):
                readings.setdefault(match[1], []).append(int(match[3]))
            elif re.fullmatch(r"R \d+", text) or "Brownout" in text:
                resets.append(text)
    port.close()
    if not readings:
        raise SystemExit("no scans: is the board running fw/antenna_test (IO0 jumper out, RST pressed)?")
    summary = {ssid: {"median": float(np.median(values)), "spread": float(np.percentile(values, 90) - np.percentile(values, 10)), "scans": len(values)}
               for ssid, values in sorted(readings.items(), key=lambda item: -np.median(item[1]))}
    print(f"{args.label}: {args.seconds:.0f} s" + (f", resets: {resets}" if resets else ""))
    for ssid, row in summary.items():
        print(f"  {ssid:20s} {row['median']:6.1f} dBm  (10-90% spread {row['spread']:.0f} dB, {row['scans']} scans)")
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as out:
        out.write(json.dumps({"label": args.label, "time": datetime.now(timezone.utc).isoformat(timespec="seconds"), "seconds": args.seconds, "resets": resets, "networks": summary}) + "\n")


if __name__ == "__main__":
    main()
