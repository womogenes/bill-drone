"""
Antenna comparison with fw/antenna_test on an ESP32 board behind a DevKit's USB-serial: uv run -m sw.antenna_test LABEL [--seconds 30] [--port /dev/ttyUSB0]

Reads the board's scans for --seconds and prints each network's median signal strength (dBm) and spread, then appends them under LABEL to sw/recordings/antenna/results.jsonl, so runs with different antennas at the same spot compare directly. Each run also prints its change from the previous run: the median over every network heard in both, which a fade on one network (a small move of the board) barely shifts. The port opens with RTS asserted, which holds the DevKit's own ESP32 in reset so only the tested board talks on the line.
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
    readings, seen, resets, buffer = {}, set(), [], b""
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        buffer += port.read(4096)
        *lines, buffer = buffer.split(b"\n")
        for line in lines:
            text = line.decode(errors="replace").strip()
            # Once per network per scan: a glitching line has repeated the same reading thousands of times (2026-11-14), and garbled names fall out by not ending in a channel and RSSI.
            if (match := re.fullmatch(r"S (\d+) ([ -~]+) (\d{1,2}) (-\d{2,3})", text)) and (match[1], match[2]) not in seen:
                seen.add((match[1], match[2]))
                readings.setdefault(match[2], []).append(int(match[4]))
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
    previous = json.loads(RESULTS.read_text().splitlines()[-1]) if RESULTS.exists() and RESULTS.read_text().strip() else None
    if previous:
        shared = [ssid for ssid in summary if ssid in previous["networks"]]
        if shared:
            changes = [summary[ssid]["median"] - previous["networks"][ssid]["median"] for ssid in shared]
            print(f"against {previous['label']}: {np.median(changes):+.1f} dB median over {len(shared)} networks (10-90%: {np.percentile(changes, 10):+.0f} to {np.percentile(changes, 90):+.0f})")
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as out:
        out.write(json.dumps({"label": args.label, "time": datetime.now(timezone.utc).isoformat(timespec="seconds"), "seconds": args.seconds, "resets": resets, "networks": summary}) + "\n")


if __name__ == "__main__":
    main()
