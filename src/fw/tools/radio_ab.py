# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Compares the drone's link on its own radio against the S3 camera's relay (radio_via_camera, fw/relay_link.h) from where the drone sits now: measures the link on its own radio, switches to the relay with the camera's transmit power at --power, measures again, switches back and prints the two side by side.

uv run tools/radio_ab.py [--seconds 30] [--power 84]

The drone disarmed on the ground, the laptop service running. Each switch stores the setting, waits for it to reach flash (1 s after the last change) and restarts the drone; both boards follow within seconds. --power is in quarter dBm (84 = 21 dBm, the most the camera takes; it kept 56, 14 dBm, before): full power at the camera's Wi-Fi start browned it out once (2026-10-26, before its capacitors), so the camera's resets are reported too; it goes back to 14 dBm at the end. On the bench on 2026-11-14 the relay was about 15 dB down at 14 dBm.
"""

import argparse
import json
import statistics
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_direct import Drone  # noqa: E402

BRIDGE = "http://127.0.0.1:8765"
# The camera's own transmit power (quarter dBm, ota.h radioPower), restored at the end.
NORMAL_POWER = 56
# LNK1 reports carry flag 4 while the drone's link goes through the camera.
RELAY_FLAG = 4


def status():
    return json.load(urllib.request.urlopen(f"{BRIDGE}/status", timeout=3))


def store(value):
    request = urllib.request.Request(f"{BRIDGE}/settings", json.dumps({"settings": {"radio_via_camera": value}}).encode(), {"Content-Type": "application/json"})
    stored = json.load(urllib.request.urlopen(request, timeout=5))["settings"]
    if stored.get("radio_via_camera") != value:
        raise SystemExit(f"radio_via_camera not stored as {value}")


def relayed():
    """Whether the drone's link goes through the camera now, or None while no report says."""
    try:
        drone = (status().get("reading") or {}).get("link", {}).get("drone")
    except OSError:
        return None
    return None if not drone else bool(drone.get("flags", 0) & RELAY_FLAG)


def switch(drone, relay, power):
    if relay:
        print(f"camera transmit power: {drone.request(b'K', b'T' + bytes([power])).decode(errors='replace')}")
    store(int(relay))
    # The setting reaches flash 1 s after the last change; a restart before that comes back in the old mode (2026-11-14).
    time.sleep(2)
    drone.request(b"I")
    drone.restart()
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        time.sleep(1)
        if relayed() is relay:
            # Let both boards settle before measuring.
            time.sleep(5)
            return
    raise SystemExit(f"the drone did not come back {'through the camera' if relay else 'on its own radio'} within 60 s")


def measure(seconds):
    """The link's reports over seconds: what each end hears of the other, latency, failed sends and gaps."""
    rows = {"drone hears controller (dBm)": [], "controller hears drone (dBm)": [], "latency (ms)": []}
    failed = gaps = sent = 0
    camera_resets = None
    seen = set()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        reading = status().get("reading") or {}
        link = reading.get("link", {})
        for source, label in (("drone", "drone hears controller (dBm)"), ("controller", "controller hears drone (dBm)")):
            report = link.get(source)
            # One report a second from each end; count each once.
            if report and (source, report.get("rx"), report.get("tx_ok"), report.get("silence_ms")) not in seen:
                seen.add((source, report.get("rx"), report.get("tx_ok"), report.get("silence_ms")))
                rows[label].append(report["rssi"])
                if source == "drone":
                    rows["latency (ms)"].append(report.get("latency_ms", 0))
                    failed += report.get("tx_fail", 0)
                    sent += report.get("tx_ok", 0) + report.get("tx_fail", 0)
                gaps += report.get("gaps", 0)
        camera = reading.get("camera") or {}
        camera_resets = camera.get("resets", camera_resets)
        time.sleep(0.5)
    summary = {label: statistics.median(values) if values else None for label, values in rows.items()}
    summary["failed sends"] = f"{failed} of {sent}"
    summary["gaps"] = gaps
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--power", type=int, default=84, help="camera transmit power in quarter dBm, 8-84")
    args = parser.parse_args()
    if relayed() is None:
        raise SystemExit("no link report from the drone: is it on and the service running?")
    drone = Drone("ws://127.0.0.1:8765/update")
    results = {}
    try:
        if relayed():
            switch(drone, False, args.power)
        print(f"own radio: measuring {args.seconds:.0f} s")
        results["own radio"] = measure(args.seconds)
        print("switching to the camera's relay")
        switch(drone, True, args.power)
        print(f"through the camera: measuring {args.seconds:.0f} s")
        results["through the camera"] = measure(args.seconds)
    finally:
        print("switching back to the drone's own radio")
        switch(drone, False, args.power)
        print(f"camera transmit power back to {drone.request(b'K', b'T' + bytes([NORMAL_POWER])).decode(errors='replace')}")
    labels = list(next(iter(results.values())))
    print(f"\n{'':32s}" + "".join(f"{name:>22s}" for name in results))
    for label in labels:
        print(f"{label:32s}" + "".join(f"{str(result[label]):>22s}" for result in results.values()))


if __name__ == "__main__":
    main()
