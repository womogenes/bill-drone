"""
Find the ESP32-CAM on any network this laptop is on and update it over Wi-Fi: make camera-wifi (from src/fw).

Tries bill-esp32cam.local, then asks every address on each local IPv4 network (up to /22, so a laptop hotspot's or phone hotspot's whole subnet) for the camera's /status. Builds and uploads with PlatformIO's espota (its progress prints as it goes), then waits for the camera to come back and prints its status. The camera pauses tracking and reboots, so update with the drone disarmed.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

ESP32CAM = Path(__file__).resolve().parents[1] / "esp32cam"
NAME = "bill-esp32cam.local"


def status(host, timeout=1.0):
    """The camera's /status as a dict, or None when host is not the camera."""
    try:
        with urllib.request.urlopen(f"http://{host}/status", timeout=timeout) as reply:
            data = json.loads(reply.read())
        return data if "camera" in data else None
    except (OSError, ValueError):
        return None


def networks():
    """The IPv4 networks of this laptop's interfaces that are up, except loopback and tunnels, largest first capped at /22."""
    found = []
    for interface in json.loads(subprocess.run(["ip", "-j", "-4", "addr"], capture_output=True, text=True, check=True).stdout):
        if interface["ifname"] == "lo" or "POINTOPOINT" in interface.get("flags", []) or "UP" not in interface.get("flags", []):
            continue
        for address in interface.get("addr_info", []):
            network = ipaddress.ip_interface(f"{address['local']}/{max(address['prefixlen'], 22)}").network
            found.append((interface["ifname"], address["local"], network))
    return found


def discover():
    try:
        address = socket.gethostbyname(NAME)
        if status(address, 3):
            print(f"camera at {address} ({NAME})")
            return address
    except OSError:
        pass
    for name, own, network in networks():
        hosts = [str(h) for h in network.hosts() if str(h) != own]
        print(f"scanning {network} on {name} ({len(hosts)} addresses)")
        with ThreadPoolExecutor(64) as pool:
            for host, found in zip(hosts, pool.map(status, hosts)):
                if found:
                    print(f"camera at {host}")
                    return host
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="the camera's address, skipping discovery")
    parser.add_argument("--find", action="store_true", help="only find the camera")
    args = parser.parse_args()
    host = args.host or discover()
    if not host:
        raise SystemExit("no camera found: is it powered, and is this laptop on the network in src/fw/esp32cam/src/wifi.h?")
    before = status(host, 3) or {}
    print(f"camera {before.get('camera')}, tag_id {before.get('tag_id')}")
    if args.find:
        return
    upload = ["uvx", "--from", "platformio==6.1.18", "--with", "intelhex", "pio", "run", "-d", str(ESP32CAM), "-e", "ota", "-t", "upload", "--upload-port", host]
    subprocess.run(upload, check=True)
    print("waiting for the camera to reboot", end="", flush=True)
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        time.sleep(2)
        after = status(host, 2)
        if after:
            print(f"\nback: camera {after.get('camera')}, tag_id {after.get('tag_id')} (-1 until it sees a tag)")
            return
        print(".", end="", flush=True)
    raise SystemExit("\nthe camera did not come back within 60 s")


if __name__ == "__main__":
    main()
