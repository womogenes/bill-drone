"""
Find the ESP32-CAM on any network this laptop is on and update it over Wi-Fi: make camera-wifi [NAME=michael-drone] (from src/fw). With --join (make camera-fast), first has the drone's camera join the update network for one boot (its K J request, fw/esp32cam/src/ota.h: the repository's .env network, which this laptop is on), so an update takes seconds without wiring and without its usual network list; the camera's name comes from the drone (K N), and afterwards the camera's running image is checked through the drone, since it goes back to its radio-only mode.

Each camera has a name by its MAC (fw/esp32cam/src/ota.h cameraNames), bill-esp32cam unless --name says otherwise. Tries NAME.local, then asks every address on each local IPv4 network (up to /22, so a laptop hotspot's or phone hotspot's whole subnet) for a camera's /status with that name (a camera from before names, without one, is taken as bill-esp32cam). Builds and uploads with PlatformIO's espota (its progress prints as it goes), then waits for the camera to come back and prints its status. The camera pauses tracking and reboots, so update with the drone disarmed.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ESP32CAM = Path(__file__).resolve().parents[1] / "esp32cam"
DEFAULT_NAME = "bill-esp32cam"


def status(host, timeout=1.0, name=None):
    """The camera's /status as a dict, or None when host is not a camera (or not the one named)."""
    try:
        with urllib.request.urlopen(f"http://{host}/status", timeout=timeout) as reply:
            data = json.loads(reply.read())
        if "camera" not in data or (name and data.get("name", DEFAULT_NAME) != name):
            return None
        return data
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


def discover(name):
    try:
        address = socket.gethostbyname(f"{name}.local")
        if status(address, 3, name):
            print(f"camera at {address} ({name}.local)")
            return address
    except OSError:
        pass
    for interface, own, network in networks():
        hosts = [str(h) for h in network.hosts() if str(h) != own]
        print(f"scanning {network} on {interface} ({len(hosts)} addresses)")
        with ThreadPoolExecutor(64) as pool:
            for host, found in zip(hosts, pool.map(lambda host: status(host, name=name), hosts)):
                if found:
                    print(f"camera at {host}")
                    return host
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="the camera's address, skipping discovery")
    parser.add_argument("--find", action="store_true", help="only find the camera")
    parser.add_argument("--name", default=DEFAULT_NAME, help="the camera's name (fw/esp32cam/src/ota.h cameraNames)")
    parser.add_argument("--env", default="s3ota", help="the PlatformIO environment to upload (s3ota for the S3 camera, ota for the old ESP32-CAM)")
    parser.add_argument("--join", action="store_true", help="have the drone's camera join the update network first (K J), then check its image through the drone")
    args = parser.parse_args()
    drone = None
    if args.join:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from ota_direct import Drone
        drone = Drone("ws://127.0.0.1:8765/update")
        args.name = drone.request(b"K", b"N").decode().split(" ")[1]
        print(f"asking {args.name} to join the update network")
        drone.request(b"K", b"J")
        drone.ws.close()
        drone.ws = None
    host = args.host
    # A camera that just restarted onto the network takes a few seconds to join and answer.
    deadline = time.monotonic() + (60 if args.join else 0)
    while not host:
        host = discover(args.name)
        if host or time.monotonic() > deadline:
            break
        time.sleep(2)
    if not host:
        raise SystemExit("no camera found: is it powered, and is this laptop on the network in src/fw/esp32cam/src/wifi.h?")
    before = status(host, 3) or {}
    print(f"camera {before.get('name', DEFAULT_NAME)}: {before.get('camera')}, tag_id {before.get('tag_id')}")
    if args.find:
        return
    upload = ["uvx", "--from", "platformio==6.1.18", "--with", "intelhex", "pio", "run", "-d", str(ESP32CAM), "-e", args.env, "-t", "upload", "--upload-port", host]
    subprocess.run(upload, check=True)
    if drone:
        # Back in its usual mode the camera is off the network; the drone still reaches it over its wire.
        print("waiting for the camera to answer the drone", end="", flush=True)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            time.sleep(2)
            try:
                print(f"\nback: {drone.request(b'K', b'I', patience=3).decode(errors='replace')}")
                return
            except (TimeoutError, RuntimeError):
                print(".", end="", flush=True)
        raise SystemExit("\nthe camera did not answer the drone within 60 s")
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
