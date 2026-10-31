# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Finds the best channel for the camera's FPV radio here: moves the camera (the drone's K H request, fw/esp32cam/src/update.h) to each candidate channel, tunes the monitor-mode adapter there and counts what it hears, then leaves the camera on the best (it keeps the channel across restarts; sw/fpv_monitor.py finds it). The drone's own link is on channel 6; its status rate is shown for each channel too.

uv run tools/fpv_channels.py [--channels 3 9 11 12 13 1] [--seconds 5]

Needs the dashboard (which asks the camera for FPV), the drone disarmed or not, and dumpcap on the adapter (the wireshark group). On 2026-10-31 indoors: channel 11 heard 20-33% of shards, 13 44-52%, 9 59-71%, 3 57-68%.
"""

import argparse
import json
import struct
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_direct import Drone  # noqa: E402

CAPTURE = "/tmp/fpv_channels.pcapng"
# Radiotap fields before the antenna signal (5): their sizes and alignments.
FIELDS = {0: (8, 8), 1: (1, 1), 2: (1, 1), 3: (4, 2), 4: (2, 2), 5: (1, 1)}


def signal(packet):
    present = struct.unpack_from("<I", packet, 4)[0]
    offset, word = 8, present
    while word & 0x80000000:
        word = struct.unpack_from("<I", packet, offset)[0]
        offset += 4
    for field in range(6):
        if not present & (1 << field):
            continue
        size, align = FIELDS[field]
        offset = (offset + align - 1) // align * align
        if field == 5:
            return struct.unpack_from("b", packet, offset)[0]
        offset += size
    return None


def measure(seconds):
    """The camera's frames a second, the share of its shards heard, the frames a second that can be rebuilt and the median signal (dBm), over seconds of capture; None if nothing was heard."""
    subprocess.run(["timeout", str(seconds + 3), "dumpcap", "-q", "-i", "wlan1", "-a", f"duration:{seconds}", "-w", CAPTURE], capture_output=True)
    data = Path(CAPTURE).read_bytes()
    position, frames, signals = 0, {}, []
    while position + 12 <= len(data):
        kind, size = struct.unpack_from("<II", data, position)
        if kind == 6:
            captured = struct.unpack_from("<I", data, position + 20)[0]
            packet = data[position + 28:position + 28 + captured]
            frame = packet[struct.unpack_from("<H", packet, 2)[0]:]
            if len(frame) > 48 and frame[16:22] == b"FPVRAW" and frame[28:32] == b"FPV2":
                number, index, total, needed = struct.unpack_from("<HBBB", frame, 32)
                frames.setdefault(number, [total, needed, set()])[2].add(index)
                strength = signal(packet)
                if strength is not None:
                    signals.append(strength)
        position += size
    if not frames:
        return None
    numbers = sorted(frames)
    span = numbers[-1] - numbers[0] + 1
    heard = sum(len(f[2]) for f in frames.values()) / sum(f[0] for f in frames.values())
    rebuilt = sum(1 for f in frames.values() if len(f[2]) >= f[1])
    signals.sort()
    return span / seconds, heard, rebuilt / seconds, signals[len(signals) // 2] if signals else 0


def status_rate(seconds):
    seen, last, start = 0, None, time.time()
    while time.time() - start < seconds:
        reading = json.load(urllib.request.urlopen("http://127.0.0.1:8765/status"))["reading"]["time"]
        seen += reading != last
        last = reading
        time.sleep(0.01)
    return seen / seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--channels", type=int, nargs="+", default=[3, 9, 11, 12, 13, 1])
    parser.add_argument("--seconds", type=float, default=5)
    args = parser.parse_args()
    drone = Drone("ws://127.0.0.1:8765/update")
    stream = subprocess.Popen(["curl", "-s", "-m", "600", "-o", "/dev/null", "http://localhost:3000/api/camera?fpv=1"])
    time.sleep(4)
    best = None
    try:
        for channel in args.channels:
            drone.request(b"K", b"H" + bytes([channel]), patience=10)
            subprocess.run(["timeout", "5", "dumpcap", "-k", f"{2407 + 5 * channel},HT20", "-i", "wlan1"], capture_output=True)
            time.sleep(1.5)
            result = measure(args.seconds)
            rate = status_rate(3)
            if not result:
                print(f"channel {channel:2d}: nothing heard | drone status {rate:4.1f}/s", flush=True)
                continue
            print(f"channel {channel:2d}: camera {result[0]:4.1f} fps, shards heard {result[1]:4.0%}, rebuilt {result[2]:4.1f} fps, "
                  f"signal {result[3]} dBm | drone status {rate:4.1f}/s", flush=True)
            if best is None or result[2] > best[1]:
                best = (channel, result[2])
    finally:
        if best:
            drone.request(b"K", b"H" + bytes([best[0]]), patience=10)
            print(f"camera left on channel {best[0]}")
        stream.terminate()


if __name__ == "__main__":
    main()
