# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
Update the drone's camera over ESP-NOW from the FPV receiver (fw/fpv_rx, the S3 on this laptop's USB): make camera-fast (from src/fw). Needs no Wi-Fi network and no wiring, only the receiver plugged in and the drone (powered, disarmed, the laptop service running) to name the camera and check the result.

The drone gives the camera's MAC (K N), channel (K S) and slots (K I). The receiver's port is taken from the dashboard for the update (its reader is stopped and the port locked; the dashboard opens it again within 20 s afterwards), told the camera (FPVF) and its channel (FPVC), and passes each message on (FPVE) by ESP-NOW. The camera (fw/esp32cam/src/air_update.h) reports the hash of every sector of its spare slot, so only sectors that differ are sent, which makes an interrupted update resume; each sector goes as 18 pieces and a W request, which the camera checks against the sector's hash, writes and reads back, or answers with the pieces it lacks. Then B selects the slot (the camera checks the whole image) and restarts it, and the drone confirms the camera runs the new image.
"""

import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import termios
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_direct import Drone  # noqa: E402

IMAGE = Path(__file__).resolve().parents[1] / "esp32cam/.pio/build/s3cam/firmware.bin"
SECTOR, HASH_SIZE, PIECE, HASHES = 4096, 8, 232, 28
MAGIC = b"CUE1"


class Receiver:
    """The S3 receiver's port, raw, with the camera's replies (FPVE) sorted out of the video stream by a reader thread."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self.fd)
            raise
        a = termios.tcgetattr(self.fd)
        a[0:4] = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0]
        a[6][termios.VMIN], a[6][termios.VTIME] = 1, 0
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        os.set_blocking(self.fd, True)
        self.replies, self.unacked, self.beacon = {}, 0, False
        self.changed = threading.Condition()
        self.open = True
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        buffer = b""
        while self.open:
            try:
                buffer += os.read(self.fd, 65536)
            except OSError:
                return
            if b"FPVS" in buffer or b"FPVT" in buffer:
                self.beacon = True
            while True:
                at, lost = buffer.find(b"FPVE"), buffer.find(b"FPVX")
                if lost >= 0 and (at < 0 or lost < at):
                    self.unacked += 1
                    buffer = buffer[lost + 4:]
                    continue
                if at < 0 or len(buffer) < at + 6:
                    break
                length = buffer[at + 4] | buffer[at + 5] << 8
                if len(buffer) < at + 6 + length:
                    break
                message = buffer[at + 6:at + 6 + length]
                buffer = buffer[at + 6 + length:]
                if message[:5] == MAGIC + b"A" and len(message) >= 10:
                    with self.changed:
                        self.replies[message[5:9]] = (message[9], message[10:])
                        self.changed.notify_all()
            buffer = buffer[-(6 + 250):] if len(buffer) > 65536 else buffer

    def write(self, data):
        os.write(self.fd, data)

    def send(self, message):
        self.write(b"FPVE" + struct.pack("<H", len(message)) + message)

    def request(self, op, body=b"", timeout=0.6, tries=8):
        """The camera's answer to one request (status, text), asked again on silence; the camera answers a repeated id from its last reply."""
        identifier = os.urandom(4)
        for _ in range(tries):
            self.send(MAGIC + b"R" + identifier + op + body)
            deadline = time.monotonic() + timeout
            with self.changed:
                while identifier not in self.replies and time.monotonic() < deadline:
                    self.changed.wait(deadline - time.monotonic())
                if identifier in self.replies:
                    return self.replies.pop(identifier)
        raise TimeoutError(f"the camera did not answer {op.decode()} over ESP-NOW")

    def close(self):
        self.open = False
        os.close(self.fd)


def take_port(candidates):
    """The receiver's port: the dashboard's reader stopped, the port locked, and the receiver's beacon or report heard on it within 2 s."""
    for pid in subprocess.run(["pgrep", "-f", r"os.read\(fd, 65536\)"], capture_output=True, text=True).stdout.split():
        os.kill(int(pid), signal.SIGTERM)
    for path in candidates:
        try:
            receiver = Receiver(path)
        except OSError:
            continue
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not receiver.beacon:
            time.sleep(0.05)
        if receiver.beacon:
            return path, receiver
        receiver.close()
    raise SystemExit("no FPV receiver found on " + ", ".join(candidates))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", nargs="?", type=Path, default=IMAGE)
    parser.add_argument("--port", help="the receiver's port (default: each /dev/ttyACM* in turn)")
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    args = parser.parse_args()
    image = args.image.read_bytes()
    expected = image[-32:-24].hex()
    drone = Drone(args.bridge)
    mac, name = drone.request(b"K", b"N").decode().split()[:2]
    running, target, size, sha = drone.request(b"K", b"I").decode().split()
    if sha == expected:
        print(f"{name}'s {running} already runs this image")
        return
    if len(image) > int(size):
        raise SystemExit(f"image of {len(image)} bytes does not fit the camera's {target} ({size} bytes)")
    channel = int(next(word for word in drone.request(b"K", b"S").decode().split() if word.startswith("ch"))[2:])
    drone.ws.close()
    drone.ws = None
    candidates = [args.port] if args.port else sorted(str(p) for p in Path("/dev").glob("ttyACM*"))
    path, receiver = take_port(candidates)
    print(f"receiver on {path}, camera {name} ({mac}) on channel {channel}: {running} -> {target}")
    try:
        receiver.write(b"FPVF" + bytes.fromhex(mac.replace(":", "")) + b"FPVC" + bytes([channel]))
        status, text = receiver.request(b"I")
        if status or text.decode().split()[1] != target:
            raise SystemExit(f"the camera answered I with {text!r}")
        count = (len(image) + SECTOR - 1) // SECTOR
        local = [image[i * SECTOR:(i + 1) * SECTOR].ljust(SECTOR, b"\xff") for i in range(count)]
        digests = [hashlib.sha256(sector).digest()[:HASH_SIZE] for sector in local]
        there = b""
        for first in range(0, count, HASHES):
            status, text = receiver.request(b"H", struct.pack("<HB", first, min(HASHES, count - first)))
            if status:
                raise SystemExit(f"the camera refused H: {text.decode(errors='replace')}")
            there += text
        stale = [i for i in range(count) if there[i * HASH_SIZE:(i + 1) * HASH_SIZE] != digests[i]]
        print(f"{len(stale)} of {count} sectors to send ({count - len(stale)} already there)", flush=True)
        started = time.monotonic()
        for done, index in enumerate(stale, 1):
            wanted = range(0, SECTOR, PIECE)
            for _ in range(10):
                receiver.write(b"FPVC" + bytes([channel]))
                for offset in wanted:
                    receiver.send(MAGIC + b"D" + struct.pack("<HH", index, offset) + local[index][offset:offset + PIECE])
                status, text = receiver.request(b"W", struct.pack("<H", index) + digests[index], timeout=1.5)
                if not status:
                    break
                text = text.decode(errors="replace")
                if not text.startswith("missing "):
                    if "corrupted" not in text:
                        raise SystemExit(f"sector {index}: {text}")
                    wanted = range(0, SECTOR, PIECE)
                    continue
                bits = int(text.split()[1], 16)
                wanted = [offset for offset in range(0, SECTOR, PIECE) if bits >> (offset // PIECE) & 1]
            else:
                raise SystemExit(f"sector {index} did not take in 10 tries")
            elapsed = time.monotonic() - started
            print(f"\rwrote {done}/{len(stale)} sectors ({done * SECTOR / 1024 / elapsed:.0f} KB/s)", end="", flush=True)
        print(f"\nsent in {time.monotonic() - started:.1f} s; booting the camera's {target}", flush=True)
        try:
            status, text = receiver.request(b"B", timeout=3, tries=3)
            if status:
                raise SystemExit(f"the camera refused B: {text.decode(errors='replace')}")
        except TimeoutError:
            print("no answer to B (it may have restarted at once); checking through the drone")
    finally:
        receiver.close()
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        time.sleep(1)
        try:
            booted = drone.request(b"K", b"I", patience=3).decode().split()
        except (TimeoutError, RuntimeError):
            continue
        if booted[3] == expected:
            print(f"the camera runs the new image from {booted[0]}")
            return
        if booted[0] != running:
            raise SystemExit(f"the camera runs {booted[0]} ({booted[3]}), not the new image")
    raise SystemExit("the camera did not answer with the new image within 60 s")


if __name__ == "__main__":
    main()
