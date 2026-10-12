"""
Upload/run the temporary recorder, download its trace, and reboot normal firmware.
"""

import argparse
from datetime import datetime
import json
import urllib.request
from pathlib import Path
import socket
import subprocess
import sys
import time

root = Path(__file__).parent.resolve()
fw = root.parents[1]
sys.path.insert(0, str(fw / "lib/webrepl"))
import webrepl_cli as client

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--kp", type=float, default=1.5)
parser.add_argument("--ki", type=float, default=0)
parser.add_argument("--kd", type=float, default=1.0)
parser.add_argument("--base", type=float, default=200)
args = parser.parse_args()
config = dict(
    line.split("=", 1)
    for line in (root.parents[3] / ".env").read_text().splitlines()
    if "=" in line and not line.startswith("#")
)
pw = config["MPY_PASSWORD"].strip().strip("\"'")
ip = "192.168.86.32"
stem = datetime.now().strftime("%Y%m%d-%H%M%S") + "-onboard"


def transfer(source, dest):
    result = subprocess.run(
        [
            sys.executable,
            str(fw / "lib/webrepl/webrepl_cli.py"),
            "-p",
            pw,
            str(source),
            str(dest),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).replace(pw, "[redacted]"))


def connect():
    s = socket.create_connection((ip, 8266), timeout=5)
    client.client_handshake(s)
    ws = client.websocket(s)
    client.login(ws, pw)
    return s, ws


def drain(s, ws, timeout):
    s.settimeout(timeout)
    output = b""
    try:
        while True:
            output += ws.read(1, text_ok=True)
    except (TimeoutError, OSError, AssertionError):
        return output.decode(errors="replace").replace(pw, "[redacted]")


transfer(root / "probe.py", ip + ":/roll_probe.py")
s, ws = connect()
drain(s, ws, 0.5)
ws.write(b"\x03", client.WEBREPL_FRAME_TXT)
drain(s, ws, 0.5)
command = (
    f'import sys,gc,asyncio; asyncio.new_event_loop(); sys.modules.pop("roll_probe",None); '
    f'gc.collect(); import roll_probe; asyncio.run(roll_probe.run({args.kp},{args.ki},{args.kd},{args.base},"{stem}")); '
    "import machine; machine.reset()\r"
)
ws.write(command.encode(), client.WEBREPL_FRAME_TXT)
print(drain(s, ws, 12), flush=True)
s.close()
# Recorder resets itself after saving; the hardware WDT is the fallback.
for attempt in range(20):
    try:
        with urllib.request.urlopen("http://" + ip, timeout=1) as response:
            response.read()
        break
    except OSError:
        time.sleep(0.5)
else:
    raise RuntimeError("ESP did not come back online")
for suffix in ("bin", "json"):
    transfer(ip + ":/roll_probe." + suffix, root / (stem + "." + suffix))
metadata = json.loads((root / (stem + ".json")).read_text())
assert metadata["runid"] == stem, "Trace does not belong to this run"
print(stem, metadata)
