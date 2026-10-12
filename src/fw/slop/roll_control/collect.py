"""
Record bounded roll responses through the existing firmware websocket.
"""

import argparse
import json
from pathlib import Path
import threading
import time
from datetime import datetime
from websocket import create_connection

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--ip", default="192.168.86.32")
parser.add_argument("--kp", type=float, default=1.5)
parser.add_argument("--ki", type=float, default=0)
parser.add_argument("--kd", type=float, default=1.5)
parser.add_argument("--base", type=float, default=200)
parser.add_argument("--seconds", type=float, default=18)
parser.add_argument("--target", type=float, default=4)
args = parser.parse_args()
path = Path(__file__).parent / (datetime.now().strftime("%Y%m%d-%H%M%S") + ".json")
ws = create_connection(f"ws://{args.ip}/ws", timeout=1)
done = threading.Event()
rows, commands = [], []
started = time.monotonic()


def send(command):
    commands.append(dict(sent_s=time.monotonic() - started, **command))
    ws.send(json.dumps(command))


def heartbeat():
    while not done.wait(0.1):
        ws.send('{"command":"heartbeat"}')


worker = threading.Thread(target=heartbeat, daemon=True)
worker.start()
reason = "duration complete"
try:
    initial = json.loads(ws.recv())
    print("Initial:", initial["imu"]["roll"], initial["control"], flush=True)
    starting_angle = initial["imu"]["roll"]
    centered = abs(starting_angle) < 30
    settings = dict(
        enabled=True,
        target=0,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        base_throttle=args.base,
    )
    send(dict(command="control", settings=settings))
    phase = -1
    active_seen = False
    while time.monotonic() - started < args.seconds:
        row = json.loads(ws.recv())
        elapsed = time.monotonic() - started
        row["received_s"] = elapsed
        rows.append(row)
        angle = row["imu"]["roll"]
        active_seen |= row["control"]["enabled"]
        if abs(angle) < 30:
            centered = True
        if (centered and abs(angle) > 30) or (
            not centered and abs(angle) > abs(starting_angle) + 0.5
        ):
            reason = "angle boundary"
            break
        if not centered and elapsed > 4:
            reason = "startup timeout"
            break
        if active_seen and not row["control"]["enabled"]:
            reason = "controller disabled externally"
            break
        if abs(row["imu"]["gyro"][0]) > 180:
            reason = "angular velocity boundary"
            break
        new_phase = int(elapsed // 3)
        if centered and new_phase != phase:
            phase = new_phase
            target = [0, args.target, -args.target, 0, -args.target, args.target][
                phase % 6
            ]
            send(dict(command="control", settings=dict(target=target)))
            print(f"{elapsed:.2f}s roll={angle:.1f}, target={target}", flush=True)
finally:
    try:
        send(dict(command="stop"))
    finally:
        done.set()
        worker.join()
        ws.close()
        path.write_text(
            json.dumps(
                dict(
                    arguments=vars(args),
                    reason=reason,
                    commands=commands,
                    readings=rows,
                )
            )
        )
        print(reason, len(rows), path, flush=True)
