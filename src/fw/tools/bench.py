"""
The compute and scheduling benchmark through the laptop service (a drone running make bench's build: drone/main/bench_flight.c), disarmed on USB with no pack.
uv run tools/bench.py idle [--seconds 20]           the disarmed drone's own ticks and the lock's sites over a while
uv run tools/bench.py run SCENARIO [--ticks 3500] [--record] [--busy] [--mpc]
    SCENARIO flight (a hold, a 60-degree quaternion burst, a tumble's recovery, a landing on the simulated plant), replay (the 2026-11-15 07:02 UTC flight's sensors) or worst (a GPS solution and a barometer reading every tick, saturating tumbles every 3 s, bursts every 6 s).
    --record streams the run's flight log as in flight; --busy also keeps the radio's request service busy (settings reads) while it runs.
uv run tools/bench.py warm                          the Kalman filter and the hold four times in a row on copies: the first with a cold cache
uv run tools/bench.py reinstall                     installs the IMU's I2C driver again from the control task (its interrupt on core 1)
Prints each metric's count and its min, median, 99th, 99.9th percentile, max and mean (us).
"""

import argparse
import json
import struct
import threading
import time

from ota_direct import Drone

SCENARIOS = {"flight": 1, "replay": 2, "worst": 3}


def table(drone):
    metrics = {}
    page = 0
    while True:
        reply = json.loads(drone.request(b"b", b"m" + bytes([page])))
        metrics.update(reply["metrics"])
        page += 1
        if page >= reply["pages"]:
            break
    print(f"{'metric':12s} {'n':>7s} {'min':>8s} {'median':>8s} {'p99':>8s} {'p99.9':>8s} {'max':>8s} {'mean':>8s}  (us)")
    for name, row in metrics.items():
        if row[0]:
            print(f"{name:12s} {row[0]:7.0f} " + " ".join(f"{v:8.1f}" for v in row[1:]))
    sites = json.loads(drone.request(b"b", b"l"))["sites"]
    print(f"\n{'lock site':34s} {'calls':>7s} {'max wait':>9s} {'max hold':>9s} {'mean hold':>9s}  (us)")
    for site, count, wait, hold, mean in sorted(sites, key=lambda s: -s[3]):
        print(f"{site:34s} {count:7d} {wait:9.1f} {hold:9.1f} {mean:9.1f}")
    return metrics, sites


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["idle", "run", "reinstall", "warm"])
    parser.add_argument("scenario", nargs="?", choices=list(SCENARIOS), default="flight")
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    parser.add_argument("--ticks", type=int, default=3500)
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--busy", action="store_true")
    parser.add_argument("--mpc", action="store_true", help="also time the MPC prototype after each armed tick (it takes core 1 time from the schedule)")
    parser.add_argument("--json", help="also write the results here")
    args = parser.parse_args()
    drone = Drone(args.bridge)
    if args.command == "reinstall":
        print(drone.request(b"b", b"i").decode())
        return
    if args.command == "warm":
        drone.request(b"b", b"w")
        time.sleep(0.5)
        progress = json.loads(drone.request(b"b", b"q"))
        print("ekf_update us:", progress["warm_ekf"], "hover_update us:", progress["warm_hover"])
        return
    if args.command == "idle":
        drone.request(b"b", b"r")
        time.sleep(args.seconds)
        metrics, sites = table(drone)
        progress = None
    else:
        drone.request(b"b", b"s" + struct.pack("<BHB", SCENARIOS[args.scenario], args.ticks, args.record | args.mpc << 1))
        stop = threading.Event()
        if args.busy:
            # A second link to the service, reading the settings (the longest reply) over and over.
            def flood():
                other = Drone(args.bridge)
                while not stop.is_set():
                    other.request(b"S")

            threading.Thread(target=flood, daemon=True).start()
        while True:
            time.sleep(1)
            progress = json.loads(drone.request(b"b", b"q"))
            print(f"  {progress['tick']}/{progress['of']} armed {progress['armed']} holding {progress['holding']} phases {progress['phases']} recovering {progress['recovering']}", flush=True)
            if not progress["active"] and not progress["requested"]:
                break
        stop.set()
        print(json.dumps(progress))
        metrics, sites = table(drone)
    if args.json:
        with open(args.json, "w") as file:
            json.dump({"metrics": metrics, "sites": sites, "run": progress}, file)


if __name__ == "__main__":
    main()
