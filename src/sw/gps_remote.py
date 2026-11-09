"""
The drone's GPS from the laptop, through the drone's N request (fw/drone/main/radio.c, gps.c gps_send and gps_take) and the laptop service: uv run -m sw.gps_remote <command>

  sats              the satellites tracked (NAV-SAT): system, number, signal (dB-Hz), elevation, azimuth, used in the fix
  watch [seconds]   the satellite count, the strongest signals, the fix and the RF health every few seconds (Ctrl-C to stop)
  rf                RF and antenna health (MON-RF): jamming indicator, noise, AGC
  status            fix state, time to first fix, time since start (NAV-STATUS)
  version           receiver firmware and supported systems (MON-VER)
  poll <MESSAGE>    any pollable UBX message by pyubx2's name, e.g. NAV-SIG, NAV-DOP, MON-COMMS
  get <KEY> ...     configuration items from the module's RAM layer, by pyubx2's names, e.g. CFG_RATE_MEAS CFG_SIGNAL_GLO_ENA
  set <KEY=VALUE> ...  configuration items in the RAM layer (lost at power-off); only with the drone disarmed

The module's own routine messages (NAV-PVT, already in the drone's status) do not come through; everything else it sends does, including the replies to the drone's own NAV-SAT and MON-RF polls each second (firmware from 2026-11-09), while the tunnel has been used in the last 5 s. Which source disturbs the GPS (camera video, camera, the drone's radio): uv run -m sw.gps_interference. A setting that leaves the module silent at the drone's baud makes the drone search and configure it again after 3 s (gps.c).
"""

import argparse
import struct
import sys
import time

from pyubx2 import POLL, UBXMessage, UBXReader

from fw.tools.ota_direct import Drone

BRIDGE = "ws://127.0.0.1:8765/update"
SYSTEMS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS", 6: "GLONASS"}


class Gps:
    def __init__(self, bridge=BRIDGE):
        self.drone = Drone(bridge)
        self.dropped = None
        self.take()

    def take(self, send=b""):
        """Sends bytes to the module and returns the UBX messages it has sent since the last take."""
        reply = self.drone.request(b"N", send)
        dropped = struct.unpack_from("<I", reply)[0]
        if self.dropped is not None and dropped > self.dropped:
            print(f"(the drone dropped {dropped - self.dropped} frames for want of room)", file=sys.stderr)
        self.dropped = dropped
        messages, data = [], reply[4:]
        while len(data) >= 8:
            length = data[4] | data[5] << 8
            messages.append(UBXReader.parse(data[: length + 8]))
            data = data[length + 8 :]
        return messages

    def ask(self, message, identity=None, wait=1.5):
        """Sends a message and waits for a reply with its identity (or an ACK for a set)."""
        identity = identity or message.identity
        got = self.take(message.serialize())
        until = time.monotonic() + wait
        while True:
            for reply in got:
                if reply.identity == identity or (identity == "ACK" and reply.identity in ("ACK-ACK", "ACK-NAK")):
                    return reply
            if time.monotonic() > until:
                return None
            time.sleep(0.1)
            got = self.take()

    def poll(self, name):
        return self.ask(UBXMessage(name.split("-")[0], name, POLL))


def satellites(reply):
    rows = []
    for i in range(1, reply.numSvs + 1):
        get = lambda field: getattr(reply, f"{field}_{i:02d}")
        rows.append((SYSTEMS.get(get("gnssId"), get("gnssId")), get("svId"), get("cno"), get("elev"), get("azim"), bool(get("svUsed"))))
    return sorted(rows, key=lambda row: -row[2])


def show_sats(gps):
    reply = gps.poll("NAV-SAT")
    if reply is None:
        print("No NAV-SAT reply")
        return
    rows = satellites(reply)
    heard = [row for row in rows if row[2] > 0]
    print(f"{len(rows)} satellites known, {len(heard)} heard, {sum(row[5] for row in rows)} used")
    print("system    sv  dB-Hz  elev  azim  used")
    for system, sv, cno, elev, azim, used in rows:
        print(f"{system:<8} {sv:3}  {cno:5}  {elev:4}  {azim:4}  {'yes' if used else ''}")


def show_rf(gps):
    reply = gps.poll("MON-RF")
    if reply is None:
        print("No MON-RF reply")
        return
    jamming = {0: "unknown", 1: "ok", 2: "warning", 3: "critical"}
    antenna = {0: "init", 1: "unknown", 2: "ok", 3: "short", 4: "open"}
    print(f"jamming {jamming.get(reply.jammingState_01)} (indicator {reply.jamInd_01}/255), noise {reply.noisePerMS_01}/ms, AGC {reply.agcCnt_01 / 8191:.0%}, antenna {antenna.get(reply.antStatus_01)}")


def watch(gps, period):
    while True:
        sat = gps.poll("NAV-SAT")
        status = gps.poll("NAV-STATUS")
        rf = gps.poll("MON-RF")
        line = time.strftime("%H:%M:%S")
        if sat:
            rows = satellites(sat)
            heard = [row for row in rows if row[2] > 0]
            best = ", ".join(f"{s[:3]}{sv}:{cno}" for s, sv, cno, *_ in heard[:6])
            line += f"  heard {len(heard)} used {sum(row[5] for row in rows)}  [{best}]"
        if status:
            line += f"  fix {status.gpsFix} ok {status.gpsFixOk} ttff {status.ttff / 1000:.0f}s up {status.msss / 1000:.0f}s"
        if rf:
            line += f"  jam {rf.jamInd_01} noise {rf.noisePerMS_01} agc {rf.agcCnt_01 / 8191:.0%}"
        print(line, flush=True)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("sats", "watch", "rf", "status", "version", "poll", "get", "set"))
    parser.add_argument("items", nargs="*")
    parser.add_argument("--bridge", default=BRIDGE)
    args = parser.parse_args()
    gps = Gps(args.bridge)
    if args.command == "sats":
        show_sats(gps)
    elif args.command == "watch":
        try:
            watch(gps, float(args.items[0]) if args.items else 3.0)
        except KeyboardInterrupt:
            pass
    elif args.command == "rf":
        show_rf(gps)
    elif args.command in ("status", "version", "poll"):
        name = {"status": "NAV-STATUS", "version": "MON-VER"}.get(args.command) or (args.items[0] if args.items else parser.error("poll needs a message name"))
        print(gps.poll(name) or f"No {name} reply")
    elif args.command == "get":
        print(gps.ask(UBXMessage.config_poll(0, 0, args.items), "CFG-VALGET") or "No reply")
    else:
        items = []
        for item in args.items:
            key, value = item.split("=", 1)
            items.append((key, int(value, 0)))
        print(gps.ask(UBXMessage.config_set(1, 0, items), "ACK") or "No acknowledgement")


if __name__ == "__main__":
    main()
