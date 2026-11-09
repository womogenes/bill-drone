"""
Which source disturbs the drone's GPS: the module's signal strengths and jamming indicators for a while in each state of the camera and the drone's radio, then a table. With the drone disarmed on the bench (outdoors, sky in view, still), the laptop service running and the dashboard tab closed (its camera panel keeps asking the camera for video or previews): uv run -m sw.gps_interference [--seconds 30] [--states fpv,idle,off,quiet] [--rounds 2] [--save samples.jsonl]

States (each sets the camera itself, so any order works):
  fpv         camera on, streaming FPV over raw 802.11 (K V every 0.5 s, as the dashboard asks)
  idle        camera on and tracking, not streaming (3 s after the last K V the camera goes back to tracking)
  off         camera held off through its EN line (sw.camera_power off: chip, radio and image sensor stopped)
  quiet       camera off and the drone's own radio silent (request M, in 5 s spells: the drone's tunnel holds about 6 s of replies)
  quiet-idle  camera on and tracking, the drone's radio silent

Each second it polls the module (sw.gps_remote's tunnel) for UBX-NAV-SAT and MON-RF. Per state: samples, the mean C/N0 of the strongest four satellites and of all heard, the C/N0 of the satellites heard in every state (the fairest comparison as the sky moves), satellites heard and used, the CW jamming indicator (0-255, mean and most), the worst jamming state (needs the jamming monitor, on in drone firmware from 2026-11-09), noise per ms and AGC. A source that jams shows as lower C/N0 and a higher noise or jamming indicator (often with the AGC turning its gain down) in the states where it runs. --rounds repeats the whole sequence, so a slow change in the sky does not pass for a difference between states. The camera is switched back on at the end.
"""

import argparse
import json
import statistics
import struct
import sys
import threading
import time
import urllib.request

from pyubx2 import POLL, UBXMessage

from fw.tools.ota_direct import Drone
from sw.gps_remote import BRIDGE, Gps, satellites

STATUS = "http://127.0.0.1:8765/status"
STATES = ("fpv", "idle", "off", "quiet", "quiet-idle")
JAMMING = {0: "unknown", 1: "ok", 2: "warning", 3: "critical"}
# The drone's quiet spells (request M takes 1-15 s; its tunnel keeps about 6 s of NAV-SAT and MON-RF replies), and when the silence starts after the drone's reply (radio.c quiet_task).
SPELL = 5
QUIET_DELAY = 0.3
# The camera goes back to tracking this long after the last K V (fw/esp32cam/src/config.h fpvLingerMs), and needs about this long to boot after its EN line is released.
LINGER = 3.0
BOOT = 4.0
# FPV's byte budget in K V (KB/s), as the dashboard asks with the ESP32 receiver (sw/dashboard/app/api/camera/route.ts radioBudget).
BUDGET = 90


class Video:
    """Keeps the camera streaming FPV over the radio while running is set, as the dashboard does: K V every 0.5 s on its own connection to the laptop service."""

    def __init__(self, bridge):
        self.drone = Drone(bridge)
        self.running = threading.Event()
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            self.running.wait()
            try:
                self.drone.request(b"K", b"V" + struct.pack("<H", BUDGET), patience=2)
            except (RuntimeError, TimeoutError) as error:
                print(f"(K V: {error})", file=sys.stderr)
            time.sleep(0.5)


def gps_time():
    """The GPS time of week (ms) of the drone's newest solution as the laptop service last heard it, or None."""
    try:
        gps = json.load(urllib.request.urlopen(STATUS, timeout=2))["reading"]["gps"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return gps["itow_ms"] + gps["age_ms"] if gps and gps["age_ms"] is not None else None


class Collector:
    """NAV-SAT and MON-RF replies: each NAV-SAT once (by its time of week), and each MON-RF with the NAV-SAT before it, since they are polled together."""

    def __init__(self):
        self.seen = set()
        self.itow = None
        self.sats, self.rf = [], []

    def add(self, messages, window=None):
        for message in messages:
            if message.identity == "NAV-SAT":
                self.itow = message.iTOW
                if message.iTOW not in self.seen and (window is None or window[0] <= message.iTOW <= window[1]):
                    self.seen.add(message.iTOW)
                    self.sats.append(satellites(message))
            elif message.identity == "MON-RF" and self.itow is not None and (window is None or window[0] <= self.itow <= window[1]):
                self.rf.append((message.jamInd_01, message.jammingState_01, message.noisePerMS_01, message.agcCnt_01 / 8191))


def listen(gps, seconds, collector):
    """Polls once a second (drone firmware from 2026-11-09 polls as well; a NAV-SAT seen twice counts once) and takes every reply."""
    request = UBXMessage("NAV", "NAV-SAT", POLL).serialize() + UBXMessage("MON", "MON-RF", POLL).serialize()
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        started = time.monotonic()
        collector.add(gps.take(request))
        while time.monotonic() - started < 1:
            time.sleep(0.1)
            collector.add(gps.take())


def quiet(gps, drone, seconds, collector):
    """The drone's radio silent in spells of SPELL s, its own polls' replies taken from the tunnel after each; only those inside the silence (by NAV-SAT's time of week) count."""
    remaining = seconds
    while remaining > 0:
        spell = min(SPELL, max(1, round(remaining)))
        gps.take()
        drone.request(b"M", bytes([spell]))
        now = gps_time()
        # Without a GPS time (no solution yet) the window is unknown and every reply counts.
        window = (now + QUIET_DELAY * 1000 + 300, now + (QUIET_DELAY + spell) * 1000 - 200) if now is not None else None
        time.sleep(QUIET_DELAY + spell + 0.5)
        # The laptop service drops its clients while the drone is silent; Drone reconnects once it is back.
        for _ in range(20):
            messages = gps.take()
            collector.add(messages, window)
            if not messages:
                break
        remaining -= spell


class Bench:
    """The tunnel, a connection for the camera and quiet requests, the video keepalive, and whether the camera is on (unknown at the start)."""

    def __init__(self, bridge):
        self.gps = Gps(bridge)
        self.drone = Drone(bridge)
        self.video = Video(bridge)
        self.camera_on = None

    def camera(self, on):
        """The camera on (released, then waited for until it answers) or held off."""
        if on == self.camera_on:
            return
        self.drone.request(b"J", b"\x05" if on else b"\x04")
        self.camera_on = on
        if on:
            time.sleep(BOOT)
            print(f"  camera: {self.drone.request(b'K', b'S', patience=20).decode()}", flush=True)
        else:
            time.sleep(1)

    def run(self, state, seconds):
        print(f"{state}:", flush=True)
        self.video.running.clear()
        self.camera(state in ("fpv", "idle", "quiet-idle"))
        if state == "fpv":
            self.video.running.set()
            time.sleep(2)
            print(f"  camera: {self.drone.request(b'K', b'S').decode()}", flush=True)
        elif state in ("idle", "quiet-idle"):
            time.sleep(LINGER + 1)
        collector = Collector()
        if state.startswith("quiet"):
            quiet(self.gps, self.drone, seconds, collector)
        else:
            listen(self.gps, seconds, collector)
        self.video.running.clear()
        print(f"  {len(collector.sats)} NAV-SAT, {len(collector.rf)} MON-RF", flush=True)
        return collector


def mean(values):
    return statistics.fmean(values) if values else float("nan")


def summarize(results):
    """One row per state, and how many satellites were heard in every state (whose C/N0 is the common column)."""
    heard = [{(row[0], row[1]) for sample in collector.sats for row in sample if row[2] > 0} for _, collector in results if collector.sats]
    common = set.intersection(*heard) if heard else set()
    rows = []
    for state, collector in results:
        strongest = [mean(sorted((row[2] for row in sample if row[2] > 0), reverse=True)[:4]) for sample in collector.sats]
        jam = [rf[0] for rf in collector.rf]
        rows.append({
            "state": state, "samples": len(collector.sats),
            "top4": mean([value for value in strongest if value == value]),
            "heard_cno": mean([row[2] for sample in collector.sats for row in sample if row[2] > 0]),
            "common_cno": mean([row[2] for sample in collector.sats for row in sample if (row[0], row[1]) in common]),
            "heard": mean([sum(row[2] > 0 for row in sample) for sample in collector.sats]),
            "used": mean([sum(row[5] for row in sample) for sample in collector.sats]),
            "jam": mean(jam), "jam_max": max(jam) if jam else float("nan"),
            "jamming": JAMMING.get(max((rf[1] for rf in collector.rf), default=0)),
            "noise": mean([rf[2] for rf in collector.rf]), "agc": mean([rf[3] for rf in collector.rf]),
        })
    return rows, len(common)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=30, help="per state (default 30)")
    parser.add_argument("--states", default="fpv,idle,off,quiet", help=f"comma-separated, in order, from {', '.join(STATES)}")
    parser.add_argument("--rounds", type=int, default=1, help="repeat the sequence (default 1)")
    parser.add_argument("--save", help="write every sample to this JSON-lines file")
    parser.add_argument("--bridge", default=BRIDGE)
    args = parser.parse_args()
    states = args.states.split(",")
    if unknown := set(states) - set(STATES):
        parser.error(f"unknown states {sorted(unknown)}")
    bench = Bench(args.bridge)
    results = []
    try:
        for number in range(1, args.rounds + 1):
            for state in states:
                results.append((state if args.rounds == 1 else f"{state} {number}", bench.run(state, args.seconds)))
    except KeyboardInterrupt:
        print("Interrupted; the table covers the states finished.", file=sys.stderr)
    finally:
        bench.video.running.clear()
        print("Camera back on.", flush=True)
        bench.camera_on = None
        bench.camera(True)
    if args.save:
        with open(args.save, "w") as file:
            for state, collector in results:
                for sample in collector.sats:
                    file.write(json.dumps({"state": state, "satellites": sample}) + "\n")
                for jam, jamming, noise, agc in collector.rf:
                    file.write(json.dumps({"state": state, "jam": jam, "jamming": jamming, "noise": noise, "agc": agc}) + "\n")
    rows, common = summarize(results)
    print(f"\nC/N0 in dB-Hz; common: the {common} satellites heard in every state")
    print(f"{'state':<13}{'samples':>8}{'top 4':>8}{'heard':>8}{'common':>8}{'n heard':>9}{'n used':>8}{'jam':>7}{'max':>5}  {'jamming':<9}{'noise':>7}{'AGC':>6}")
    for row in rows:
        print(f"{row['state']:<13}{row['samples']:>8}{row['top4']:>8.1f}{row['heard_cno']:>8.1f}{row['common_cno']:>8.1f}{row['heard']:>9.1f}{row['used']:>8.1f}"
              f"{row['jam']:>7.1f}{row['jam_max']:>5.0f}  {row['jamming']:<9}{row['noise']:>7.0f}{row['agc']:>6.0%}")


if __name__ == "__main__":
    main()
