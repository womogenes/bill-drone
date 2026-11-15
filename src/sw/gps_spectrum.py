"""
The drone's GPS front end as its own spectrum analyzer: UBX-MON-SPAN (256 bins around the module's RF center, as the receiver sees it after its gain stage) and MON-RF polled through the drone's GPS tunnel once a second in each state of the camera, averaged, plotted and searched for spurs, to find which of the camera's clocks lands in the GPS band. With the drone disarmed on the bench, the laptop service running and the dashboard tab closed (its camera panel keeps asking for FPV): uv run -m sw.gps_spectrum [--seconds 30] [--states fpv,idle,off] [--rounds 1]

States are sw.gps_interference's (fpv: the camera streaming, idle: tracking tags, off: held in reset through its EN line), plus "as-is", which leaves the camera alone. Writes src/sw/recordings/gps_spectrum_<time>/spectra.npz and spectrum.png, and prints per state the spurs (bins standing above the local floor, a running median) and the jamming indicator, then each state's bins that rise above the last state's (the reference; put "off" last). MON-SPAN's levels are after the programmable gain (pga, dB), which the AGC turns down against strong interference, so the plot also shows each state referred to the input (level minus pga). Only polls; the module's configuration is untouched, and the tunnel refuses to send while the drone is armed (radio.c), as this tool stops when the laptop service shows a throttle or a recording.
"""

import argparse
import json
import struct
import sys
import time
import urllib.request
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyubx2 import POLL, UBXMessage

from sw.gps_interference import LINGER, Bench
from sw.gps_remote import BRIDGE

STATUS = "http://127.0.0.1:8765/status"
RECORDINGS = Path(__file__).parent / "recordings"
L1 = 1575.42
# A spur stands this far (dB) above the running median of FLOOR_BINS bins around it; the receiver's spectrum is shaped by its SAW filter, so a single floor level would flag the passband's shoulders.
SPUR_DB = 2.0
FLOOR_BINS = 31


def disarmed():
    """True while the laptop service shows both throttles at zero and no recording; a stale or missing status counts as armed."""
    try:
        status = json.load(urllib.request.urlopen(STATUS, timeout=2))
        reading = status["reading"]
        return status.get("online") and all(motor["throttle"] == 0 for motor in reading["actuators"].values()) and not reading["recorder"]["writing"]
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return False


def span_blocks(message):
    """MON-SPAN's RF blocks from its raw payload (version, block count, two reserved, then per block 256 spectrum bytes in quarter dB, span, resolution and center in Hz, pga in dB, three reserved): (frequencies in MHz, levels in dB, pga) each."""
    payload = message.payload
    blocks = []
    for block in range(payload[1]):
        offset = 4 + 272 * block
        levels = np.frombuffer(payload, np.uint8, 256, offset) * 0.25
        span, resolution, center = struct.unpack_from("<III", payload, offset + 256)
        # Bin i is at center + span * (i - 128) / 256 (the interface description's formula).
        frequencies = (center + span * (np.arange(256) - 128) / 256) / 1e6
        blocks.append((frequencies, levels, payload[offset + 268]))
    return blocks


def capture(gps, seconds):
    """MON-SPAN and MON-RF once a second for seconds; the drone's own MON-RF polls' replies count too."""
    request = UBXMessage("MON", "MON-SPAN", POLL).serialize() + UBXMessage("MON", "MON-RF", POLL).serialize()
    spectra, pgas, rf = [], [], []
    frequencies = None
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if not disarmed():
            raise SystemExit("The drone is armed, recording or out of reach; stopped.")
        started = time.monotonic()
        messages = gps.take(request)
        while time.monotonic() - started < 1:
            time.sleep(0.1)
            messages += gps.take()
        for message in messages:
            if message.identity == "MON-SPAN":
                frequencies, levels, pga = span_blocks(message)[0]
                spectra.append(levels)
                pgas.append(pga)
            elif message.identity == "MON-RF":
                rf.append((message.jamInd_01, message.noisePerMS_01, message.agcCnt_01 / 8191))
    return frequencies, np.array(spectra), np.array(pgas), np.array(rf)


def floor(levels):
    """The running median of FLOOR_BINS bins, the edges padded with the end values."""
    half = FLOOR_BINS // 2
    padded = np.pad(levels, half, mode="edge")
    return np.array([np.median(padded[i : i + FLOOR_BINS]) for i in range(len(levels))])


def spurs(frequencies, excess, threshold=SPUR_DB):
    """Local maxima of excess (dB) above threshold, strongest first: (MHz, dB)."""
    found = [(frequencies[i], excess[i]) for i in range(1, len(excess) - 1) if excess[i] >= threshold and excess[i] >= excess[i - 1] and excess[i] >= excess[i + 1]]
    return sorted(found, key=lambda spur: -spur[1])


def describe(found):
    return ", ".join(f"{mhz:.2f} ({db:+.1f})" for mhz, db in found[:10]) or "none"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=30, help="per state (default 30)")
    parser.add_argument("--states", default="fpv,idle,off", help="comma-separated, in order, from fpv, idle, off, as-is; the last is the reference")
    parser.add_argument("--rounds", type=int, default=1, help="repeat the sequence (default 1)")
    parser.add_argument("--bridge", default=BRIDGE)
    args = parser.parse_args()
    states = args.states.split(",")
    if unknown := set(states) - {"fpv", "idle", "off", "as-is"}:
        parser.error(f"unknown states {sorted(unknown)}")
    if not disarmed():
        raise SystemExit("The laptop service shows the drone armed, recording or out of reach.")
    bench = Bench(args.bridge)
    results = {}
    changed = False
    try:
        for number in range(1, args.rounds + 1):
            for state in states:
                print(f"{state}:", flush=True)
                if state != "as-is":
                    # sw.gps_interference's settling: the camera switched (and booted), FPV started, or the FPV linger run out.
                    changed = True
                    bench.video.running.clear()
                    bench.camera(state != "off")
                    if state == "fpv":
                        bench.video.running.set()
                        time.sleep(2)
                        print(f"  camera: {bench.drone.request(b'K', b'S').decode()}", flush=True)
                    elif state == "idle":
                        time.sleep(LINGER + 1)
                frequencies, spectra, pgas, rf = capture(bench.gps, args.seconds)
                print(f"  {len(spectra)} spectra, {len(rf)} MON-RF", flush=True)
                if len(spectra):
                    name = state if args.rounds == 1 else f"{state} {number}"
                    results[name] = (frequencies, spectra, pgas, rf)
    except KeyboardInterrupt:
        print("Interrupted; the results cover the states finished.", file=sys.stderr)
    finally:
        bench.video.running.clear()
        if changed:
            print("Camera back on.", flush=True)
            bench.camera_on = None
            bench.camera(True)
    if not results:
        raise SystemExit("No MON-SPAN replies (the module may not support it, or the tunnel is down).")

    folder = RECORDINGS / time.strftime("gps_spectrum_%Y-%m-%d_%H%M%S")
    folder.mkdir(parents=True)
    np.savez(folder / "spectra.npz", **{f"{name}_{field}": value for name, (frequencies, spectra, pgas, rf) in results.items() for field, value in (("frequencies", frequencies), ("spectra", spectra), ("pga", pgas), ("rf", rf))})
    names = list(results)
    reference = names[-1]
    means = {name: spectra.mean(axis=0) for name, (frequencies, spectra, pgas, rf) in results.items()}
    frequencies = results[reference][0]
    print(f"\nRF center {frequencies[128]:.3f} MHz, span {frequencies[-1] - frequencies[0] + (frequencies[1] - frequencies[0]):.2f} MHz, bin {(frequencies[1] - frequencies[0]) * 1000:.0f} kHz; spurs in MHz (dB above the local floor)")
    for name in names:
        pgas, rf = results[name][2], results[name][3]
        jam = f"jam {rf[:, 0].mean():.0f} (max {rf[:, 0].max():.0f}), noise {rf[:, 1].mean():.0f}, AGC {rf[:, 2].mean():.0%}" if len(rf) else "no MON-RF"
        print(f"{name:<10} pga {pgas.mean():.1f} dB, {jam}\n  spurs: {describe(spurs(frequencies, means[name] - floor(means[name])))}")
    print(f"\nInput-referred (level - pga) above {reference}:")
    referred = {name: means[name] - results[name][2].mean() for name in names}
    for name in names[:-1]:
        difference = referred[name] - referred[reference]
        print(f"{name:<10} mean {difference.mean():+.1f} dB; rising bins: {describe(spurs(frequencies, difference - np.median(difference)))}")

    figure, axes = plt.subplots(3 if len(names) > 1 else 2, 1, figsize=(12, 11 if len(names) > 1 else 7.5), sharex=True)
    top, middle, bottom = axes if len(names) > 1 else (*axes, None)
    for name in names:
        top.plot(frequencies, means[name], label=f"{name} (pga {results[name][2].mean():.0f} dB)", linewidth=1)
        middle.plot(frequencies, referred[name], label=name, linewidth=1)
        if bottom and name != reference:
            bottom.plot(frequencies, referred[name] - referred[reference], label=f"{name} - {reference}", linewidth=1)
    top.set_ylabel("MON-SPAN level (dB)")
    middle.set_ylabel("level - pga (dB)")
    if bottom:
        bottom.set_ylabel(f"input-referred, minus {reference} (dB)")
    axes[-1].set_xlabel("frequency (MHz)")
    for axis in axes:
        axis.axvline(L1, color="grey", linestyle="--", linewidth=0.8)
        axis.grid(alpha=0.3)
        axis.legend(loc="upper right", fontsize=8)
    top.set_title(f"GPS front-end spectrum (UBX-MON-SPAN), mean of each state; dashed: L1 {L1} MHz")
    figure.tight_layout()
    figure.savefig(folder / "spectrum.png", dpi=120)
    print(f"\nSaved {folder}/spectrum.png and spectra.npz")


if __name__ == "__main__":
    main()
