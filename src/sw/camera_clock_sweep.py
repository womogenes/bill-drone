"""
Which camera clock or drive setting keeps the camera's harmonics off the GPS: for each setting, the camera's clocks through K C (fw/esp32cam/src/clocks.h), then the GPS front end's spectrum (UBX-MON-SPAN) and jamming indicator (MON-RF) for --seconds through the drone's GPS tunnel (sw.gps_spectrum's capture), the tones at 1560/1570/1580/1590 MHz above the local floor, and the camera's health: its packet rate at the laptop service, K S (whose request counter shows a restart), and in FPV every JPEG of the dashboard's FPV stream decoded, with K F and K G. With the drone disarmed (stops if it arms or records) and the camera answering requests; FPV is kept going with K V as the dashboard does.

From src: uv run -m sw.camera_clock_sweep tag|fpv name:xclk_hz:pin_drive:pad_drive,... [seconds]
e.g. uv run -m sw.camera_clock_sweep tag base:0:2:3,weakest:0:0:0,x16:16000000:2:3

xclk_hz 0 is the mode's default; drives 0-3 or 0xff for config.h's. A point is marked ! when the camera's clocks or request count changed during it (a restart). Each point's spectra go to sw/recordings/camera_clock_sweep/<mode>_<name>_<time>.npz and FPV's stream to fpv_<name>.mjpeg. The camera keeps the last setting until it restarts.
"""
import io
import json
import struct
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

from sw.gps_remote import BRIDGE, Gps
from sw.gps_spectrum import capture, disarmed, floor
from sw.gps_interference import Video
from fw.tools.ota_direct import Drone

OUT = Path(__file__).parent / "recordings" / "camera_clock_sweep"
OUT.mkdir(parents=True, exist_ok=True)
TONES = (1560.0, 1570.0, 1580.0, 1590.0)
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 30


def status():
    return json.load(urllib.request.urlopen("http://127.0.0.1:8765/status", timeout=3))


def kc(drone, mode, xclk=0, pin=0xFF, pad=0xFF):
    body = bytes([mode]) + struct.pack("<IBBBBBB", xclk, 0, pin, pad, 0, 0, 0)
    return drone.request(b"K", b"C" + body, patience=10).decode(errors="replace")


def tones(frequencies, spectra, pgas):
    mean = spectra.mean(axis=0)
    excess = mean - floor(mean)
    result = {}
    for tone in TONES:
        near = np.abs(frequencies - tone) <= 0.25
        result[tone] = (float(excess[near].max()), float((mean - pgas.mean())[near].max())) if near.any() else (float("nan"), float("nan"))
    return result


def jpeg_check(path):
    from PIL import Image
    data = path.read_bytes()
    frames, bad, sizes = 0, 0, set()
    start = 0
    while True:
        a = data.find(b"Content-Length: ", start)
        if a < 0:
            break
        length = int(data[a + 16 : data.find(b"\r\n", a)])
        body = data.find(b"\r\n\r\n", a) + 4
        if body + length > len(data):
            break
        frames += 1
        try:
            image = Image.open(io.BytesIO(data[body : body + length]))
            image.load()
            sizes.add(image.size)
        except Exception:
            bad += 1
        start = body + length
    return frames, bad, sizes


def main():
    mode = sys.argv[1]
    settings = [s.split(":") for s in sys.argv[2].split(",")]  # name:xclk:pin:pad (0xff = default)
    drone = Drone(BRIDGE)
    gps = Gps(BRIDGE)
    jpeg = mode == "fpv"
    video = Video(BRIDGE)
    if jpeg:
        video.running.set()
    rows = []
    for name, xclk, pin, pad in settings:
        if not disarmed():
            raise SystemExit("not idle; stopped")
        stream = None
        if jpeg:
            # The dashboard's FPV stream keeps K V going (and lets the JPEGs be checked).
            stream_path = OUT / f"fpv_{name}.mjpeg"
            stream = subprocess.Popen(["curl", "-s", "-N", "-o", str(stream_path), "http://127.0.0.1:3000/api/camera?fpv=1"])
            time.sleep(4)
        try:
            print(name, kc(drone, int(jpeg), int(xclk), int(pin, 0), int(pad, 0)), flush=True)
            time.sleep(4)
            described = drone.request(b"K", b"C" + bytes([int(jpeg)]), patience=10).decode(errors="replace")
            requests_before = int(drone.request(b"K", b"S", patience=10).decode(errors="replace").split()[7].split("/")[0])
        except TimeoutError as error:
            print(f"{name}: camera not answering ({error}); stopped", flush=True)
            if stream:
                stream.terminate()
            break
        before = status()["reading"]["camera"]
        if jpeg:
            stream_path.write_bytes(b"")
        t0 = time.monotonic()
        frequencies, spectra, pgas, rf = capture(gps, SECONDS)
        elapsed = time.monotonic() - t0
        after = status()["reading"]["camera"]
        try:
            ks = drone.request(b"K", b"S", patience=10).decode(errors="replace")
            after_clocks = drone.request(b"K", b"C" + bytes([int(jpeg)]), patience=10).decode(errors="replace")
        except TimeoutError:
            ks = after_clocks = "no answer"
        # A restart during the point resets the request counter and drops the K C setting back to plan/defaults.
        try:
            requests_after = int(ks.split()[7].split("/")[0])
        except (IndexError, ValueError):
            requests_after = -1
        valid = after_clocks == described and requests_after >= requests_before
        health = {"frames_per_s": (after["frames"] - before["frames"]) / elapsed, "poses": after["poses"] - before["poses"], "ks": ks}
        if jpeg:
            stream.terminate()
            stream.wait()
            health["jpeg"] = jpeg_check(stream_path)
            health["kf"] = drone.request(b"K", b"F", patience=10).decode(errors="replace")
            health["kg"] = drone.request(b"K", b"G", patience=10).decode(errors="replace")
        t = tones(frequencies, spectra, pgas)
        row = {"name": name, "clocks": described, "pga": float(pgas.mean()), "jam": float(rf[:, 0].mean()) if len(rf) else None, "jam_max": float(rf[:, 0].max()) if len(rf) else None, "noise": float(rf[:, 1].mean()) if len(rf) else None,
               "tones": {f"{k:.0f}": v for k, v in t.items()}, "spectra": len(spectra), "valid": valid, "after_clocks": after_clocks, **health}
        rows.append(row)
        np.savez(OUT / f"{mode}_{name}_{time.strftime('%H%M%S')}.npz", frequencies=frequencies, spectra=spectra, pga=pgas, rf=rf)
        print(json.dumps(row, default=str), flush=True)
    print("\nsetting    pga  jam(max) noise | tone dB above floor (level-pga) at " + " ".join(f"{t:.0f}" for t in TONES))
    for r in rows:
        print(f"{r['name'] + ('' if r['valid'] else '!'):<10} {r['pga']:4.1f} {r['jam']:4.1f}({r['jam_max']:.0f}) {r['noise']:5.1f} | " + "  ".join(f"{v[0]:+5.1f}({v[1]:5.1f})" for v in r["tones"].values()))


if __name__ == "__main__":
    main()
