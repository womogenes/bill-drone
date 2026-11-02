"""
Record the camera's tag-mode preview while the drone is armed: uv run -m sw.camera_record

The preview (fw/esp32cam/src/web.h radioPreviewLoop: the 80x60 grayscale frame as JPEG, about 14 a second) comes over the camera's radio through the FPV receiver and the dashboard's /api/camera, which keeps it coming while this listens, without taking the camera out of tag mode. From arming until POST_ROLL_S after disarming each flight is saved under sw/recordings/video/ as <UTC start>.mjpeg (the JPEGs back to back), <UTC start>.csv (per frame: laptop time, capture and send time on the camera's clock in us) and <UTC start>.mp4 (by ffmpeg at the frames' measured rate, scaled up 4x and turned upright by the drone's hover_cam_roll, as the dashboard shows it; the MJPEG keeps the camera's own orientation). The UTC start lines up with the flight log of the same arming within a second or so.
"""

import argparse
import csv
import json
import shutil
import subprocess
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

VIDEO = Path(__file__).resolve().parent / "recordings/video"
POST_ROLL_S = 2.0


def armed(bridge):
    try:
        with urllib.request.urlopen(bridge + "/status", timeout=2) as reply:
            return bool(json.load(reply)["reading"]["control"].get("enabled"))
    except Exception:
        return None


def camera_roll(bridge):
    """
    The camera's roll on its mount (hover_cam_roll, degrees; 180 when mounted upside down), 0 if the bridge does not say.
    """
    try:
        with urllib.request.urlopen(bridge + "/settings", timeout=2) as reply:
            body = json.load(reply)
        return float(body.get("settings", body).get("hover_cam_roll", 0))
    except Exception:
        return 0.0


def upright(roll):
    """
    The ffmpeg filters that turn the picture back by roll, as the dashboard's camera view does.
    """
    turn = round(roll) % 360
    if turn == 0:
        return []
    if turn == 180:
        return ["hflip", "vflip"]
    return [f"rotate={-roll}*PI/180"]


def frames(stream):
    """
    (capture_us, sent_us, jpeg) from the dashboard's multipart stream.
    """
    while True:
        line = stream.readline()
        if not line:
            return
        if not line.startswith(b"--frame"):
            continue
        headers = {}
        while (line := stream.readline().strip()):
            key, _, value = line.decode(errors="replace").partition(":")
            headers[key.strip().lower()] = value.strip()
        jpeg = stream.read(int(headers.get("content-length", 0)))
        capture, _, sent = headers.get("x-time", "0 0").partition(" ")
        yield int(capture or 0), int(sent or 0), jpeg


def record(dashboard, bridge):
    started = datetime.now(timezone.utc)
    roll = camera_roll(bridge)
    name = started.strftime("%Y-%m-%dT%H-%M-%SZ")
    VIDEO.mkdir(parents=True, exist_ok=True)
    mjpeg, index = VIDEO / f"{name}.mjpeg", VIDEO / f"{name}.csv"
    count, disarmed_at, checked = 0, None, 0.0
    with urllib.request.urlopen(dashboard + "/api/camera", timeout=5) as stream, mjpeg.open("wb") as out, index.open("w", newline="") as table:
        writer = csv.writer(table)
        writer.writerow(["laptop_s", "capture_us", "sent_us"])
        for capture, sent, jpeg in frames(stream):
            now = time.time()
            out.write(jpeg)
            writer.writerow([f"{now:.3f}", capture, sent])
            count += 1
            if now - checked > 0.5:
                checked = now
                if armed(bridge) is False:
                    disarmed_at = disarmed_at or now
                else:
                    disarmed_at = None
            if disarmed_at and now - disarmed_at > POST_ROLL_S:
                break
    seconds = (datetime.now(timezone.utc) - started).total_seconds()
    print(f"{name}: {count} frames in {seconds:.1f} s", flush=True)
    if count > 1 and shutil.which("ffmpeg"):
        rate = max(1.0, count / seconds)
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", f"{rate:.2f}", "-f", "mjpeg", "-i", str(mjpeg),
                        "-vf", ",".join(upright(roll) + ["scale=iw*4:ih*4:flags=neighbor"]), "-pix_fmt", "yuv420p", str(VIDEO / f"{name}.mp4")], check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--dashboard", default="http://localhost:3000")
    parser.add_argument("--bridge", default="http://127.0.0.1:8765")
    args = parser.parse_args()
    while True:
        if armed(args.bridge):
            try:
                record(args.dashboard, args.bridge)
            except Exception as error:
                print("recording stopped:", error, flush=True)
                time.sleep(1)
        time.sleep(0.2)


if __name__ == "__main__":
    main()
