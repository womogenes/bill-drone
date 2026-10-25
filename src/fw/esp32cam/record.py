"""
Record the ESP32-CAM's per-frame records: uv run fw/esp32cam/record.py [--host 192.168.86.28]
Subscribes to /record, saves every frame to sw/recordings/camera-<time>.frm and prints a line a second. Ctrl-C stops.
With --imu, the drone's IMU readings from the running bridge (fw/controller/bridge.py /status, 20 Hz, armed or not) go to <name>.imu.jsonl beside it, one {host_ns, drone_ms, gyro_deg_s, accel_g, roll_deg, pitch_deg} per reading, on the same laptop clock as the frames' receive times; load_imu() reads it back.
Summarize a recording, or a black-box file's records: uv run fw/esp32cam/record.py --analyze sw/recordings/camera-<time>.frm

File format: repeated [int64 laptop receive time, ns][record from fw/esp32cam/src/record.h], 224 bytes (TAG3 packets) or, in older recordings, 144 (TAG2).
"""

import argparse
import json
import socket
import struct
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import numpy as np

PACKET = "<4sIIIHH16fIfHh16fHH"  # PosePacket, 164 bytes
RECORD = struct.Struct(PACKET + "8fIIHBBHBBBBHHHHH")
assert RECORD.size == 224
OLD_RECORD = struct.Struct("<4sIIIHH16fIfHhHH" + "8fIHHBBH")  # TAG2, until 2026-10-22
assert OLD_RECORD.size == 144
FRAME_US = 33974  # sensor frame period
PER_DATAGRAM = 6


def decode(record):
    if len(record) == OLD_RECORD.size:
        v = OLD_RECORD.unpack(record)
        return {
            "sequence": v[1], "capture_us": v[2], "age_us": v[3], "valid": v[5],
            "position": v[6:9], "rotation": v[9:18], "margin": v[18], "reprojection_px": v[19],
            "flow": v[20:22], "flow_interval_us": v[22], "flow_residual_px": v[23], "flow_inliers": v[24],
            "tag_position": (0.0, 0.0, 0.0), "alt_reprojection_px": 0.0, "trusted": int(v[5] > 0),
            "corners": v[28:36], "frame_start_us": v[36], "process_us": v[37], "flow_wait_us": v[38],
            "found": v[39], "preview": v[40], "short_frames": v[41],
            "track_failed": 0, "alignment": 0.0, "measured": 0, "measures": 0, "align_coarse": 0, "align_fine": 0, "window_pixels": 0, "frame_pixels": 0,
        }
    v = RECORD.unpack(record)
    return {
        "sequence": v[1], "capture_us": v[2], "age_us": v[3], "valid": v[5],
        "position": v[6:9], "rotation": v[9:18], "margin": v[18], "reprojection_px": v[19],
        "flow": v[20:22], "flow_interval_us": v[22], "flow_residual_px": v[23], "flow_inliers": v[24],
        "tag_position": v[26:29], "alt_reprojection_px": v[41], "trusted": v[42],
        "corners": v[44:52], "frame_start_us": v[52], "process_us": v[53], "flow_wait_us": v[54],
        "found": v[55], "preview": v[56], "short_frames": v[57],
        "track_failed": v[58], "alignment": v[59] / 100, "measured": v[60], "measures": v[61], "align_coarse": v[62], "align_fine": v[63], "window_pixels": v[64], "frame_pixels": v[65],
    }


def load(path):
    data = Path(path).read_bytes()
    if data[:8] == b"BILLBBX1":
        from blackbox import entries

        rows = [e["record"] | {"host_ns": 0} for e in entries(data) if "record" in e]
        rows.sort(key=lambda r: r["sequence"])
        return {k: np.array([r[k] for r in rows]) for k in rows[0]}
    size = RECORD.size if data[8:12] == b"TAG3" else OLD_RECORD.size
    step = 8 + size
    rows = [decode(data[i + 8:i + step]) | {"host_ns": struct.unpack_from("<q", data, i)[0]} for i in range(0, len(data) - step + 1, step)]
    rows.sort(key=lambda r: r["sequence"])
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


def log_imu(path, stop, bridge="http://127.0.0.1:8765/status"):
    last = None
    with path.open("w") as file:
        while not stop.is_set():
            try:
                reading = json.load(urllib.request.urlopen(bridge, timeout=1))["reading"]
            except (OSError, ValueError, KeyError, TypeError):
                stop.wait(0.5)
                continue
            imu = reading and reading.get("imu")
            if imu and reading["time"] != last:
                last = reading["time"]
                file.write(json.dumps({"host_ns": time.time_ns(), "drone_ms": reading["time"], "gyro_deg_s": imu["gyro"], "accel_g": imu["accel"], "roll_deg": imu["roll"], "pitch_deg": imu["pitch"]}) + "\n")
            stop.wait(0.02)


def load_imu(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    return {k: np.array([r[k] for r in rows]) for k in rows[0]} if rows else {}


def subscribe(host, port, stop):
    while not stop.is_set():
        try:
            urllib.request.urlopen(f"http://{host}/record?port={port}", timeout=2).read()
        except OSError as error:
            print(f"subscribe failed: {error}")
        stop.wait(2)


def record(host, out, seconds=None, imu=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("", 0))
    sock.settimeout(0.5)
    stop = threading.Event()
    threading.Thread(target=subscribe, args=(host, sock.getsockname()[1], stop), daemon=True).start()
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"recording to {out}")
    if imu:
        threading.Thread(target=log_imu, args=(out.with_suffix(".imu.jsonl"), stop), daemon=True).start()
        print(f"and the drone's IMU to {out.with_suffix('.imu.jsonl')}")
    count, last_sequence, lost = 0, None, 0
    window = []
    shown = started = time.monotonic()
    with out.open("wb") as file:
        try:
            while seconds is None or time.monotonic() - started < seconds:
                try:
                    datagram = sock.recv(4 + PER_DATAGRAM * RECORD.size)
                except TimeoutError:
                    datagram = b""
                now = time.time_ns()
                if datagram[:4] == b"FRM2":
                    for i in range(4, len(datagram) - RECORD.size + 1, RECORD.size):
                        chunk = datagram[i:i + RECORD.size]
                        file.write(struct.pack("<q", now) + chunk)
                        r = decode(chunk)
                        if last_sequence is not None and r["sequence"] > last_sequence + 1:
                            lost += r["sequence"] - last_sequence - 1
                        last_sequence = r["sequence"]
                        count += 1
                        window.append(r)
                if time.monotonic() - shown >= 1 and window:
                    shown = time.monotonic()
                    found = sum(r["found"] > 0 for r in window) / len(window)
                    valid = sum(r["valid"] > 0 for r in window) / len(window)
                    starts = [r["frame_start_us"] for r in window]
                    skipped = sum(round(((b - a) & 0xFFFFFFFF) / FRAME_US) - 1 for a, b in zip(starts, starts[1:]))
                    p = window[-1]["position"]
                    print(f"{count:6d} frames  {len(window):2d}/s  found {found:4.0%}  valid {valid:4.0%}  sensor frames skipped {skipped:2d}  records lost {lost}  pos {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}")
                    window = []
        except KeyboardInterrupt:
            pass
    stop.set()
    print(f"saved {count} frames to {out}")


def cost_model(d):
    """Least-squares processing time per unit of work, as in the simulator's cost model (sim/harness.cpp)."""
    columns = {"fixed": np.ones(len(d["process_us"])), "measure": d["measures"], "align_coarse": d["align_coarse"], "align_fine": d["align_fine"],
               "window_run": d["window_pixels"] > 0, "window_pixel": d["window_pixels"], "frame_run": d["frame_pixels"] > 0, "frame_pixel": d["frame_pixels"]}
    columns = {k: np.asarray(v, float) for k, v in columns.items()}
    used = {k: v for k, v in columns.items() if np.any(v != 0) and (k == "fixed" or np.ptp(v) > 0)}
    if len(used) < 2:
        return None
    a = np.stack(list(used.values()), axis=1)
    solution, *_ = np.linalg.lstsq(a, d["process_us"].astype(float), rcond=None)
    return {k: round(float(x), 2) for k, x in zip(used, solution)} | {"whole_frame_searches": int((d["frame_pixels"] > 0).sum()), "whole_frame_us_median": int(np.median(d["process_us"][d["frame_pixels"] > 0])) if (d["frame_pixels"] > 0).any() else None}


def analyze(path):
    d = load(path)
    n = len(d["sequence"])
    seconds = ((d["frame_start_us"][-1] - d["frame_start_us"][0]) & 0xFFFFFFFF) / 1e6
    gaps = np.diff(d["frame_start_us"].astype(np.int64)) & 0xFFFFFFFF
    skipped = np.round(gaps / FRAME_US).astype(int) - 1
    found, valid = d["found"] > 0, d["valid"] > 0
    corners = d["corners"].reshape(-1, 4, 2)
    centre = corners.mean(axis=1)
    both = found[1:] & found[:-1]
    speed = np.linalg.norm(np.diff(centre, axis=0), axis=1)[both]  # px per frame
    # Tag losses: found then not found, with the corner speed just before.
    losses = np.flatnonzero(found[:-1] & ~found[1:])
    pre = [np.linalg.norm(centre[i] - centre[i - 1]) if i > 0 and found[i - 1] else np.nan for i in losses]
    position = d["position"][valid]
    step = np.linalg.norm(np.diff(position, axis=0), axis=1) if len(position) > 1 else np.array([0.0])
    flow = d["flow"][1:][both & (d["flow_interval_us"][1:] > 0)]
    motion = np.diff(centre, axis=0)[both & (d["flow_interval_us"][1:] > 0)]
    print(json.dumps({
        "frames": n, "seconds": round(seconds, 1), "rate_hz": round(n / seconds, 1) if seconds else None,
        "records_lost": int(d["sequence"][-1] - d["sequence"][0] + 1 - n),
        "sensor_frames_skipped": int(skipped.clip(0).sum()), "frames_with_preview": int(d["preview"].sum()),
        "skipped_after_preview_frames": int(skipped.clip(0)[d["preview"][:-1] > 0].sum()),
        "short_frames": int((d["short_frames"][-1] - d["short_frames"][0]) & 0xFFFF),
        "found": round(found.mean(), 3), "tracked": round((d["found"] == 1).mean(), 3), "detected": int((d["found"] == 2).sum()),
        "valid": round(valid.mean(), 3), "ambiguous_resolved": int((d["valid"] == 2).sum()), "trusted": round(float((d["trusted"] > 0).mean()), 3),
        "measured": int(d["measured"].sum()), "track_failures_by_reason": [int((d["track_failed"] == k).sum()) for k in range(1, 6)],
        "alignment_median": round(float(np.median(d["alignment"][d["found"] == 1])), 2) if (d["found"] == 1).any() else None,
        "tag_losses": len(losses), "speed_before_loss_px_per_frame": [round(float(v), 1) for v in pre][:20],
        "corner_speed_px_per_frame": {"median": round(float(np.median(speed)), 2) if len(speed) else None, "p95": round(float(np.percentile(speed, 95)), 2) if len(speed) else None, "max": round(float(speed.max()), 2) if len(speed) else None},
        "position_step_m": {"median": round(float(np.median(step)), 4), "p99": round(float(np.percentile(step, 99)), 4), "max": round(float(step.max()), 4)},
        "reprojection_px": {"median": round(float(np.median(d["reprojection_px"][valid])), 3) if valid.any() else None, "p95": round(float(np.percentile(d["reprojection_px"][valid], 95)), 3) if valid.any() else None},
        "margin_median": round(float(np.median(d["margin"][found])), 1) if found.any() else None,
        "age_us": {"median": int(np.median(d["age_us"])), "p95": int(np.percentile(d["age_us"], 95))},
        "process_us": {"median": int(np.median(d["process_us"])), "p95": int(np.percentile(d["process_us"], 95)), "max": int(d["process_us"].max())},
        "flow_wait_us": {"median": int(np.median(d["flow_wait_us"])), "p95": int(np.percentile(d["flow_wait_us"], 95))},
        "process_us_by_work": cost_model(d),
        # Flow against the tag's own motion: the tag is excluded from the flow, so with a still tag this is camera motion.
        "flow_vs_tag_px": {"frames": len(flow), "rms_difference": round(float(np.sqrt(np.mean(np.sum((flow - motion) ** 2, axis=1)))), 3) if len(flow) else None},
    }, indent=1))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.86.28")
    parser.add_argument("--analyze", help="summarize a .frm recording or a black-box .bin file")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--seconds", type=float, help="stop after this long instead of on Ctrl-C")
    parser.add_argument("--imu", action="store_true", help="also log the drone's IMU from the running bridge")
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
    else:
        default = Path(__file__).resolve().parents[2] / "sw" / "recordings" / f"camera-{datetime.now():%Y%m%d-%H%M%S}.frm"
        record(args.host, args.out or default, args.seconds, args.imu)
