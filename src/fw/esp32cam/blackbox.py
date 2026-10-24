"""
Read the ESP32-CAM's black-box files (src/blackbox.h, written to the microSD card by src/sd.h).

Summarize a file:                 uv run fw/esp32cam/blackbox.py sw/recordings/blackbox/0003.bin
Export its images as PNGs:        uv run fw/esp32cam/blackbox.py 0003.bin --png out/  [--every 10] [--limit 500]
Convert its records for record.py: uv run fw/esp32cam/blackbox.py 0003.bin --frm 0003.frm
                                  uv run fw/esp32cam/record.py --analyze 0003.frm
Download new files after landing: uv run fw/esp32cam/blackbox.py --fetch [--host 192.168.86.28] [--out sw/recordings/blackbox]

File format: a 104-byte header (HEADER below: magic BILLBBX1, header size, file number, record size, reduced frame size,
crop size, full frame size, how often a frame carried the full frame, CAM clock at the start, calibration, tag, build),
then one entry per frame: a 16-byte entry header (BBE1, flags, reserved, payload length, frames dropped so far in this
file), then the 224-byte FrameRecord (record.py), the 80x60 reduced frame, if flags & 1 an 8-byte crop header (left, top,
width, height) and the crop's pixels, if flags & 2 the full 320x240 frame, and padding to a multiple of four bytes.
"""

import argparse
import json
import struct
import urllib.request
from pathlib import Path

import numpy as np

from record import decode

HEADER = struct.Struct("<8sIIIHHHHHHIffff5ffI24s")
assert HEADER.size == 104
ENTRY = struct.Struct("<4sHHII")
CROP = struct.Struct("<hhHH")
HAS_CROP, HAS_FULL = 1, 2


def read_header(data):
    v = HEADER.unpack_from(data)
    if v[0] != b"BILLBBX1":
        raise ValueError("not a black-box file")
    return {
        "header_bytes": v[1], "file": v[2], "record_bytes": v[3], "reduced": (v[4], v[5]), "crop_size": v[6],
        "full": (v[7], v[8]), "full_every": v[9], "start_us": v[10], "fx": v[11], "fy": v[12], "cx": v[13], "cy": v[14],
        "distortion": v[15:20], "tag_size_m": v[20], "tag_id": v[21], "build": v[22].rstrip(b"\0").decode(),
    }


def entries(data):
    """Yields each entry as a dict: the decoded record and its bytes (raw), dropped (frames dropped so far), reduced (an image), and crop
    ((left, top), image) and full (an image) or None. Damaged stretches are skipped to the next entry magic; a truncated
    last entry (power cut before the final write) is ignored."""
    header = read_header(data)
    rw, rh = header["reduced"]
    fw, fh = header["full"]
    at = header["header_bytes"]
    while at + ENTRY.size <= len(data):
        magic, flags, _, length, dropped = ENTRY.unpack_from(data, at)
        if magic != b"BBE1" or length % 4:
            following = data.find(b"BBE1", at + 1)
            yield {"damaged_bytes": (following if following >= 0 else len(data)) - at}
            if following < 0:
                return
            at = following
            continue
        body = at + ENTRY.size
        if body + length > len(data):
            return
        p = body
        raw = data[p:p + header["record_bytes"]]
        record = decode(raw)
        p += header["record_bytes"]
        reduced = np.frombuffer(data, np.uint8, rw * rh, p).reshape(rh, rw)
        p += rw * rh
        crop = full = None
        if flags & HAS_CROP:
            left, top, w, h = CROP.unpack_from(data, p)
            p += CROP.size
            crop = ((left, top), np.frombuffer(data, np.uint8, w * h, p).reshape(h, w))
            p += w * h
        if flags & HAS_FULL:
            full = np.frombuffer(data, np.uint8, fw * fh, p).reshape(fh, fw)
            p += fw * fh
        yield {"record": record, "raw": raw, "dropped": dropped, "reduced": reduced, "crop": crop, "full": full}
        at = body + length


def summarize(path):
    data = Path(path).read_bytes()
    header = read_header(data)
    frames = [e for e in entries(data) if "record" in e]
    damaged = sum(e.get("damaged_bytes", 0) for e in entries(data))
    sequences = np.array([e["record"]["sequence"] for e in frames], np.int64)
    starts = np.array([e["record"]["frame_start_us"] for e in frames], np.int64)
    seconds = float(((starts[-1] - starts[0]) & 0xFFFFFFFF) / 1e6) if len(frames) > 1 else 0.0
    gaps = np.diff(sequences) - 1 if len(frames) > 1 else np.array([0])
    dropped = frames[-1]["dropped"] if frames else 0
    found = np.array([e["record"]["found"] > 0 for e in frames])
    valid = np.array([e["record"]["valid"] > 0 for e in frames])
    summary = {
        "file": header["file"], "build": header["build"], "frames": len(frames), "seconds": round(seconds, 1),
        "rate_hz": round(len(frames) / seconds, 1) if seconds else None,
        # The black box's own drops (the card fell behind) against frames the detector never produced or sent.
        "dropped_by_black_box": dropped, "sequence_gaps": int(gaps.clip(0).sum()),
        "sequence_gaps_not_black_box": int(gaps.clip(0).sum()) - dropped,
        "drop_runs": int((gaps > 0).sum()), "longest_drop_run": int(gaps.max()) if len(gaps) else 0,
        "damaged_bytes": damaged, "with_crop": sum(e["crop"] is not None for e in frames),
        "with_full": sum(e["full"] is not None for e in frames),
        "found": round(float(found.mean()), 3) if len(frames) else None,
        "valid": round(float(valid.mean()), 3) if len(frames) else None,
        "bytes": len(data), "bytes_per_second": round(len(data) / seconds) if seconds else None,
        "calibration": {k: header[k] for k in ("fx", "fy", "cx", "cy", "tag_size_m", "tag_id")},
    }
    print(json.dumps(summary, indent=1))
    return summary


def export_png(path, out, every=1, limit=None):
    import cv2

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    written = 0
    for i, e in enumerate(e for e in entries(Path(path).read_bytes()) if "record" in e):
        if i % every or (limit is not None and written >= limit):
            continue
        s = e["record"]["sequence"]
        cv2.imwrite(str(out / f"{s:07d}_reduced.png"), e["reduced"])
        if e["crop"] is not None:
            (left, top), image = e["crop"]
            cv2.imwrite(str(out / f"{s:07d}_crop_{left}_{top}.png"), image)
        if e["full"] is not None:
            cv2.imwrite(str(out / f"{s:07d}_full.png"), e["full"])
        written += 1
    print(f"wrote {written} frames to {out}")


def export_frm(path, out):
    with Path(out).open("wb") as file:
        count = 0
        for e in entries(Path(path).read_bytes()):
            if "record" in e:
                file.write(struct.pack("<q", 0) + e["raw"])
                count += 1
    print(f"wrote {count} records to {out}")


def fetch(host, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    listing = json.loads(urllib.request.urlopen(f"http://{host}/blackbox", timeout=5).read())
    print(f"card {listing['state']}, recording file {listing['file']}")
    for item in listing["files"]:
        target = out / item["name"]
        if target.exists() and target.stat().st_size == item["bytes"]:
            continue
        print(f"downloading {item['name']} ({item['bytes'] / 1e6:.1f} MB)")
        with urllib.request.urlopen(f"http://{host}/blackbox/file?name={item['name']}", timeout=30) as response:
            target.write_bytes(response.read())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs="?")
    parser.add_argument("--png", help="export the frames' images to this directory")
    parser.add_argument("--every", type=int, default=1, help="with --png, every Nth frame")
    parser.add_argument("--limit", type=int, help="with --png, at most this many frames")
    parser.add_argument("--frm", help="write the records in record.py's format to this file")
    parser.add_argument("--fetch", action="store_true", help="download the camera's black-box files")
    parser.add_argument("--host", default="192.168.86.28")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[2] / "sw" / "recordings" / "blackbox")
    args = parser.parse_args()
    if args.fetch:
        fetch(args.host, args.out)
    elif args.file:
        summarize(args.file)
        if args.png:
            export_png(args.file, args.png, args.every, args.limit)
        if args.frm:
            export_frm(args.file, args.frm)
    else:
        parser.print_help()
