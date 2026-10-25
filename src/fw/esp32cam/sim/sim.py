# /// script
# dependencies = ["numpy", "scipy", "pillow"]
# ///
"""
Camera tag-pipeline simulator: uv run fw/esp32cam/sim/sim.py [scenario ...]
Renders 320x240 Y8 frames of the tag16h5 tag on a textured wall as the ESP32-CAM sees them (calibrated pinhole, rolling shutter at 138 us a row, motion blur over the exposure, lens blur, noise), runs them through the camera's own pipeline (harness.cpp: tag.h, track.h, pose.h, flow.h, the AprilTag library) and scores it against the truth.
"""

import argparse
import hashlib
import inspect
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import cache
from pathlib import Path

import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
LIBRARY = HERE.parent / ".pio" / "libdeps" / "esp32cam" / "Apriltag library for Arduino ESP32" / "src"
BUILD = HERE / "build"
WIDTH, HEIGHT = 320, 240
FX, FY, CX, CY = 492.66, 494.56, 160.56, 107.77
ROW_US = 138.0
FRAME_US = 33974.0
TAG_M = 0.15135  # black square
CODE = 0x27C8  # tag16h5 id 0
BITS = [(1, 1), (2, 1), (3, 1), (2, 2), (4, 1), (4, 2), (4, 3), (3, 2), (4, 4), (3, 4), (2, 4), (3, 3), (1, 4), (1, 3), (1, 2), (2, 3)]
TEXTURE_MM = TAG_M * 1000 / 6 / 25  # tag cells exactly 25 texture pixels
WALL_M = 1.6  # texture covers +-WALL_M


def build():
    BUILD.mkdir(exist_ok=True)
    stub = BUILD / "stub"
    stub.mkdir(exist_ok=True)
    (stub / "esp_attr.h").write_text("#pragma once\n#define IRAM_ATTR\n#define DRAM_ATTR\n")
    sources = [LIBRARY / "apriltag.c", LIBRARY / "apriltag_quad_thresh.c", LIBRARY / "tag16h5.c"] + sorted((LIBRARY / "common").glob("*.c"))
    objects = []
    for source in sources:
        target = BUILD / (source.stem + ".o")
        if not target.exists() or target.stat().st_mtime < source.stat().st_mtime:
            subprocess.run(["gcc", "-O2", "-c", f"-I{stub}", f"-I{LIBRARY}", f"-I{LIBRARY / 'common'}", str(source), "-o", str(target)], check=True)
        objects.append(str(target))
    binary = BUILD / "harness"
    headers = list((HERE.parent / "src").glob("*.h")) + [HERE / "harness.cpp"]
    if not binary.exists() or binary.stat().st_mtime < max(h.stat().st_mtime for h in headers):
        subprocess.run(["g++", "-O2", "-std=c++17", f"-I{stub}", f"-I{LIBRARY}", f"-I{LIBRARY / 'common'}", str(HERE / "harness.cpp"), *objects, "-lm", "-lpthread", "-o", str(binary)], check=True)
    return binary


@cache
def texture():
    """Wall texture in grey levels, TEXTURE_MM per pixel, origin at the tag centre, y down: blotchy background, white paper, the tag."""
    n = 2 * int(WALL_M * 1000 / TEXTURE_MM)  # even, so the tag centre falls on a texture pixel boundary
    rng = np.random.default_rng(1)
    wall = np.zeros((n, n), np.float32)
    for scale, weight in ((120, 40), (30, 30), (8, 18)):
        wall += weight * ndimage.gaussian_filter(rng.standard_normal((n, n)).astype(np.float32), scale / TEXTURE_MM) * scale ** 0.5
    wall = 125 + wall / wall.std() * 35
    centre = n / 2
    cell = TAG_M * 1000 / 6 / TEXTURE_MM  # 6 cells across the black square
    paper = 6 / 2 * cell + cell + 30 / TEXTURE_MM  # white border cell plus 30 mm of paper
    y, x = np.mgrid[0:n, 0:n]
    wall[(abs(x - centre) < paper) & (abs(y - centre) < paper)] = 205
    black = 35
    for cy in range(6):
        for cx in range(6):
            x0, y0 = centre + (cx - 3) * cell, centre + (cy - 3) * cell
            wall[int(round(y0)):int(round(y0 + cell)), int(round(x0)):int(round(x0 + cell))] = black
    # tag16h5's data bits in the library's order (bit_x, bit_y in cells of the black square), most significant first.
    for i, (cx, cy) in enumerate(BITS):
        if CODE >> (15 - i) & 1:
            x0, y0 = centre + (cx - 3) * cell, centre + (cy - 3) * cell
            wall[int(round(y0)):int(round(y0 + cell)), int(round(x0)):int(round(x0 + cell))] = 205
    return np.clip(wall, 0, 255)


def rotation(yaw, pitch, roll):
    """Camera to wall rotation for a camera facing +z (the wall) turned by yaw (about y), pitch (about x), roll (about z), radians."""
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return ry @ rx @ rz


def render(motion, frames, exposure_us=10000, noise=2.5, seed=0):
    """motion(t seconds) -> (R camera->wall 3x3, camera position in the wall/tag frame). Returns frames, starts, truth."""
    wall = texture()
    rng = np.random.default_rng(seed)
    v, u = np.mgrid[0:HEIGHT, 0:WIDTH].astype(np.float32)
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], axis=-1)  # camera frame
    images, starts, truth = [], [], []
    for n in range(frames):
        start = n * FRAME_US
        # Enough exposure samples for the blur to look continuous: about one per half pixel of motion.
        r0, c0 = motion(start * 1e-6)
        r1, c1 = motion((start + exposure_us) * 1e-6)
        spin = np.degrees(np.arccos(np.clip((np.trace(r0.T @ r1) - 1) / 2, -1, 1)))
        # At least 16, each offset within the pixel (4x4 stratified), so pixels integrate their area as a sensor's do; point samples would put every edge at a pixel boundary.
        samples = int(np.clip(np.ceil(spin * FX * np.pi / 180 * 2 + np.linalg.norm(c1 - c0) * 1000), 16, 48))
        accumulated = np.zeros((HEIGHT, WIDTH), np.float32)
        for k in range(samples):
            du, dv = ((k * 5) % 16 % 4 + 0.5) / 4 - 0.5, ((k * 5) % 16 // 4 + 0.5) / 4 - 0.5
            sub = rays + np.array([du / FX, dv / FY, 0], np.float32)
            # Row r is exposed over the exposure before its readout at start + r * ROW_US.
            times = (start + np.arange(HEIGHT) * ROW_US - exposure_us + (k + 0.5) / samples * exposure_us) * 1e-6
            poses = [motion(t) for t in times]
            rs = np.stack([p[0] for p in poses]).astype(np.float32)
            cs = np.stack([p[1] for p in poses]).astype(np.float32)
            d = np.einsum("rij,rwj->rwi", rs, sub)
            s = -cs[:, None, 2] / d[..., 2]
            x = cs[:, None, 0] + s * d[..., 0]
            y = cs[:, None, 1] + s * d[..., 1]
            px = x * 1000 / TEXTURE_MM + wall.shape[1] / 2 - 0.5
            py = y * 1000 / TEXTURE_MM + wall.shape[0] / 2 - 0.5
            accumulated += ndimage.map_coordinates(wall, [py.ravel(), px.ravel()], order=1, mode="nearest").reshape(HEIGHT, WIDTH)
        image = ndimage.gaussian_filter(accumulated / samples, 0.6)
        image += rng.normal(0, noise, image.shape)
        images.append(np.clip(np.round(image), 0, 255).astype(np.uint8))
        starts.append(int(start))
        # Truth at the middle of the exposure of the tag's centre row.
        r, c = motion((start + CY * ROW_US - exposure_us / 2) * 1e-6)
        for _ in range(2):
            corners, centre_row = project_corners(r, c)
            r, c = motion((start + centre_row * ROW_US - exposure_us / 2) * 1e-6)
        corners, _ = project_corners(r, c)
        truth.append({"corners": corners, "position": c.copy(), "rotation": r.copy()})
    return np.stack(images), np.array(starts), truth


def project_corners(r, c):
    half = TAG_M / 2
    world = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])  # detection order
    cam = (world - c) @ r  # r maps camera to wall, so wall to camera is r.T
    pix = np.stack([FX * cam[:, 0] / cam[:, 2] + CX, FY * cam[:, 1] / cam[:, 2] + CY], axis=1)
    return pix, pix[:, 1].mean()


def run(images, starts, timed=True, extra=()):
    binary = build()
    header = b"FRM0" + np.array([WIDTH, HEIGHT, len(images)], np.int32).tobytes()
    body = b"".join(np.int64(s).tobytes() + im.tobytes() for s, im in zip(starts, images))
    out = subprocess.run([str(binary), str(FX), str(FY), str(CX), str(CY), str(int(timed)), *map(str, extra)], input=header + body, capture_output=True, check=True).stdout
    rows = np.array([[float(x) for x in line.split()] for line in out.decode().splitlines()])
    return {"kind": rows[:, 0].astype(int), "valid": rows[:, 1].astype(int), "margin": rows[:, 2], "reprojection": rows[:, 3], "host_us": rows[:, 4],
            "corners": rows[:, 5:13].reshape(-1, 4, 2), "position": rows[:, 13:16], "rotation": rows[:, 16:25].reshape(-1, 3, 3), "flow": rows[:, 25:27], "failure": rows[:, 28].astype(int), "predicted": rows[:, 29:37].reshape(-1, 4, 2), "trusted": rows[:, 37].astype(int), "tag_position": rows[:, 38:41]}


def score(result, truth, name):
    found = result["kind"] > 0
    valid = result["valid"] > 0
    processed = result["kind"] >= 0
    true_corners = np.stack([t["corners"] for t in truth])
    corner_error = np.sqrt(np.mean(np.sum((result["corners"] - true_corners) ** 2, axis=2), axis=1))
    position_error = np.linalg.norm(result["position"] - np.stack([t["position"] for t in truth]), axis=1)
    rotation_error = np.array([np.degrees(np.arccos(np.clip((np.trace(r.T @ t["rotation"]) - 1) / 2, -1, 1))) for r, t in zip(result["rotation"], truth)])
    wrong = valid & (rotation_error > 10)
    in_view = np.array([np.all((t["corners"] > 2) & (t["corners"] < [WIDTH - 3, HEIGHT - 3])) for t in truth])
    # Outages: runs of in-view frames without the tag, after it was first found.
    outages, run = [], 0
    seen = False
    for f, iv in zip(found, in_view):
        seen |= bool(f)
        if seen and iv and not f:
            run += 1
        elif run:
            outages.append(run)
            run = 0
    good = valid & ~wrong
    trusted = result["trusted"] > 0
    true_tag = np.stack([-t["position"] @ t["rotation"] for t in truth])  # tag centre in the camera frame
    tag_error = np.linalg.norm(result["tag_position"] - true_tag, axis=1)
    speed = np.r_[0, np.linalg.norm(np.diff(true_corners.mean(axis=1), axis=0), axis=1)]
    return {
        "scenario": name, "frames": len(found), "track_failures": "/".join(str(int((result["failure"] == k).sum())) for k in range(1, 6)), "processed": f"{processed.mean():.0%}", "in_view": f"{in_view.mean():.0%}",
        "found": f"{found[in_view].mean():.1%}", "tracked": int((result["kind"] == 1).sum()), "detected": int((result["kind"] == 2).sum()),
        "valid": f"{valid[in_view].mean():.1%}", "wrong_tilt": f"{int((wrong & (result['valid'] == 1)).sum())}+{int((wrong & (result['valid'] == 2)).sum())}",
        "tilt_deg": f"{np.median(rotation_error[good]):.1f}/{np.percentile(rotation_error[good], 95):.1f}" if good.any() else "-",
        "outages": len(outages), "longest_outage": max(outages, default=0),
        "corner_px": f"{np.median(corner_error[found]):.2f}/{np.percentile(corner_error[found], 95):.2f}" if found.any() else "-",
        "position_mm": f"{np.median(position_error[good]) * 1000:.0f}/{np.percentile(position_error[good], 95) * 1000:.0f}" if good.any() else "-",
        "trusted": f"{trusted[in_view].mean():.1%}", "tag_mm": f"{np.median(tag_error[trusted]) * 1000:.1f}/{np.percentile(tag_error[trusted], 95) * 1000:.1f}" if trusted.any() else "-",
        "max_speed_px": f"{speed.max():.0f}", "lost_at_speed_px": f"{np.median(speed[in_view & ~found]):.0f}" if (in_view & ~found).any() else "-",
        "camera_ms": f"{np.median(result['host_us'][processed]) / 1000:.0f}/{result['host_us'][processed].max() / 1000:.0f}",
    }


# Scenarios: camera 1.7 m from the wall, tag slightly off-centre, viewed with a small tilt like the real setup.
DISTANCE = 1.7
BASE = np.array([0.18, 0.37, -DISTANCE])


def facing(yaw=0.0, pitch=0.0, roll=0.0, offset=(0.0, 0.0, 0.0), base=None):
    position = (BASE if base is None else np.asarray(base)) + np.array(offset)
    # Aim at the tag, then apply the deviation.
    aim_yaw = np.arctan2(-position[0], -position[2])
    aim_pitch = np.arctan2(position[1], np.hypot(position[0], position[2]))
    return rotation(aim_yaw + yaw, aim_pitch + pitch, roll), position


def hand(seed, rate_dps, base=None, aim=(0.0, 0.0)):
    """Hand-held: angular velocity a smoothed random process with rms about rate_dps, around a view whose centre is
    aim (yaw, pitch degrees) away from the tag, from base."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, 12, 0.001)
    w = ndimage.gaussian_filter1d(rng.standard_normal((len(t), 3)), 60, axis=0)
    w = w / w.std(axis=0) * np.radians(rate_dps) * np.array([1, 1, 0.3])
    angle = np.cumsum(w, axis=0) * 0.001
    angle -= ndimage.gaussian_filter1d(angle, 400, axis=0)  # stay roughly pointed at the tag
    offset = np.radians([aim[0], aim[1], 0])
    return lambda s: facing(*(np.array([np.interp(s, t, angle[:, i]) for i in range(3)]) + offset), base=base)


SCENARIOS = {
    "still": (lambda t: facing(), 60),
    "pan30": (lambda t: facing(yaw=np.radians(30 / (2 * np.pi * 0.6)) * np.sin(2 * np.pi * 0.6 * t)), 120),
    "pan60": (lambda t: facing(yaw=np.radians(60 / (2 * np.pi * 1.0)) * np.sin(2 * np.pi * 1.0 * t)), 120),
    "pan120": (lambda t: facing(yaw=np.radians(120 / (2 * np.pi * 1.6)) * np.sin(2 * np.pi * 1.6 * t)), 120),
    "wobble": (lambda t: facing(pitch=np.radians(3) * np.sin(2 * np.pi * 2 * t), yaw=np.radians(1.5) * np.sin(2 * np.pi * 0.7 * t), offset=(0.05 * np.sin(t), 0, 0.05 * np.sin(0.8 * t))), 150),
    "hand40": (hand(3, 40), 200),
    "hand80": (hand(4, 80), 200),
    # The tag carried slowly through the whole image, corners included (yaw +-16 deg, pitch +-12 deg).
    "sweep": (lambda t: facing(yaw=np.radians(16) * np.sin(2 * np.pi * 0.23 * t), pitch=np.radians(12) * np.sin(2 * np.pi * 0.31 * t + 1)), 240),
    "corner_hand": (hand(5, 40, aim=(13, -9)), 200),
    "near_hand": (hand(6, 40, base=(0.05, 0.1, -0.7)), 200),
    "far_still": (lambda t: facing(base=(0.3, 0.5, -3.0)), 60),
    "far_hand": (hand(7, 25, base=(0.3, 0.5, -3.0)), 200),
    # Seen from 50 degrees off the tag's normal.
    "steep_wobble": (lambda t: facing(pitch=np.radians(3) * np.sin(2 * np.pi * 2 * t), base=(1.3, 0.2, -1.1)), 150),
    "flick": (lambda t: facing(yaw=np.radians(12) * (np.clip((t - 1) / 0.06, 0, 1) - np.clip((t - 1.6) / 0.06, 0, 1))), 90),
}


def rendered(name, exposure_us, noise):
    """A scenario's frames and truth, cached in build/frames by the scenario's and the renderer's source."""
    motion, frames = SCENARIOS[name]
    key = hashlib.sha1(repr((name, frames, exposure_us, noise, inspect.getsource(sys.modules[__name__]).split("\ndef run(")[0])).encode()).hexdigest()[:12]
    path = BUILD / "frames" / f"{name}-{key}.npz"
    if path.exists():
        data = np.load(path)
        truth = [{"corners": c, "position": p, "rotation": r} for c, p, r in zip(data["corners"], data["position"], data["rotation"])]
        return data["images"], data["starts"], truth
    images, starts, truth = render(motion, frames, exposure_us, noise)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, images=images, starts=starts, corners=np.stack([t["corners"] for t in truth]), position=np.stack([t["position"] for t in truth]), rotation=np.stack([t["rotation"] for t in truth]))
    return images, starts, truth


def simulate(name, exposure_us, noise, extra):
    images, starts, truth = rendered(name, exposure_us, noise)
    return score(run(images, starts, extra=extra), truth, name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("scenarios", nargs="*", default=list(SCENARIOS))
    parser.add_argument("--exposure-us", type=float, default=10000)
    parser.add_argument("--noise", type=float, default=2.5)
    parser.add_argument("--save", type=Path, help="write the rendered frames of the first scenario as PNGs here")
    parser.add_argument("--harness", nargs="*", default=[], help="extra harness arguments (see harness.cpp)")
    args = parser.parse_args()
    if args.save:
        from PIL import Image
        args.save.mkdir(parents=True, exist_ok=True)
        for i, im in enumerate(rendered(args.scenarios[0], args.exposure_us, args.noise)[0]):
            Image.fromarray(im).save(args.save / f"{i:04d}.png")
    build()
    with ProcessPoolExecutor() as pool:
        rows = list(pool.map(simulate, args.scenarios, *zip(*[(args.exposure_us, args.noise, args.harness)] * len(args.scenarios))))
    keys = list(rows[0])
    print(" | ".join(keys))
    for row in rows:
        print(" | ".join(str(row[k]) for k in keys))
