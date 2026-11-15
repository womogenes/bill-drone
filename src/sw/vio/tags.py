"""
Ground truth for a recorded visual-inertial dataset (sw/vio/recorded.py) from the AprilTags in its frames, and an estimate's error against it.

uv run --with opencv-contrib-python-headless -m sw.vio.tags <dataset> [--size 0.150] [--estimate <dataset>/estimate.txt]

Finds every tag16h5 tag in the frames; calibrates the camera from their corners, starting from the dataset's own calibration; places the tags relative to the most-seen one from frames that see two; and gives the camera's pose in that tag's frame wherever a placed tag is in view (truth.csv: time, camera position in meters, camera-to-world quaternion w, x, y, z). With an estimate (OpenVINS's IMU poses, sw/vio/openvins/run_euroc.cpp), the estimate's camera positions are aligned to the truth by the rotation and translation that fit best (no scale) and the position error is reported over time.
"""

import argparse
import collections
import csv
import re
from pathlib import Path

import cv2
import numpy as np
import scipy.optimize
import scipy.sparse
import yaml

from sw.vio import dataset

DICTIONARY = cv2.aruco.DICT_APRILTAG_16h5
# Smaller tags in the image than this are too coarse to place.
MIN_SIDE_PX = 20


def detect(folder):
    """(time s, tag id, 4x2 corners) for every tag in the frames."""
    parameters = cv2.aruco.DetectorParameters()
    # tag16h5 has only 16 data bits, so any correction lets wallpaper and posters through as tags (2026-10-31: IDs never printed).
    parameters.errorCorrectionRate = 0
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), parameters)
    found = []
    for ns, name in dataset.read(folder, "cam0/data.csv"):
        gray = cv2.imread(str(Path(folder) / "mav0/cam0/data" / name), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = detector.detectMarkers(gray)
        for corner, tag in zip(corners or [], [] if ids is None else ids.ravel()):
            c = corner[0]
            if np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)) >= MIN_SIDE_PX:
                found.append((ns / 1e9, int(tag), c.astype(np.float64)))
    return found


def square(size):
    half = size / 2
    return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float64)


def rotate(vectors, points):
    """Points (n, 3) turned by rotation vectors (n, 3), row by row (Rodrigues)."""
    theta = np.linalg.norm(vectors, axis=1, keepdims=True)
    axis = vectors / np.maximum(theta, 1e-12)
    cos, sin = np.cos(theta), np.sin(theta)
    return points * cos + np.cross(axis, points) * sin + axis * np.sum(axis * points, axis=1, keepdims=True) * (1 - cos)


def calibrate(found, size, matrix, distortion, shape, views=100):
    """
    The camera matrix and k1 (k2 held at 0) by bundle adjustment over up to views frames: the intrinsics, every tag's pose relative to the most-seen one (the tags must stay put) and each frame's camera pose, fitted together to all the tags' corners. On synthetic 640x480 frames of four tags it finds fx 698.4, fy 700.3, cx 329.8, cy 235.5 against the true 700, 702, 330, 236 from any starting guess. Fitting each sighting on its own instead leaves the focal length trading against distance. Even fitted together, tags close to each other and small in the image leave it loose: on 2026-11-14's 480x320 recording of three tags 30-45 cm apart the fit was within 0.03 px for any fx from 450 to 900, so spread the tags across the whole view.
    """
    seen, corners = collections.defaultdict(dict), collections.defaultdict(dict)
    for time, tag, c in found:
        transform = pose(c, size, matrix, distortion)
        if transform is not None:
            seen[time][tag], corners[time][tag] = transform, c
    reference, placed = place(seen)
    tag_ids = [tag for tag in placed if tag != reference]
    frames = [time for time in sorted(corners) if corners[time].keys() & placed.keys()]
    frames = frames[:: max(1, len(frames) // views)]
    # Each observation: its frame's index, its tag's index (-1 the reference), its four corners.
    rows = [(f, tag_ids.index(tag) if tag != reference else -1, corners[time][tag]) for f, time in enumerate(frames) for tag in corners[time] if tag in placed]
    frame_of = np.repeat([r[0] for r in rows], 4)
    tag_of = np.repeat([r[1] for r in rows], 4)
    seen_px = np.concatenate([r[2] for r in rows])
    local = np.tile(square(size), (len(rows), 1))
    start = [matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2], distortion.ravel()[0], distortion.ravel()[1]]
    for tag in tag_ids:
        start += [*cv2.Rodrigues(placed[tag][:3, :3])[0].ravel(), *placed[tag][:3, 3]]
    for time in frames:
        # The camera from the frame's tags in the reference tag's frame (a global solver: the iterative one found wrong minima on these nearly flat layouts).
        points = np.concatenate([(placed[tag] @ np.c_[square(size), np.ones(4)].T).T[:, :3] for tag in corners[time] if tag in placed])
        image = np.concatenate([corners[time][tag] for tag in corners[time] if tag in placed])
        _, rvec, tvec = cv2.solvePnP(points, image, matrix, distortion, flags=cv2.SOLVEPNP_SQPNP)
        start += [*rvec.ravel(), *tvec.ravel()]
    start = np.array(start)
    tag_base, frame_base = 6, 6 + 6 * len(tag_ids)

    def residual(x):
        fx, fy, cx, cy, k1 = x[:5]
        k2 = 0.0
        tag_pose = x[tag_base:frame_base].reshape(-1, 6)
        frame_pose = x[frame_base:].reshape(-1, 6)
        world = local.copy()
        moved = tag_of >= 0
        if moved.any():
            world[moved] = rotate(tag_pose[tag_of[moved], :3], local[moved]) + tag_pose[tag_of[moved], 3:]
        camera = rotate(frame_pose[frame_of, :3], world) + frame_pose[frame_of, 3:]
        x_n, y_n = camera[:, 0] / camera[:, 2], camera[:, 1] / camera[:, 2]
        r2 = x_n ** 2 + y_n ** 2
        scale = 1 + k1 * r2 + k2 * r2 ** 2
        return np.concatenate([fx * x_n * scale + cx - seen_px[:, 0], fy * y_n * scale + cy - seen_px[:, 1]])

    # Which parameters each residual depends on, for the sparse solver.
    n = len(seen_px)
    sparsity = scipy.sparse.lil_matrix((2 * n, len(start)), dtype=int)
    sparsity[:, :6] = 1
    for i in range(n):
        for row in (i, n + i):
            sparsity[row, frame_base + 6 * frame_of[i]:frame_base + 6 * frame_of[i] + 6] = 1
            if tag_of[i] >= 0:
                sparsity[row, tag_base + 6 * tag_of[i]:tag_base + 6 * tag_of[i] + 6] = 1
    fit = scipy.optimize.least_squares(residual, start, method="lm")
    fx, fy, cx, cy, k1, k2 = fit.x[:6]
    k2 = 0.0
    error = float(np.sqrt(np.mean(residual(fit.x).reshape(2, -1).T ** 2) * 2))
    return error, np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), np.array([k1, k2, 0.0, 0.0, 0.0])


def pose(corners, size, matrix, distortion):
    """The tag's pose in the camera (4x4) from its corners."""
    ok, rvec, tvec = cv2.solvePnP(square(size), corners, matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(tvec).all():
        return None
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    return transform


def mean_transform(transforms):
    """A robust mean of rigid transforms: the median translation and the rotation nearest the mean of the rotation matrices."""
    t = np.median([m[:3, 3] for m in transforms], axis=0)
    u, _, vt = np.linalg.svd(np.mean([m[:3, :3] for m in transforms], axis=0))
    out = np.eye(4)
    out[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    out[:3, 3] = t
    return out


def place(seen):
    """Each tag's pose in the most-seen tag's frame, from frames that see two (breadth first). seen: time -> {tag: tag-in-camera}."""
    counts = collections.Counter(tag for tags in seen.values() for tag in tags)
    reference = counts.most_common(1)[0][0]
    placed = {reference: np.eye(4)}
    while True:
        between = collections.defaultdict(list)
        for tags in seen.values():
            for known in tags.keys() & placed.keys():
                for other in tags.keys() - placed.keys():
                    # other in the reference frame = known-in-reference . camera-in-known . other-in-camera
                    between[other].append(placed[known] @ np.linalg.inv(tags[known]) @ tags[other])
        if not between:
            return reference, placed
        best = max(between, key=lambda tag: len(between[tag]))
        placed[best] = mean_transform(between[best])


def truth(seen, placed):
    """(time, camera-in-world 4x4) for every frame that sees a placed tag."""
    out = []
    for time, tags in sorted(seen.items()):
        estimates = [placed[tag] @ np.linalg.inv(transform) for tag, transform in tags.items() if tag in placed]
        if estimates:
            out.append((time, mean_transform(estimates)))
    return out


def align(source, target):
    """The rotation and translation taking source points onto target ones in least squares (Kabsch)."""
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    rotation = (u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt).T
    return rotation, b - rotation @ a


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--size", type=float, default=0.150, help="the tags' black square (m)")
    parser.add_argument("--estimate", type=Path)
    parser.add_argument("--save", action="store_true", help="write the calibration fitted from the tags into the dataset's kalibr_imucam_chain.yaml")
    args = parser.parse_args()
    folder = args.dataset.expanduser()
    camera = yaml.safe_load((folder / "kalibr_imucam_chain.yaml").read_text().replace("%YAML:1.0", ""))["cam0"]
    fx, fy, cx, cy = camera["intrinsics"]
    matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
    distortion = np.array(camera["distortion_coeffs"] + [0.0], np.float64)
    found = detect(folder)
    print(f"{len(found)} tag sightings, tags {dict(sorted(collections.Counter(tag for _, tag, _ in found).items()))}")
    # One tag alone leaves the focal length trading against distance (64 sightings of one put the principal point 13 px from the image's edge; 1175 on 2026-11-14 gave 890-1030 px with the starting guess): calibration needs two or more fixed tags seen together.
    if len(found) >= 200 and len({tag for _, tag, _ in found}) >= 2:
        error, refined, refined_distortion = calibrate(found, args.size, matrix, distortion, tuple(camera["resolution"]))
        width, height = camera["resolution"]
        if abs(refined[0, 2] - width / 2) < width / 8 and abs(refined[1, 2] - height / 2) < height / 8:
            matrix, distortion = refined, refined_distortion
        else:
            print(f"calibration rejected: principal point ({refined[0, 2]:.0f}, {refined[1, 2]:.0f}) too far from the center")
    else:
        error = float("nan")
        print("too few tags to calibrate: the dataset's calibration stays")
    print(f"calibration: reprojection {error:.2f} px, fx {matrix[0, 0]:.1f} fy {matrix[1, 1]:.1f} cx {matrix[0, 2]:.1f} cy {matrix[1, 2]:.1f} k1 {distortion.ravel()[0]:.3f} k2 {distortion.ravel()[1]:.3f}")
    if args.save and np.isfinite(error):
        path = folder / "kalibr_imucam_chain.yaml"
        text = path.read_text()
        text = re.sub(r"intrinsics: \[.*\]", f"intrinsics: [{matrix[0, 0]:.3f}, {matrix[1, 1]:.3f}, {matrix[0, 2]:.3f}, {matrix[1, 2]:.3f}]", text)
        text = re.sub(r"distortion_coeffs: \[.*\]", "distortion_coeffs: [" + ", ".join(f"{v:.6f}" for v in distortion.ravel()[:4]) + "]", text)
        path.write_text(text)
        print(f"saved to {path}")
    seen = collections.defaultdict(dict)
    for time, tag, corners in found:
        transform = pose(corners, args.size, matrix, distortion)
        if transform is not None:
            seen[time][tag] = transform
    reference, placed = place(seen)
    print(f"tags placed relative to tag {reference}: " + ", ".join(f"{tag} at {np.round(m[:3, 3], 2).tolist()}" for tag, m in sorted(placed.items())))
    poses = truth(seen, placed)
    with open(folder / "truth.csv", "w", newline="") as file:
        rows = csv.writer(file)
        rows.writerow(["time_s", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for time, m in poses:
            rows.writerow([f"{time:.6f}", *np.round(m[:3, 3], 4), *np.round(dataset.quaternion(m[:3, :3]), 6)])
    times = np.array([t for t, _ in poses])
    print(f"ground truth in {len(poses)} frames, covering {len(np.unique(times.astype(int)))} of the recording's seconds")
    if args.estimate:
        estimate = np.loadtxt(args.estimate.expanduser(), delimiter=",")
        imu_cam = np.array(camera["T_imu_cam"])
        cams = []
        for row in estimate:
            body = np.eye(4)
            w, x, y, z = row[4:8]
            body[:3, :3] = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
            body[:3, 3] = row[1:4]
            cams.append((body @ imu_cam)[:3, 3])
        cams = np.array(cams)
        inside = (times >= estimate[0, 0]) & (times <= estimate[-1, 0])
        if inside.sum() < 10:
            print("too little ground truth within the estimate")
            return
        est = np.array([np.interp(times[inside], estimate[:, 0], cams[:, k]) for k in range(3)]).T
        tru = np.array([m[:3, 3] for (_, m), keep in zip(poses, inside) if keep])
        rotation, shift = align(est, tru)
        error = np.linalg.norm(est @ rotation.T + shift - tru, axis=1)
        print(f"estimate against truth over {inside.sum()} frames: position error RMS {np.sqrt(np.mean(error ** 2)):.2f} m, median {np.median(error):.2f} m, max {error.max():.2f} m")
        for second in np.unique(times[inside].astype(int))[::5]:
            pick = times[inside].astype(int) == second
            print(f"  t {second:4d} s: {error[pick].mean():.2f} m")


if __name__ == "__main__":
    main()
