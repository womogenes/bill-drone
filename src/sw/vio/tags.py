"""
Ground truth for a recorded visual-inertial dataset (sw/vio/recorded.py) from the AprilTags in its frames, and an estimate's error against it.

uv run --with opencv-contrib-python-headless -m sw.vio.tags <dataset> [--size 0.150] [--estimate <dataset>/estimate.txt]

Finds every tag16h5 tag in the frames; calibrates the camera from their corners, starting from the dataset's own calibration; places the tags relative to the most-seen one from frames that see two; and gives the camera's pose in that tag's frame wherever a placed tag is in view (truth.csv: time, camera position in metres, camera-to-world quaternion w, x, y, z). With an estimate (OpenVINS's IMU poses, sw/vio/openvins/run_euroc.cpp), the estimate's camera positions are aligned to the truth by the rotation and translation that fit best (no scale) and the position error is reported over time.
"""

import argparse
import collections
import csv
from pathlib import Path

import cv2
import numpy as np
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
        grey = cv2.imread(str(Path(folder) / "mav0/cam0/data" / name), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = detector.detectMarkers(grey)
        for corner, tag in zip(corners or [], [] if ids is None else ids.ravel()):
            c = corner[0]
            if np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)) >= MIN_SIDE_PX:
                found.append((ns / 1e9, int(tag), c.astype(np.float64)))
    return found


def square(size):
    half = size / 2
    return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float64)


def calibrate(found, size, matrix, distortion, shape, views=400):
    """The camera matrix and (k1, k2) refined on up to views tag sightings spread through the recording."""
    pick = found[:: max(1, len(found) // views)]
    objects = [square(size).astype(np.float32) for _ in pick]
    images = [c.astype(np.float32) for _, _, c in pick]
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    error, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, shape, matrix.copy(), distortion.copy(), flags=flags)
    return error, matrix, distortion


def pose(corners, size, matrix, distortion):
    """The tag's pose in the camera (4x4) from its corners."""
    ok, rvec, tvec = cv2.solvePnP(square(size), corners, matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    return transform if ok else None


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
    args = parser.parse_args()
    folder = args.dataset.expanduser()
    camera = yaml.safe_load((folder / "kalibr_imucam_chain.yaml").read_text().replace("%YAML:1.0", ""))["cam0"]
    fx, fy, cx, cy = camera["intrinsics"]
    matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
    distortion = np.array(camera["distortion_coeffs"] + [0.0], np.float64)
    found = detect(folder)
    print(f"{len(found)} tag sightings, tags {dict(sorted(collections.Counter(tag for _, tag, _ in found).items()))}")
    # One tag seen briefly calibrates nothing: 64 sightings of one tag put the principal point 13 px from the image's edge.
    if len(found) >= 200 and len({tag for _, tag, _ in found}) >= 3:
        error, refined, refined_distortion = calibrate(found, args.size, matrix, distortion, tuple(camera["resolution"]))
        width, height = camera["resolution"]
        if abs(refined[0, 2] - width / 2) < width / 8 and abs(refined[1, 2] - height / 2) < height / 8:
            matrix, distortion = refined, refined_distortion
        else:
            print(f"calibration rejected: principal point ({refined[0, 2]:.0f}, {refined[1, 2]:.0f}) too far from the centre")
    else:
        error = float("nan")
        print("too few tags to calibrate: the dataset's calibration stays")
    print(f"calibration: reprojection {error:.2f} px, fx {matrix[0, 0]:.1f} fy {matrix[1, 1]:.1f} cx {matrix[0, 2]:.1f} cy {matrix[1, 2]:.1f} k1 {distortion.ravel()[0]:.3f} k2 {distortion.ravel()[1]:.3f}")
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
