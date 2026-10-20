"""
AprilTag lens calibration for the dashboard: JSON in, JSON out.
Capture one tag16h5 marker at varied tilts and locations, then fit and save.
"""

import base64
import json
from pathlib import Path
import re
import sys

import cv2
import numpy as np

SIZE = (320, 240)
CONFIG = Path(__file__).with_name("src") / "config.h"


def calibrate(request):
    """
    Detect four ordered tag corners, or fit the captured views.
    """
    tag_id, size_mm = int(request["tag_id"]), float(request["tag_size_mm"])
    if not 0 <= tag_id < 30 or not np.isfinite(size_mm) or size_mm <= 0:
        raise ValueError(
            "Enter tag16h5 ID 0–29 and a positive black-square width in mm."
        )
    if "image" in request:
        gray = cv2.imdecode(
            np.frombuffer(base64.b64decode(request["image"]), np.uint8), 0
        )
        if gray is None or gray.shape[::-1] != SIZE:
            raise ValueError("Expected a 320×240 camera frame.")
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5), parameters
        )
        corners, ids, rejected = detector.detectMarkers(gray)
        found = [] if ids is None else ids.ravel().tolist()
        if found.count(tag_id) != 1:
            raise ValueError(f"Need one tag {tag_id}. Detected IDs: {found}.")
        return {"corners": corners[found.index(tag_id)].reshape(4, 2).tolist()}

    views = np.asarray(request["views"], np.float32)
    if (
        views.shape[1:] != (4, 2)
        or not 25 <= len(views) <= 60
        or not np.isfinite(views).all()
    ):
        raise ValueError("Capture 25–60 varied AprilTag views.")
    for i, view in enumerate(views):
        if any(np.linalg.norm(view - other, axis=1).mean() < 5 for other in views[:i]):
            raise ValueError(
                "Some views are too similar. Move or tilt the tag between captures."
            )
    square = np.float32([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]) * (size_mm / 1000)
    # Four corners/view: constrain the less observable distortion terms.
    rms, matrix, distortion, *rest = cv2.calibrateCameraExtended(
        [square] * len(views),
        list(views),
        SIZE,
        None,
        None,
        flags=cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
    )
    if (
        not np.isfinite(matrix).all()
        or not np.isfinite(distortion).all()
        or matrix[0, 0] <= 0
        or matrix[1, 1] <= 0
        or not 0 < matrix[0, 2] < 320
        or not 0 < matrix[1, 2] < 240
    ):
        raise ValueError(
            "Lens fit failed. Include varied tilts and positions near all image edges."
        )
    if not np.isfinite(rms) or rms > 1:
        raise ValueError(
            f"Reprojection error is {rms:.2f} px. Recapture sharper, varied views."
        )
    camera = dict(
        zip(
            ("fx", "fy", "cx", "cy"),
            map(float, (matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2])),
        )
    )
    camera.update(
        tag_id=tag_id, tag_size_mm=size_mm, distortion=distortion.ravel().tolist()
    )
    text = CONFIG.read_text()
    declarations = {
        r"float fx = [^;]+;": "float "
        + ", ".join(f"{key} = {camera[key]:.9f}f" for key in ("fx", "fy", "cx", "cy"))
        + ";",
        r"float distortion\[5\] = [^;]+;": "float distortion[5] = {"
        + ", ".join(f"{v:.9f}f" for v in camera["distortion"])
        + "};",
        r"float tagSizeM = [^;]+;": f"float tagSizeM = {size_mm / 1000:.9f}f;",
        r"int tagId = [^;]+;": f"int tagId = {tag_id};",
    }
    for pattern, replacement in declarations.items():
        text, count = re.subn(pattern, replacement, text)
        assert count == 1, "Camera configuration declaration missing"
    CONFIG.write_text(text)
    return {"rms": rms, "views": len(views), "camera": camera}


if __name__ == "__main__":
    try:
        print(json.dumps(calibrate(json.load(sys.stdin))))
    except (ValueError, KeyError, cv2.error) as error:
        print(json.dumps({"error": str(error)}))
        sys.exit(1)
