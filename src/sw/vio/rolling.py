"""
Rolling-shutter rectification from the gyro: each row of a frame is warped to where it would have been seen at the frame's stamp (the middle of its readout), undoing the camera's rotation between the two (its translation over a few milliseconds is left). OpenVINS models a global shutter.
"""

import cv2
import numpy as np


def rectify(gray, readout_s, omega_body, camera_to_body, matrix, bottom_first=False):
    """
    gray: the frame, stamped at the middle of its readout; readout_s: first row to last; omega_body: the gyro (rad/s, body axes) over the frame; camera_to_body: the OpenCV camera's axes in the body; matrix: the pinhole camera matrix; bottom_first: the sensor read the image's bottom row first (a frame turned by 180 degrees).
    """
    height, width = gray.shape
    omega = np.asarray(camera_to_body).T @ np.asarray(omega_body)
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    rays = np.linalg.inv(matrix) @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
    row = v.ravel()
    for _ in range(2):
        # The source row's time from the middle, then the camera's rotation back to it: a direction seen at the stamp as d is seen at time dt as exp(-omega dt) d.
        dt = ((height - 1 - row) if bottom_first else row) / (height - 1) * readout_s - readout_s / 2
        angle = -omega[:, None] * dt
        theta = np.linalg.norm(angle, axis=0)
        axis = angle / np.maximum(theta, 1e-12)
        cos, sin = np.cos(theta), np.sin(theta)
        rotated = rays * cos + np.cross(axis.T, rays.T).T * sin + axis * np.sum(axis * rays, axis=0) * (1 - cos)
        pixels = matrix @ rotated
        row = pixels[1] / pixels[2]
    map_x = (pixels[0] / pixels[2]).reshape(height, width).astype(np.float32)
    map_y = row.reshape(height, width).astype(np.float32)
    return cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
