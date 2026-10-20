"""
Render the exported camera clamp mesh without CAD or image generation.
"""

from pathlib import Path
import struct
import numpy as np
from PIL import Image, ImageDraw

root = Path(__file__).parent / "stl"
data = (root / "camera_bottom_clamp.stl").read_bytes()
count = struct.unpack_from("<I", data, 80)[0]
points = np.array(
    [struct.unpack_from("<12fH", data, 84 + i * 50)[3:12] for i in range(count)]
).reshape(-1, 3)
vertices, index = np.unique(points, axis=0, return_inverse=True)
faces = index.reshape(-1, 3)
edges = np.sort(
    np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1
)
assert np.all(np.unique(edges, axis=0, return_counts=True)[1] == 2)
view = np.array([0.65, -1.25, 1.15])
view /= np.linalg.norm(view)
right = np.cross([0, 0, 1], view)
right /= np.linalg.norm(right)
up = np.cross(view, right)
xy = np.stack([points @ right, points @ up], axis=1)
low, high = xy.min(0), xy.max(0)
scale = min(1000 / (high - low)[0], 620 / (high - low)[1])
image = Image.new("RGB", (1200, 800), "#eef3f6")
draw = ImageDraw.Draw(image)
for face in sorted(points.reshape(-1, 3, 3), key=lambda f: f.mean(0) @ view):
    normal = np.cross(face[1] - face[0], face[2] - face[0])
    normal /= max(np.linalg.norm(normal), 1e-12)
    shade = 0.5 + 0.45 * abs(normal @ view)
    color = tuple((np.array([48, 135, 182]) * shade).astype(int))
    coords = (np.stack([face @ right, face @ up], axis=1) - low) * scale
    coords[:, 0] += 100
    coords[:, 1] = 720 - coords[:, 1]
    draw.polygon([tuple(v) for v in coords], fill=color)
draw.text(
    (35, 25),
    "Camera bottom clamp - actual exported STL; board and screws omitted",
    fill="black",
)
image.save(root / "camera_bottom_clamp_preview.png")
print("Closed manifold; size mm:", points.max(0) - points.min(0))
