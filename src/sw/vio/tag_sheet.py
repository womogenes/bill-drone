"""
A page of tag16h5 tags for calibrating the camera and placing ground truth (sw/vio/tags.py): print it at 100% or show it full screen, then measure one tag's black square and pass that as tags.py --size.

uv run --with opencv-contrib-python-headless --with pillow -m sw.vio.tag_sheet --out ~/sw/vio/tags.pdf [--ids 1 2 3 4 5 6] [--square-mm 60]

The tags are laid out three by two on a Letter page (fits A4) with a white margin of at least two cells around each, as detection needs. The camera tracks any tag16h5 ID, so keep these out of view while the drone holds on its own tag.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

DPI = 300
PAGE_MM = (215.9, 279.4)
CELLS = 6  # tag16h5: 4x4 data cells inside a one-cell black border


def px(mm):
    return round(mm / 25.4 * DPI)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    parser.add_argument("--square-mm", type=float, default=60.0)
    args = parser.parse_args()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    page = Image.new("L", (px(PAGE_MM[0]), px(PAGE_MM[1])), 255)
    draw = ImageDraw.Draw(page)
    side = px(args.square_mm)
    columns, rows = 2, 3
    for n, tag in enumerate(args.ids[: columns * rows]):
        x = round((n % columns + 0.5) * page.width / columns - side / 2)
        y = round((n // columns + 0.5) * page.height / rows - side / 2)
        marker = cv2.resize(cv2.aruco.generateImageMarker(dictionary, tag, CELLS), (side, side), interpolation=cv2.INTER_NEAREST)
        page.paste(Image.fromarray(np.ascontiguousarray(marker)), (x, y))
        draw.text((x, y + side + px(3)), f"tag16h5 id {tag}, black square {args.square_mm:g} mm", fill=0)
    out = args.out.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    page.save(out, resolution=DPI)
    print(f"{out}: ids {args.ids[: columns * rows]}, {args.square_mm:g} mm squares")


if __name__ == "__main__":
    main()
