"""
Read the power supply and scale from the ESP32-CAM's latest frame.

[codex vibe code hell yeah]
"""

import argparse
from datetime import datetime
from functools import cache
from pathlib import Path
from threading import Condition, Thread

import cv2
import numpy as np
from websocket import create_connection

cv2.setNumThreads(1)
CAM_ADDR = "ws://192.168.86.33:8082/ws/stream"
# Clockwise corners in the 1024x768 reference, starting at each display's upright top-left.
QUADS = {
    "supply": [(185, 688), (7, 623), (79, 501), (257, 566)],
    "scale": [(461, 482), (512, 337), (590, 363), (539, 508)],
}
# Match the instrument bodies, excluding digits that change between frames.
HOUSINGS = {
    "supply": [(0, 320), (360, 390), (265, 748), (0, 650)],
    "scale": [(640, 280), (800, 320), (730, 535), (570, 495)],
}
CODES = dict(zip(("1111110", "0110000", "1101101", "1111001", "0110011",
                 "1011011", "1011111", "1110000", "1111111", "1111011", "1000111"),
                "0123456789F"))
REGIONS = ((.25, 0, .75, .2), (.7, .15, 1, .45), (.6, .55, .95, .85),
           (0, .8, .45, 1), (0, .55, .3, .85), (0, .15, .3, .45), (.25, .4, .65, .6))


class Camera:
    def __init__(self):
        self.ws = create_connection(CAM_ADDR)
        self.ws.send("stream")
        self.ready = Condition()
        self.latest = self.error = None
        Thread(target=self.receive, daemon=True).start()

    def receive(self):
        try:
            while packet := self.ws.recv():
                with self.ready:
                    self.latest = packet
                    self.ready.notify()
            raise ConnectionError("Camera stream closed")
        except Exception as error:
            # Forward worker failures to the caller instead of leaving it waiting.
            with self.ready:
                self.error = error
                self.ready.notify()

    def read(self):
        with self.ready:
            self.ready.wait_for(lambda: self.latest is not None or self.error is not None)
            if self.error is not None:
                raise self.error
            packet = self.latest
            self.latest = None
        return cv2.imdecode(np.frombuffer(packet, np.uint8, offset=16), cv2.IMREAD_COLOR)

    def close(self):
        self.ws.close()


@cache
def camera():
    return Camera()


def get_frame():
    """
    Decode the newest JPEG; intermediate frames are discarded while parsing.
    """
    return camera().read()


WARPS = {}


def calibrate(gray):
    """
    Match both instrument bodies once at startup; restart after repositioning.
    """
    reference = cv2.imread(str(Path(__file__).with_name("alignment.png")), 0)
    sift = cv2.SIFT_create(contrastThreshold=.015)
    current, descriptors = sift.detectAndCompute(gray, None)
    for name, corners in QUADS.items():
        mask = np.zeros_like(reference)
        cv2.fillConvexPoly(mask, np.int32(HOUSINGS[name]), 255)
        cv2.fillConvexPoly(mask, np.int32(corners), 0)
        keys, template = sift.detectAndCompute(reference, mask)
        pairs = cv2.BFMatcher().knnMatch(template, descriptors, k=2)
        matches = [a for a, b in pairs if a.distance < .75 * b.distance]
        source = np.float32([keys[m.queryIdx].pt for m in matches])
        target = np.float32([current[m.trainIdx].pt for m in matches])
        transform, inliers = cv2.estimateAffine2D(source, target, ransacReprojThreshold=3)
        if transform is None or inliers.sum() < 8:
            raise RuntimeError(f"Cannot calibrate {name}; keep its whole display and housing visible")
        located = cv2.transform(np.float32([corners]), transform)[0]
        w, h = (220, 190) if name == "supply" else (190, 90)
        WARPS[name] = cv2.getPerspectiveTransform(located, np.float32([(0, 0), (w, 0), (w, h), (0, h)]))


def digit(gray, scale=False):
    """
    Remove uneven lighting, threshold, trim the glyph, and read its seven segments.
    """
    contrast = cv2.subtract(gray, cv2.GaussianBlur(gray, (0, 0), 9))
    if np.percentile(contrast, 95) < max(10, np.median(gray) * .15):
        return " "
    if scale:
        # Fading LCD segments have sharp edges too; use their brightness instead.
        binary = np.uint8(gray > (np.median(gray) + np.percentile(gray, 98)) / 2) * 255
    else:
        level, _ = cv2.threshold(contrast, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary = np.uint8(contrast > level * .85) * 255
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.zeros_like(binary)
    min_area = gray.shape[0] ** 2 * .004
    for contour in contours:
        if cv2.contourArea(contour) > min_area and max(cv2.boundingRect(contour)[2:]) > gray.shape[0] * .2:
            cv2.drawContours(mask, [contour], -1, 255, -1)
    points = cv2.findNonZero(mask)
    if points is None:
        return "?"
    x, y, w, h = cv2.boundingRect(points)
    sides = cv2.minAreaRect(points)[1]
    if min(sides) < max(sides) * .35:
        return "1"
    glyph = cv2.resize(cv2.bitwise_and(binary, mask)[y:y+h, x:x+w], (40, 64))
    bits = "".join("1" if glyph[int(t*64):int(b*64), int(l*40):int(r*40)].mean() > 255*.25
                   else "0" for l, t, r, b in REGIONS)
    return CODES.get(bits, "?")


def read_row(gray, scale=False):
    """
    Find digit columns rather than assuming fixed character spacing.
    """
    contrast = cv2.subtract(gray, cv2.GaussianBlur(gray, (0, 0), 9))
    if np.percentile(contrast, 95) < max(8, np.median(gray) * (.25 if scale else .15)):
        return ""
    if scale:
        return "".join(digit(part, scale=True) for part in np.array_split(gray, 3, axis=1))
    level, _ = cv2.threshold(contrast, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary = np.uint8(contrast > level * .85) * 255
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.zeros_like(binary)
    for contour in contours:
        if cv2.contourArea(contour) > gray.shape[0] ** 2 * .003:
            cv2.drawContours(mask, [contour], -1, 255, -1)
    edges = np.diff(np.r_[False, mask.any(axis=0), False].astype(int))
    text = ""
    for left, right in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        _, _, _, height = cv2.boundingRect(cv2.findNonZero(mask[:, left:right]))
        if height > gray.shape[0] * .35:
            text += digit(gray[:, max(0, left-1):min(gray.shape[1], right+1)])
    return text


def number(text, divisor):
    return int(text) / divisor if text.isdecimal() else None


def parse_frame(frame):
    """
    Unknown numbers return None. OFF supply values are displayed setpoints.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if not WARPS:
        calibrate(gray)
    supply = cv2.warpPerspective(gray, WARPS["supply"], (220, 190))
    scale = cv2.warpPerspective(gray, WARPS["scale"], (190, 90))
    rows = [read_row(supply[y:y+60]) for y in (5, 65, 125)]
    weight = read_row(scale, True)
    supply_on = False if "F" in rows[2] else True if rows[2].isdecimal() else None
    scale_on = bool(weight.strip())
    return dict(supply_volts=number(rows[0], 100) if len(rows[0]) == 4 else None,
                supply_amps=number(rows[1], 100 if supply_on is False else 1000) if supply_on is not None and len(rows[1]) == 4 else None,
                supply_watts=0.0 if supply_on is False else number(rows[2], 10),
                scale_kg=number(weight, 100) if len(weight) == 3 else None, supply_on=supply_on, scale_on=scale_on)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Yay
    parser.add_argument("--live", action="store_true", help="Print readings continuously")
    args = parser.parse_args()

    if args.live:
        while True:
            print(parse_frame(get_frame()), flush=True)

    else:
        frame = get_frame()
        logs = Path(__file__).parent / "logs"
        logs.mkdir(exist_ok=True)
        path = logs / f"frame_{datetime.now():%Y%m%d_%H%M%S_%f}.png"
        cv2.imwrite(str(path), frame)
        camera().close()
        print(path)
