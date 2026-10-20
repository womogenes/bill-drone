"""Local CAD mirror of chassis.fs direct-mount deck; requires cadquery.

The existing bridge/frame and clamp screw seats are preserved. This builder is
also usable when Onshape session login is unavailable.
"""

from pathlib import Path
import math
import cadquery as cq

ROOT = Path(__file__).parent / "stl"


def block(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cylinder(x, y, z, radius, height):
    return cq.Solid.makeCylinder(radius, height, cq.Vector(x, y, z))


def hex_prism(x, y, z, flats, height):
    radius = flats / math.sqrt(3)
    points = [
        (x + radius * math.cos(i * math.pi / 3), y + radius * math.sin(i * math.pi / 3))
        for i in range(6)
    ]
    return (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(points)
        .close()
        .extrude(height)
        .val()
    )


outline = [
    (-23, -45.5),
    (23, -45.5),
    (25, -43.5),
    (25, -18),
    (38, -14),
    (40, -12),
    (40, 82),
    (38, 84),
    (-38, 84),
    (-40, 82),
    (-40, -12),
    (-38, -14),
    (-25, -18),
    (-25, -43.5),
]
deck = (
    cq.Workplane("XY").workplane(offset=5.3).polyline(outline).close().extrude(3).val()
)
for y in [-10, 10]:
    deck = deck.fuse(cylinder(0, y, 8.3, 5, 2.7))
    deck = deck.cut(cylinder(0, y, 5, 1.75, 7))
    deck = deck.cut(cylinder(0, y, 9.3, 3.3, 3))
for x in [-33, 33]:
    for y in [-8, 78]:
        deck = deck.fuse(cylinder(x, y, 8.3, 4, 5))
        deck = deck.cut(cylinder(x, y, 5, 1.2, 9))
        deck = deck.cut(hex_prism(x, y, 9.3, 4.5, 2))
        deck = deck.cut(
            block(
                x if x > 0 else x - 4.1,
                x + 4.1 if x > 0 else x,
                y - 2.25,
                y + 2.25,
                9.3,
                11.3,
            )
        )
for x in [-19, 19]:
    for y in [-36, -24]:
        deck = deck.cut(cylinder(x, y, 5, 1.75, 4))
for x0, x1 in [(-28, -7), (7, 28)]:
    deck = deck.cut(block(x0, x1, 0, 68, 5, 9))
deck = deck.fuse(block(-35, 35, 58.7, 63.9, 5.3, 8.3))
for side in [-1, 1]:
    x = side * 24
    tab = block(x - 5, x + 5, 58.7, 63.9, -18, 8.3)
    edges = [
        edge
        for edge in tab.Edges()
        if abs(edge.Center().z + 18) < 1e-6 and abs(edge.Center().y - 61.3) < 1e-6
    ]
    tab = tab.chamfer(1, None, edges)
    rib = (
        cq.Workplane("YZ", origin=(26 if side > 0 else -29, 0, 0))
        .polyline([(63.9, -8), (77.2, 5.3), (63.9, 5.3)])
        .close()
        .extrude(3)
        .val()
    )
    deck = deck.fuse(tab, rib)
    deck = deck.cut(
        cq.Solid.makeCylinder(1.75, 6, cq.Vector(x, 64.2, -12), cq.Vector(0, -1, 0))
    )
deck = deck.clean()
assert deck.isValid() and len(deck.Solids()) == 1
# Validate empty through bores, seated head clearance, and nominal M2 nut fit.
for y in [-10, 10]:
    assert deck.intersect(cylinder(0, y, 5.3, 1.7, 5.7)).Volume() < 1e-6
    assert deck.intersect(cylinder(0, y, 9.3, 3, 1.7)).Volume() < 1e-6
for x in [-33, 33]:
    for y in [-8, 78]:
        assert deck.intersect(hex_prism(x, y, 9.5, 4, 1.6)).Volume() < 1e-6
        assert deck.intersect(cylinder(x, y, 5.3, 1, 8)).Volume() < 1e-6
cq.exporters.export(
    deck, str(ROOT / "electronics_deck.stl"), tolerance=0.01, angularTolerance=0.05
)
cq.exporters.export(deck, str(ROOT / "electronics_deck.step"))
print("Valid single solid; clamp bores and M2 nut envelopes clear.")
print("Volume mm3:", deck.Volume(), "Solid PLA equivalent g:", deck.Volume() * 0.00124)
