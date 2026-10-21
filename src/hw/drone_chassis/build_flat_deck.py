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
    (-29, -54.7),
    (29, -54.7),
    (29, -45.5),
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
    (-29, -45.5),
]
deck = (
    cq.Workplane("XY").workplane(offset=5.3).polyline(outline).close().extrude(3).val()
)
for y in [-10, 10]:
    deck = deck.cut(cylinder(0, y, 5, 1.75, 4))
for x in [-33, 33]:
    for y in [-8, 78]:
        deck = deck.cut(cylinder(x, y, 5, 1.2, 4))
        # Captive M2 nut in the underside: the screw comes down through perfboard, spacer and deck, so tightening clamps all three.
        deck = deck.cut(hex_prism(x, y, 5.3 - 0.2, 4.5, 2.0))
for x in [-19, 19]:
    for y in [-36, -24]:
        deck = deck.cut(cylinder(x, y, 5, 1.75, 4))
for x0, x1 in [(-28, -7), (7, 28)]:
    deck = deck.cut(block(x0, x1, 0, 68, 5, 9))
# 8 mm deep so the rear tab roots join a solid bar, not the 3 mm plate.
deck = deck.fuse(block(-35, 35, 58.7, 63.9, 0.3, 8.3))
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
# The camera end hangs two 4 mm tabs between the front landing frame and the side-frame mount bosses, clamped by the
# front landing-leg screws (M3 x 16 -> M3 x 20; the front frame sits 4 mm further forward).
# The tongue is 8 mm deep so the tabs grow out of a solid slab.
deck = deck.fuse(block(-29, 29, -54.7, -45.5, 0.3, 5.3), block(-29, -19, -54.7, -50.7, -19, 5.3), block(19, 29, -54.7, -50.7, -19, 5.3)).clean()
for x in [-24, 24]:
    deck = deck.cut(cq.Solid.makeCylinder(1.75, 5, cq.Vector(x, -55, -12), cq.Vector(0, 1, 0)))
edges = [edge for edge in deck.Edges() if cq.Vertex.makeVertex(0, -54.7, 8.3).distance(edge) < 1e-6]
assert len(edges) == 1, len(edges)
deck = deck.chamfer(1, None, edges)
deck = deck.clean()
assert deck.isValid() and len(deck.Solids()) == 1
# Validate empty bores, a flat top (nothing above z=8.3) and the M2 nut pockets.
bb = deck.BoundingBox()
assert abs(bb.zmax - 8.3) < 1e-6, bb.zmax
for y in [-10, 10]:
    assert deck.intersect(cylinder(0, y, 5.3, 1.7, 3)).Volume() < 1e-6
for x in [-33, 33]:
    for y in [-8, 78]:
        assert deck.intersect(hex_prism(x, y, 5.3, 4.3, 1.7)).Volume() < 1e-6
        assert deck.intersect(cylinder(x, y, 5.3, 1.1, 3)).Volume() < 1e-6
cq.exporters.export(
    deck, str(ROOT / "electronics_deck_flat.stl"), tolerance=0.01, angularTolerance=0.05
)
cq.exporters.export(deck, str(ROOT / "electronics_deck_flat.step"))
spacer = cylinder(0, 0, 0, 2.75, 5).cut(cylinder(0, 0, -1, 1.25, 7))
cq.exporters.export(cq.Workplane().add(spacer), str(ROOT / "perfboard_spacer.stl"), tolerance=0.01, angularTolerance=0.05)
print("Valid single solid with a flat top; bores and M2 nut pockets clear. Spacer: 5 mm tall, 5.5 mm OD, 2.5 mm bore.")
print("Volume mm3:", deck.Volume(), "Solid PLA equivalent g:", deck.Volume() * 0.00124)
