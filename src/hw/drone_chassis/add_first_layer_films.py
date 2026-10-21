"""Close deck-hole mouths with removable 0.4 mm print-only films."""
from pathlib import Path
import json
import trimesh

root = Path(__file__).parent / "stl"
deck = trimesh.load(root / "electronics_deck_six_mount.stl", force="mesh")
holes = ([(x, y, 3.15) for x in [-24, 24] for y in [-10, 10]]
         + [(0, y, 1.8) for y in [-10, 10]]
         + [(x, y, 1.25) for x in [-33, 33] for y in [-8, 78]]
         + [(x, y, 1.8) for x in [-19, 19] for y in [-36, -24]])
parts = [deck]
for x, y, radius in holes:
    film = trimesh.creation.cylinder(radius=radius, height=0.4, sections=96)
    film.apply_translation([x, y, 8.1])
    parts.append(film)
result = trimesh.boolean.union(parts, engine="manifold")
assert result.is_volume and len(result.split()) == 1
result.export(root / "electronics_deck_first_layer_films.stl")
(root / "first_layer_films.json").write_text(json.dumps({"thickness_mm": 0.4, "z_range_mm": [7.9, 8.3], "hole_centers_mm": [[x, y] for x, y, radius in holes], "remove_before_assembly": True}, indent=2))
