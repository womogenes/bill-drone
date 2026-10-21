"""
Creality Print project for the support-free deck and its four perfboard spacers, from the existing deck project's printer/filament settings.
"""

import json
import struct
import zipfile
from pathlib import Path
from xml.etree import ElementTree as E

import numpy as np

root = Path(__file__).parent / "stl"
with zipfile.ZipFile(root / "electronics_deck.3mf") as z:
    template = {n: z.read(n) for n in z.namelist() if n not in ["3D/3dmodel.model", "Metadata/model_settings.config"]}
settings = json.loads(template["Metadata/project_settings.config"])
# Flat top face down, tabs up: no supports. Four walls carry the tab roots; 0.28 mm layers and 20% infill keep it quick.
for key, value in {"enable_support": "0", "layer_height": "0.28", "wall_loops": "4", "sparse_infill_density": "20%", "top_shell_layers": "4", "bottom_shell_layers": "4"}.items():
    settings[key] = value
    changed = set(settings["different_settings_to_system"][0].split(";"))
    changed.add(key)
    settings["different_settings_to_system"][0] = ";".join(sorted(changed))
template["Metadata/project_settings.config"] = json.dumps(settings).encode()


def load(stem, flip):
    b = (root / (stem + ".stl")).read_bytes()
    count = struct.unpack_from("<I", b, 80)[0]
    assert len(b) == 84 + 50 * count
    a = np.array([struct.unpack_from("<12fH", b, 84 + i * 50)[3:12] for i in range(count)]).reshape(-1, 3)
    if flip:
        a = a * np.array([1, -1, -1])
    low, high = a.min(0), a.max(0)
    return a - np.array([(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]), high - low


ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
E.register_namespace("", ns)
model = E.Element("{" + ns + "}model", {"unit": "millimeter"})
E.SubElement(model, "{" + ns + "}metadata", {"name": "Application"}).text = "Creality_Print V7.0.1.4212 Release"
E.SubElement(model, "{" + ns + "}metadata", {"name": "BambuStudio:3mfVersion"}).text = "1"
resources = E.SubElement(model, "{" + ns + "}resources")
build = E.SubElement(model, "{" + ns + "}build")
config = E.Element("config")
plate = E.SubElement(config, "plate")
for k, v in [("plater_id", "1"), ("plater_name", "Flat electronics deck")]:
    E.SubElement(plate, "metadata", {"key": k, "value": v})
# Deck centred on the 220 mm bed; spacers in a row beside it.
items = [("electronics_deck_flat", "Flat electronics deck", True, (110, 110))] + [
    ("perfboard_spacer", f"Perfboard spacer {i + 1}", False, (175, 85 + 12 * i)) for i in range(4)]
for number, (stem, name, flip, (x, y)) in enumerate(items, start=1):
    a, size = load(stem, flip)
    vertices, index = np.unique(a, axis=0, return_inverse=True)
    triangles = index.reshape(-1, 3)
    edges = np.sort(np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]]), axis=1)
    assert np.all(np.unique(edges, axis=0, return_counts=True)[1] == 2), f"Non-manifold {stem}"
    obj = E.SubElement(resources, "{" + ns + "}object", {"id": str(number), "type": "model", "name": name})
    mesh = E.SubElement(obj, "{" + ns + "}mesh")
    vs = E.SubElement(mesh, "{" + ns + "}vertices")
    ts = E.SubElement(mesh, "{" + ns + "}triangles")
    for v in vertices:
        E.SubElement(vs, "{" + ns + "}vertex", dict(zip(["x", "y", "z"], map(str, v))))
    for t in triangles:
        E.SubElement(ts, "{" + ns + "}triangle", dict(zip(["v1", "v2", "v3"], map(str, t))))
    E.SubElement(build, "{" + ns + "}item", {"objectid": str(number), "transform": f"1 0 0 0 1 0 0 0 1 {x} {y} 0", "printable": "1"})
    entry = E.SubElement(config, "object", {"id": str(number)})
    for k, v in [("name", name), ("extruder", "1")]:
        E.SubElement(entry, "metadata", {"key": k, "value": v})
    part = E.SubElement(entry, "part", {"id": "1", "subtype": "normal_part"})
    E.SubElement(part, "metadata", {"key": "name", "value": name})
    instance = E.SubElement(plate, "model_instance")
    for k, v in [("object_id", str(number)), ("instance_id", "0"), ("identify_id", str(number))]:
        E.SubElement(instance, "metadata", {"key": k, "value": v})
    print(name, "envelope", size.round(1), "at", (x, y))
with zipfile.ZipFile(root / "electronics_deck_flat.3mf", "w", zipfile.ZIP_DEFLATED) as z:
    for k, v in template.items():
        z.writestr(k, v)
    z.writestr("3D/3dmodel.model", E.tostring(model, encoding="utf-8", xml_declaration=True))
    z.writestr("Metadata/model_settings.config", E.tostring(config, encoding="utf-8", xml_declaration=True))
