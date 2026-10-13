"""Combine the current oriented projects into four editable objects on one plate."""
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as E
import json
import zipfile
import numpy as np

root = Path(__file__).parent / "stl"
ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
E.register_namespace("", ns)
q = lambda name: "{" + ns + "}" + name
parts = [
    ("electronics_deck", "Electronics deck - 90x70 board", 70, 110),
    ("camera_fixture", "Camera frame with lens fork", 155, 120),
    ("camera_retaining_bar", "PCB retaining bar", 155, 70),
    ("camera_lens_keeper", "Lens keeper", 155, 165),
]
with zipfile.ZipFile(root / "electronics_deck.3mf") as source:
    template = {name: source.read(name) for name in source.namelist()
                if name not in ["3D/3dmodel.model", "Metadata/model_settings.config"]}
settings = json.loads(template["Metadata/project_settings.config"])
assert settings["print_sequence"] == "by layer"
brim = float(settings["brim_width"])
model = E.Element(q("model"), {"unit": "millimeter"})
E.SubElement(model, q("metadata"), {"name": "Application"}).text = "Creality_Print V7.0.1.4212 Release"
E.SubElement(model, q("metadata"), {"name": "BambuStudio:3mfVersion"}).text = "1"
resources = E.SubElement(model, q("resources"))
build = E.SubElement(model, q("build"))
config = E.Element("config")
plate = E.Element("plate")
for key, value in [("plater_id", "1"), ("plater_name", "90x70 electronics deck and camera")]:
    E.SubElement(plate, "metadata", {"key": key, "value": value})
bounds = []
for index, (stem, name, x, y) in enumerate(parts, 1):
    with zipfile.ZipFile(root / (stem + ".3mf")) as source:
        original = E.fromstring(source.read("3D/3dmodel.model"))
    obj = deepcopy(original.find(q("resources") + "/" + q("object")))
    obj.set("id", str(index))
    obj.set("name", name)
    resources.append(obj)
    vertices = np.array([[float(v.get(axis)) for axis in ["x", "y", "z"]]
                         for v in obj.find(q("mesh") + "/" + q("vertices"))])
    low, high = vertices.min(0), vertices.max(0)
    assert abs(low[2]) < 0.00001
    box = (low[:2] + [x, y] - brim, high[:2] + [x, y] + brim)
    assert np.all(box[0] >= 5) and np.all(box[1] <= 215)
    for other in bounds:
        assert np.any(box[1] < other[0]) or np.any(other[1] < box[0]), "Brim envelopes overlap"
    bounds.append(box)
    E.SubElement(build, q("item"), {"objectid": str(index), "transform": f"1 0 0 0 1 0 0 0 1 {x} {y} 0", "printable": "1"})
    record = E.SubElement(config, "object", {"id": str(index)})
    for key, value in [("name", name), ("extruder", "1"), ("enable_support", "1" if stem == "electronics_deck" else "0")]:
        E.SubElement(record, "metadata", {"key": key, "value": value})
    part = E.SubElement(record, "part", {"id": str(index), "subtype": "normal_part"})
    E.SubElement(part, "metadata", {"key": "name", "value": name})
    instance = E.SubElement(plate, "model_instance")
    for key, value in [("object_id", str(index)), ("instance_id", "0"), ("identify_id", str(index))]:
        E.SubElement(instance, "metadata", {"key": key, "value": value})
    print(name, "size mm", (high - low).round(2).tolist(), "center", [x, y])
config.append(plate)
output = root / "electronics_deck_and_camera.3mf"
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
    for name, data in template.items():
        archive.writestr(name, data)
    archive.writestr("3D/3dmodel.model", E.tostring(model, encoding="utf-8", xml_declaration=True))
    archive.writestr("Metadata/model_settings.config", E.tostring(config, encoding="utf-8", xml_declaration=True))
with zipfile.ZipFile(output) as archive:
    assert archive.testzip() is None
    saved = E.fromstring(archive.read("3D/3dmodel.model"))
    assert len(saved.find(q("build"))) == 4
print("Four separate objects, all flat on the bed, with separated 6 mm brims.")
print(output)
