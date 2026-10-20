"""
Combine the current oriented projects into four editable objects on one plate.
"""

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
    ("legs", "Reinforced landing frame 1", 87, 54),
    ("legs", "Reinforced landing frame 2", 87, 151),
    ("cradle", "M2 horn motor cradle 1", 192, 55),
    ("cradle", "M2 horn motor cradle 2", 192, 143),
]
projects = {
    "legs": root / "reinforced_landing_legs_pair.3mf",
    "cradle": root.parent.parent / "end_attachment/stl/motor_cradles_m2_pair.3mf",
}
with zipfile.ZipFile(projects["legs"]) as source:
    template = {
        name: source.read(name)
        for name in source.namelist()
        if name not in ["3D/3dmodel.model", "Metadata/model_settings.config"]
    }
settings = json.loads(template["Metadata/project_settings.config"])
assert settings["print_sequence"] == "by layer"
settings["brim_width"] = "4"
template["Metadata/project_settings.config"] = json.dumps(settings).encode()
model = E.Element(q("model"), {"unit": "millimeter"})
E.SubElement(
    model, q("metadata"), {"name": "Application"}
).text = "Creality_Print V7.0.1.4212 Release"
E.SubElement(model, q("metadata"), {"name": "BambuStudio:3mfVersion"}).text = "1"
resources = E.SubElement(model, q("resources"))
build = E.SubElement(model, q("build"))
config = E.Element("config")
plate = E.Element("plate")
for key, value in [
    ("plater_id", "1"),
    ("plater_name", "Reinforced legs and M2 motor cradles"),
]:
    E.SubElement(plate, "metadata", {"key": key, "value": value})
bounds = []
for index, (stem, name, x, y) in enumerate(parts, 1):
    with zipfile.ZipFile(projects[stem]) as source:
        original = E.fromstring(source.read("3D/3dmodel.model"))
    obj = deepcopy(original.find(q("resources") + "/" + q("object")))
    obj.set("id", str(index))
    obj.set("name", name)
    resources.append(obj)
    vertices = np.array(
        [
            [float(v.get(axis)) for axis in ["x", "y", "z"]]
            for v in obj.find(q("mesh") + "/" + q("vertices"))
        ]
    )
    if stem == "cradle":
        vertices = vertices[:, [1, 0, 2]] * [-1, 1, 1]
        for element, vertex in zip(obj.find(q("mesh") + "/" + q("vertices")), vertices):
            for axis, value in zip(["x", "y", "z"], vertex):
                element.set(axis, str(value))
    brim = 4 if stem == "legs" else 3
    low, high = vertices.min(0), vertices.max(0)
    assert abs(low[2]) < 0.00001
    box = (low[:2] + [x, y] - brim, high[:2] + [x, y] + brim)
    assert np.all(box[0] >= 5) and np.all(box[1] <= 215)
    for other in bounds:
        assert np.any(box[1] < other[0]) or np.any(other[1] < box[0]), (
            "Brim envelopes overlap"
        )
    bounds.append(box)
    E.SubElement(
        build,
        q("item"),
        {
            "objectid": str(index),
            "transform": f"1 0 0 0 1 0 0 0 1 {x} {y} 0",
            "printable": "1",
        },
    )
    record = E.SubElement(config, "object", {"id": str(index)})
    for key, value in [
        ("name", name),
        ("extruder", "1"),
        ("enable_support", "1" if stem == "cradle" else "0"),
        ("brim_width", str(brim)),
        ("support_type", "tree(auto)"),
        ("support_style", "organic"),
        ("wall_loops", "6" if stem == "legs" else "4"),
        ("sparse_infill_density", "35%" if stem == "legs" else "25%"),
    ]:
        E.SubElement(record, "metadata", {"key": key, "value": value})
    part = E.SubElement(record, "part", {"id": str(index), "subtype": "normal_part"})
    E.SubElement(part, "metadata", {"key": "name", "value": name})
    instance = E.SubElement(plate, "model_instance")
    for key, value in [
        ("object_id", str(index)),
        ("instance_id", "0"),
        ("identify_id", str(index)),
    ]:
        E.SubElement(instance, "metadata", {"key": key, "value": value})
    print(name, "size mm", (high - low).round(2).tolist(), "center", [x, y])
config.append(plate)
output = root / "legs_and_motor_cradles.3mf"
with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
    for name, data in template.items():
        archive.writestr(name, data)
    archive.writestr(
        "3D/3dmodel.model", E.tostring(model, encoding="utf-8", xml_declaration=True)
    )
    archive.writestr(
        "Metadata/model_settings.config",
        E.tostring(config, encoding="utf-8", xml_declaration=True),
    )
with zipfile.ZipFile(output) as archive:
    assert archive.testzip() is None
    saved = E.fromstring(archive.read("3D/3dmodel.model"))
    assert len(saved.find(q("build"))) == 4
print(
    "Four separate objects, all flat on the bed, with separated brims (4 mm legs, 3 mm cradles)."
)
print(output)
