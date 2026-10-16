"""Replacement-only, unsliced projects for reinforced legs and M2 horn cradles."""
import json
import struct
import zipfile
from pathlib import Path
from xml.etree import ElementTree as E
import numpy as np

root = Path(__file__).parent
ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
E.register_namespace("", ns)
q = lambda tag: "{" + ns + "}" + tag
with zipfile.ZipFile(root / "stl/electronics_deck.3mf") as z:
    template = {n: z.read(n) for n in z.namelist() if n not in ["3D/3dmodel.model", "Metadata/model_settings.config"]}


def read_mesh(path, orientation):
    b = path.read_bytes()
    count = struct.unpack_from("<I", b, 80)[0]
    assert len(b) == 84 + count * 50
    a = np.array([struct.unpack_from("<12fH", b, 84+i*50)[3:12] for i in range(count)]).reshape(-1, 3)
    if orientation == "legs":
        a = a[:, [0, 2, 1]] * [1, 1, -1]
    else:
        a = a * [1, -1, -1]
    low, high = a.min(0), a.max(0)
    a -= [(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]]
    vertices, indices = np.unique(a, axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    assert np.all(np.unique(edges, axis=0, return_counts=True)[1] == 2)
    return vertices, faces, high-low


for stem, source, orientation, centers in [
    ("reinforced_landing_legs_pair", root / "stl/landing_legs.stl", "legs", [(110,57),(110,157)]),
    ("motor_cradles_m2_pair", root.parent / "end_attachment/stl/motor_cradle_m2.stl", "cradle", [(70,110),(150,110)]),
]:
    vertices, faces, size = read_mesh(source, orientation)
    settings = json.loads(template["Metadata/project_settings.config"])
    changes = {"wall_loops":"4", "sparse_infill_density":"25%", "top_shell_layers":"5", "bottom_shell_layers":"5", "top_shell_thickness":"1", "bottom_shell_thickness":"1", "enable_support":"0" if orientation=="legs" else "1", "support_on_build_plate_only":"1"}
    if orientation == "legs":
        changes.update({"wall_loops": "6", "sparse_infill_density": "35%"})
    if orientation == "cradle":
        changes.update({"support_type": "tree(auto)", "support_style": "organic"})
    settings.update(changes)
    settings["different_settings_to_system"][0] = ";".join(sorted(set(settings["different_settings_to_system"][0].split(";")) | set(changes)))
    model = E.Element(q("model"), {"unit":"millimeter"})
    E.SubElement(model,q("metadata"),{"name":"Application"}).text="Creality_Print V7.0.1.4212 Release"
    E.SubElement(model,q("metadata"),{"name":"BambuStudio:3mfVersion"}).text="1"
    resources=E.SubElement(model,q("resources")); build=E.SubElement(model,q("build")); config=E.Element("config"); plate=E.Element("plate")
    E.SubElement(plate,"metadata",{"key":"plater_id","value":"1"})
    bounds=[]
    for index, (x,y) in enumerate(centers,1):
        box=(np.array([x,y])-size[:2]/2-6,np.array([x,y])+size[:2]/2+6)
        assert np.all(box[0]>=5) and np.all(box[1]<=215)
        for old in bounds: assert np.any(box[1]<old[0]) or np.any(old[1]<box[0])
        bounds.append(box)
        obj=E.SubElement(resources,q("object"),{"id":str(index),"type":"model","name":stem})
        mesh=E.SubElement(obj,q("mesh"));vs=E.SubElement(mesh,q("vertices"));ts=E.SubElement(mesh,q("triangles"))
        for v in vertices:E.SubElement(vs,q("vertex"),dict(zip(["x","y","z"],map(str,v))))
        for t in faces:E.SubElement(ts,q("triangle"),dict(zip(["v1","v2","v3"],map(str,t))))
        E.SubElement(build,q("item"),{"objectid":str(index),"transform":f"1 0 0 0 1 0 0 0 1 {x} {y} 0","printable":"1"})
        o=E.SubElement(config,"object",{"id":str(index)})
        for k,v in [("name",stem+" "+str(index)),("extruder","1")]:E.SubElement(o,"metadata",{"key":k,"value":v})
        part=E.SubElement(o,"part",{"id":str(index),"subtype":"normal_part"});E.SubElement(part,"metadata",{"key":"name","value":stem})
        instance=E.SubElement(plate,"model_instance")
        for k,v in [("object_id",str(index)),("instance_id","0"),("identify_id",str(index))]:E.SubElement(instance,"metadata",{"key":k,"value":v})
    config.append(plate)
    output=source.parent/(stem+".3mf")
    with zipfile.ZipFile(output,"w",zipfile.ZIP_DEFLATED) as z:
        for n,b in template.items():z.writestr(n,json.dumps(settings) if n=="Metadata/project_settings.config" else b)
        z.writestr("3D/3dmodel.model",E.tostring(model,encoding="utf-8",xml_declaration=True))
        z.writestr("Metadata/model_settings.config",E.tostring(config,encoding="utf-8",xml_declaration=True))
    print(output, "each size",size,"closed manifold; brim clearance passed")
