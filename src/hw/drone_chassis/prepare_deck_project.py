import json, struct, zipfile
from pathlib import Path
from xml.etree import ElementTree as E
import numpy as np
root=Path(__file__).parent / "stl"
with zipfile.ZipFile(root / "electronics_deck.3mf") as z:
 template={n:z.read(n) for n in z.namelist() if n not in ["3D/3dmodel.model","Metadata/model_settings.config"]}
settings=json.loads(template["Metadata/project_settings.config"])
for key,value in {"enable_support":"1","support_type":"normal(auto)","support_on_build_plate_only":"1","support_top_z_distance":"0.2","support_interface_top_layers":"2"}.items():
 settings[key]=value
 changed=set(settings["different_settings_to_system"][0].split(";"));changed.add(key)
 settings["different_settings_to_system"][0]=";".join(sorted(changed))
template["Metadata/project_settings.config"]=json.dumps(settings).encode()
ns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
E.register_namespace("",ns)
meshes=[]
print_meshes=[]
for stem,name,rotate in [("electronics_deck","Braced electronics deck 90x70 perfboard","upside")]: 
 if stem=="camera_fixture_with_tabs":
  a=np.concatenate([points+np.array([dx,0,0]) for points,dx in zip(print_meshes,[0,57,106])])
  count=len(a)//3
 else:
  b=(root / (stem+".stl")).read_bytes();count=struct.unpack_from("<I",b,80)[0]
  assert len(b)==84+50*count
  a=np.array([struct.unpack_from("<12fH",b,84+i*50)[3:12] for i in range(count)]).reshape(-1,3)
  meshes.append((name,a.copy()))
  if rotate=="upside":a=a*np.array([1,-1,-1])
  elif rotate=="back":a=a[:,[0,2,1]]*np.array([1,1,-1])
  elif rotate:a=a[:,[0,2,1]]*np.array([1,-1,1])
 low=a.min(0);high=a.max(0);a-=np.array([(low[0]+high[0])/2,(low[1]+high[1])/2,low[2]])
 if stem!="camera_fixture_with_tabs":print_meshes.append(a.copy())
 vertices,index=np.unique(a,axis=0,return_inverse=True); triangles=index.reshape(-1,3)
 edges=np.sort(np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]]),axis=1)
 assert np.all(np.unique(edges,axis=0,return_counts=True)[1]==2),"Non-manifold STL"
 model=E.Element("{"+ns+"}model",{"unit":"millimeter"})
 E.SubElement(model,"{"+ns+"}metadata",{"name":"Application"}).text="Creality_Print V7.0.1.4212 Release"
 E.SubElement(model,"{"+ns+"}metadata",{"name":"BambuStudio:3mfVersion"}).text="1"
 res=E.SubElement(model,"{"+ns+"}resources");obj=E.SubElement(res,"{"+ns+"}object",{"id":"1","type":"model","name":name});mesh=E.SubElement(obj,"{"+ns+"}mesh")
 vs=E.SubElement(mesh,"{"+ns+"}vertices");ts=E.SubElement(mesh,"{"+ns+"}triangles")
 for v in vertices:E.SubElement(vs,"{"+ns+"}vertex",dict(zip(["x","y","z"],map(str,v))))
 for t in triangles:E.SubElement(ts,"{"+ns+"}triangle",dict(zip(["v1","v2","v3"],map(str,t))))
 E.SubElement(E.SubElement(model,"{"+ns+"}build"),"{"+ns+"}item",{"objectid":"1","transform":"1 0 0 0 1 0 0 0 1 110 110 0","printable":"1"})
 config=E.Element("config");obj=E.SubElement(config,"object",{"id":"1"})
 for k,v in [("name",name),("extruder","1")]:E.SubElement(obj,"metadata",{"key":k,"value":v})
 part=E.SubElement(obj,"part",{"id":"1","subtype":"normal_part"});E.SubElement(part,"metadata",{"key":"name","value":name})
 plate=E.SubElement(config,"plate")
 for k,v in [("plater_id","1"),("plater_name",name)]:E.SubElement(plate,"metadata",{"key":k,"value":v})
 inst=E.SubElement(plate,"model_instance")
 for k,v in [("object_id","1"),("instance_id","0"),("identify_id","1")]:E.SubElement(inst,"metadata",{"key":k,"value":v})
 with zipfile.ZipFile(root / (stem+".3mf"),"w",zipfile.ZIP_DEFLATED) as z:
  for k,v in template.items():z.writestr(k,v)
  z.writestr("3D/3dmodel.model",E.tostring(model,encoding="utf-8",xml_declaration=True));z.writestr("Metadata/model_settings.config",E.tostring(config,encoding="utf-8",xml_declaration=True))
 print(name,"print envelope",high-low,"watertight",count,"triangles")
