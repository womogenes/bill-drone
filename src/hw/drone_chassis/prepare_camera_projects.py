import json, struct, zipfile
from pathlib import Path
from xml.etree import ElementTree as E
import numpy as np
root=Path(__file__).parent / "stl"
with zipfile.ZipFile(root / "electronics_deck.3mf") as z:
 template={n:z.read(n) for n in z.namelist() if n not in ["3D/3dmodel.model","Metadata/model_settings.config"]}
# The braced deck needs supports; the separate flat camera parts do not.
settings=json.loads(template["Metadata/project_settings.config"])
settings["enable_support"]="0"
changed=set(settings["different_settings_to_system"][0].split(";"));changed.add("enable_support")
settings["different_settings_to_system"][0]=";".join(sorted(changed))
template["Metadata/project_settings.config"]=json.dumps(settings).encode()
ns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
E.register_namespace("",ns)
meshes=[]
print_meshes=[]
for stem,name,rotate in [("camera_fixture","Camera fixture",True),("camera_retaining_bar","Camera retaining bar","back"),("camera_lens_keeper","Camera lens keeper",True),("camera_fixture_with_tabs","Camera frame, PCB bar and lens keeper",False)]:
 if stem=="camera_fixture_with_tabs":
  a=np.concatenate([points+np.array([dx,0,0]) for points,dx in zip(print_meshes,[0,57,106])])
  count=len(a)//3
 else:
  b=(root / (stem+".stl")).read_bytes();count=struct.unpack_from("<I",b,80)[0]
  assert len(b)==84+50*count
  a=np.array([struct.unpack_from("<12fH",b,84+i*50)[3:12] for i in range(count)]).reshape(-1,3)
  meshes.append((name,a.copy()))
  if rotate=="back":a=a[:,[0,2,1]]*np.array([1,1,-1])
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
# Orthographic mesh preview, actual exported geometry.
from PIL import Image,ImageDraw
view=np.array([0.8,-1.4,1.0]);view/=np.linalg.norm(view)
right=np.cross([0,0,1],view);right/=np.linalg.norm(right);up=np.cross(view,right)
items=[];allpoints=[]
for name,a in meshes:
 tri=a.reshape(-1,3,3); xy=np.stack([a@right,a@up],axis=1);allpoints.append(xy)
 for face in tri:
  n=np.cross(face[1]-face[0],face[2]-face[0]);n/=max(np.linalg.norm(n),1e-12)
  shade=0.5+0.45*abs(n@view)
  color=np.array([220,132,48] if name=="Electronics deck" else [52,134,176])*shade
  items.append((face.mean(0)@view,np.stack([face@right,face@up],axis=1),tuple(color.astype(int))))
points=np.concatenate(allpoints);lo=points.min(0);hi=points.max(0);scale=min(1000/(hi-lo)[0],760/(hi-lo)[1])
im=Image.new("RGB",(1200,950),"#f0f3f5");draw=ImageDraw.Draw(im)
for depth,xy,c in sorted(items,key=lambda t:t[0]):
 xy=(xy-lo)*scale;xy[:,0]+=100;xy[:,1]=850-xy[:,1];draw.polygon([tuple(p) for p in xy],fill=c)
draw.text((35,25),"Open camera frame (PCB omitted; retaining bar shown loose)",fill="black")
im.save(root / "camera_fixture_preview.png")

# Clear filename for the current three-piece design; preserve prior download path.
(root / "camera_mount_simple.3mf").write_bytes((root / "camera_fixture_with_tabs.3mf").read_bytes())

(root / "camera_mount_with_lens_keeper.3mf").write_bytes((root / "camera_fixture_with_tabs.3mf").read_bytes())
