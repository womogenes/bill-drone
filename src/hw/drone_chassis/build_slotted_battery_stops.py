"""Support-free frame/stop geometry from the previously validated lowered-frame mesh."""
from pathlib import Path
import json
import numpy as np
import trimesh
import manifold3d as mf

root = Path(__file__).parent / "stl"

def box(low, high):
    low, high = np.array(low), np.array(high)
    solid = trimesh.creation.box(extents=high-low)
    solid.apply_translation((high+low)/2)
    return solid

def prism(points, x0, x1):
    points=np.array(points,float)
    area=np.sum(points[:,0]*np.roll(points[:,1],-1)-points[:,1]*np.roll(points[:,0],-1))
    if area<0: points=points[::-1]
    count=len(points)
    vertices=np.vstack([np.column_stack([np.full(count,x),points]) for x in [x0,x1]])
    triangles=np.array(mf.triangulate([points]))
    faces=[*triangles[:,::-1],*(triangles+count)]
    for i in range(count):
        j=(i+1)%count
        faces.extend([[i,j,j+count],[i,j+count,i+count]])
    result=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    trimesh.repair.fix_normals(result)
    assert result.is_volume
    return result

def hole(x0,x1,y,z,diameter):
    solid=trimesh.creation.cylinder(radius=diameter/2,height=x1-x0,sections=96)
    solid.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]))
    solid.apply_translation([(x0+x1)/2,y,z])
    return solid

# Remove only the old inward lip; preserve the reinforced hanger and neck rounds.
old=trimesh.load(root/"side_frame_lower_battery_5mm_lips.stl",force="mesh")
frame=trimesh.boolean.difference([old,box([-18.5,-80,-80],[0,80,10])],engine="manifold")
slot=[[-5,-27.8],[5,-27.8],[5.3,-27.5],[5.3,-23.5],[5,-23.2],[-5,-23.2],[-5.3,-23.5],[-5.3,-27.5]]
nut=[[6.2/np.sqrt(3)*np.cos(a),-33.5+6.2/np.sqrt(3)*np.sin(a)] for a in np.arange(6)*np.pi/3]
frame=trimesh.boolean.difference([frame,prism(slot,-25,-18),hole(-25,-18,0,-33.5,3.5),prism(nut,-21.7,-18.4)],engine="manifold")
# Tab is 10 mm wide and 4 mm high, with 0.3 mm per-side slot clearance.
tab=prism([[-4.7,-27.5],[4.7,-27.5],[5,-27.2],[5,-23.8],[4.7,-23.5],[-4.7,-23.5],[-5,-23.8],[-5,-27.2]],-27.5,-13.5)
flange=prism([[-4.4,-37.5],[4.4,-37.5],[5,-36.9],[5,-24.1],[4.4,-23.5],[-4.4,-23.5],[-5,-24.1],[-5,-36.9]],-27.5,-24.5)
stop=trimesh.boolean.union([tab,flange],engine="manifold")
stop=trimesh.boolean.difference([stop,hole(-28,-24,0,-33.5,3.5)],engine="manifold")

checks={}
for name,mesh in [("side_frame_slotted_battery",frame),("battery_stop_slide_in",stop)]:
    assert mesh.is_volume and len(mesh.split())==1
    mesh.export(root/(name+".stl"))
    checks[name]={"volume_mm3":float(mesh.volume),"extents_mm":mesh.extents.tolist(),"watertight":True,"components":1}

def overlap(a,b):
    return abs(trimesh.boolean.intersection([a,b],engine="manifold").volume)

checks["installed_stop_frame_overlap_mm3"]=overlap(frame,stop)
assert checks["installed_stop_frame_overlap_mm3"]<.001
battery=box([-18,-37,-63],[18,37,-28])
assert overlap(stop,battery)<.001 and overlap(frame,battery)<.001
# Insertion path: translate outward along X; test both end positions and intermediate poses.
checks["insertion_overlap_mm3"]={}
for shift in [0,-2,-5,-10,-15]:
    trial=stop.copy();trial.apply_translation([shift,0,0]);v=overlap(frame,trial)
    checks["insertion_overlap_mm3"][str(shift)]=v
    assert v<.001
shaft=hole(-27.5,-19.5,0,-33.5,3)
assert overlap(frame,shaft)<.001 and overlap(stop,shaft)<.001 and overlap(battery,shaft)<.001
checks.update(battery_top_clearance_mm=.5,retention_projection_mm=5,slot_clearance_per_side_mm=.3,fastener="2x M3x8 socket heads + 2x M3 hex nuts",frame_nut_pocket_af_mm=6.2,nominal_nut_engagement_mm=2.2)
(root/"slotted_battery_validation.json").write_text(json.dumps(checks,indent=2)+"\n")
print(json.dumps(checks,indent=2))
