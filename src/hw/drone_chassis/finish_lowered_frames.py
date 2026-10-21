import trimesh,numpy as np,manifold3d as mf
from pathlib import Path
root=Path('/home/willi/coding/ee/bill-drone/src/hw/drone_chassis/stl')
frame=trimesh.load(root/'side_frame_lower_battery.stl',force='mesh')
# Exported body contains the thickened hanger; replace the detached inner lip with an integral 5 mm projection.
lip=trimesh.creation.box(extents=[11,20,3]);lip.apply_translation([-19,0,-26])
parts=[frame,lip]
# Three-mm concave rounds at Y +/-9, Z=-35, matching the pending FeatureScript edit.
r=3.;u=np.array([5.,23.]);u/=np.linalg.norm(u);theta=np.arccos(np.dot([0,-1],u));dist=r/np.tan(theta/2)
c=np.array([9.,-35.]);p0=c+[0,-dist];p1=c+u*dist;center=p0+[r,0]
a1=np.arctan2(*(p1-center)[::-1]);angles=np.linspace(np.pi,a1,30)
poly=np.vstack([c,center+np.column_stack([np.cos(angles),np.sin(angles)])*r])
for sign in [-1,1]:
 yz=poly*np.array([sign,1]);signed=np.sum(yz[:,0]*np.roll(yz[:,1],-1)-yz[:,1]*np.roll(yz[:,0],-1))
 if signed<0:yz=yz[::-1]
 n=len(yz);vs=np.vstack([np.column_stack([np.full(n,x),yz]) for x in [-24.5,-18.5]])
 tri=np.array(mf.triangulate([yz]));faces=[*tri[:,::-1],*(tri+n)]
 for i in range(n):
  j=(i+1)%n;faces.extend([[i,j,j+n],[i,j+n,i+n]])
 m=trimesh.Trimesh(vertices=vs,faces=faces,process=True);trimesh.repair.fix_normals(m)
 parts.append(m)
result=trimesh.boolean.union(parts,engine='manifold')
assert result.is_volume and len(result.split())==1
result.export(root/'side_frame_lower_battery_5mm_lips.stl')
print(result.bounds, result.volume)
