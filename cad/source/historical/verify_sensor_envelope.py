from pathlib import Path
import numpy as np,json,hashlib
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[1];p=root/'reference'/'RPLIDAR A1M8 kit.STL'
a=np.memmap(p,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),mode='r',offset=84)
v=a['v'].astype('float64');raw_min=v.min(axis=(0,1));raw_max=v.max(axis=(0,1));v-=np.array([37.10319,.07303969,36.53911])
chunks=[v[v[:,:,1]>=31-1e-5][:,[0,2]]]
for i,j in [(0,1),(1,2),(2,0)]:
 aa,bb=v[:,i],v[:,j];m=(aa[:,1]-31)*(bb[:,1]-31)<0;aa,bb=aa[m],bb[m];t=(31-aa[:,1])/(bb[:,1]-aa[:,1]);chunks.append((aa+(bb-aa)*t[:,None])[:,[0,2]])
u=np.concatenate(chunks);inner=u[ConvexHull(u).vertices]
profile=json.loads((root/'source'/'aperture_profile.json').read_text());outer=np.array(profile['vertices']);edges=np.roll(outer,-1,axis=0)-outer;lengths=np.linalg.norm(edges,axis=1)
def margin(points):
 d=points[:,None,:]-outer[None,:,:];return ((edges[None,:,0]*d[:,:,1]-edges[None,:,1]*d[:,:,0])/lengths[None,:]).min()
angles=np.linspace(0,np.pi*2,4096,endpoint=False);rotor=35.1*np.stack([np.cos(angles),np.sin(angles)],axis=1)
feet=v[np.abs(v[:,:,1])<1e-5];foot_checks=[]
for x,z in [(-28,-28),(-28,28),(42,-20),(42,20)]:
 q=feet[np.linalg.norm(feet[:,[0,2]]-[x,z],axis=1)<5];rr=np.linalg.norm(q[:,[0,2]]-[x,z],axis=1);foot_checks.append({'expected_center_xz':[x,z],'bottom_mesh_points':len(q),'modeled_minor_bore_radius_mm':float(rr.min()),'foot_outer_radius_mm':float(rr.max())})
r={'source_triangle_count':len(a),'source_bounds':{'minimum':raw_min.tolist(),'maximum':raw_max.tolist(),'dimensions_xyz':(raw_max-raw_min).tolist()},'raw_axes':'X motor-direction,Y vertical,Z transverse','source_header':p.open('rb').read(80).decode('latin1'),'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'feet':foot_checks,'roof_height_from_feet_mm':[31,34],'verified_nominal_full_head_lower_line_mm':35.5,'nominal_vertical_head_margin_mm':1.5,'minimum_static_projected_margin_mm':float(margin(inner)),'minimum_rotor_radius35_1_margin_mm':float(margin(rotor)),'scope':'Exact upper mesh including plane intersections inside aperture; guarantees nominal straight vertical roof sweep, not tilted lid or physical fit.'}
assert r['minimum_static_projected_margin_mm']>1.99
assert r['minimum_rotor_radius35_1_margin_mm']>3.88
(root/'validation'/'exact_sensor_envelope_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
