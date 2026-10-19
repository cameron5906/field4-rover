import struct,collections,json,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out={}
for p in sorted((root/'stl').rglob('*.stl')):
 b=p.read_bytes();n=struct.unpack_from('<I',b,80)[0];edges=collections.Counter();verts=set();vol=0;deg=0
 for i in range(n):
  v=struct.unpack_from('<12fH',b,84+50*i);pts=[tuple(round(v[3+j*3+k],6) for k in range(3)) for j in range(3)];verts.update(pts)
  for j in range(3):edges[tuple(sorted([pts[j],pts[(j+1)%3]]))]+=1
  a,c,d=pts;u=[c[k]-a[k] for k in range(3)];w=[d[k]-a[k] for k in range(3)];cross=[u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]]
  deg+=sum(q*q for q in cross)<1e-16
  vol+=(a[0]*(c[1]*d[2]-c[2]*d[1])+a[1]*(c[2]*d[0]-c[0]*d[2])+a[2]*(c[0]*d[1]-c[1]*d[0]))/6
 r={'triangles':n,'file_length_valid':len(b)==84+50*n,'edges_not_used_twice':sum(v!=2 for v in edges.values()),'degenerate_triangles':deg,'signed_volume_mm3':round(vol,4),'bounds_mm':[[min(p[k] for p in verts) for k in range(3)],[max(p[k] for p in verts) for k in range(3)]],'sha256':hashlib.sha256(b).hexdigest()};out[str(p.relative_to(root))]=r
 assert r['file_length_valid'] and not r['edges_not_used_twice'] and not deg and vol>0,(p,r)
(root/'validation'/'independent_stl_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
