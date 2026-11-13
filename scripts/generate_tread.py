"""Generate the convex oblate tread contact mesh; run from the repository root."""
import math,struct
from pathlib import Path
v=[]; n=64; m=16
for j in range(m+1):
 a=math.pi*j/m
 for i in range(n):
  b=2*math.pi*i/n
  v.append((.135*math.sin(a)*math.cos(b),.135*math.sin(a)*math.sin(b),.015*math.cos(a)))
tri=[]
for j in range(m):
 for i in range(n):
  a=j*n+i;b=j*n+(i+1)%n;c=(j+1)*n+i;d=(j+1)*n+(i+1)%n
  if j>0:tri.append((v[a],v[c],v[b]))
  if j<m-1:tri.append((v[b],v[c],v[d]))
p=Path('src/ringquad_description/meshes/rounded_tread.stl')
with p.open('wb') as f:
 f.write(b'Rounded tread contact approximation'.ljust(80,b'\0'));f.write(struct.pack('<I',len(tri)))
 for t in tri:f.write(struct.pack('<12fH',0,0,0,*t[0],*t[1],*t[2],0))
