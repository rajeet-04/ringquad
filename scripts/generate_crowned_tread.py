#!/usr/bin/env python3
"""Physical elliptical-section annular rim and its conservative convex collision hull.

Dimensions: major radius 127.5 mm, radial semi-axis 7.5 mm, axial semi-axis
15 mm. Outer/inner diameter 270/240 mm, full axial width 30 mm. SI units.
"""
import argparse
from pathlib import Path
import struct
import numpy as np
from scipy.spatial import ConvexHull


def stl(path, vertices, faces):
    with path.open('wb') as out:
        out.write(b'RingQuad elliptical crown, dimensions in metres'.ljust(80,b'\0'))
        out.write(struct.pack('<I',len(faces)))
        for face in faces:
            tri=vertices[face];n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n/=np.linalg.norm(n)
            out.write(struct.pack('<12fH',*n,*tri.flatten(),0))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--fine-contact',action='store_true');args=parser.parse_args()
    stride=1 if args.fine_contact else 2
    padding=.0001 if args.fine_contact else .0002
    na,nb=128,64
    vertices=np.array([[(.1275+.0075*np.cos(b))*np.cos(a),(.1275+.0075*np.cos(b))*np.sin(a),.015*np.sin(b)]
                       for a in np.arange(na)*2*np.pi/na for b in np.arange(nb)*2*np.pi/nb])
    faces=[]
    for i in range(na):
        for j in range(nb):
            a=i*nb+j;b=((i+1)%na)*nb+j;c=((i+1)%na)*nb+(j+1)%nb;d=i*nb+(j+1)%nb
            faces.extend([[a,b,c],[a,c,d]])
    root=Path('src/ringquad_description/meshes')
    stl(root/'crowned_rim.stl',vertices,np.asarray(faces))
    # Use 64x32 samples for contact: independent of the 128x64 physical mesh.
    # Inflate by 0.2 mm to enclose polygon sampling error, verified separately.
    hull_vertices=vertices.reshape(na,nb,3)[::stride,::stride].reshape(-1,3).copy()
    hull_vertices[:,:2]*=(.135+padding)/.135;hull_vertices[:,2]*=(.015+padding)/.015
    hull=ConvexHull(hull_vertices)
    # QHull simplices do not guarantee an outward winding; correct each face.
    faces=hull.simplices.copy()
    for i,f in enumerate(faces):
        tri=hull_vertices[f]
        if np.dot(np.cross(tri[1]-tri[0],tri[2]-tri[0]),hull.equations[i,:3])<0:faces[i]=f[::-1]
    stl(root/'crowned_tread_hull.stl',hull_vertices,faces)
    print('Wrote crowned rim and inflated convex collision hull')
