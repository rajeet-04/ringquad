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
    body_vertices=vertices.copy()
    body_faces=np.asarray(faces)
    # Rear V-race lies behind the propeller plane; twelve narrow tabs connect it.
    profile=np.array([[.101,-.0225],[.103,-.0245],[.105,-.0225],[.103,-.0205]])
    race=np.array([[r*np.cos(a),r*np.sin(a),z] for a in np.arange(na)*2*np.pi/na for r,z in profile])
    race_faces=[]
    for i in range(na):
        for j in range(4):
            a=i*4+j;b=((i+1)%na)*4+j;c=((i+1)%na)*4+(j+1)%4;d=i*4+(j+1)%4
            race_faces.extend([[a,b,c],[a,c,d]])
    vertices=np.vstack([vertices,race]);faces=np.vstack([body_faces,np.asarray(race_faces)+len(body_vertices)])
    tab_points=[]
    tabs=[]
    cube_faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    tilt=np.arctan2(.021,.0085)
    ry=np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]])
    for a in np.arange(12)*2*np.pi/12:
        rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        v=np.array([[x,y,z] for x in [-.0005,.0005] for y in [-.0015,.0015] for z in [-.01143,.01143]])
        v=(v@ry.T+np.array([.1155,0,-.01825]))@rz.T
        faces=np.vstack([faces,cube_faces+len(vertices)]);vertices=np.vstack([vertices,v]);tab_points.extend(v);tabs.append(v)
    root=Path('src/ringquad_description/meshes')
    stl(root/'crowned_rim.stl',vertices,np.asarray(faces))
    # Use 64x32 samples for contact: independent of the 128x64 physical mesh.
    # Inflate by 0.2 mm to enclose polygon sampling error, verified separately.
    hull_vertices=body_vertices.reshape(na,nb,3)[::stride,::stride].reshape(-1,3).copy()
    hull_vertices[:,:2]*=(.135+padding)/.135;hull_vertices[:,2]*=(.015+padding)/.015
    hull=ConvexHull(hull_vertices)
    # QHull simplices do not guarantee an outward winding; correct each face.
    faces=hull.simplices.copy()
    for i,f in enumerate(faces):
        tri=hull_vertices[f]
        if np.dot(np.cross(tri[1]-tri[0],tri[2]-tri[0]),hull.equations[i,:3])<0:faces[i]=f[::-1]
    stl(root/'crowned_tread_hull.stl',hull_vertices,faces)
    # The rear race must retain its hollow centre: a single convex hull would
    # create an imaginary rear disc intersecting the stationary carrier.
    race_dir=root/'rear_race_segments';race_dir.mkdir(exist_ok=True)
    for i in range(16):
        angles=np.linspace(i*2*np.pi/16,(i+1)*2*np.pi/16,5)
        v=np.array([[r*np.cos(a),r*np.sin(a),z] for a in angles for r in [.1008,.1052/np.cos(np.pi/64)] for z in [-.0247,-.0203]])
        h=ConvexHull(v);f=h.simplices.copy()
        for j,tri in enumerate(f):
            t=v[tri]
            if np.dot(np.cross(t[1]-t[0],t[2]-t[0]),h.equations[j,:3])<0:f[j]=tri[::-1]
        stl(race_dir/f'race_{i}.stl',v,f)
    tab_dir=root/'rear_tab_segments';tab_dir.mkdir(exist_ok=True)
    for i,v in enumerate(tabs):
        centre=v.mean(axis=0);v=centre+(v-centre)*1.01
        h=ConvexHull(v);f=h.simplices.copy()
        for j,tri in enumerate(f):
            t=v[tri]
            if np.dot(np.cross(t[1]-t[0],t[2]-t[0]),h.equations[j,:3])<0:f[j]=tri[::-1]
        stl(tab_dir/f'tab_{i}.stl',v,f)
    print('Wrote physical crowned wheel and hollow rear-race collision decomposition')
