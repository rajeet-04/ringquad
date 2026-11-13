#!/usr/bin/env python3
"""Offline swept collision audit using the shipped convex tread mesh.

Run from repository root with numpy/scipy; outputs JSON. Positive intersection
radius means a strictly feasible common interior of ring and chassis volumes.
"""
import json
import math
from pathlib import Path
import struct
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog


def rotation(axis, angle):
    axis=np.asarray(axis,float); x,y,z=axis
    skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def mesh(path='src/ringquad_description/meshes/rounded_tread.stl'):
    data=Path(path).read_bytes()
    count=struct.unpack_from('<I',data,80)[0]
    vertices=[]
    for i in range(count):
        vertices.extend(np.array(struct.unpack_from('<12fH',data,84+i*50)[3:12]).reshape(3,3))
    return np.unique(vertices,axis=0)


def pose(name,fold,angle=0,ring_offset=.035, fold_ground_deg=80):
    side=1 if name.endswith('l') else -1
    front=1 if name.startswith('f') else -1
    axis=-side
    r=rotation([1,0,0],axis*math.radians(fold))
    centre=np.array([front*.08,side*.08,0])+r@np.array([front*.12/math.sqrt(2),side*.12/math.sqrt(2),ring_offset])
    steering=rotation([0,axis,0],math.radians(angle)/math.sin(math.radians(fold_ground_deg)))
    return centre,r@steering


def intersection(planes, centre, rot, clearance=0):
    # Transform canonical outward halfspaces n.x+d <= 0 to the body frame.
    normals=planes[:,:3]@rot.T
    bounds=-planes[:,3]+normals@centre
    box=np.vstack([np.eye(3),-np.eye(3)])
    a=np.vstack([normals,box]);b=np.concatenate([bounds,np.array([.08,.08,.04]*2)+clearance])
    # Find the maximum common inscribed sphere radius, with unit plane normals.
    result=linprog([0,0,0,-1], A_ub=np.column_stack([a,np.ones(len(a))]),b_ub=b,
                   bounds=[(None,None)]*4, method='highs')
    if not result.success:raise RuntimeError(result.message)
    return float(result.x[3]),result.x[:3].tolist()


def audit():
    vertices=mesh();planes=ConvexHull(vertices).equations
    collisions=[];close=[];safe_angles=[]
    # Folded steering axis changes slightly at 75..80 degrees: audit each.
    for fold in [0,20,40,60,74,75,77.5,80]:
        for name in ('fl','fr','rl','rr'):
            angles=range(-35,36) if fold>=75 and name.startswith('f') else (0,)
            for angle in angles:
                centre,rot=pose(name,fold,angle)
                radius,witness=intersection(planes,centre,rot)
                record={'fold_deg':fold,'ring':name,'steer_deg':angle,'intersection_radius_m':radius,'witness_m':witness}
                if radius>1e-6:collisions.append(record)
                if abs(angle)<=15:
                    inflated,_=intersection(planes,centre,rot,.002)
                    if inflated>1e-6:close.append(record)
    # Maximum folded width and ground-to-top height, including body box.
    envelopes=[]
    for angle in range(-35,36):
        points=[]
        for name in ('fl','fr','rl','rr'):
            centre,rot=pose(name,80,angle if name.startswith('f') else 0)
            points.extend(vertices@rot.T+centre)
        points=np.asarray(points)
        envelopes.append({'steer_deg':angle,'width_m':float(np.ptp(points[:,1])),
                          'ground_to_body_top_m':float(.04-points[:,2].min())})
    return {'chassis_intersections':collisions,'under_2mm_margin_at_up_to_15deg':close,'envelopes':envelopes,
            'method':'convex shipped mesh vs chassis box; excludes arms, blades outside collision mesh and unmodelled servo housings'}


if __name__=='__main__':
    result=audit();Path('hardware-results').mkdir(exist_ok=True)
    Path('hardware-results/geometry.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({'intersections':len(result['chassis_intersections']),
       'examples':result['chassis_intersections'][:2],
       'near_chassis_up_to_15deg':len(result['under_2mm_margin_at_up_to_15deg']),
       'max_width_m':max(x['width_m'] for x in result['envelopes'])},indent=2))
