"""Dense ring/body and ring/ring clearance check for the 60 mm / 25 deg candidate."""
import argparse
import json
import itertools
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from hardware_geometry import mesh,pose,intersection

parser=argparse.ArgumentParser();parser.add_argument('--carrier',action='store_true');parser.add_argument('--envelope-padding',type=float,default=0.);parser.add_argument('--fold-ground-deg',type=int,default=80);parser.add_argument('--steering-limit-deg',type=int,default=25);parser.add_argument('--clearance',type=float,default=.002);parser.add_argument('--physical-rim',action='store_true');parser.add_argument('--ring-offset',type=float,default=.06);parser.add_argument('--output',default='hardware-results/candidate-dense-geometry.json');args=parser.parse_args()
if args.physical_rim:
    # Circumscribed polygon conservatively encloses the full 30mm-wide rim sweep.
    theta=np.arange(128)*2*np.pi/128
    radius=(.135+args.envelope_padding)/np.cos(np.pi/128)
    v=np.array([[radius*np.cos(a),radius*np.sin(a),z] for a in theta for z in ((-.035 if args.carrier else -.015)-args.envelope_padding,.015+args.envelope_padding)])
else:
    v=mesh()
planes=ConvexHull(v).equations
body_failures=[];pair_failures=[];checks=0
stop=args.steering_limit_deg+1
for fold in range(args.fold_ground_deg+1):
    for name in ('fl','fr','rl','rr'):
        for angle in range(-stop,stop+1) if fold>=75 and name.startswith('f') else (0,):
            centre,rot=pose(name,fold,angle,args.ring_offset,args.fold_ground_deg)
            value,witness=intersection(planes,centre,rot,args.clearance);checks+=1
            if value>1e-6:body_failures.append({'fold':fold,'ring':name,'angle':angle,'witness':witness,'local_radial_m':float(np.linalg.norm((rot.T@(np.array(witness)-centre))[:2])),'local_axial_m':float((rot.T@(np.array(witness)-centre))[2])})
for fold in sorted(set(range(0,args.fold_ground_deg+1,2)) | {args.fold_ground_deg}):
    for angles in ((0,0),(stop,stop),(-stop,-stop),(stop,-stop),(-stop,stop)) if fold>=75 else ((0,0),):
        shapes=[]
        for i,name in enumerate(('fl','fr','rl','rr')):
            c,r=pose(name,fold,angles[i] if i<2 else 0,args.ring_offset,args.fold_ground_deg)
            a=planes[:,:3]@r.T;b=-planes[:,3]+a@c
            shapes.append((a,b))
        for i,j in itertools.combinations(range(4),2):
            a=np.vstack([shapes[i][0],shapes[j][0]]);b=np.concatenate([shapes[i][1],shapes[j][1]])
            result=linprog([0,0,0,-1],A_ub=np.column_stack([a,np.ones(len(a))]),b_ub=b,bounds=[(None,None)]*4,method='highs')
            if not result.success:raise RuntimeError(result.message)
            if result.x[3]>1e-6:pair_failures.append({'fold':fold,'angles':angles,'pair':[i,j]})
result={'candidate_ring_offset_m':args.ring_offset,'includes_rear_carrier':args.carrier,'envelope_padding_m':args.envelope_padding,'envelope':'full_30mm_rim_sweep' if args.physical_rim else 'shipped_rounded_collision_mesh','fold_ground_deg':args.fold_ground_deg,'commanded_stop_deg':args.steering_limit_deg,'audited_angle_including_stop_margin_deg':stop,'body_box_inflation_m':args.clearance,'body_checks':checks,'body_failures':body_failures,'ring_pair_failures':pair_failures}
Path(args.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
