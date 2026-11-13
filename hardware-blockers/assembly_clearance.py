"""Conservative sampled wheel-motor case versus chassis and other module envelopes."""
import itertools,json,sys
from pathlib import Path
sys.path.insert(0,'scripts')
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from hardware_geometry import pose,intersection
angles=np.arange(32)*2*np.pi/32
motor=np.array([[.09+r*np.cos(a),r*np.sin(a),z] for a in angles for r in [.0102/np.cos(np.pi/32)] for z in [-.0854,-.0418]])
ring=np.array([[.1352/np.cos(np.pi/64)*np.cos(a),.1352/np.cos(np.pi/64)*np.sin(a),z] for a in np.arange(64)*2*np.pi/64 for z in [-.0402,.0152]])
checks=0;failures=[]
for fold in range(0,91,2):
    for steer in ((0,0),(26,26),(-26,-26),(26,-26),(-26,26)) if fold>=75 else ((0,0),):
        modules=[];motors=[]
        for i,name in enumerate(('fl','fr','rl','rr')):
            c,r=pose(name,fold,steer[i] if i<2 else 0,.1,90)
            points=motor.copy()
            if i>=2:points[:,:2]*=-1
            h=ConvexHull(points).equations
            radius,_=intersection(h,c,r,.002);checks+=1
            if radius>1e-6:failures.append(dict(fold=fold,steer=steer,case='motor/body',module=name))
            motors.append(ConvexHull(points@r.T+c).equations)
            modules.append(ConvexHull(ring@r.T+c).equations)
        for i,j in itertools.product(range(4),repeat=2):
            if i==j:continue
            h=np.vstack([motors[i],modules[j]])
            q=linprog([0,0,0,-1],A_ub=np.column_stack([h[:,:3],np.ones(len(h))]),b_ub=-h[:,3],bounds=[(None,None)]*4,method='highs');checks+=1
            if not q.success:raise RuntimeError(q.message)
            if q.x[3]>1e-6:failures.append(dict(fold=fold,steer=steer,case='motor/other-module',modules=[i,j]))
result=dict(checks=checks,failures=failures,passed=not failures,method='2deg folds, zero or four +/-26deg steering combinations; circumscribed convex module and motor-case envelopes with 0.2mm padding; 2mm chassis inflation; excludes wiring and unsized housings')
Path('hardware-blockers/results/assembly-clearance.json').write_text(json.dumps(result,indent=2));print(result)
assert result['passed']
