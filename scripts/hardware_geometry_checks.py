"""Sanity checks on the independent convex swept-envelope audit."""
import numpy as np
from scipy.spatial import ConvexHull
from hardware_geometry import mesh, pose, intersection
planes=ConvexHull(mesh()).equations
assert intersection(planes,np.array([1.,0.,0.]),np.eye(3))[0]<0
assert intersection(planes,np.zeros(3),np.eye(3))[0]>0
centre,rot=pose('fl',0,0,.035)
assert intersection(planes,centre,rot)[0]>0
centre,rot=pose('fl',80,25,.06)
assert intersection(planes,centre,rot,.002)[0]<0
left=intersection(planes,*pose('fl',80,25,.06),.002)[0]
right=intersection(planes,*pose('fr',80,-25,.06),.002)[0]
assert abs(left-right)<1e-9
print('Convex audit checks passed: disjoint, overlap, baseline interference, candidate gap, symmetry')
