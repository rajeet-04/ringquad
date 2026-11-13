"""Check physical rim dimensions and collision-hull enclosure independently."""
import argparse
import json
import numpy as np
from scipy.spatial import ConvexHull
from hardware_geometry import mesh

parser=argparse.ArgumentParser();parser.add_argument('--output',default='hardware-results/crowned-mesh-check.json');args=parser.parse_args()
visual=mesh('src/ringquad_description/meshes/crowned_rim.stl')
hull_mesh=mesh('src/ringquad_description/meshes/crowned_tread_hull.stl')
radial=np.linalg.norm(visual[:,:2],axis=1)
assert abs(radial.max()-.135)<1e-7 and abs(radial.min()-.101)<1e-7
assert abs(np.ptp(visual[:,2])-.0395)<1e-7
planes=ConvexHull(hull_mesh).equations
rng=np.random.default_rng(42);angles=rng.uniform(0,2*np.pi,(12000,2));a,b=angles.T
surface=np.column_stack([(.1275+.0075*np.cos(b))*np.cos(a),(.1275+.0075*np.cos(b))*np.sin(a),.015*np.sin(b)])
worst=max(float(np.max(points@planes[:,:3].T+planes[:,3])) for points in np.array_split(surface,100))
assert worst<0, f'collision hull underbounds the physical crown by {worst} m'
n=planes[:,:3];r=np.linalg.norm(n[:,:2],axis=1)
analytic_max=float(np.max(.1275*r+np.sqrt((.0075*r)**2+(.015*n[:,2])**2)+planes[:,3]))
assert analytic_max<0, f'analytic crown support exceeds hull by {analytic_max} m'
result={'physical_outer_radius_m':float(radial.max()),'physical_inner_radius_m':float(radial.min()),'physical_width_m':float(np.ptp(visual[:,2])),
        'exact_analytic_support_max_m':analytic_max,'analytic_surface_samples':len(surface),'largest_signed_hull_distance_m':worst,'passed':True}
open(args.output,'w').write(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
