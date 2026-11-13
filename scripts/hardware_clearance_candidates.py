"""Search candidate ring offsets and steering stops without changing the robot."""
import json
import numpy as np
from scipy.spatial import ConvexHull
from hardware_geometry import mesh, pose, intersection
from pathlib import Path

vertices=mesh();planes=ConvexHull(vertices).equations
results=[]
for offset in (.035,.055,.06,.065,.07,.075):
    for limit in (10,15,20,25,30,35):
        failures=[]
        for fold in (0,20,40,60,74,75,77.5,80):
            for angle in (-limit,0,limit) if fold>=75 else (0,):
                centre,rot=pose('fl',fold,angle,offset)
                radius,witness=intersection(planes,centre,rot,.002)
                if radius>1e-6:failures.append((fold,angle))
        results.append({'offset_m':offset,'steering_limit_deg':limit,'has_2mm_clearance_in_samples':not failures,'failed_samples':failures})
Path('hardware-results/clearance-candidates.json').write_text(json.dumps(results,indent=2))
print(json.dumps([r for r in results if r['has_2mm_clearance_in_samples']],indent=2))
