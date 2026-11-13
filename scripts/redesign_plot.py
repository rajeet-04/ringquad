"""Plan-view comparison of the pre-redesign and upright full-rim envelopes."""
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from hardware_geometry import pose, rotation

fig,axes=plt.subplots(1,2,figsize=(9,5),sharex=True,sharey=True)
theta=np.linspace(0,2*np.pi,128)
v=np.array([[.135*np.cos(a),.135*np.sin(a),z] for a in theta for z in [-.015,.015]])
for ax,(fold,offset,label) in zip(axes,[(80,.075,'Earlier candidate: 80° / 75 mm'),(90,.1,'Independent drive: 90° / 100 mm')]):
    pts=[];body=rotation([0,0,1],math.radians(1))
    for n in ['fl','fr','rl','rr']:
        c,r=pose(n,fold,10 if n.startswith('f') else 0,offset,fold)
        shape=v if fold==80 else np.array([[.135*np.cos(a),.135*np.sin(a),z] for a in theta for z in [-.035,.015]])
        points=(shape@r.T+c)@body.T
        xy=points[:,[1,0]];hull=ConvexHull(xy)
        ax.fill(*xy[hull.vertices].T,alpha=.35,color='#e77930' if n.startswith('f') else '#347bb3')
        pts.extend(points)
    pts=np.asarray(pts)
    for wall in [-.25,.25]:ax.axvline(wall,color='black');ax.axvline(wall-math.copysign(.01,wall),color='red',ls=':',lw=.8)
    ax.set_title(label);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_xlabel('Lateral position (m)')
    ax.text(0,-.43,f'Width {np.ptp(pts[:,1])*1000:.0f} mm',ha='center')
    ax.set_xlim(-.29,.29);ax.set_ylim(-.46,.46)
axes[0].set_ylabel('Forward position (m)')
fig.suptitle('Conservative ring/carrier envelope; 10° steering and 1° heading\nBlack: 500 mm corridor; red: 10 mm wall buffer')
fig.tight_layout();fig.savefig('hardware-results/redesign-footprint.png',dpi=180);fig.savefig('hardware-results/redesign-footprint.svg')
