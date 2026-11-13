"""Full-rim candidate tunnel margin for nominal fold and sampled headings."""
import json
import math
import numpy as np
from hardware_geometry import pose,rotation

rows=[]
for steering in [0,10,25]:
    for heading in [-1,0,1]:
        body=rotation([0,0,1],math.radians(heading));lo=1e9;hi=-1e9
        for name in ['fl','fr','rl','rr']:
            centre,rot=pose(name,80,steering if name.startswith('f') else 0,.075)
            centre=body@centre;normal=body@rot[:,2]
            extent=.135*math.sqrt(max(0.,1-normal[1]**2))+.015*abs(normal[1])
            lo=min(lo,centre[1]-extent);hi=max(hi,centre[1]+extent)
        rows.append(dict(steering_deg=steering,heading_deg=heading,width_m=hi-lo,
          conservative_cross_track_allowance_with_5mm_buffer_m=min(.25-hi,.25+lo)-.005))
open('hardware-results/physical-tunnel-envelope.json','w').write(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
