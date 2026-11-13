"""Sample independent support functions for the crown and all rear collision pieces."""
import json, sys
from pathlib import Path
sys.path.insert(0,'scripts')
import numpy as np
from hardware_geometry import mesh
pieces=list(Path('src/ringquad_description/meshes/rear_race_segments').glob('*.stl'))+list(Path('src/ringquad_description/meshes/rear_tab_segments').glob('*.stl'))
rear=np.vstack([mesh(str(p)) for p in pieces]);rows=[]
for degree in np.linspace(0,90,1801):
    t=np.deg2rad(degree)
    crown=.1275*np.sin(t)+np.hypot(.0075*np.sin(t),.015*np.cos(t))
    # Radial support bounds every wheel rotation, independent of mesh spin phase.
    support=float(np.max(np.linalg.norm(rear[:,:2],axis=1)*np.sin(t)-rear[:,2]*np.cos(t)))
    centre=.1*np.cos(t)-(.12/np.sqrt(2))*np.sin(t)
    base=max(.04,crown-centre) # chassis half height is 40mm
    rows.append({'fold_deg':float(degree),'rear_floor_clearance_m':base+centre-support})
worst=min(rows,key=lambda r:r['rear_floor_clearance_m'])
result={'checks':len(rows),'worst':worst,'passed':bool(worst['rear_floor_clearance_m']>=0),'assumption':'level body, ground following exact crown support; conservative rear radial envelopes and chassis 40mm support'}
Path('hardware-blockers/results/rear-support.json').write_text(json.dumps(result,indent=2));print(result)
assert result['passed']
