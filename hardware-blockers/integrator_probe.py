import sys
sys.path.insert(0,'scripts')
import json
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import hardware_flight_sweep as h
import separate_drive_flight_sweep as s
from ringquad_control.flight import FlightController,Gains

# Test a bounded disturbance-integrator candidate without changing production code.
class Candidate(FlightController):
 def update(self,pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp=None):
  # Allow integration at a persistent off-target equilibrium; keep fast transients gated.
  old=self.g.i_zone
  if np.linalg.norm(vel)<.2:self.g.i_zone=10.
  out=super().update(pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp)
  self.g.i_zone=old
  return out

def task(t):
 variant,name=t
 h.CASES={n:{'independent_propeller':True,**v} for n,v in s.CASES.items()}
 class C(Candidate):
  def __init__(self,*a,**kw):
   super().__init__(*a,**kw);self.g.i_max=variant
 h.FlightController=C
 return dict(integral_limit=variant,**h.evaluate((name,0)))
if __name__=='__main__':
 names=['nominal','thrust_minus20pct','one_motor_thrust_minus20pct','combined_adverse']
 with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(task,[(v,n) for v in [2.,3.,4.,5.] for n in names]))
 open('hardware-blockers/results/integrator-probe.json','w').write(json.dumps(rows,indent=2))
 for r in rows:print(r['integral_limit'],r['case'],r['passed'],r['crash'],round(r['position_error_m'],3))
