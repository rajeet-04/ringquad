import sys
sys.path.insert(0,'scripts')
import json
import math
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import hardware_flight_sweep as h
import separate_drive_flight_sweep as s
from ringquad_control.flight import FlightController,vee,E3
from ringquad_control import params as p

class Candidate(FlightController):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.bias=np.zeros(3);self.g.i_max=4.;self.g.ki=np.array([POS,POS,2*POS])
 def update(self,pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp=None):
  old=self.g.i_zone
  if np.linalg.norm(vel)<.3:self.g.i_zone=10.
  thrust,torque=super().update(pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp)
  self.g.i_zone=old
  acc=self.g.kp*(pos_sp-pos)+self.g.kd*(-vel)+self.integral
  acc[2]=np.clip(acc[2],-self.g.max_accel_z,self.g.max_accel_z)
  axy=np.linalg.norm(acc[:2]);limit=np.tan(self.g.max_tilt)*(p.GRAVITY+acc[2])
  if axy>limit:acc[:2]*=limit/axy
  b3=acc+p.GRAVITY*E3;b3/=np.linalg.norm(b3)
  b1=np.array([np.cos(yaw_sp),np.sin(yaw_sp),0]);b2=np.cross(b3,b1);b2/=np.linalg.norm(b2)
  desired=np.column_stack([np.cross(b2,b3),b2,b3])
  err=.5*vee(desired.T@rot-rot.T@desired)
  if np.linalg.norm(err)<.5 and np.linalg.norm(rate)<1.:
   self.bias=np.clip(self.bias-np.array([ATT,ATT,.05])*err*dt,-8.,8.)
  return thrust,torque+p.INERTIA@self.bias

def task(t):
 global POS,ATT
 POS,ATT,name=t
 h.CASES={n:{'independent_propeller':True,**v} for n,v in s.CASES.items()}
 h.FlightController=Candidate
 return dict(position_ki=POS,attitude_ki=ATT,**h.evaluate((name,0)))
if __name__=='__main__':
 names=['nominal','thrust_minus20pct','one_motor_thrust_minus20pct','combined_adverse']
 with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(task,[(v,a,n) for v in [1.,1.5,2.] for a in [2.,5.,10.] for n in names]))
 open('hardware-blockers/results/disturbance-probe.json','w').write(json.dumps(rows,indent=2))
 for r in rows:print(r['position_ki'],r['attitude_ki'],r['case'],r['passed'],r['crash'],round(r['position_error_m'],3))
