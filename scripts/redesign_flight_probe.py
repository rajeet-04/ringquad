"""Exploratory controller probe; preserves old hardware stress results."""
import sys
import json
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import hardware_flight_sweep as h
from ringquad_control.flight import FlightController, vee, E3
from ringquad_control import params as p

class IntegralController(FlightController):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.bias=np.zeros(3)
    def update(self,pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp=None):
        thrust,torque=super().update(pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp)
        # Estimate disturbance torque from attitude error against the desired force.
        acc=self.g.kp*(pos_sp-pos)+self.g.kd*(-vel)+self.integral
        acc[2]=np.clip(acc[2],-self.g.max_accel_z,self.g.max_accel_z)
        axy=np.linalg.norm(acc[:2]);limit=np.tan(self.g.max_tilt)*(p.GRAVITY+acc[2])
        if axy>limit:acc[:2]*=limit/axy
        b3=acc+p.GRAVITY*E3;b3/=np.linalg.norm(b3)
        b1=np.array([np.cos(yaw_sp),np.sin(yaw_sp),0]);b2=np.cross(b3,b1);b2/=np.linalg.norm(b2)
        desired=np.column_stack([np.cross(b2,b3),b2,b3])
        err=.5*vee(desired.T@rot-rot.T@desired)
        self.bias=np.clip(self.bias-GAIN*err*dt,-10.,10.)
        return thrust,torque+p.INERTIA@self.bias


def probe(task):
    global GAIN
    gain,name=task;GAIN=gain
    h.FlightController=IntegralController
    return dict(gain=gain,**h.evaluate((name,0)))

if __name__=='__main__':
    cases=['nominal','cg_x_10mm_16s','cg_y_10mm','thrust_minus20pct','one_motor_thrust_minus20pct','torque_025']
    with ProcessPoolExecutor(max_workers=4) as pool:r=list(pool.map(probe,[(g,c) for g in [0.,5.,10.,15.] for c in cases]))
    open('hardware-results/redesign-flight-probe.json','w').write(json.dumps(r,indent=2))
    for v in r: print(v['gain'],v['case'],v['passed'],v['crash'],round(v['position_error_m'],3),flush=True)
