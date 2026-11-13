"""Probe bounded flight references and measured CG/thrust calibration.

Old uncalibrated step failures remain in flight-sweep.json. This evaluates an
explicitly calibrated vehicle with the same torque limits and noise, not blind
robustness to an unknown loss of thrust.
"""
import json
import math
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import hardware_flight_sweep as h
from ringquad_control import mixer, params as p
from ringquad_control.flight import FlightController

BASE_ALLOCATE=mixer.wrench_to_thrusts
BASE_COMMAND=mixer.thrusts_to_commands


def run(task):
    name,seed=task
    c=h.CASES[name]
    mass=c.get('mass',h.NOMINAL_MASS)
    com=h.ESTIMATED_COM+np.array([c.get('cg_x',0.),c.get('cg_y',0.),0.])
    scale=np.full(4,c.get('thrust',1.));scale[0]*=c.get('weak_motor',1.)
    class CalibratedController(FlightController):
        def __init__(self,*a,**kw):
            super().__init__(mass=mass)
            self.time=0.
        def update(self,pos,vel,rot,rate,pos_sp,yaw_sp,dt,vel_sp=None):
            self.time+=dt
            target=np.array([1.,.5,1.5]);start=np.array([0.,0.,1.])
            distance=np.linalg.norm(target-start)
            carrot=start+(target-start)*min(1., .5*self.time/distance)
            yaw=min(yaw_sp,.2*self.time)
            return super().update(pos,vel,rot,rate,carrot,yaw,dt)
    h.FlightController=CalibratedController
    mixer.wrench_to_thrusts=lambda thrust,torque,centre_of_mass=None: BASE_ALLOCATE(thrust,torque,com)
    mixer.thrusts_to_commands=lambda force: BASE_COMMAND(force/scale)
    row=h.evaluate((name,seed))
    row['calibrated_mass_kg']=mass;row['calibrated_com_m']=com.tolist();row['calibrated_thrust_scale']=scale.tolist()
    return row


if __name__=='__main__':
    cases=['nominal','cg_x_10mm_16s','cg_y_10mm','thrust_minus20pct','one_motor_thrust_minus20pct','torque_025','combined_adverse']
    with ProcessPoolExecutor(max_workers=4) as pool: rows=list(pool.map(run,[(n,s) for n in cases for s in range(3)]))
    output={'reference_limits':{'position_carrot_m_s':.5,'yaw_carrot_rad_s':.2},'requires_measured_calibration':True,'runs':rows}
    open('hardware-results/redesign-calibrated-flight.json','w').write(json.dumps(output,indent=2))
    for n in cases:print(n,sum(r['passed'] for r in rows if r['case']==n),'/3',flush=True)
