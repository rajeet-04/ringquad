#!/usr/bin/env python3
"""Seeded hardware-uncertainty stress tests of the torque-limited Python plant.

This is not the Gazebo actuator implementation, CFD or hardware qualification.
Limits are evaluated with fixed controller settings, no scenario retuning.
"""
import json
import math
from pathlib import Path
from collections import deque
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from ringquad_control import mixer, params as p
from ringquad_control.flight import FlightController, yaw_of
from ringquad_control.plant import Plant, Rotor, skew

NOMINAL_MASS=p.MASS+2*p.STEERING_KNUCKLE_MASS
ESTIMATED_COM=np.array([2*p.STEERING_KNUCKLE_MASS*p.ARM_XY/NOMINAL_MASS,0.,0.])
CASES={
 'nominal':{}, 'payload_100g':{'mass':1.82}, 'payload_250g':{'mass':1.97},
 'cg_x_5mm':{'cg_x':.005}, 'cg_x_10mm':{'cg_x':.01}, 'cg_y_5mm':{'cg_y':.005},
 'cg_y_10mm':{'cg_y':.01}, 'torque_030':{'torque':.3}, 'torque_025':{'torque':.25},
 'rim_drag_half':{'drag':.5}, 'rim_drag_double':{'drag':2.},
 'thrust_minus20pct':{'thrust':.8}, 'one_motor_thrust_minus20pct':{'weak_motor':.8},
 'inertia_minus20pct':{'inertia':.8}, 'inertia_plus20pct':{'inertia':1.2},
 'delay_20ms':{'delay':.02}, 'delay_40ms':{'delay':.04}, 'sensor_noise_double':{'noise':2.},
 'gust_3N':{'gust':3.}, 'gust_5N':{'gust':5.},
 'ring_80g':{'ring_mass':.08,'mass':1.8},
 'cg_x_5mm_16s':{'cg_x':.005,'seconds':16},
 'cg_y_5mm_16s':{'cg_y':.005,'seconds':16},
 'cg_x_10mm_16s':{'cg_x':.01,'seconds':16},
 'combined_adverse':{'mass':1.97,'torque':.3,'drag':2.,'thrust':.85,'cg_x':.005,'delay':.02},
}


def evaluate(task):
    name,seed=task;c=CASES[name];rng=np.random.default_rng(seed)
    rotor=(Rotor(izz=p.MOTOR_ROTOR_IZZ+p.PLAIN_PROP_IZZ,max_torque=c.get('torque',.4),rim_drag=0.)
           if c.get('independent_propeller',False) else
           Rotor.ring(c.get('ring_mass',.06),max_torque=c.get('torque',.4),rim_drag=p.RIM_DRAG*c.get('drag',1.)))
    actual_com=ESTIMATED_COM+np.array([c.get('cg_x',0.),c.get('cg_y',0.),0.])
    scale=np.full(4,c.get('thrust',1.));scale[0]*=c.get('weak_motor',1.)
    plant=Plant(rotor=rotor,mass=c.get('mass',NOMINAL_MASS),inertia=p.INERTIA*c.get('inertia',1.),centre_of_mass=actual_com,thrust_scale=scale)
    plant.pos[:]=[0,0,1]
    ctl=FlightController(mass=NOMINAL_MASS)
    dt=.004;delay=deque(maxlen=round(c.get('delay',.008)/dt)+1)
    command=np.full(4,math.sqrt(plant.mass*p.GRAVITY/(4*p.K_THRUST)))
    target=np.array([1.,.5,1.5]);target_yaw=math.pi/4
    peak_tilt=peak_error=peak_lock_gyro=0.;sat=0;count=0;power=[];crash=False
    position=np.zeros(3)
    for step in range(round(c.get('seconds',8)/.001)):
        t=step*.001
        plant.ext_force=np.array([0,c.get('gust',1.5),0]) if 1<t<2 else np.zeros(3)
        if step%4==0:
            noise=c.get('noise',1.)
            rot=plant.rot@(np.eye(3)+skew(rng.normal(0,math.radians(.3)*noise,3)))
            u,_,vt=np.linalg.svd(rot);rot=u@vt
            position=plant.pos-plant.rot@actual_com
            state=(position+rng.normal(0,.01*noise,3),plant.vel+rng.normal(0,.03*noise,3),rot,plant.rate+rng.normal(0,.05*noise,3))
            delay.append(state)
            thrust,torque=ctl.update(*delay[0],target,target_yaw,dt)
            command=mixer.thrusts_to_commands(mixer.wrench_to_thrusts(thrust,torque,centre_of_mass=ESTIMATED_COM))
            error=float(np.linalg.norm(position-target));peak_error=max(peak_error,error)
            peak_tilt=max(peak_tilt,plant.tilt_deg())
            peak_lock_gyro=max(peak_lock_gyro,float(max(abs(plant.joint_vel[:2]))*rotor.izz*abs(plant.rate[0])))
            sat+=int(np.any(abs(plant.motor_torque)>.99*rotor.max_torque));count+=1
            if t>c.get('seconds',8)-2:power.append(plant.power())
            if plant.tilt_deg()>60 or position[2]<.1 or np.linalg.norm(position)>10:
                crash=True;break
        plant.step(command)
    position=plant.pos-plant.rot@actual_com
    error=float(np.linalg.norm(position-target));yaw_error=abs(math.degrees((yaw_of(plant.rot)-target_yaw+math.pi)%(2*math.pi)-math.pi))
    passed=not crash and error<.1 and yaw_error<5 and np.linalg.norm(plant.vel)<.2
    return dict(case=name,seed=seed,passed=bool(passed),crash=crash,position_error_m=error,yaw_error_deg=yaw_error,
                final_speed_m_s=float(np.linalg.norm(plant.vel)),peak_tilt_deg=peak_tilt,peak_position_error_m=peak_error,
                torque_saturation_fraction=sat/max(count,1),mean_final_shaft_power_w=float(np.mean(power)) if power else None,
                peak_gyro_lock_y_proxy_Nm=peak_lock_gyro)


def static_envelope():
    rows=[]
    for mass in (1.72,1.97,2.22):
        for drag in (.5,1.,2.):
            for torque in (.25,.3,.4):
                w=math.sqrt(mass*p.GRAVITY/(4*p.K_THRUST))
                load=(p.PROP_DRAG+p.RIM_DRAG*drag)*w*w
                max_w=min(p.MAX_ROTOR_SPEED,math.sqrt(torque/(p.PROP_DRAG+p.RIM_DRAG*drag)))
                rows.append(dict(mass_kg=mass,rim_drag_scale=drag,torque_limit_Nm=torque,hover_speed_rad_s=w,
                  hover_shaft_torque_Nm=load,hover_torque_margin_Nm=torque-load,static_thrust_to_weight=4*p.K_THRUST*max_w**2/(mass*p.GRAVITY),
                  hover_shaft_power_w=4*load*w,endurance_min_estimate=35.52/(4*load*w/.7)*60))
    return rows


if __name__=='__main__':
    tasks=[(name,seed) for name in CASES for seed in range(3)]
    with ProcessPoolExecutor(max_workers=4) as pool:results=list(pool.map(evaluate,tasks))
    output={'criteria':'8-second (16-second CG follow-ups explicitly labelled) 3D step + 45deg yaw + 1-second gust; final position <10cm, yaw <5deg, speed <0.2m/s; crash if >60deg tilt or height<0.1m',
            'cases':CASES,'runs':results,'static_envelope':static_envelope(),
            'limits':'approximate rigid body; no contact, aero interference, voltage sag, heat or lock compliance; gyro lock estimate is a proxy, not measured reaction'}
    Path('hardware-results').mkdir(exist_ok=True);Path('hardware-results/flight-sweep.json').write_text(json.dumps(output,indent=2))
    print(json.dumps({name:sum(r['passed'] for r in results if r['case']==name) for name in CASES},indent=2))
