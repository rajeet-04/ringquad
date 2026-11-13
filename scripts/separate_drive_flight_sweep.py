"""Seeded flight uncertainty sweep for the non-spinning-ring architecture."""
import json
from concurrent.futures import ProcessPoolExecutor
import hardware_flight_sweep as h

CASES={
 'nominal':{},'payload_100g':{'mass':h.NOMINAL_MASS+.1},'payload_250g':{'mass':h.NOMINAL_MASS+.25},
 'cg_x_5mm':{'cg_x':.005,'seconds':16},'cg_y_5mm':{'cg_y':.005,'seconds':16},
 'cg_x_10mm':{'cg_x':.01,'seconds':16},'cg_y_10mm':{'cg_y':.01,'seconds':16},
 'torque_030':{'torque':.3},'torque_025':{'torque':.25},
 'thrust_minus20pct':{'thrust':.8},'one_motor_thrust_minus20pct':{'weak_motor':.8},
 'inertia_minus20pct':{'inertia':.8},'inertia_plus20pct':{'inertia':1.2},
 'delay_20ms':{'delay':.02},'delay_40ms':{'delay':.04},'sensor_noise_double':{'noise':2.},
 'gust_3N':{'gust':3.},'gust_5N':{'gust':5.},
 'drive_budget_80g_each':{'mass':h.NOMINAL_MASS+.12},
 'combined_adverse':{'mass':h.NOMINAL_MASS+.25,'torque':.3,'thrust':.85,'cg_x':.005,'delay':.02,'seconds':16},
}


def evaluate(task):
    h.CASES={name:{'independent_propeller':True,**settings} for name,settings in CASES.items()}
    return h.evaluate(task)


if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=4) as pool: rows=list(pool.map(evaluate,[(n,s) for n in CASES for s in range(3)]))
    result={'nominal_mass_kg':h.NOMINAL_MASS,'cases':CASES,'runs':rows,
      'limits':'approximate Python flight plant; separate lightweight props, stationary rims; old stress criteria retained; no CFD, contact, electrical or thermal model'}
    open('hardware-results/separate-drive-flight.json','w').write(json.dumps(result,indent=2))
    for n in CASES:print(n,sum(r['passed'] for r in rows if r['case']==n),'/3; crashes',sum(r['crash'] for r in rows if r['case']==n),flush=True)
