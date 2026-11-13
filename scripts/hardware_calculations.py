"""First-principles sizing bounds; assumptions are explicit, not material approval."""
import json
import math
from pathlib import Path
from ringquad_control import params as p
mass=p.MASS+.04
hover=math.sqrt(mass*p.GRAVITY/(4*p.K_THRUST))
kt=60/(2*math.pi*700)  # Ideal SI conversion for the assumed 700 KV motor.
rows=[]
for name,w in [('stopped_fold_limit',5.),('hover',hover),('maximum',p.MAX_ROTOR_SPEED)]:
    torque=(p.PROP_DRAG+p.RIM_DRAG)*w*w
    rows.append({'state':name,'speed_rad_s':w,'rpm':w*60/(2*math.pi),
      'outer_rim_speed_m_s':.135*w,'stored_energy_per_rotor_J':.5*(p.RING_IZZ+p.MOTOR_ROTOR_IZZ)*w*w,
      'rim_tension_N_if_all_60g_is_uniform_rim':.06*.1275*w*w/(2*math.pi),
      'gyro_torque_Nm_at_fold_rate_04':p.RING_IZZ*w*.4,
      'shaft_drag_Nm':torque,'ideal_torque_current_A_at_700KV':torque/kt})
result={'assumptions':{'ring_mass_kg':.06,'mean_radius_m':.1275,'motor_KV':700,'motor_Kt_Nm_per_A':kt,
 'battery_usable_Wh':35.52,'electrical_to_shaft_efficiency':.7,'mass_kg':mass},
 'rotor_states':rows,'peak_motor_current_A_ideal':p.MOTOR_MAX_TORQUE/kt,
 'ground_rpm_at_04m_s':.4/p.WHEEL_RADIUS*60/(2*math.pi),
 'limits':'no strength allowance without actual material/section/joint data; no thermal or voltage-sag validation without resistance, motor/ESC maps and cooling data; no sensorless-startup/cogging model'}
Path('hardware-results').mkdir(exist_ok=True);Path('hardware-results/sizing.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
