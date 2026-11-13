"""Transparent energy/thrust estimates; no electrical or structural qualification."""
import json
import math
from ringquad_control import params as p
mass=p.MASS+.04
hover=math.sqrt(mass*p.GRAVITY/(4*p.K_THRUST))
flight_inertia=p.PLAIN_PROP_IZZ+p.MOTOR_ROTOR_IZZ
old_hover=math.sqrt(1.72*p.GRAVITY/(4*p.LEGACY_K_THRUST))
old_inertia=p.RING_IZZ+p.MOTOR_ROTOR_IZZ
result={'nominal_mass_kg':mass,'hover_speed_rad_s':hover,'hover_speed_rpm':hover*60/(2*math.pi),
        'static_max_thrust_to_weight':4*p.MAX_THRUST/(mass*p.GRAVITY),
        'assumed_flight_spin_inertia_kg_m2':flight_inertia,'estimated_prop_and_bell_energy_at_hover_J_each':.5*flight_inertia*hover**2,
        'estimated_prop_and_bell_energy_at_max_J_each':.5*flight_inertia*p.MAX_ROTOR_SPEED**2,
        'estimated_legacy_spinning_rim_energy_at_legacy_hover_J_each':.5*old_inertia*old_hover**2,
        'wheel_energy_at_0_4m_s_J_each':.5*p.RING_IZZ*(.4/p.WHEEL_RADIUS)**2,
        'hover_angular_momentum_Nms_each':flight_inertia*hover,'legacy_hover_angular_momentum_Nms_each':old_inertia*old_hover,
        'limits':'assumed rotor inertia/thrust coefficients; no aerodynamic interference, electrical, thermal, fatigue or containment certification'}
open('hardware-results/independent-sizing.json','w').write(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
