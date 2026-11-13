"""Conservative beam/pin/drive/power screening, not material or impact certification."""
import json
import math
from pathlib import Path

F=300.;E=71e9;allow=200e6
carrier_length=.095
steel_allow=100e6
I=math.pi/64*(.017**4-.010**4);J=2*I
bending=(F*.1+10)*.0085/I
shear=16*.0085/J
standoff_stress=1.3*math.sqrt(bending*bending+3*shear*shear)
strong=(.016*.018**3-.014*.016**3)/12
weak=(.018*.016**3-.016*.014**3)/12
beam_stress=1.5*max(F*carrier_length*.009/strong,F*carrier_length*.008/weak)
beam_deflection=F*carrier_length**3/(3*E*min(strong,weak))
proof_torque=16.;pin_force=proof_torque/.012
pin_shear=pin_force/(2*math.pi*.004**2/4)
ears_bearing=pin_force/(2*.003*.004)
ears_bending=(pin_force/2)*.008*.007/(.003*.014**3/12)
motor_stall=.0980665*1.9;ratio=.103/.013;efficiency=.8
wheel_stall=motor_stall*ratio*efficiency
speed=.4/.135
no_load_speed=(550*2*math.pi/60)/ratio*.8 # manufacturer speed tolerance screen
available=wheel_stall*(1-speed/no_load_speed)
preload=10.;friction=.4
rows={
 'standoff_combined_stress_Pa':standoff_stress,'carrier_root_stress_Pa':beam_stress,
 'carrier_tip_deflection_m':beam_deflection,'pin_proof_shear_Pa':pin_shear,
 'ears_proof_bearing_Pa':ears_bearing,'ears_proof_bending_Pa':ears_bending,
 'material_allowable_assumed_Pa':allow,'steel_allowable_assumed_Pa':steel_allow,'design_force_N':F,'carrier_design_span_m':carrier_length,'minimum_room_temperature_yield_required_Pa':460e6,'temperature_derating_assumed':.87,'yield_safety_factor':2.,'proof_force_to_observed_141_09N_ratio':F/141.09070271470762,'wheel_torque_available_at_0_4m_s_Nm':available,
 'friction_drive_torque_at_minimum_mu_Nm':preload*friction*.103,
 'wheel_required_peak_Nm':.33,'roller_ratio':ratio,
 'stress_screen_passed':max(standoff_stress,beam_stress,ears_bearing,ears_bending)<allow and pin_shear<steel_allow,
 'deflection_screen_passed':beam_deflection<.001,'drive_screen_passed':min(available,preload*friction*.103)>.33,
 'physical_proof_passed':False,
 'assumptions':'300N concentrated on one carrier; CAD bounds carrier span at95mm (roller centre90mm); 1.5 carrier and1.3 standoff stress factors; 7075-T651 carrier/7075-T6 seamless tube, E71GPa, room-temperature certified yield>=460MPa; assumed0.87 temperature derating and SF2 give200MPa allowable. Elevated-temperature coupons, joints, fatigue and physical proof remain required.'}
Path('hardware-blockers/results/design-checks.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
