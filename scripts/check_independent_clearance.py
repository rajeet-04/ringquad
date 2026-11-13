"""Independent mechanical checks for propeller/rim and rear-carrier clearance."""
import json
import math
# Box blade vertices bound the entire rotation sweep. Hub is smaller.
radial_tip=.115
half_chord=.0125
half_thickness=.0015
pitch=math.radians(15)
tangent=half_chord*math.cos(pitch)+half_thickness*math.sin(pitch)
axial=half_chord*math.sin(pitch)+half_thickness*math.cos(pitch)
blade_sweep_radius=math.hypot(radial_tip,tangent)
minimum_ring_hole_radius=.120
carrier_front=-.027
assert blade_sweep_radius < minimum_ring_hole_radius
assert carrier_front < -max(.006,axial)
result={'propeller_swept_radius_m':blade_sweep_radius,'minimum_rim_hole_radius_m':minimum_ring_hole_radius,
        'radial_clearance_lower_bound_m':minimum_ring_hole_radius-blade_sweep_radius,
        'propeller_half_axial_envelope_m':max(.006,axial),'rear_carrier_to_propeller_axial_gap_m':-max(.006,axial)-carrier_front,
        'limits':'rigid design dimensions; blade deflection, bearings, tolerances and detailed belt/gear packaging need CAD and tests','passed':True}
open('hardware-results/independent-prop-clearance.json','w').write(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
