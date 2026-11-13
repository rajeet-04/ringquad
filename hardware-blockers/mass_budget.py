"""Check CAD-derived component masses against the unchanged provisional allocations."""
import json
from pathlib import Path
cad=json.loads(Path('hardware-blockers/cad/volumes.json').read_text())
mass=lambda name:cad[name]['mass_g']
flight={'motor':67.5,'standoff':mass('standoff'),'mounts_allowance':8.}
module={'gearmotor':44.,'carrier':mass('carrier'),'three_rollers':3*mass('v_roller'),
        'four_bearings_allowance':20.,'fasteners_allowance':10.,'shaft_adapter_allowance':5.,
        'latch_ears':mass('latch_ears'),'latch_tongue':mass('latch_tongue'),'latch_pin':mass('latch_pin')}
result={'provisional_not_weighed':True,'flight_arm_g':flight,'flight_arm_sum_g':sum(flight.values()),'flight_arm_allocation_g':120.,
 'wheel_module_g':module,'wheel_module_sum_g':sum(module.values()),'wheel_module_allocation_g':160.,
 'total_vehicle_budget_kg':(1102+4*(120+160+72+15)+2*40)/1000,
 'passed':sum(flight.values())<=120 and sum(module.values())<=160,
 'unresolved':'complete assembly, fold linkage and final purchased parts remain unweighed; assumed mounts/bearings/fasteners/adapter masses require detailed BOM closure'}
Path('hardware-blockers/results/mass-budget.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert result['passed']
