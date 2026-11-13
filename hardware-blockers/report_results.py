"""Collect the final candidate evidence; never promotes simulation to physical release."""
import csv,hashlib,json,math,re
from pathlib import Path
root=Path('hardware-blockers/results')
def read(name):return json.loads((root/name).read_text())
nominal=read('release-nominal/result.json');matrix=read('release-matrix/results.json');provenance=read('release-matrix/provenance.json')
image=read('release-image-mission/result.json');loads_mission=read('release-load-mission/result.json')
steering=read('release-steering/result.json');fault=read('release-fault-inhibit/result.json')
flight=read('final-flight-finite033.json');geometry=read('final-clearance.json');assembly=read('assembly-clearance.json');rear=read('rear-support.json')
units=int(re.search(r'(\d+) passed', (root/'unit-tests.log').read_text())[1])
mission_rows=[*nominal['runs'],*matrix,*image['runs'],*loads_mission['runs']]
observed=[];discarded=0
for row in csv.DictReader((root/'release-loads.csv').open()):
    try:
        fields=['time','force_x','force_y','force_z','torque_x','torque_y','torque_z','lock_torque','lock_broken']
        values={k:float(row[k]) for k in fields}
        if not all(math.isfinite(v) for v in values.values()):raise ValueError('non-finite sample')
        observed.append(values)
    except (TypeError,ValueError,KeyError):discarded+=1
assert observed
force=max(math.sqrt(sum(r[k]**2 for k in ['force_x','force_y','force_z'])) for r in observed)
moment=max(math.sqrt(sum(r[k]**2 for k in ['torque_x','torque_y','torque_z'])) for r in observed)
latch=max(abs(r['lock_torque']) for r in observed)
checks={
 'python_tests':units==61,
 'native_actuator_and_latch_checks':'passed' in (root/'native-tests.log').read_text(),
 'nominal_missions':nominal['passed'] and len(nominal['runs'])==3,
 'sensitivity_missions':len(matrix)==5 and all(r['passed'] for r in matrix) and provenance['sources_unchanged'],
 'image_mission':image['passed'] and read('image-check.json')['passed'],
 'load_mission':loads_mission['passed'] and all(not r['lock_broken'] for r in observed),
 'interface_and_steering':steering['passed'] and all(math.isfinite(r['measured_yaw_rate']) and abs(r['measured_yaw_rate']-r['requested_yaw_rate'])<=.08 for r in steering['measurements']),
 'fault_inhibit':fault['passed'],
 'flight_sweep':len(flight['runs'])==60 and all(r['passed'] and not r['crash'] for r in flight['runs']),
 'geometry':not geometry['body_failures'] and not geometry['ring_pair_failures'] and assembly['passed'] and rear['passed'] and read('mesh-check.json')['passed'],
 'mass_budget':read('mass-budget.json')['passed'],
 'structural_screen':all(read('design-checks.json')[k] for k in ['stress_screen_passed','deflection_screen_passed','drive_screen_passed']),
 'cad_parts':all(r['valid'] for r in json.loads(Path('hardware-blockers/cad/volumes.json').read_text()).values())}
result={'simulation_acceptance_passed':all(checks.values()),'physical_release_ready':False,'checks':checks,
 'python_tests_passed':units,'full_missions_passed':sum(r['passed'] for r in mission_rows),'full_missions_total':len(mission_rows),
 'worst_goal_error_cm':max(r['goal_error_cm'] for r in mission_rows),'worst_cross_track_cm':max(r['cross_track_cm'] for r in mission_rows),
 'minimum_buffered_margin_mm':min(r['buffered_margin_mm'] for r in mission_rows),'steering_measurements':steering['measurements'],
 'numerical_flight_runs_passed':sum(r['passed'] for r in flight['runs']),
 'sampled_front_knuckle_loads':{'samples':len(observed),'discarded_partial_rows':discarded,'max_force_norm_N':force,'max_moment_norm_Nm':moment,'max_latch_torque_Nm':latch,'last_sample_time_s':max(r['time'] for r in observed),'limit':'250Hz samples in joint frames; not continuous peak loads, stress/strain or proof tests'},
 'engine_version':Path('build/ringquad-physics/version.txt').read_text().strip(),
 'physical_gates':['final parts, full assembly and fold actuator/linkage selection','installed thrust and mass/CG calibration','thermal/current enforcement and endurance','floor/contact/preload/wear calibration','structural proof/fatigue and rotor containment']}
(root/'SUMMARY.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(result,indent=2,allow_nan=False))
assert result['simulation_acceptance_passed']
