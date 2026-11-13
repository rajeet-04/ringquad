#!/usr/bin/env python3
"""Collect final redesign results without hiding earlier failed iterations."""
import hashlib
import json
import re
import subprocess
from pathlib import Path


def metrics(path):
    log=path.read_text()
    goal=re.search(r'mission complete in (\d+) s, (\d+) cm',log)
    drive=re.search(r'drive done, worst cross-track ([\d.]+) cm; minimum buffered footprint margin ([\d.-]+) mm',log)
    return dict(log=str(path),completed=bool(goal),
        passed=bool(goal and drive and int(goal[2])<=10 and float(drive[1])<=5 and float(drive[2])>0),
        duration_s=int(goal[1]) if goal else None,goal_error_cm=int(goal[2]) if goal else None,
        cross_track_cm=float(drive[1]) if drive else None,buffered_margin_mm=float(drive[2]) if drive else None,
        aborted='mission aborted:' in log)


if __name__=='__main__':
    root=Path('hardware-results')
    nominal=[metrics(p) for p in sorted((root/'final-default').glob('run-*.log'))]
    matrix=[json.loads(p.read_text()) for p in sorted((root/'final-matrix').glob('*/result.json'))]
    geom=json.loads((root/'independent-final-geometry.json').read_text())
    sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('src').rglob('*') if p.is_file() and p.suffix in ('.py','.cc','.xacro','.stl','.yaml','.sdf')}
    historical=[metrics(p) for d in ['redesign-90-initial','redesign-default','redesign-tight-landing','redesign-final-default','redesign-crowned-default','redesign-bounded-default','separate-drive-default','independent-final-default'] for p in sorted((root/d).glob('run-*.log'))]
    flight=json.loads((root/'separate-drive-flight.json').read_text())
    stress={'runs':len(flight['runs']),'accuracy_passes':sum(r['passed'] for r in flight['runs']),'crashes':sum(r['crash'] for r in flight['runs'])}
    interface=json.loads((root/'redesign-steering/result.json').read_text())
    recorded=json.loads((root/'final-matrix/source-hashes.json').read_text())
    hashes_match=all(sources.get(name)==value for name,value in recorded.items())
    result=dict(interface_checks=interface,source_hashes_match_matrix=hashes_match,flight_stress=stress,base_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        design={'fold_deg':90,'axis_standoff_m':.1,'drivetrain':'independent_propeller_and_wheel','nominal_mass_kg':1.98,'carrier_drive_mass_budget_kg_each':.05,'steering_stop_deg':25,'tread':'elliptical_crown_270x30mm','self_collision':True},
        source_hashes=sources,geometry=geom,nominal=nominal,sensitivity=matrix,
        simulation_regressions_passed=(hashes_match and interface['passed'] and len(nominal)==3 and all(r['passed'] for r in nominal) and len(matrix)==5 and all(r['passed'] for r in matrix) and not geom['body_failures'] and not geom['ring_pair_failures']),
        hardware_qualified=False,earlier_upright_square_rim_iterations=historical,
        limitations=['sampled rigid geometry','uncalibrated contact law','ideal positive latch','unknown material and actuator maps','9 of 60 thrust-loss/combined challenge runs miss position accuracy; none crash'])
    (root/'redesign-summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_hashes','geometry','earlier_upright_square_rim_iterations')},indent=2))
