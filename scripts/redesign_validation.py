#!/usr/bin/env python3
"""Regression matrix for upright full-rim redesign; old failures stay untouched."""
import argparse
from validation_sources import snapshot, drift
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

CASES={
    'servo_quarter_torque': {'servo_torque': '.25'},
    'servo_slow': {'servo_speed': '.5'},
    'low_traction': {'friction_scale': '.4'},
    'high_traction': {'friction_scale': '1.5'},
    'heavier_knuckles_calibrated': {'knuckle_mass': '.06'},
}


def run(item):
    name, settings=item
    folder=OUT/name
    folder.mkdir(parents=True,exist_ok=False)
    command=[sys.executable,'scripts/demo_test.py','--runs','1','--timeout','900','--output',str(folder)]
    for key,value in settings.items():command+=['--model-arg',key,value]
    result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=960)
    (folder/'runner.log').write_text(result.stdout)
    log=(folder/'run-1.log').read_text()
    goal=re.search(r'mission complete in (\d+) s, (\d+) cm',log)
    drive=re.search(r'drive done, worst cross-track ([\d.]+) cm; minimum buffered footprint margin ([\d.-]+) mm',log)
    passed=(result.returncode==0 and goal is not None and drive is not None
            and int(goal[2])<=10 and float(drive[1])<=5 and float(drive[2])>0)
    row=dict(case=name,settings=settings,passed=passed,returncode=result.returncode,
             duration_s=int(goal[1]) if goal else None,goal_error_cm=int(goal[2]) if goal else None,
             cross_track_cm=float(drive[1]) if drive else None,buffered_margin_mm=float(drive[2]) if drive else None)
    print(json.dumps(row),flush=True)
    (folder/'result.json').write_text(json.dumps(row,indent=2))
    return row


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='hardware-results/redesign-final-matrix');parser.add_argument('--workers',type=int,default=1)
    args=parser.parse_args();OUT=Path(args.output);OUT.mkdir(parents=True,exist_ok=False)
    sources=snapshot()
    (OUT/'source-hashes.json').write_text(json.dumps(sources,indent=2))
    with ThreadPoolExecutor(max_workers=args.workers) as pool: rows=list(pool.map(run,CASES.items()))
    (OUT/'results.json').write_text(json.dumps(rows,indent=2))
    changed=drift(sources,snapshot())
    (OUT/'provenance.json').write_text(json.dumps({'sources_unchanged':not changed,'changed_sources':changed},indent=2))
    raise SystemExit(0 if all(r['passed'] for r in rows) and not changed else 1)
