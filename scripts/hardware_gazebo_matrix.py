#!/usr/bin/env python3
"""Historical shared-rim comparison matrix; use redesign_validation.py for the final architecture."""
import argparse
import json
import time
import os
from pathlib import Path
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT=Path.cwd()
OUT=ROOT/'hardware-results'/'gazebo'
CANDIDATE={'drivetrain':'shared','fold_ground_deg':'80','tread_collision':'rounded','ring_offset':'.06','steering_limit_deg':'25','self_collision':'true'}
RESUME=False
CASES={
 'clearance_candidate':{},
 'servo_half_torque':{'servo_torque':'.5'},
 'servo_quarter_torque':{'servo_torque':'.25'},
 'servo_slow':{'servo_speed':'.5'},
 'low_traction':{'friction_scale':'.4'},
 'high_traction':{'friction_scale':'1.5'},
 'heavier_knuckles':{'knuckle_mass':'.04'},
 'legacy_self_collision':{'ring_offset':'.035','steering_limit_deg':'35'},
}


def run(item):
    name,changes=item;folder=OUT/name;folder.mkdir(parents=True,exist_ok=True)
    settings={**CANDIDATE,**changes}
    previous_log=folder/'run-1.log'
    if RESUME and previous_log.exists() and 'mission complete in' in previous_log.read_text() and (folder/'runner.log').exists() and 'run 1: PASS' in (folder/'runner.log').read_text():
        print(f'{name}: cached PASS',flush=True)
        return {'case':name,'settings':settings,'passed':True,'returncode':0,'cached':True}
    if previous_log.exists():
        archive=folder/f'prior-{time.time_ns()}';archive.mkdir()
        for file in list(folder.iterdir()):
            if file!=archive and not file.name.startswith('prior-'):file.rename(archive/file.name)
    command=[sys.executable,'scripts/demo_test.py','--runs','1','--steering','front','--tread','rubber','--timeout','900','--output',str(folder)]
    for key,value in settings.items():command+=['--model-arg',key,value]
    env=dict(os.environ,RINGQUAD_HARDWARE_LOG=str(folder/'joint-loads.csv'))
    result=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=940)
    (folder/'runner.log').write_text(result.stdout)
    print(f'{name}: {"PASS" if result.returncode==0 else "FAIL"}',flush=True)
    return {'case':name,'settings':settings,'passed':result.returncode==0,'returncode':result.returncode}


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    parser=argparse.ArgumentParser();parser.add_argument('--resume',action='store_true');RESUME=parser.parse_args().resume
    with ThreadPoolExecutor(max_workers=2) as executor:results=list(executor.map(run,CASES.items()))
    (OUT/'matrix.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))
