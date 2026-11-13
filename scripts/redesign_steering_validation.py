"""Isolated yaw-tracking check for the physical crowned rim."""
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

folder=Path('hardware-results/redesign-steering');folder.mkdir(exist_ok=False)
env=dict(os.environ,ROS_DOMAIN_ID='180',GZ_PARTITION='ringquad-steering-'+uuid.uuid4().hex)
with (folder/'sim.log').open('w') as log:
    sim=subprocess.Popen(['ros2','launch','ringquad_control','sim.launch.py','gui:=false','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    try:
        integration=subprocess.run([sys.executable,'scripts/mission_test.py'],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=600)
        (folder/'integration.log').write_text(integration.stdout)
        check=subprocess.run([sys.executable,'scripts/steering_test.py'],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=600)
        (folder/'tracking.log').write_text(check.stdout)
        rows=[json.loads(line) for line in check.stdout.splitlines() if line.startswith('{')]
        result={'interface_returncode':integration.returncode,'returncode':check.returncode,'passed':integration.returncode==0 and check.returncode==0 and len(rows)==2,'measurements':rows}
        (folder/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    finally:
        if sim.poll() is None:
            sim.send_signal(signal.SIGINT)
            try:sim.wait(timeout=10)
            except subprocess.TimeoutExpired:pass
        try:os.killpg(sim.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    raise SystemExit(0 if result['passed'] else 1)
