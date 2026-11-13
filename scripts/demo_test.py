#!/usr/bin/env python3
"""Run fresh headless demo worlds; fail on abort, timeout, or missing completion.

Run in the ROS container after sourcing install/setup.bash:
  python3 scripts/demo_test.py --runs 3 --output /tmp/ringquad-demo
"""
import argparse
import json
import re
from validation_sources import snapshot, drift
import fcntl
import uuid
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-arg', nargs=2, action='append', default=[], metavar=('NAME','VALUE'))
    parser.add_argument('--steering', choices=('fixed', 'front'), default='front')
    parser.add_argument('--tread', choices=('rubber', 'mixed', 'omni'), default='rubber')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--output', type=Path, default=Path('/tmp/ringquad-demo'))
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    allowed = {'lock_break_torque','drivetrain','fold_ground_deg','servo_torque','servo_speed','friction_scale','knuckle_mass','ring_offset','steering_limit_deg','self_collision','tread_collision'}
    for name, value in args.model_arg:
        if name not in allowed:
            parser.error(f'unsupported model argument: {name}')
        if name == 'drivetrain':
            if value not in ('shared','separate'):parser.error('unknown drivetrain')
        elif name == 'tread_collision':
            if value not in ('rounded','cylinder','crowned'):parser.error('unknown tread collision')
        elif name == 'self_collision':
            if value not in ('true','false'):parser.error('self_collision must be true or false')
        else:
            import math
            if not math.isfinite(float(value)) or float(value) <= 0:
                parser.error('numeric model arguments must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=True)
    sources=snapshot();rows=[]
    (args.output/'source-hashes.json').write_text(json.dumps(sources,indent=2))
    def finish(code):
        changed=drift(sources,snapshot())
        result={'passed':code==0 and not changed,'sources_unchanged':not changed,'changed_sources':changed,'runs':rows}
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        return 1 if code or changed else 0
    for run in range(1, args.runs + 1):
        path = args.output / f'run-{run}.log'
        passed = False
        # Lock a ROS domain for this run, including concurrent test processes.
        domain_lock = None
        for domain in range(50, 150):
            candidate = open(f'/tmp/ringquad-test-domain-{domain}.lock', 'a')
            try:
                fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
                domain_lock = candidate
                break
            except BlockingIOError:
                candidate.close()
        if domain_lock is None:
            raise RuntimeError('no free test ROS domain')
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                   GZ_PARTITION='ringquad-test-' + uuid.uuid4().hex)
        print(f'run {run}: ROS domain {domain}, Gazebo partition {env["GZ_PARTITION"]}', flush=True)
        with path.open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'ringquad_control', 'demo.launch.py',
                 f'tread:={args.tread}', f'steering:={args.steering}', 'rviz:=false', 'gui:=false'] + [f'{n}:={v}' for n,v in args.model_arg],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
            try:
                deadline = time.monotonic() + args.timeout
                while process.poll() is None and time.monotonic() < deadline:
                    content = path.read_text()
                    if 'mission aborted:' in content:
                        break
                    if 'mission complete in' in content:
                        passed = True
                        break
                    time.sleep(0.2)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                # Gazebo's wrapper can exit while leaving its server orphaned.
                # Kill the remaining session group even when the launcher exited.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                domain_lock.close()
        print(f'run {run}: {"PASS" if passed else "FAIL"} ({path})', flush=True)
        content=path.read_text()
        goal=re.search(r'mission complete in (\d+) s, (\d+) cm',content)
        drive=re.search(r'drive done, worst cross-track ([\d.]+) cm; minimum buffered footprint margin ([\d.-]+) mm',content)
        row={'run':run,'completed':passed,'duration_s':int(goal[1]) if goal else None,
             'goal_error_cm':int(goal[2]) if goal else None,'cross_track_cm':float(drive[1]) if drive else None,
             'buffered_margin_mm':float(drive[2]) if drive else None}
        passed=passed and goal is not None and drive is not None and int(goal[2])<=10 and float(drive[1])<=5 and float(drive[2])>0
        row['passed']=passed;rows.append(row)
        if not passed:
            print('\n'.join(line for line in path.read_text().splitlines()
                            if 'ringquad_mission' in line)[-3000:])
            return finish(1)
    return finish(0)


if __name__ == '__main__':
    raise SystemExit(main())
