#!/usr/bin/env python3
"""Run fresh headless demo worlds; fail on abort, timeout, or missing completion.

Run in the ROS container after sourcing install/setup.bash:
  python3 scripts/demo_test.py --runs 3 --output /tmp/ringquad-demo
"""
import argparse
import fcntl
import uuid
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steering', choices=('fixed', 'front'), default='fixed')
    parser.add_argument('--tread', choices=('rubber', 'mixed', 'omni'), default='omni')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--output', type=Path, default=Path('/tmp/ringquad-demo'))
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
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
                 f'tread:={args.tread}', f'steering:={args.steering}', 'rviz:=false', 'gui:=false'],
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
        if not passed:
            print('\n'.join(line for line in path.read_text().splitlines()
                            if 'ringquad_mission' in line)[-3000:])
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
