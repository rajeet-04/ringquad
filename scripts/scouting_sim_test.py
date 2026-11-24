#!/usr/bin/env python3
"""End-to-end inspection loop against the simulator running in Podman (host side).

Prereqs: container `ringquad-harness` (or any container publishing 127.0.0.1:8765) running
  ros2 launch ringquad_control demo.launch.py mission:=false camera:=true world:=scouting gui:=false rviz:=false
and a local MLX model directory. Scenarios:
  success      hover, observe the patch, review, request a closer view, collect it
  stale        same, but wait until the evidence pose is too old: the bridge must reject
  interrupted  same, but force `land` mid-request: the request must end failed
Usage: scripts/scouting_sim_test.py --model-path DIR --out DIR [--scenario success]
"""
import argparse
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

CONTAINER = 'ringquad-harness'
BRIDGE = 'http://127.0.0.1:8765'
ROOT = Path(__file__).resolve().parent.parent
HARNESS = ['uv', 'run', '--project', str(ROOT / 'harness'), '--extra', 'mlx', 'ringquad-harness']


def ros(cmd, timeout=30):
    full = f'source /opt/ros/jazzy/setup.bash && source ~/ringquad_ws/install/setup.bash && {cmd}'
    return subprocess.run(['podman', 'exec', '-u', 'ubuntu', CONTAINER, 'bash', '-lc', full],
                          capture_output=True, text=True, timeout=timeout)


def state():
    out = ros('ros2 topic echo --once --field data /ringquad/state std_msgs/msg/String', 20).stdout
    return out.split('\n')[0].strip()


def wait_for(pred, timeout, what):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return
        time.sleep(1)
    sys.exit(f'FAIL: timed out waiting for {what}')


def run(args, **kw):
    print('+', ' '.join(map(str, args)))
    return subprocess.run(args, text=True, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-path', required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--scenario', choices=('success', 'stale', 'interrupted'), default='success')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    wait_for(lambda: json.load(urllib.request.urlopen(BRIDGE + '/health', timeout=5))['calibrated'], 60, 'camera bridge')
    ros('ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: fly}"')
    wait_for(lambda: state().startswith('mode=fly') and float(state().split('z=')[1]) > 1.0, 60, 'hover')
    time.sleep(5)  # settle

    # The 2B model sometimes fails or says monitor on a rendered frame; both are valid outcomes, so
    # observe again (up to 3 sessions) until one observation gets an inspect decision.
    session = None
    for attempt in range(1, 4):
        session = args.out / f'{args.scenario}-session-{attempt}'
        if run(HARNESS + ['live', '--url', BRIDGE + '/stream', '--output', session, '--stop-after-observations', '1',
                          '--max-frames', '600']).returncode:
            sys.exit('FAIL: live observation')
        run(HARNESS + ['review', session, '--model-path', args.model_path, '--review-timeout', '120'])  # rc 1 = some reviews failed
        events = [json.loads(line) for line in (session / 'events.jsonl').read_text().splitlines()]
        if any(e['event_type'] == 'review' and (e['data'].get('decision') or {}).get('outcome') == 'inspect' for e in events):
            break
        print(f'attempt {attempt}: no inspect decision, observing again')
    else:
        sys.exit('FAIL: the model never proposed an inspection')

    if args.scenario == 'stale':
        time.sleep(40)  # sim runs in real time: evidence pose is now older than the 30 s limit
    interrupter = None
    if args.scenario == 'interrupted':
        ros('ros2 param set /camera_bridge inspection_speed 0.08')  # ~6 s move, long enough to interrupt
        # React inside the container: the first setpoint the bridge publishes means the request is
        # running; command `land` right then.
        interrupter = subprocess.Popen(['podman', 'exec', '-u', 'ubuntu', CONTAINER, 'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash && source ~/ringquad_ws/install/setup.bash && '
            'ros2 topic echo --once /ringquad/goal geometry_msgs/msg/PoseStamped >/dev/null && '
            'ros2 topic pub -t 6 -r 5 /ringquad/mode std_msgs/msg/String "{data: land}"'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)  # let the subscription come up before the request is submitted
    inspect = subprocess.Popen(HARNESS + ['inspect', session, '--bridge', BRIDGE, '--output', args.out / f'{args.scenario}-inspection'],
                               text=True)
    code = inspect.wait(timeout=300)
    if interrupter:
        interrupter.wait(timeout=60)
    lines = [json.loads(line) for line in (args.out / f'{args.scenario}-inspection' / 'inspection.jsonl').read_text().splitlines()]
    trace = next((line['trace'] for line in lines if line['event'] == 'trace'), [])
    result = next(line for line in lines if line['event'] == 'result')
    states = [t['state'] for t in trace]
    print(json.dumps({'scenario': args.scenario, 'exit': code, 'states': states, 'result': result}, indent=2))
    expected = {'success': (0, ['accepted', 'running', 'completed'], 'closed'),
                'stale': (1, ['rejected'], 'rejected'),
                'interrupted': (1, ['accepted', 'running', 'failed'], 'failed')}[args.scenario]
    if (code, states, result['outcome']) != expected:
        sys.exit(f'FAIL: expected {expected}')
    print('PASS')


if __name__ == '__main__':
    main()
