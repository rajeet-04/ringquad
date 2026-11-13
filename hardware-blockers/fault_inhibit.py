"""Inject an intentionally weak physical latch, verify its fault inhibits takeoff."""
import argparse,json,os,signal,subprocess,sys,uuid
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--output',default='hardware-blockers/results/fault-inhibit');args=parser.parse_args()
folder=Path(args.output);folder.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,ROS_DOMAIN_ID='174',GZ_PARTITION='fault-'+uuid.uuid4().hex)
with (folder/'sim.log').open('w') as log:
    sim=subprocess.Popen(['ros2','launch','ringquad_control','sim.launch.py','gui:=false','rviz:=false','lock_break_torque:=.00001'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        code='''import rclpy,time,json
from rclpy.node import Node
from std_msgs.msg import Bool,String
rclpy.init();n=Node('fault_acceptance');s={'fault':False,'state':''}
def fault(m):s['fault']=s['fault'] or m.data
def state(m):s['state']=m.data
n.create_subscription(Bool,'/ringquad/lock_fault',fault,10);n.create_subscription(String,'/ringquad/state',state,10)
p=n.create_publisher(String,'/ringquad/mode',10);deadline=time.monotonic()+60
while not s['fault'] and time.monotonic()<deadline:
 p.publish(String(data='drive'));rclpy.spin_once(n,timeout_sec=.05)
end=time.monotonic()+3
while time.monotonic()<end:
 p.publish(String(data='fly'));rclpy.spin_once(n,timeout_sec=.05)
print(json.dumps(s));n.destroy_node();rclpy.shutdown()
'''
        check=subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True,timeout=75)
        (folder/'client.log').write_text(check.stdout+check.stderr)
        state=json.loads(check.stdout.strip().splitlines()[-1]);log.flush()
        refused='fly refused: inspect and repair the steering latch' in (folder/'sim.log').read_text()
        result={'passed':check.returncode==0 and state['fault'] and any(('mode='+mode) in state['state'] for mode in ('idle','drive','land')) and refused,'takeoff_refusal_logged':refused,**state}
        (folder/'result.json').write_text(json.dumps(result,indent=2));print(result,flush=True)
    finally:
        sim.send_signal(signal.SIGINT)
        try:sim.wait(timeout=10)
        except subprocess.TimeoutExpired:pass
        try:os.killpg(sim.pid,signal.SIGKILL)
        except ProcessLookupError:pass
raise SystemExit(0 if result['passed'] else 1)
