import os,signal,subprocess,time,uuid,json,math
from pathlib import Path
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--mode',default='drive',choices=['drive','fly']);parser.add_argument('--output',default='hardware-blockers/results/fold-debug');parser.add_argument('--model-arg',nargs=2,action='append',default=[]);args=parser.parse_args()
folder=Path(args.output);folder.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,ROS_DOMAIN_ID=str(190+os.getpid()%30),GZ_PARTITION='fold-'+uuid.uuid4().hex,
         RINGQUAD_HARDWARE_LOG=str(folder.resolve()/'loads.csv'))
passed=False
log=(folder/'sim.log').open('w')
proc=subprocess.Popen(['ros2','launch','ringquad_control','sim.launch.py','gui:=false','rviz:=false']+[f'{k}:={v}' for k,v in args.model_arg],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 code='''import rclpy,time,json
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
rclpy.init();n=Node('fold_probe');s={'t':0.,'hinges':{},'velocities':{},'motor_commands':[],'z':0.}
def odom(m):s['t']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;s['z']=m.pose.pose.position.z
def joints(m):s['hinges'].update((a,b) for a,b in zip(m.name,m.position) if a.startswith('hinge_'));s['velocities'].update(zip(m.name,m.velocity))
n.create_subscription(Odometry,'/odom',odom,10);n.create_subscription(JointState,'/joint_states',joints,10)
n.create_subscription(Actuators,'/ringquad/motor_speed',lambda m:s.update(motor_commands=list(m.velocity)),10)
p=n.create_publisher(String,'/ringquad/mode',10);deadline=time.monotonic()+300
while s['t']<15 and time.monotonic()<deadline:
 p.publish(String(data='drive'));rclpy.spin_once(n,timeout_sec=.01)
print(json.dumps(s));n.destroy_node();rclpy.shutdown()
'''
 code=code.replace("data='drive'",f"data='{args.mode}'")
 result=subprocess.run(['python3','-c',code],env=env,capture_output=True,text=True,timeout=320)
 (folder/'state.json').write_text(result.stdout);print(result.stdout,result.stderr,flush=True)
 state=json.loads(result.stdout.strip().splitlines()[-1])
 passed=result.returncode==0 and len(state['hinges'])==4 and all(abs(v-math.pi/2)<math.radians(3) for v in state['hinges'].values())
 (folder/'result.json').write_text(json.dumps({'passed':passed,**state},indent=2))
finally:
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:pass
 try:os.killpg(proc.pid,signal.SIGKILL)
 except ProcessLookupError:pass

raise SystemExit(0 if passed else 1)
