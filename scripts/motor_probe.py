#!/usr/bin/env python3
"""Hold one speed command on all four motors and report rotor speeds and height.

Stop the controller first. Usage: python3 motor_probe.py [command_rad_s] [seconds]
"""
import sys
import time

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

cmd = float(sys.argv[1]) if len(sys.argv) > 1 else 500.0
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

rclpy.init()
node = rclpy.create_node('motor_probe')
pub = node.create_publisher(Actuators, '/ringquad/motor_speed', 10)
state = {'spin': {}, 'z': None}
node.create_subscription(
    JointState, '/joint_states',
    lambda m: state['spin'].update((n, v) for n, v in zip(m.name, m.velocity) if n.startswith('spin')), 10)
node.create_subscription(Odometry, '/odom', lambda m: state.update(z=m.pose.pose.position.z), 10)

end = time.time() + seconds
next_print = time.time()
while time.time() < end:
    pub.publish(Actuators(velocity=[cmd] * 4))
    rclpy.spin_once(node, timeout_sec=0.01)
    if time.time() >= next_print:
        speeds = ' '.join(f'{n[5:]}={v:7.1f}' for n, v in sorted(state['spin'].items()))
        print(f'z={state["z"]}  {speeds}', flush=True)
        next_print += 0.5
pub.publish(Actuators(velocity=[0.0] * 4))
rclpy.shutdown()
