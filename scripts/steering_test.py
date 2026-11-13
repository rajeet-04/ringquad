#!/usr/bin/env python3
"""Check left/right yaw tracking in an empty sim launched with steering:=front tread:=rubber."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('steering_test')
    state = {'time': None, 'rate': 0.0, 'steering': {}, 'prop_speed': {}, 'wheel_speed': {}}
    def odom(m):
        state.update(time=m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                     rate=m.twist.twist.angular.z)
    def joints(m):
        state['steering'].update((n, a) for n,a in zip(m.name,m.position) if n.startswith('steer_'))
        state['prop_speed'].update((n,v) for n,v in zip(m.name,m.velocity) if n.startswith('spin_'))
        state['wheel_speed'].update((n,v) for n,v in zip(m.name,m.velocity) if n.startswith('wheel_'))
    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(JointState, '/joint_states', joints, 10)
    mode=node.create_publisher(String, '/ringquad/mode', 10)
    cmd=node.create_publisher(Twist, '/cmd_vel', 10)
    def drive(seconds, speed, rate):
        started=state['time']
        deadline=time.monotonic()+max(60.,seconds*15)
        values=[]
        while started is None or state['time']-started<seconds:
            if time.monotonic()>deadline:
                raise RuntimeError('simulation did not advance')
            mode.publish(String(data='drive'))
            message=Twist();message.linear.x=float(speed);message.angular.z=float(rate)
            cmd.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if started is None:started=state['time']
            if state['time'] is not None:values.append(state['rate'])
        return sum(values)/len(values)
    try:
        drive(10,0,0)
        if len(state['steering'])!=2:
            raise RuntimeError('launch with steering:=front')
        for requested in (.2,-.2):
            drive(2,.4,requested)
            measured=drive(3,.4,requested)
            print(json.dumps({'requested_yaw_rate':requested,'measured_yaw_rate':measured if math.isfinite(measured) else None,
                              'steering':state['steering'], 'max_prop_speed_rad_s':max(abs(v) for v in state['prop_speed'].values()),
                              'mean_wheel_speed_rad_s':sum(abs(v) for v in state['wheel_speed'].values())/4}),flush=True)
            if len(state['wheel_speed'])!=4 or len(state['prop_speed'])!=4 or max(abs(v) for v in state['prop_speed'].values())>5:
                raise AssertionError('independent drivetrain missing or propellers spinning while driving')
            if not math.isfinite(measured) or abs(measured-requested)>.08:
                raise AssertionError('yaw tracking error exceeds 0.08 rad/s')
            drive(1,0,0)
    finally:
        cmd.publish(Twist())
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
