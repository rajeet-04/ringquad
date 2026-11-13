#!/usr/bin/env python3
"""Check left/right yaw tracking in an empty sim launched with steering:=front tread:=rubber."""
import json
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
    state = {'time': None, 'rate': 0.0, 'steering': {}}
    def odom(m):
        state.update(time=m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                     rate=m.twist.twist.angular.z)
    def joints(m):
        state['steering'].update((n, a) for n,a in zip(m.name,m.position) if n.startswith('steer_'))
    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(JointState, '/joint_states', joints, 10)
    mode=node.create_publisher(String, '/ringquad/mode', 10)
    cmd=node.create_publisher(Twist, '/cmd_vel', 10)
    def drive(seconds, speed, rate):
        started=state['time']
        deadline=time.monotonic()+max(15.,seconds*5)
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
            print(json.dumps({'requested_yaw_rate':requested,'measured_yaw_rate':measured,
                              'steering':state['steering']}),flush=True)
            if abs(measured-requested)>.08:
                raise AssertionError('yaw tracking error exceeds 0.08 rad/s')
            drive(1,0,0)
    finally:
        cmd.publish(Twist())
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
