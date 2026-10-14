#!/usr/bin/env python3
"""Switch to fly mode and print attitude, body rates and motor commands at 20 Hz.

Usage (sim.launch.py running, robot on the ground): python3 flight_log.py [seconds]
"""
import math
import sys
import time

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
rclpy.init()
node = rclpy.create_node('flight_log')
s = {'odom': None, 'imu': None, 'motors': None}
node.create_subscription(Odometry, '/odom', lambda m: s.update(odom=m), 10)
node.create_subscription(Imu, '/imu', lambda m: s.update(imu=m), 10)
node.create_subscription(Actuators, '/ringquad/motor_speed', lambda m: s.update(motors=m), 10)
mode = node.create_publisher(String, '/ringquad/mode', 10)


def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


end = time.time() + 2.0
while time.time() < end:  # discovery
    rclpy.spin_once(node, timeout_sec=0.05)
start = time.time()
nxt = start
while time.time() - start < seconds:
    mode.publish(String(data='fly'))
    rclpy.spin_once(node, timeout_sec=0.01)
    if time.time() >= nxt and s['odom'] and s['imu']:
        p = s['odom'].pose.pose
        r, pt, y = euler(p.orientation)
        w = s['imu'].angular_velocity
        m = s['motors'].velocity if s['motors'] else []
        print(f"t={time.time() - start:4.2f} z={p.position.z:5.2f} rpy=({r:6.1f},{pt:6.1f},{y:6.1f}) "
              f"rate=({w.x:6.2f},{w.y:6.2f},{w.z:6.2f}) motors={[round(v) for v in m]}", flush=True)
        nxt += 0.05
mode.publish(String(data='idle'))
rclpy.shutdown()
