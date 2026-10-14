#!/usr/bin/env python3
"""Ground steering check against a running sim.launch.py (empty world), robot on the ground.

1. Line following: drive 4 m along +x starting 0.2 m off the line; print cross-track and heading.
2. Arc turns: forward speed with a yaw-rate request; report the yaw rate achieved.
Run inside the container:  python3 ~/ringquad_ws/scripts/drive_test.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from ringquad_control.ground import follow_line


class DriveTest:
    def __init__(self):
        self.node = rclpy.create_node('drive_test')
        self.pos, self.yaw, self.t, self.state = None, 0.0, 0.0, ''
        self.node.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.node.create_subscription(String, '/ringquad/state', lambda m: setattr(self, 'state', m.data), 10)
        self.mode = self.node.create_publisher(String, '/ringquad/mode', 10)
        self.cmd = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pos = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def spin(self, seconds, speed=None, yaw_rate=0.0):
        end = time.time() + seconds
        while time.time() < end:
            self.mode.publish(String(data='drive'))
            if speed is not None:
                t = Twist()
                t.linear.x, t.angular.z = float(speed), float(yaw_rate)
                self.cmd.publish(t)
            rclpy.spin_once(self.node, timeout_sec=0.01)


def main():
    rclpy.init()
    d = DriveTest()
    d.spin(10.0, 0.0)  # wait for the rings to stop and the ~3.5 s fold
    x0, y0 = d.pos
    start, end = (x0, y0 - 0.2), (x0 + 4.0, y0 - 0.2)   # line 0.2 m to the right
    print('line following, start 0.2 m left of the line:')
    deadline, nxt = time.time() + 30, time.time()
    while time.time() < deadline:
        v, w, remaining, cross = follow_line(d.pos, d.yaw, start, end, 0.4)
        if remaining <= 0.02:
            break
        d.spin(0.05, v, w)
        if time.time() >= nxt:
            print(f'  along {4 - remaining:4.2f} m  cross-track {cross * 100:6.1f} cm  heading {math.degrees(d.yaw):6.1f} deg', flush=True)
            nxt += 1.0
    d.spin(1.0, 0.0)

    print('arc turns at 0.4 m/s:')
    for w in (0.3, 0.6, 1.0):
        d.spin(1.5, 0.4, w)
        y0, t0 = d.yaw, d.t
        d.spin(2.0, 0.4, w)
        rate = ((d.yaw - y0 + math.pi) % (2 * math.pi) - math.pi) / (d.t - t0)
        print(f'  requested {w:.1f} rad/s -> achieved {rate:.2f} rad/s', flush=True)
        d.spin(1.0, 0.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
