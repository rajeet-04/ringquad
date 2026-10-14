#!/usr/bin/env python3
"""Smoke test against a running gazebo.launch.py: fold, drive, unfold.

Run inside the container:  python3 ~/ringquad_ws/scripts/smoke_test.py
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

FOLD = math.radians(80)
HINGES = ['hinge_fl', 'hinge_fr', 'hinge_rl', 'hinge_rr']
LEFT, RIGHT = ['fl', 'rl'], ['fr', 'rr']


class SmokeTest(Node):
    def __init__(self):
        super().__init__('ringquad_smoke_test')
        self.joints, self.odom = {}, None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.fold_pub = self.create_publisher(Float64, '/ringquad/fold', 10)
        self.ring_pubs = {c: self.create_publisher(Float64, f'/ringquad/ring_{c}', 10)
                          for c in LEFT + RIGHT}
        self.fold_target, self.speed = None, 0.0

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_odom(self, msg):
        self.odom = msg.pose.pose.position

    def wait(self, seconds):
        """Spin and resend setpoints at 10 Hz, so a dropped message does not matter."""
        end = time.time() + seconds
        while time.time() < end:
            self.send()
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self):
        if self.fold_target is not None:
            self.fold_pub.publish(Float64(data=self.fold_target))
        # forward is positive on left rings, negative on right rings (see the xacro header)
        for c in LEFT:
            self.ring_pubs[c].publish(Float64(data=self.speed))
        for c in RIGHT:
            self.ring_pubs[c].publish(Float64(data=-self.speed))

    def fold(self, angle):
        self.fold_target = angle

    def drive(self, speed):
        self.speed = speed

    def hinges(self):
        return [self.joints.get(h, float('nan')) for h in HINGES]


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    return ok


def main():
    rclpy.init()
    n = SmokeTest()
    n.wait(2.0)
    results = []

    n.fold(FOLD)
    n.wait(6.0)
    h = n.hinges()
    results.append(check('fold to 80 deg', all(abs(a - FOLD) < math.radians(2) for a in h),
                         ', '.join(f'{math.degrees(a):.1f}' for a in h)))

    start = n.odom
    n.drive(10.0)
    n.wait(4.0)
    n.drive(0.0)
    n.wait(1.0)
    end = n.odom
    if start is None or end is None:
        results.append(check('drive forward', False, 'no /odom received'))
    else:
        dx, dy = end.x - start.x, end.y - start.y
        results.append(check('drive forward', dx > 0.3 and abs(dy) < 0.25 * dx,
                             f'dx={dx:.2f} m, dy={dy:.2f} m'))

    n.fold(0.0)
    n.wait(6.0)
    h = n.hinges()
    results.append(check('unfold to 0 deg', all(abs(a) < math.radians(2) for a in h),
                         ', '.join(f'{math.degrees(a):.1f}' for a in h)))

    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
