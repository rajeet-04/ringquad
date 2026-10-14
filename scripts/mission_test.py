#!/usr/bin/env python3
"""End-to-end check against a running sim.launch.py (Gazebo + controller).

Take off, fly to a point while turning 90 deg, land, fold into wheels, drive, turn, unfold.
Run inside the container:  python3 ~/ringquad_ws/scripts/mission_test.py
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

HINGES = ('hinge_fl', 'hinge_fr', 'hinge_rl', 'hinge_rr')


def yaw_from(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class MissionTest(Node):
    def __init__(self):
        super().__init__('ringquad_mission_test')
        self.pose, self.state, self.hinges, self.sim_t = None, '', {}, 0.0
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/ringquad/state', self.on_state, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.mode_pub = self.create_publisher(String, '/ringquad/mode', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/ringquad/goal', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode, self.goal, self.cmd = None, None, None

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_state(self, msg):
        self.state = msg.data

    def on_joints(self, msg):
        self.hinges.update((n, a) for n, a in zip(msg.name, msg.position) if n in HINGES)

    def wait(self, seconds):
        """Spin and resend the current mode, goal and cmd_vel at 10 Hz."""
        end = time.time() + seconds
        while time.time() < end:
            if self.mode:
                self.mode_pub.publish(String(data=self.mode))
            if self.goal:
                self.goal_pub.publish(self.goal)
            if self.cmd:
                self.cmd_pub.publish(self.cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_goal(self, x, y, z, yaw):
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, z
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal = g

    def drive(self, speed, yaw_rate):
        t = Twist()
        t.linear.x, t.angular.z = speed, yaw_rate
        self.cmd = t

    def xyz(self):
        return self.pose.position.x, self.pose.position.y, self.pose.position.z


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)
    return ok


def main():
    rclpy.init()
    n = MissionTest()
    n.wait(3.0)
    if n.pose is None:
        print('FAIL  no /odom; is sim.launch.py running?')
        sys.exit(1)
    results = []
    x0, y0, z0 = n.xyz()
    yaw0 = yaw_from(n.pose.orientation)

    # 1. take off: controller climbs 1 m above the take-off spot
    n.mode = 'fly'
    n.wait(8.0)
    x, y, z = n.xyz()
    err = math.dist((x, y, z), (x0, y0, z0 + 1.0))
    results.append(check('take off to 1 m', err < 0.1, f'error {err * 100:.1f} cm'))

    # 2. fly to a point 1.5 m up, 1 m forward and 0.5 m left, turning 90 deg
    target = (x0 + 1.0, y0 + 0.5, z0 + 1.5)
    n.set_goal(*target, yaw0 + math.pi / 2)
    n.wait(8.0)
    err = math.dist(n.xyz(), target)
    yaw_err = abs(angle_diff(yaw_from(n.pose.orientation), yaw0 + math.pi / 2))
    results.append(check('fly to point + yaw 90', err < 0.1 and yaw_err < math.radians(5),
                         f'error {err * 100:.1f} cm, yaw error {math.degrees(yaw_err):.1f} deg'))

    # 3. land: descends and disarms itself
    n.goal = None
    n.mode = 'land'
    n.wait(8.0)
    n.mode = None
    _, _, z = n.xyz()
    landed = 'mode=idle' in n.state
    results.append(check('land and disarm', landed and z < z0 + 0.1,
                         f'{n.state}, height above start {(z - z0) * 100:.1f} cm'))

    # 4. drive: fold into wheels, then 0.5 m/s forward (speeds use sim time: Gazebo runs slower than real time)
    n.mode = 'drive'
    n.wait(4.0)
    folded = all(abs(a - math.radians(80)) < math.radians(3) for a in n.hinges.values())
    results.append(check('fold into wheels', folded and len(n.hinges) == 4,
                         ', '.join(f'{math.degrees(a):.1f}' for a in n.hinges.values())))
    heading = yaw_from(n.pose.orientation)
    n.drive(0.5, 0.0)
    n.wait(1.0)
    xs, ys, _ = n.xyz()
    ts = n.sim_t
    n.wait(3.0)
    x, y, _ = n.xyz()
    dt = n.sim_t - ts
    n.drive(0.0, 0.0)
    n.wait(1.0)
    along = ((x - xs) * math.cos(heading) + (y - ys) * math.sin(heading)) / dt
    across = (-(x - xs) * math.sin(heading) + (y - ys) * math.cos(heading)) / dt
    results.append(check('drive forward at 0.5 m/s', abs(along - 0.5) < 0.05 and abs(across) < 0.05,
                         f'{along:.2f} m/s forward, {across:.2f} m/s sideways'))

    # 5. turn on the spot at 1 rad/s: measure the rate over the last 2 s
    n.drive(0.0, 1.0)
    n.wait(1.0)
    yaw_start, ts = yaw_from(n.pose.orientation), n.sim_t
    n.wait(2.0)
    rate = angle_diff(yaw_from(n.pose.orientation), yaw_start) / (n.sim_t - ts)
    n.drive(0.0, 0.0)
    n.wait(1.0)
    results.append(check('skid-steer turn at 1 rad/s', abs(rate - 1.0) < 0.1, f'{rate:.2f} rad/s'))

    # 6. back to idle: unfold
    n.cmd = None
    n.mode = 'idle'
    n.wait(4.0)
    unfolded = all(abs(a) < math.radians(3) for a in n.hinges.values())
    results.append(check('unfold', unfolded, ', '.join(f'{math.degrees(a):.1f}' for a in n.hinges.values())))

    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
