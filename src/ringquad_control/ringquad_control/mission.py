"""Demo mission for worlds/demo.sdf: fly over the wall, drive through the tunnel, land on the goal.

Drives the controller node through its public interface only (/ringquad/mode, /ringquad/goal,
/cmd_vel). Each step is a generator that yields until it is done; a step that times out
aborts the mission and lands (or stops, if driving).

Positions are world/odom frame (the odometry publisher reports world poses).
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .ground import follow_line

HINGES = ('hinge_fl', 'hinge_fr', 'hinge_rl', 'hinge_rr')
FOLD_GROUND = math.radians(80)
TICK = 0.05               # s
CRUISE_ALT = 2.2          # m, wall is 1.5 m
FLY_SPEED = 1.0           # m/s, carrot speed
DRIVE_SPEED = 0.4         # m/s

WALL_X = 2.5
TUNNEL_PAD = (4.5, 0.0)
TUNNEL_LINE = ((4.5, 0.0), (9.4, 0.0))   # tunnel spans x 5.5 .. 8.5
GOAL_PAD = (10.5, 1.0)


class MissionAbort(Exception):
    pass


class DemoMission(Node):
    def __init__(self):
        super().__init__('ringquad_mission')
        self.pos = self.yaw = None
        self.vel = np.zeros(3)
        self.state = ''
        self.hinges = {}
        self.goal = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/ringquad/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.mode_pub = self.create_publisher(String, '/ringquad/mode', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/ringquad/goal', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.plan = self.run()
        self.timer = self.create_timer(TICK, self.tick)
        self.started = time.time()

    # ---------- inputs ----------
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def on_joints(self, msg):
        self.hinges.update((n, a) for n, a in zip(msg.name, msg.position) if n in HINGES)

    # ---------- outputs ----------
    def send_goal(self, xyz, yaw=0.0):
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = (float(c) for c in xyz)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(g)

    def send_cmd(self, speed, yaw_rate):
        t = Twist()
        t.linear.x, t.angular.z = float(speed), float(yaw_rate)
        self.cmd_pub.publish(t)

    # ---------- step helpers (generators) ----------
    def mode_now(self):
        return self.state.split()[0].removeprefix('mode=') if self.state else ''

    def set_mode(self, mode, timeout=3.0):
        end = time.time() + timeout
        while self.mode_now() != mode:
            if time.time() > end:
                raise MissionAbort(f'controller did not enter mode {mode} (state: {self.state})')
            self.mode_pub.publish(String(data=mode))
            yield

    def wait_until(self, cond, timeout, what):
        end = time.time() + timeout
        while not cond():
            if time.time() > end:
                raise MissionAbort(f'timed out: {what}')
            yield

    def fly_to(self, target, timeout=20.0, tol=0.15):
        """Move a carrot from the current position to target at FLY_SPEED; wait to arrive."""
        target = np.asarray(target, float)
        carrot = self.pos.copy() if self.goal is None else self.goal.copy()
        end = time.time() + timeout
        while True:
            step = target - carrot
            dist = np.linalg.norm(step)
            carrot = target.copy() if dist < FLY_SPEED * TICK else carrot + step / dist * FLY_SPEED * TICK
            self.goal = carrot
            self.send_goal(carrot)
            if dist < 1e-6 and np.linalg.norm(self.pos - target) < tol and np.linalg.norm(self.vel) < 0.2:
                return
            if time.time() > end:
                raise MissionAbort(f'timed out flying to {target.round(2).tolist()}')
            yield

    def take_off(self):
        yield from self.set_mode('fly')
        self.goal = None
        start = self.pos.copy()
        yield from self.wait_until(lambda: self.pos[2] > start[2] + 0.8, 10.0, 'take-off')

    def land(self):
        self.goal = None
        yield from self.set_mode('land')
        yield from self.wait_until(lambda: self.mode_now() == 'idle', 15.0, 'landing')

    def fold_into_wheels(self):
        yield from self.set_mode('drive')
        yield from self.wait_until(
            lambda: len(self.hinges) == 4 and all(abs(a - FOLD_GROUND) < 0.05 for a in self.hinges.values()),
            6.0, 'folding')

    def drive_line(self, start, end, timeout=40.0):
        deadline = time.time() + timeout
        worst = 0.0
        while True:
            v, w, remaining, cross = follow_line(self.pos, self.yaw, start, end, DRIVE_SPEED)
            worst = max(worst, abs(cross))
            if remaining <= 0.02:
                self.send_cmd(0.0, 0.0)
                self.get_logger().info(f'drive done, worst cross-track {worst * 100:.1f} cm')
                return
            if time.time() > deadline:
                self.send_cmd(0.0, 0.0)
                raise MissionAbort('timed out driving')
            self.send_cmd(v, w)
            yield

    def pause(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            yield

    # ---------- the mission ----------
    def run(self):
        yield from self.wait_until(lambda: self.pos is not None and self.state, 15.0, 'controller and odometry')
        log = self.get_logger().info

        log('1/6 take off')
        yield from self.take_off()
        log('2/6 climb and fly over the wall')
        yield from self.fly_to((self.pos[0], self.pos[1], CRUISE_ALT))
        yield from self.fly_to((TUNNEL_PAD[0], TUNNEL_PAD[1], CRUISE_ALT))
        yield from self.fly_to((TUNNEL_PAD[0], TUNNEL_PAD[1], 0.8))
        log('3/6 land at the tunnel pad')
        yield from self.land()
        log('4/6 fold into wheels and drive through the tunnel')
        yield from self.fold_into_wheels()
        yield from self.drive_line(*TUNNEL_LINE)
        yield from self.pause(1.0)
        log('5/6 unfold and take off')
        yield from self.set_mode('idle')
        yield from self.pause(3.0)
        yield from self.take_off()
        log('6/6 fly to the goal pad and land')
        yield from self.fly_to((GOAL_PAD[0], GOAL_PAD[1], 1.0))
        yield from self.land()
        err = np.linalg.norm(self.pos[:2] - np.array(GOAL_PAD))
        log(f'mission complete in {time.time() - self.started:.0f} s, {err * 100:.0f} cm from the goal pad centre')

    def tick(self):
        try:
            next(self.plan)
        except StopIteration:
            self.timer.cancel()
            self.done = True
        except MissionAbort as e:
            self.get_logger().error(f'mission aborted: {e}')
            self.send_cmd(0.0, 0.0)
            self.mode_pub.publish(String(data='land'))
            self.timer.cancel()
            self.done = True


def main():
    rclpy.init()
    node = DemoMission()
    node.done = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
