"""Demo mission for worlds/demo.sdf: fly over the wall, drive through the tunnel, land on the goal.

Drives the controller node through its public interface only (/ringquad/mode, /ringquad/goal,
/cmd_vel). Each step is a generator that yields until it is done; a step that times out aborts
the mission and lands (or stops, if driving). Positions are world/odom frame (the odometry
publisher reports world poses).

Ground steering is weak with realistic motors (see ground.py), so the robot lines up in the air,
lands, folds, and checks it is still lined up; if not, it hops (unfold, take off, re-align, land)
and tries again. In the tunnel it drives straight and stops if it drifts too far.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .ground import LineFollowGains, follow_line, wrap
from . import params as p

HINGES = ('hinge_fl', 'hinge_fr', 'hinge_rl', 'hinge_rr')
FOLD_GROUND = math.radians(80)
TICK = 0.05               # s
CRUISE_ALT = 2.2          # m, wall is 1.5 m
FLY_SPEED = 1.0           # m/s, carrot speed
DRIVE_SPEED = 0.4         # m/s

WALL_X = 2.5
TUNNEL_PAD = (3.8, 0.0)
TUNNEL_LINE = ((3.8, 0.0), (9.4, 0.0))   # tunnel spans x 5.5 .. 8.5; 1.4 m approach runway
GOAL_PAD = (10.5, 1.0)

# Evaluate steering halfway toward the rear axle (-0.08 m) to damp CoM lever-arm swing in tread:=mixed
MISSION_LINE_GAINS = LineFollowGains(axle_offset=-0.08)

ALIGN_HEADING_TOL = math.radians(1)
ALIGN_CROSS_TOL = 0.05                  # m
DRIVE_CROSS_ABORT = 0.05                # m, from TUNNEL_STRICT_FROM onward (tunnel: ~8 cm each side)
APPROACH_CROSS_ABORT = 0.15             # m, on the open approach before that
TUNNEL_STRICT_FROM = 5.2                # m (x): 0.3 m before the tunnel entrance
ALIGN_ATTEMPTS = 3
GROUND_ALIGN_TIMEOUT = 60.0  # ground turning is slow; cheaper than another flight hop


class MissionAbort(Exception):
    pass


class DemoMission(Node):
    def __init__(self):
        super().__init__('ringquad_mission')
        self.active_steering = self.declare_parameter('steering', 'fixed').value == 'front'
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
        self.started = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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
        end = self.now() + timeout
        while self.mode_now() != mode:
            if self.now() > end:
                raise MissionAbort(f'controller did not enter mode {mode} (state: {self.state})')
            self.mode_pub.publish(String(data=mode))
            yield

    def wait_until(self, cond, timeout, what):
        end = self.now() + timeout
        while not cond():
            if self.now() > end:
                raise MissionAbort(f'timed out: {what}')
            yield

    def fly_to(self, target, timeout=20.0, tol=0.15, yaw=0.0):
        """Move a carrot from the current position to target at FLY_SPEED; wait to arrive."""
        target = np.asarray(target, float)
        carrot = self.pos.copy() if self.goal is None else self.goal.copy()
        end = self.now() + timeout * (1.5 if getattr(self, 'active_steering', False) else 1.0)
        while True:
            step = target - carrot
            dist = np.linalg.norm(step)
            carrot = target.copy() if dist < FLY_SPEED * TICK else carrot + step / dist * FLY_SPEED * TICK
            self.goal = carrot
            self.send_goal(carrot, yaw)
            if (dist < 1e-6 and np.linalg.norm(self.pos - target) < tol
                    and np.linalg.norm(self.vel) < 0.2
                    and abs(wrap(self.yaw - yaw)) < math.radians(1.0)):
                return
            if self.now() > end:
                raise MissionAbort(f'timed out flying to {target.round(2).tolist()}; position {self.pos.round(2).tolist()}, speed {np.linalg.norm(self.vel):.2f} m/s, yaw error {math.degrees(wrap(self.yaw-yaw)):.1f} deg')
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
            12.0, 'folding')

    def line_error(self, start, end):
        """(heading error rad, cross-track m) of the robot relative to the line."""
        _, _, _, cross = follow_line(self.pos, self.yaw, start, end, DRIVE_SPEED, gains=MISSION_LINE_GAINS)
        line_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        return wrap(self.yaw - line_yaw), cross

    def land_folded_on_line(self, start, end):
        """Land at the line start facing along it, fold, and check alignment; hop and retry if off."""
        line_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        landing_yaw = line_yaw
        for attempt in range(1, ALIGN_ATTEMPTS + 1):
            yield from self.fly_to((start[0], start[1], 0.8), yaw=landing_yaw)
            yield from self.land()
            yield from self.fold_into_wheels()
            yield from self.pause(0.5)
            deadline = self.now() + GROUND_ALIGN_TIMEOUT
            while abs(wrap(line_yaw - self.yaw)) > ALIGN_HEADING_TOL:
                if self.now() > deadline:
                    self.send_cmd(0.0, 0.0)
                    self.get_logger().warn('ground alignment stalled; retrying in the air')
                    break
                self.send_cmd((0.08 if int((deadline-self.now())/3) % 2 == 0 else -0.08) if getattr(self, 'active_steering', False) else 0.0,
                              float(np.clip(3.0 * wrap(line_yaw - self.yaw), -0.8, 0.8)))
                yield
            self.send_cmd(0.0, 0.0)
            yield from self.pause(0.5)
            heading, cross = self.line_error(start, end)
            self.get_logger().info(f'folded on the line: heading {math.degrees(heading):+.2f} deg, '
                                   f'cross-track {cross * 100:+.1f} cm (attempt {attempt})')
            if abs(heading) <= ALIGN_HEADING_TOL and abs(cross) <= ALIGN_CROSS_TOL:
                return
            if attempt == ALIGN_ATTEMPTS:
                raise MissionAbort('could not line up with the tunnel')
            landing_yaw = wrap(landing_yaw - heading)
            self.get_logger().info('not lined up: compensate fold heading, unfold, hop and land again')
            yield from self.set_mode('idle')
            yield from self.pause(6.0)
            yield from self.take_off()

    def drive_line(self, start, end, timeout=45.0):
        deadline = self.now() + timeout
        worst = 0.0
        next_log = self.now()
        while True:
            v, w, remaining, cross = follow_line(
                self.pos, self.yaw, start, end, DRIVE_SPEED, gains=MISSION_LINE_GAINS)
            worst = max(worst, abs(cross))
            if self.now() >= next_log:
                heading, _ = self.line_error(start, end)
                self.get_logger().info(f'  x {self.pos[0]:5.2f} m  cross-track {cross * 100:+5.1f} cm  '
                                       f'heading {math.degrees(heading):+5.1f} deg  yaw-rate cmd {w:+.2f}')
                next_log += 0.5
            limit = DRIVE_CROSS_ABORT if self.pos[0] >= TUNNEL_STRICT_FROM else APPROACH_CROSS_ABORT
            if abs(cross) > limit:
                self.send_cmd(0.0, 0.0)
                raise MissionAbort(f'drifted {cross * 100:.1f} cm off the line, stopped')
            if remaining <= 0.02:
                self.send_cmd(0.0, 0.0)
                self.get_logger().info(f'drive done, worst cross-track {worst * 100:.1f} cm')
                return
            if self.now() > deadline:
                self.send_cmd(0.0, 0.0)
                raise MissionAbort('timed out driving')
            if getattr(self, 'active_steering', False) and self.pos[0] >= TUNNEL_STRICT_FROM:
                # Keep the swept front-ring envelope small in the narrow corridor.
                tangent = math.tan(math.radians(10))
                limit = abs(v) * tangent / (2*p.ARM_XY + p.TRACK/2*tangent)
                w = float(np.clip(w, -limit, limit))
            self.send_cmd(v, w)
            yield

    def pause(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            yield

    # ---------- the mission ----------

    def run(self):
        yield from self.wait_until(
            lambda: self.pos is not None and self.state and self.now() > 0.0,
            15.0, 'controller and odometry')
        self.started = self.now()
        log = self.get_logger().info

        log('1/6 take off')
        yield from self.take_off()

        log('2/6 climb and fly over the wall')
        yield from self.fly_to((self.pos[0], self.pos[1], CRUISE_ALT))
        yield from self.fly_to((TUNNEL_PAD[0], TUNNEL_PAD[1], CRUISE_ALT))

        log('3/6 land at the tunnel pad, lined up, and fold into wheels')
        yield from self.land_folded_on_line(*TUNNEL_LINE)

        log('4/6 drive through the tunnel')
        yield from self.drive_line(*TUNNEL_LINE)
        yield from self.pause(1.0)

        log('5/6 unfold and take off')
        yield from self.set_mode('idle')
        yield from self.pause(6.0)
        yield from self.take_off()

        log('6/6 fly to the goal pad and land')
        yield from self.fly_to((GOAL_PAD[0], GOAL_PAD[1], 1.0))
        yield from self.land()
        err = np.linalg.norm(self.pos[:2] - np.array(GOAL_PAD))
        log(f'mission complete in {self.now() - self.started:.0f} s, {err * 100:.0f} cm from the goal pad centre')

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
