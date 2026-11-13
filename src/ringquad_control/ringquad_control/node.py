"""RingQuad controller node: the only publisher of motor and fold commands.

Modes (std_msgs/String on /ringquad/mode):
  idle   motors off, arms unfolded
  fly    unfold, then arm and fly to /ringquad/goal (default: 1 m above the take-off spot)
  land   descend at LAND_SPEED, switch to idle on touchdown
  drive  only on the ground: fold to wheels, then follow /cmd_vel (skid steer,
         yaw rate closed on the IMU, see ground.py)

Control runs on each IMU message (250 Hz). Position and attitude come from /odom
(simulator ground truth for now); body rates from the IMU.
"""
import math

import numpy as np
import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String

from . import mixer
from . import params as p
from .flight import FlightController, quat_to_rot, yaw_of
from .ground import YawRateController

MODES = ('idle', 'fly', 'land', 'drive')
HINGES = ('hinge_fl', 'hinge_fr', 'hinge_rl', 'hinge_rr')
FOLD_TOLERANCE = math.radians(0.5)
TAKEOFF_HEIGHT = 1.0
LAND_SPEED = 0.5
TOUCHDOWN_HEIGHT = 0.06   # above the take-off height
ON_GROUND_HEIGHT = 0.15   # above the last ground height
CMD_VEL_TIMEOUT = 0.5
FOLD_SPIN_MAX = 5.0       # rad/s: rings must be (nearly) stopped before folding
FOLD_RATE = 0.4           # rad/s: ~3.5 s for the full 80 deg fold


class RingQuadController(Node):
    def __init__(self):
        super().__init__('ringquad_controller')
        self.separate_drive = self.declare_parameter('drivetrain', 'separate').value == 'separate'
        self.base_mass = p.MASS if self.separate_drive else p.LEGACY_MASS
        self.active_steering = self.declare_parameter('steering', 'front').value == 'front'
        self.steering_limit_deg = self.declare_parameter('steering_limit_deg', 25.0).value
        self.fold_ground = math.radians(self.declare_parameter('fold_ground_deg', 90.0).value)
        self.knuckle_mass = self.declare_parameter('knuckle_mass', p.STEERING_KNUCKLE_MASS).value
        self.contact_track = mixer.folded_contact_track(self.declare_parameter('ring_offset', .1).value, self.declare_parameter('tread_collision', 'crowned').value, self.fold_ground)
        nominal_mass = self.base_mass + (2*self.knuckle_mass if self.active_steering else 0.0)
        self.vehicle_mass = self.declare_parameter('vehicle_mass', nominal_mass).value
        nominal_com = 2*self.knuckle_mass*p.ARM_XY/self.vehicle_mass if self.active_steering else 0.
        self.centre_of_mass = np.array(self.declare_parameter('centre_of_mass', [nominal_com,0.,0.]).value)
        self.thrust_coefficients = np.array(self.declare_parameter('thrust_coefficients', [p.K_THRUST]*4).value)
        if not np.isfinite(self.vehicle_mass) or self.vehicle_mass <= 0 or self.centre_of_mass.shape != (3,) or not np.all(np.isfinite(self.centre_of_mass)):
            raise ValueError('invalid measured mass or centre of mass')
        mixer.thrusts_to_commands(np.zeros(4), self.thrust_coefficients)
        self.flight = FlightController(mass=self.vehicle_mass)
        self.steer_pubs = [self.create_publisher(Float64, '/ringquad/steer_' + name, 10) for name in ('fl', 'fr')]
        self.lock_fault = False
        self.create_subscription(Bool, '/ringquad/lock_fault', self.on_lock_fault, 10)
        self.yaw_rate = YawRateController()
        self.mode = 'idle'
        self.pos = self.vel = self.rot = None
        self.last_odom_t = None
        self.ground_z = None
        self.hinges = {}
        self.steer_angles = {}
        self.spin = {}
        self.fold_allowed = False  # latched once the rings have stopped in drive mode
        self.fold_done = False     # latched once the 80 deg fold completes in drive mode
        self.fold_cmd = None       # ramped fold command (rad)
        self.goal_pos, self.goal_yaw = None, 0.0
        self.cmd_vel, self.cmd_vel_t = (0.0, 0.0), -1.0
        self.last_imu_t = None

        self.wheel_pub = self.create_publisher(Actuators, '/ringquad/wheel_speed', 10)
        self.motor_pub = self.create_publisher(Actuators, '/ringquad/motor_speed', 10)
        self.fold_pub = self.create_publisher(Float64, '/ringquad/fold', 10)
        self.state_pub = self.create_publisher(String, '/ringquad/state', 10)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/ringquad/mode', self.on_mode, 10)
        self.create_subscription(PoseStamped, '/ringquad/goal', self.on_goal, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.1, self.publish_state)

    # ---------- inputs ----------

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = msg.pose.pose.orientation
        rot = quat_to_rot(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        body_vel = np.array([v.x, v.y, v.z])
        # OdometryPublisher reports twist in base_link frame; rotate to world frame for FlightController
        self.vel = rot @ body_vel
        self.pos, self.rot, self.last_odom_t = pos, rot, t
        if self.ground_z is None:
            self.ground_z = pos[2]

    def on_joints(self, msg):
        self.hinges.update((n, a) for n, a in zip(msg.name, msg.position) if n in HINGES)
        self.steer_angles.update((n, a) for n, a in zip(msg.name, msg.position) if n.startswith('steer_'))
        self.spin.update((n, v) for n, v in zip(msg.name, msg.velocity) if n.startswith('spin_'))

    def on_goal(self, msg):
        q = msg.pose.orientation
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.goal_yaw = yaw_of(quat_to_rot(q.x, q.y, q.z, q.w))

    def on_cmd_vel(self, msg):
        self.cmd_vel = (msg.linear.x, msg.angular.z)
        self.cmd_vel_t = self.now()

    def on_lock_fault(self, msg):
        if not msg.data or self.lock_fault:
            return
        self.lock_fault = True
        self.get_logger().error('steering latch failure: takeoff inhibited; landing if airborne')
        if self.mode == 'fly':
            self.on_mode(String(data='land'))

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode not in MODES:
            self.get_logger().warn(f'unknown mode {mode!r}, expected one of {MODES}')
            return
        if mode == 'fly' and self.lock_fault:
            self.get_logger().error('fly refused: inspect and repair the steering latch')
            return
        if mode == self.mode or self.pos is None:
            return
        if mode == 'drive' and not self.on_ground():
            self.get_logger().warn('drive refused: not on the ground, land first')
            return
        if mode == 'land' and self.mode != 'fly':
            return  # nothing to land
        if mode == 'fly' and self.mode != 'land':
            self.ground_z = self.pos[2]
            self.goal_pos = self.pos + np.array([0.0, 0.0, TAKEOFF_HEIGHT])
            self.goal_yaw = yaw_of(self.rot)
        if mode == 'land':
            self.goal_pos = self.pos.copy()
            self.goal_yaw = yaw_of(self.rot)
        # Keep learned hover trim when entering descent; discarding it causes
        # a lateral transient precisely where landing accuracy matters.
        if not (self.mode == 'fly' and mode == 'land'):
            self.flight.reset()
        self.yaw_rate.reset()
        self.fold_allowed = False
        self.fold_done = False
        self.get_logger().info(f'mode {self.mode} -> {mode}')
        self.mode = mode

    # ---------- helpers ----------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ground(self):
        return self.ground_z is not None and self.pos[2] < self.ground_z + ON_GROUND_HEIGHT

    def fold_reached(self, target):
        return len(self.hinges) == 4 and all(abs(a - target) < FOLD_TOLERANCE for a in self.hinges.values())

    def fold_to(self, target, dt):
        """Ramp the fold command toward target. A fast fold drags the ring edges across the
        floor against the motor brake and can twist the robot by tens of degrees."""
        if self.fold_cmd is None:
            self.fold_cmd = float(np.mean(list(self.hinges.values()))) if self.hinges else target
        step = FOLD_RATE * dt
        self.fold_cmd += float(np.clip(target - self.fold_cmd, -step, step))
        self.fold_pub.publish(Float64(data=self.fold_cmd))

    def send_motors(self, commands):
        self.motor_pub.publish(Actuators(velocity=[float(c) for c in commands]))

    def publish_state(self):
        if self.pos is None:
            return
        self.state_pub.publish(String(data=f'mode={self.mode} z={self.pos[2]:.2f}'))

    # ---------- control loop ----------

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.004 if self.last_imu_t is None else min(max(t - self.last_imu_t, 1e-4), 0.02)
        self.last_imu_t = t
        if self.pos is None:
            return
        rate = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Independent wheel drives brake in flight and throughout folding.
        if self.separate_drive and (self.mode != 'drive' or not self.fold_reached(self.fold_ground)):
            self.wheel_pub.publish(Actuators(velocity=[0.0]*4))

        if self.mode == 'drive':
            # Never fold spinning rings: tipping a spinning ring's axis makes a gyroscopic torque
            # that twists the whole robot (each 60 g ring holds ~0.4 Nms at landing speed).
            if not self.fold_allowed:
                self.fold_allowed = len(self.spin) == 4 and max(abs(v) for v in self.spin.values()) < FOLD_SPIN_MAX
            if not self.fold_allowed:
                self.fold_to(p.FOLD_FLIGHT, dt)
                self.send_motors(np.zeros(4))
                return
            self.fold_to(self.fold_ground, dt)
            if not self.fold_reached(self.fold_ground):
                self.send_motors(np.zeros(4))
                return
            if not self.fold_done:
                self.yaw_rate.reset()
                self.fold_done = True
            speed, yaw_rate = self.cmd_vel if self.now() - self.cmd_vel_t < CMD_VEL_TIMEOUT else (0.0, 0.0)
            if self.active_steering:
                motors, angles = mixer.steered_drive_commands(speed, yaw_rate, max_angle_deg=self.steering_limit_deg, track=self.contact_track, fold_angle=self.fold_ground)
                for pub, angle in zip(self.steer_pubs, angles):
                    pub.publish(Float64(data=float(angle)))
                if self.separate_drive:
                    self.send_motors(np.zeros(4))
                    self.wheel_pub.publish(Actuators(velocity=[float(c) for c in motors]))
                else:
                    self.send_motors(motors)
            else:
                yaw_cmd = self.yaw_rate.update(yaw_rate, rate[2], dt)
                motors = mixer.drive_commands(speed, yaw_cmd)
                if self.separate_drive:
                    self.send_motors(np.zeros(4))
                    self.wheel_pub.publish(Actuators(velocity=[float(c) for c in motors]))
                else:
                    self.send_motors(motors)
            self.ground_z = self.pos[2]
            return

        for pub in self.steer_pubs:
            pub.publish(Float64(data=0.0))
        if self.active_steering and not self.fold_reached(p.FOLD_FLIGHT) and (len(self.steer_angles) != 2 or any(abs(a) > math.radians(1) for a in self.steer_angles.values())):
            self.send_motors(np.zeros(4))
            return
        self.fold_to(p.FOLD_FLIGHT, dt)
        if self.mode == 'idle' or not self.fold_reached(p.FOLD_FLIGHT):
            self.send_motors(np.zeros(4))
            if self.mode == 'idle' and abs(self.vel[2]) < 0.05:
                self.ground_z = self.pos[2]
            return

        if self.mode == 'land':
            self.goal_pos[2] = max(self.goal_pos[2] - LAND_SPEED * dt, self.ground_z - 0.2)
            if self.pos[2] < self.ground_z + TOUCHDOWN_HEIGHT and abs(self.vel[2]) < 0.15:
                self.get_logger().info('touchdown, mode land -> idle')
                self.mode = 'idle'
                self.send_motors(np.zeros(4))
                return

        thrust, torque = self.flight.update(self.pos, self.vel, self.rot, rate,
                                            self.goal_pos, self.goal_yaw, dt)
        self.send_motors(mixer.thrusts_to_commands(mixer.wrench_to_thrusts(thrust, torque,
            self.centre_of_mass), self.thrust_coefficients))


def main():
    rclpy.init()
    node = RingQuadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
