"""Geometric position and attitude controller (after Lee, Leok, McClamroch 2010).

Position PID -> desired force vector -> desired attitude and collective thrust.
Attitude error on SO(3) -> body torque. All quantities in SI units.
World frame: z up. Body frame: x forward, y left, z up.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import params as p

E3 = np.array([0.0, 0.0, 1.0])


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(rot):
    return math.atan2(rot[1, 0], rot[0, 0])


@dataclass
class Gains:
    kp: np.ndarray = field(default_factory=lambda: np.array([4.0, 4.0, 6.0]))
    kd: np.ndarray = field(default_factory=lambda: np.array([3.5, 3.5, 4.5]))
    ki: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 1.5]))
    i_max: float = 2.0                 # m/s^2, integrator clamp
    i_zone: float = 0.3                # m, integrate only within this error
    # Yaw gains are ~50x lower than roll/pitch: a yaw torque command changes rotor speeds, and
    # spinning the heavy rings up/down kicks the body far harder than the drag torque the
    # mixer plans for (see params.RING_IZZ). Higher yaw gains chatter and saturate the motors.
    k_att: np.ndarray = field(default_factory=lambda: np.array([150.0, 150.0, 2.0]))
    k_rate: np.ndarray = field(default_factory=lambda: np.array([20.0, 20.0, 0.4]))
    max_tilt: float = math.radians(30)
    max_accel_z: float = 5.0


class FlightController:
    def __init__(self, gains=None):
        self.g = gains or Gains()
        self.integral = np.zeros(3)

    def reset(self):
        self.integral[:] = 0.0

    def update(self, pos, vel, rot, rate, pos_sp, yaw_sp, dt, vel_sp=None):
        """Returns (collective thrust N, body torque Nm)."""
        g = self.g
        vel_sp = np.zeros(3) if vel_sp is None else vel_sp

        e_pos = pos_sp - pos
        # integrate only near the setpoint, so large steps do not wind up and overshoot
        near = np.abs(e_pos) < g.i_zone
        self.integral = np.clip(self.integral + near * g.ki * e_pos * dt, -g.i_max, g.i_max)
        acc = g.kp * e_pos + g.kd * (vel_sp - vel) + self.integral

        # limit tilt and vertical acceleration
        acc[2] = np.clip(acc[2], -g.max_accel_z, g.max_accel_z)
        a_xy_max = math.tan(g.max_tilt) * (p.GRAVITY + acc[2])
        a_xy = np.linalg.norm(acc[:2])
        if a_xy > a_xy_max:
            acc[:2] *= a_xy_max / a_xy

        force = p.MASS * (acc + p.GRAVITY * E3)
        thrust = float(force @ (rot @ E3))

        b3 = force / np.linalg.norm(force)
        b1c = np.array([math.cos(yaw_sp), math.sin(yaw_sp), 0.0])
        b2 = np.cross(b3, b1c)
        b2 /= np.linalg.norm(b2)
        rot_sp = np.column_stack([np.cross(b2, b3), b2, b3])

        e_rot = 0.5 * vee(rot_sp.T @ rot - rot.T @ rot_sp)
        ang_acc = -g.k_att * e_rot - g.k_rate * rate
        torque = p.INERTIA @ ang_acc + np.cross(rate, p.INERTIA @ rate)

        return max(thrust, 0.0), torque
