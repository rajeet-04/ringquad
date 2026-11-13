"""Rigid-body model of the vehicle in flight, with realistic motors. For tests and design sweeps.

Each motor is a speed controller (like FOC firmware) whose torque is limited:
    torque = clip(I_rot * (w_cmd - w) / tau + drag(w), -T_max, T_max)
    I_rot * dw/dt = torque - drag(w),      drag(w) = (PROP_DRAG + rim_drag) * w * |w|
The body feels thrust k w^2 at each rotor, the reaction of every motor torque about z,
and gyroscopic precession from the spinning rings.
"""
from dataclasses import dataclass, field

import numpy as np

from . import params as p
from .flight import E3


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


@dataclass
class Rotor:
    izz: float = p.MOTOR_ROTOR_IZZ + p.RING_IZZ
    max_torque: float = p.MOTOR_MAX_TORQUE
    rim_drag: float = p.RIM_DRAG
    tau: float = p.MOTOR_TAU_UP          # speed-loop time constant when not torque limited

    @classmethod
    def ideal(cls):
        """What the first Gazebo setup did: practically unlimited torque, no rim drag."""
        return cls(max_torque=50.0, rim_drag=0.0)

    @classmethod
    def ring(cls, mass, **kw):
        return cls(izz=p.MOTOR_ROTOR_IZZ + p.ring_izz(mass), **kw)

    @classmethod
    def plain_prop(cls, **kw):
        return cls(izz=p.MOTOR_ROTOR_IZZ + p.PLAIN_PROP_IZZ, rim_drag=0.0, **kw)


@dataclass
class Plant:
    rotor: Rotor = field(default_factory=Rotor)
    mass: float = p.MASS
    inertia: np.ndarray = field(default_factory=lambda: p.INERTIA.copy())
    dt: float = 0.001

    def __post_init__(self):
        self.pos, self.vel = np.zeros(3), np.zeros(3)
        self.rot, self.rate = np.eye(3), np.zeros(3)
        hover = np.sqrt(self.mass * p.GRAVITY / 4 / p.K_THRUST)
        self.joint_vel = p.ROTOR_DIR * hover   # signed rotor speeds about body z
        self.motor_torque = np.zeros(4)
        self.ext_force = np.zeros(3)           # world-frame disturbance (e.g. a gust)

    def drag(self, w):
        return (p.PROP_DRAG + self.rotor.rim_drag) * w * np.abs(w)

    def step(self, commands):
        """commands: rotor speed commands as sent to the motors (positive = up)."""
        r = self.rotor
        target = p.ROTOR_DIR * np.asarray(commands)
        drag = self.drag(self.joint_vel)
        torque = np.clip(r.izz * (target - self.joint_vel) / r.tau + drag, -r.max_torque, r.max_torque)
        self.joint_vel += (torque - drag) / r.izz * self.dt
        self.motor_torque = torque

        f = p.K_THRUST * self.joint_vel ** 2
        x, y = p.ROTOR_XY[:, 0], p.ROTOR_XY[:, 1]
        rotor_h = np.array([0.0, 0.0, r.izz * float(self.joint_vel.sum())])
        gyro_torque = -np.cross(self.rate, rotor_h)
        body_torque = np.array([y @ f, -x @ f, -torque.sum()]) + gyro_torque

        acc = self.rot @ E3 * f.sum() / self.mass - p.GRAVITY * E3 + self.ext_force / self.mass
        ang_acc = np.linalg.solve(self.inertia, body_torque - np.cross(self.rate, self.inertia @ self.rate))
        self.vel += acc * self.dt
        self.pos += self.vel * self.dt
        self.rate += ang_acc * self.dt
        rot = self.rot @ (np.eye(3) + skew(self.rate * self.dt))
        u, _, vt = np.linalg.svd(rot)
        self.rot = u @ vt

    def tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(self.rot[2, 2], -1, 1))))

    def power(self):
        """Mechanical shaft power of all four motors (W)."""
        return float(np.sum(np.abs(self.motor_torque * self.joint_vel)))
