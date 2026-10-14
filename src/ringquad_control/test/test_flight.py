"""Closed-loop check of controller + mixer on a rigid body with motor lag and ring inertia.

Matches what Gazebo does: each rotor speed follows its command with a first-order lag,
thrust = k w^2, and the body feels both the drag torque and the reaction torque of
spinning the heavy ring up or down.
"""
import math

import numpy as np

from ringquad_control import mixer
from ringquad_control import params as p
from ringquad_control.flight import E3, FlightController, yaw_of


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def simulate(pos_sp, yaw_sp, seconds, pos0=(0, 0, 0), ctl_dt=0.004, gains=None):
    dt = 0.001
    ctl = FlightController(gains)
    pos, vel = np.array(pos0, float), np.zeros(3)
    rot, rate = np.eye(3), np.zeros(3)
    joint_vel = np.zeros(4)            # signed rotor speeds about body z (dir x speed)
    cmd = np.zeros(4)
    x, y = p.ROTOR_XY[:, 0], p.ROTOR_XY[:, 1]
    steps_per_ctl = round(ctl_dt / dt)
    max_tilt = 0.0
    for i in range(int(seconds / dt)):
        if i % steps_per_ctl == 0:
            thrust, torque = ctl.update(pos, vel, rot, rate, np.array(pos_sp, float), yaw_sp, ctl_dt)
            cmd = mixer.thrusts_to_commands(mixer.wrench_to_thrusts(thrust, torque))
        target = p.ROTOR_DIR * cmd
        tau = np.where(np.abs(target) > np.abs(joint_vel), p.MOTOR_TAU_UP, p.MOTOR_TAU_DOWN)
        joint_acc = (target - joint_vel) / tau
        joint_vel += joint_acc * dt

        speed = np.abs(joint_vel)
        f = p.K_THRUST * speed ** 2
        torque_body = np.array([
            y @ f,
            -x @ f,
            -(p.ROTOR_DIR * p.K_MOMENT) @ f - p.RING_IZZ * joint_acc.sum(),  # drag + spin-up reaction
        ])
        acc = rot @ E3 * f.sum() / p.MASS - p.GRAVITY * E3
        ang_acc = np.linalg.solve(p.INERTIA, torque_body - np.cross(rate, p.INERTIA @ rate))
        vel += acc * dt
        pos += vel * dt
        rate += ang_acc * dt
        rot = rot @ (np.eye(3) + skew(rate * dt))
        u, _, vt = np.linalg.svd(rot)  # keep rot orthonormal
        rot = u @ vt
        max_tilt = max(max_tilt, math.degrees(math.acos(min(1.0, rot[2, 2]))))
    return pos, vel, rot, rate, max_tilt


def test_takeoff_to_one_metre():
    pos, vel, _, rate, max_tilt = simulate([0, 0, 1], 0.0, 6.0)
    assert np.linalg.norm(pos - [0, 0, 1]) < 0.05
    assert np.linalg.norm(vel) < 0.05
    assert np.linalg.norm(rate) < 0.05
    assert max_tilt < 2.0


def test_step_sideways():
    pos, _, rot, rate, max_tilt = simulate([1, 0.5, 1], 0.0, 8.0, pos0=(0, 0, 1))
    assert np.linalg.norm(pos - [1, 0.5, 1]) < 0.05
    assert rot[2, 2] > math.cos(math.radians(2))  # level again
    assert max_tilt < 32.0


def test_yaw_ninety_degrees():
    pos, _, rot, rate, _ = simulate([0, 0, 1], math.radians(90), 10.0, pos0=(0, 0, 1))
    assert abs(yaw_of(rot) - math.radians(90)) < math.radians(3)
    assert np.linalg.norm(pos - [0, 0, 1]) < 0.05
    assert abs(rate[2]) < 0.05
