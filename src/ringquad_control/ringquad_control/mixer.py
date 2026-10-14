"""Thrust/torque allocation for flight and wheel speeds for driving."""
import numpy as np

from . import params as p


def allocation_matrix():
    """[T, tau_x, tau_y, tau_z] = A @ f, with f the four rotor thrusts.

    Thrust acts along body +z at each rotor. The motor model applies a reaction
    torque of -direction * K_MOMENT * f about the rotor axis.
    """
    x, y = p.ROTOR_XY[:, 0], p.ROTOR_XY[:, 1]
    return np.vstack([
        np.ones(4),
        y,
        -x,
        -p.ROTOR_DIR * p.K_MOMENT,
    ])


_A_INV = np.linalg.inv(allocation_matrix())


def wrench_to_thrusts(thrust, torque):
    f = _A_INV @ np.array([thrust, *torque])
    return np.clip(f, 0.0, p.MAX_THRUST)


def thrusts_to_commands(f):
    """Rotor speed commands (rad/s). Positive command = upward thrust for every rotor."""
    return np.sqrt(np.clip(f, 0.0, None) / p.K_THRUST)


def drive_commands(speed, yaw_rate):
    """Skid steering in ground mode: body speed (m/s) and yaw rate (rad/s) -> rotor commands.

    Left rings roll forward with positive joint velocity, right rings with negative
    (see the xacro header). Joint velocity = direction x command, so command = direction x joint velocity.
    """
    v_left = speed - yaw_rate * p.TRACK / 2
    v_right = speed + yaw_rate * p.TRACK / 2
    joint_vel = np.where(p.LEFT, v_left, -v_right) / p.WHEEL_RADIUS
    return np.clip(p.ROTOR_DIR * joint_vel, -p.MAX_ROTOR_SPEED, p.MAX_ROTOR_SPEED)
