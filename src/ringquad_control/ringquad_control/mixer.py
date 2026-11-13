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


def wrench_to_thrusts(thrust, torque, centre_of_mass=None):
    if centre_of_mass is not None:
        # Allocation uses rotor positions about the chassis origin; requested
        # torque is about the vehicle CoM, shifted by added steering hardware.
        torque = np.asarray(torque) + np.cross(centre_of_mass, [0.0, 0.0, thrust])
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


def steered_drive_commands(speed, yaw_rate):
    """Ackermann front steering; commands refer to rear axle centre velocity.

    Clamp curvature to the inner wheel's 35 degree steering limit. At rest,
    brake all motors; this geometry cannot pivot on the spot.
    """
    import math
    wheelbase = 2 * p.ARM_XY
    if abs(speed) < 1e-6:
        return np.zeros(4), np.zeros(2)
    curvature_max = math.tan(math.radians(35)) / (wheelbase + p.TRACK / 2 * math.tan(math.radians(35)))
    rate = float(np.clip(yaw_rate, -abs(speed)*curvature_max, abs(speed)*curvature_max))
    sides = np.array([speed-rate*p.TRACK/2, speed+rate*p.TRACK/2])
    angles = np.arctan(wheelbase*rate/sides)
    forward = np.copysign(np.hypot(sides, wheelbase*rate), speed)
    wheel_speeds = np.array([forward[0], forward[1], sides[0], sides[1]])
    joint_vel = np.where(p.LEFT, wheel_speeds, -wheel_speeds) / p.WHEEL_RADIUS
    return np.clip(p.ROTOR_DIR * joint_vel, -p.MAX_ROTOR_SPEED, p.MAX_ROTOR_SPEED), angles / math.sin(p.FOLD_GROUND)
