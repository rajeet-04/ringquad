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


def steered_drive_commands(speed, yaw_rate, max_angle_deg=25.0, track=p.TRACK, fold_angle=p.FOLD_GROUND):
    """Ackermann front steering; commands refer to rear axle centre velocity.

    Clamp curvature to the configured inner wheel's steering limit. At rest,
    brake all motors; this geometry cannot pivot on the spot.
    """
    import math
    wheelbase = 2 * p.ARM_XY
    if abs(speed) < 1e-6:
        return np.zeros(4), np.zeros(2)
    curvature_max = math.tan(math.radians(max_angle_deg)) / (wheelbase + track / 2 * math.tan(math.radians(max_angle_deg)))
    rate = float(np.clip(yaw_rate, -abs(speed)*curvature_max, abs(speed)*curvature_max))
    sides = np.array([speed-rate*track/2, speed+rate*track/2])
    angles = np.arctan(wheelbase*rate/sides)
    forward = np.copysign(np.hypot(sides, wheelbase*rate), speed)
    wheel_speeds = np.array([forward[0], forward[1], sides[0], sides[1]])
    joint_vel = np.where(p.LEFT, wheel_speeds, -wheel_speeds) / p.WHEEL_RADIUS
    return np.clip(p.ROTOR_DIR * joint_vel, -p.MAX_ROTOR_SPEED, p.MAX_ROTOR_SPEED), angles / math.sin(fold_angle)


def folded_contact_track(ring_offset=.1, profile='crowned', fold_angle=p.FOLD_GROUND):
    """Contact-point track including camber; upright wheels use their axle track."""
    import math
    s, c = math.sin(fold_angle), math.cos(fold_angle)
    centre = .08 + .12/math.sqrt(2)*c + ring_offset*s
    if profile == 'crowned':
        offset = .1275*c + (.0075**2-.015**2)*s*c / math.sqrt(.0075**2*s*s+.015**2*c*c)
        return 2*(centre+offset)
    if profile == 'cylinder':
        if abs(c) < 1e-8:
            return 2*centre  # upright cylinder has a whole contact line across its width
        return 2*(centre + p.WHEEL_RADIUS*c - .015*s)
    contact_offset = (p.WHEEL_RADIUS**2-.015**2)*s*c / math.sqrt(p.WHEEL_RADIUS**2*s*s+.015**2*c*c)
    return 2*(centre+contact_offset)
