import math
import numpy as np
from ringquad_control import mixer, params as p


def wheel_speeds(commands):
    joint = commands * p.ROTOR_DIR
    return np.where(p.LEFT, joint, -joint) * p.WHEEL_RADIUS


def test_straight_and_stop():
    commands, angles = mixer.steered_drive_commands(.4, 0)
    assert np.allclose(wheel_speeds(commands), .4)
    assert np.allclose(angles, 0)
    commands, angles = mixer.steered_drive_commands(0, 1)
    assert np.allclose(commands, 0) and np.allclose(angles, 0)


def test_left_turn_wheels_share_an_instantaneous_centre():
    commands, joint_angles = mixer.steered_drive_commands(.4, .3)
    angles = joint_angles * math.sin(p.FOLD_GROUND)
    wheelbase = 2*p.ARM_XY
    radius = .4/.3
    assert np.allclose(np.tan(angles), wheelbase/np.array([radius-p.TRACK/2, radius+p.TRACK/2]))
    speeds = wheel_speeds(commands)
    assert speeds[1] > speeds[0] and speeds[3] > speeds[2]
    assert np.allclose(speeds[:2]*np.cos(angles), speeds[2:])


def test_mirrored_and_reverse_turns():
    left, angles = mixer.steered_drive_commands(.4, .3)
    right, other_angles = mixer.steered_drive_commands(.4, -.3)
    assert np.allclose(wheel_speeds(left)[[1,0,3,2]], wheel_speeds(right))
    assert np.allclose(angles[::-1], -other_angles)
    reverse, reversed_angles = mixer.steered_drive_commands(-.4, -.3)
    assert np.allclose(reverse, -left)
    assert np.allclose(angles, reversed_angles)


def test_inner_wheel_angle_and_joint_limits_hold_for_extreme_requests():
    for speed in (-.4, -.01, .01, .4):
        for rate in (-100, 100):
            commands, angles = mixer.steered_drive_commands(speed, rate)
            assert np.all(np.isfinite(commands))
            assert max(abs(angles)) < .63
            assert max(abs(angles))*math.sin(p.FOLD_GROUND) <= math.radians(35)+1e-12


def test_added_front_hardware_hover_is_balanced_about_actual_com():
    mass = p.MASS + .04
    com = np.array([.04*p.ARM_XY/mass, 0., 0.])
    thrust = mass*p.GRAVITY
    forces = mixer.wrench_to_thrusts(thrust, np.zeros(3), centre_of_mass=com)
    positions = np.column_stack([p.ROTOR_XY, np.zeros(4)]) - com
    moments = np.cross(positions, np.column_stack([np.zeros(4), np.zeros(4), forces]))
    assert np.allclose(moments.sum(axis=0), 0, atol=1e-12)
    assert np.isclose(forces.sum(), thrust)


def test_motor_speed_limits_and_tunnel_envelope():
    commands, _ = mixer.steered_drive_commands(1000., 1000.)
    assert max(abs(commands)) <= p.MAX_ROTOR_SPEED
    angle = math.radians(10)
    # Conservative projected width including the 10-degree wheel camber.
    width = p.TRACK + 2*(p.WHEEL_RADIUS*math.sin(angle) + p.WHEEL_RADIUS*math.sin(math.radians(10)) + .015)
    assert width < .5
