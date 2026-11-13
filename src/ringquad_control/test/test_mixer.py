import numpy as np

from ringquad_control import mixer
from ringquad_control import params as p


def test_allocation_round_trip():
    a = mixer.allocation_matrix()
    f = np.array([3.0, 4.0, 5.0, 4.5])
    w = a @ f
    assert np.allclose(mixer.wrench_to_thrusts(w[0], w[1:]), f)


def test_hover_splits_evenly():
    f = mixer.wrench_to_thrusts(p.MASS * p.GRAVITY, np.zeros(3))
    assert np.allclose(f, p.MASS * p.GRAVITY / 4)


def test_roll_torque_raises_left_rotors():
    f = mixer.wrench_to_thrusts(p.MASS * p.GRAVITY, np.array([0.2, 0.0, 0.0]))
    left = f[p.LEFT]
    right = f[~p.LEFT]
    assert left.min() > right.max()


def test_pitch_torque_raises_rear_rotors():
    # positive pitch (about +y) tips the nose down: rear rotors push harder
    f = mixer.wrench_to_thrusts(p.MASS * p.GRAVITY, np.array([0.0, 0.2, 0.0]))
    front, rear = f[[0, 1]], f[[2, 3]]
    assert rear.min() > front.max()


def test_yaw_torque_uses_cw_rotors():
    # cw rotors (direction -1) give +z reaction torque
    f = mixer.wrench_to_thrusts(p.MASS * p.GRAVITY, np.array([0.0, 0.0, 0.05]))
    cw = f[p.ROTOR_DIR < 0]
    ccw = f[p.ROTOR_DIR > 0]
    assert cw.min() > ccw.max()


def test_hover_speed_is_reasonable():
    w = mixer.thrusts_to_commands(np.full(4, p.MASS * p.GRAVITY / 4))
    assert 400 < w[0] < p.MAX_ROTOR_SPEED
    assert 4*p.MAX_THRUST/(p.MASS*p.GRAVITY) > 1.5  # revised heavier carrier retains thrust reserve


def test_drive_forward_joint_directions():
    cmd = mixer.drive_commands(1.0, 0.0)
    joint_vel = p.ROTOR_DIR * cmd
    assert np.all(joint_vel[p.LEFT] > 0)
    assert np.all(joint_vel[~p.LEFT] < 0)
    assert np.allclose(np.abs(joint_vel), 1.0 / p.WHEEL_RADIUS)


def test_drive_turn_left_slows_left_side():
    cmd = mixer.drive_commands(0.5, 1.0)
    joint_vel = np.abs(p.ROTOR_DIR * cmd)
    assert joint_vel[~p.LEFT].min() > joint_vel[p.LEFT].max()
