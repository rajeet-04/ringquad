"""Closed-loop checks of controller + mixer on the realistic plant (ringquad_control/plant.py):
torque-limited motors, 60 g rings, blade and rim drag."""
import math

import numpy as np

from ringquad_control import mixer
from ringquad_control.flight import FlightController, Gains, yaw_of
from ringquad_control.plant import Plant, Rotor


def simulate(pos_sp, yaw_sp, seconds, pos0=(0, 0, 0), ctl_dt=0.004, gains=None, rotor=None):
    plant = Plant(rotor=rotor or Rotor())
    plant.pos[:] = pos0
    ctl = FlightController(gains)
    steps = round(ctl_dt / plant.dt)
    max_tilt = 0.0
    cmd = np.zeros(4)
    for i in range(int(seconds / plant.dt)):
        if i % steps == 0:
            thrust, torque = ctl.update(plant.pos, plant.vel, plant.rot, plant.rate,
                                        np.array(pos_sp, float), yaw_sp, ctl_dt)
            cmd = mixer.thrusts_to_commands(mixer.wrench_to_thrusts(thrust, torque))
        plant.step(cmd)
        max_tilt = max(max_tilt, plant.tilt_deg())
        if max_tilt > 80:
            break
    return plant.pos, plant.vel, plant.rot, plant.rate, max_tilt


def test_climb_one_metre():
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


def test_large_diagonal_move():
    pos, _, rot, _, max_tilt = simulate([2, 2, 1], 0.0, 10.0, pos0=(0, 0, 1))
    assert np.linalg.norm(pos - [2, 2, 1]) < 0.05
    assert max_tilt < 35.0


def test_yaw_ninety_degrees():
    pos, _, rot, rate, _ = simulate([0, 0, 1], math.radians(90), 10.0, pos0=(0, 0, 1))
    assert abs(yaw_of(rot) - math.radians(90)) < math.radians(3)
    assert np.linalg.norm(pos - [0, 0, 1]) < 0.05
    assert abs(rate[2]) < 0.05


def test_first_tuning_is_unstable_with_realistic_motors():
    """Documents why the roll/pitch gains were lowered: the first tuning diverges."""
    first = Gains()
    first.k_att = np.array([150.0, 150.0, 2.0])
    first.k_rate = np.array([20.0, 20.0, 0.4])
    *_, max_tilt = simulate([1, 0, 1], 0.0, 8.0, pos0=(0, 0, 1), gains=first)
    assert max_tilt > 80


def test_same_gains_fly_a_plain_prop():
    pos, *_ = simulate([1, 0, 1], 0.0, 8.0, pos0=(0, 0, 1), rotor=Rotor.plain_prop())
    assert np.linalg.norm(pos - [1, 0, 1]) < 0.05
