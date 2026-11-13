import math
import numpy as np
from ringquad_control.footprint import corridor_margin, lateral_bounds
from ringquad_control import params as p


def test_upright_straight_width_and_buffer():
    folds = {'hinge_'+n: math.pi/2 for n in p.ROTOR_NAMES}
    assert np.allclose(lateral_bounds(0., folds, {}, .08), [-.175, .175])
    assert np.isclose(corridor_margin(0., 0., folds, {}, .08), .065)
    assert corridor_margin(.066, 0., folds, {}, .08) < 0


def test_stuck_steering_and_heading_reduce_clearance():
    folds = {'hinge_'+n: math.pi/2 for n in p.ROTOR_NAMES}
    stuck = {'steer_fl': math.radians(25), 'steer_fr': math.radians(25)}
    assert corridor_margin(.025, .1, folds, stuck, .08) < 0
    assert corridor_margin(.025, .1, folds, {}, .08) > 0


def test_reflection_of_measured_pose():
    folds = {'hinge_'+n: math.radians(88) for n in p.ROTOR_NAMES}
    a = {'steer_fl': .13, 'steer_fr': .11}
    b = {'steer_fl': -.11, 'steer_fr': -.13}
    assert np.isclose(corridor_margin(.01, .03, folds, a, .08),
                      corridor_margin(-.01, -.03, folds, b, .08))
