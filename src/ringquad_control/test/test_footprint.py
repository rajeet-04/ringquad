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


def test_motor_cases_fit_guard_during_ground_manoeuvres():
    # Independent point transform of the complete outboard motor cylinders.
    from ringquad_control.plant import skew
    for yaw in (-.2,0.,.2):
        for fold in (math.radians(75),math.radians(88),math.pi/2):
            folds={'hinge_'+n:fold for n in p.ROTOR_NAMES}
            steering={'steer_fl':.44,'steer_fr':-.44}
            lo,hi=lateral_bounds(yaw,folds,steering,.1)
            for name in p.ROTOR_NAMES:
                side=1 if name.endswith('l') else -1
                front=1 if name.startswith('f') else -1
                def rotation(axis,angle):
                    k=skew(np.array(axis,float));return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)
                r=rotation([1,0,0],-side*fold)
                centre=np.array([front*.08,side*.08,0])+r@np.array([front*.12/math.sqrt(2),side*.12/math.sqrt(2),.1])
                r=r@rotation([0,1,0],-side*steering.get('steer_'+name,0.))
                for angle in np.linspace(0,2*math.pi,60):
                    for z in (-.0852,-.042):
                        point=centre+r@np.array([front*.09+.01*math.cos(angle),.01*math.sin(angle),z])
                        lateral=math.sin(yaw)*point[0]+math.cos(yaw)*point[1]
                        assert lo<=lateral<=hi
