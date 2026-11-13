import numpy as np
from ringquad_control.flight import FlightController


def test_persistent_position_bias_is_learned_outside_old_integral_zone():
    ctl=FlightController()
    ctl.update(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.array([1.,0.,0.]),0.,.004)
    assert ctl.integral[0]>0


def test_large_moving_transient_does_not_accumulate_position_bias():
    ctl=FlightController()
    ctl.update(np.zeros(3),np.ones(3),np.eye(3),np.zeros(3),np.ones(3),0.,.004)
    assert np.allclose(ctl.integral,0.)


def test_saturation_freezes_position_integral_that_would_deepen_it():
    ctl=FlightController()
    args=(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.array([0.,0.,100.]),0.,.004)
    ctl.update(*args);first=ctl.integral.copy()
    ctl.update(*args)
    assert np.allclose(ctl.integral,first)


def test_reset_removes_both_learned_disturbances():
    ctl=FlightController();ctl.integral[:]=1.;ctl.attitude_integral[:]=2.;ctl.acc_excess[:]=3.
    ctl.reset()
    assert not np.any(ctl.integral) and not np.any(ctl.attitude_integral) and not np.any(ctl.acc_excess)
