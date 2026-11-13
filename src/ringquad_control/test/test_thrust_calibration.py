import numpy as np
import pytest
from ringquad_control import mixer,params as p


def test_per_rotor_coefficients_compensate_measured_thrust_and_respect_rpm_limit():
    coefficients=p.K_THRUST*np.array([.8,1.,1.,1.])
    force=np.full(4,5.)
    speeds=mixer.thrusts_to_commands(force,coefficients)
    assert speeds[0]>speeds[1]
    assert np.allclose(coefficients*speeds**2,force)
    assert np.all(mixer.thrusts_to_commands(np.full(4,100.),coefficients)<=p.MAX_ROTOR_SPEED)


@pytest.mark.parametrize('coefficients', [[0,1,1,1],[-1,1,1,1],[1,1],float('nan')])
def test_invalid_calibration_cannot_arm(coefficients):
    with pytest.raises(ValueError):mixer.thrusts_to_commands(np.ones(4),coefficients)
