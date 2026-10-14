from ringquad_control.ground import YawRateController


def run(rate_sp, seconds=3.0, dt=0.004, efficiency=0.8):
    """Plant: body yaw rate follows a fraction of the kinematic command with a 0.1 s lag
    (scrub loses some of it; Gazebo measured close to 1.0)."""
    ctl, rate = YawRateController(), 0.0
    for _ in range(int(seconds / dt)):
        cmd = ctl.update(rate_sp, rate, dt)
        rate += (efficiency * cmd - rate) * dt / 0.1
    return rate


def test_tracks_turn_left():
    assert abs(run(1.0) - 1.0) < 0.05


def test_tracks_turn_right_with_more_scrub():
    assert abs(run(-0.5, efficiency=0.6) + 0.5) < 0.05


def test_zero_request_stays_still():
    assert abs(run(0.0)) < 1e-6
