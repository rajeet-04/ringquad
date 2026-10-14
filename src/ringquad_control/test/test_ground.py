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


def drive_line(x, y, yaw, start=(0, 0), end=(4, 0), speed=0.4, dt=0.02):
    import math
    from ringquad_control.ground import follow_line
    worst = 0.0
    for _ in range(int(30 / dt)):
        v, w, remaining, cross = follow_line((x, y), yaw, start, end, speed)
        if remaining <= 0.01:
            break
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        if x > 1.0:  # after the entry section, track must stay tight
            worst = max(worst, abs(cross))
    return x, y, worst


def test_line_follow_corrects_offset_and_heading():
    import math
    x, y, worst = drive_line(0.0, 0.15, math.radians(-10))
    assert abs(x - 4.0) < 0.05
    assert abs(y) < 0.01
    assert worst < 0.02   # tunnel gives about 7 cm each side


def test_line_follow_diagonal_line():
    import math
    from ringquad_control.ground import follow_line
    v, w, remaining, cross = follow_line((0, 0), math.radians(45), (0, 0), (3, 3), 0.4)
    assert abs(cross) < 1e-9 and abs(w) < 1e-9 and abs(remaining - 3 * math.sqrt(2)) < 1e-9
