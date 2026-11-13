from ringquad_control.ground import LineFollowGains, YawRateController, follow_line


def run(rate_sp, seconds=3.0, dt=0.004, efficiency=0.8):
    """Plant: body yaw rate follows a fraction of the kinematic command with a 0.1 s lag
    (scrub loses some of it; Gazebo measured close to 1.0)."""
    ctl, rate = YawRateController(), 0.0
    for _ in range(int(seconds / dt)):
        cmd = ctl.update(rate_sp, rate, dt)
        rate += (efficiency * cmd - rate) * dt / 0.1
    return rate


def test_tracks_gentle_turn():
    assert abs(run(0.5) - 0.5) < 0.05


def test_tracks_turn_right_with_more_scrub():
    assert abs(run(-0.4, efficiency=0.6) + 0.4) < 0.05


def test_no_windup_when_robot_cannot_turn():
    """Grippy floor: the robot does not turn at all. Command must stay small and bounded."""
    ctl = YawRateController()
    for _ in range(2000):
        cmd = ctl.update(1.0, 0.0, 0.004)
    assert abs(cmd) <= ctl.g.cmd_max
    assert abs(ctl.integral) <= ctl.g.i_max


def test_zero_request_stays_still():
    assert abs(run(0.0)) < 1e-6


def drive_line(x, y, yaw, start=(0, 0), end=(4, 0), speed=0.4, dt=0.02):
    import math
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
    v, w, remaining, cross = follow_line((0, 0), math.radians(45), (0, 0), (3, 3), 0.4)
    assert abs(cross) < 1e-9 and abs(w) < 1e-9 and abs(remaining - 3 * math.sqrt(2)) < 1e-9


def test_coupled_line_follow_with_yaw_lag_and_lever_arm():
    """Simulates tread:=mixed (pivot at rear axle 16.5 cm behind CoM + YawRateController + 0.08 s lag)
    from the new tunnel pad at x = 3.8 m into the tunnel at x = 5.2 .. 8.5 m."""
    import math
    rear_x, rear_y, yaw = 3.8 - 0.165, 0.04, math.radians(-8)
    yaw_ctl, rate = YawRateController(), 0.0
    gains = LineFollowGains(axle_offset=-0.08)
    dt = 0.004
    worst_in_tunnel = 0.0
    for _ in range(int(20.0 / dt)):
        com_x = rear_x + 0.165 * math.cos(yaw)
        com_y = rear_y + 0.165 * math.sin(yaw)
        v, w_sp, remaining, cross = follow_line(
            (com_x, com_y), yaw, (3.8, 0.0), (9.4, 0.0), 0.4, gains=gains)
        if remaining <= 0.02:
            break
        cmd = yaw_ctl.update(w_sp, rate, dt)
        rate += (cmd - rate) * dt / 0.08
        yaw += rate * dt
        rear_x += v * math.cos(yaw) * dt
        rear_y += v * math.sin(yaw) * dt
        if com_x >= 5.2:
            worst_in_tunnel = max(worst_in_tunnel, abs(cross))
    assert worst_in_tunnel < 0.025
