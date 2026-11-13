"""Exercise the actual mission generator with stalled ground steering."""
import math
import pytest

pytest.importorskip('rclpy')
from ringquad_control.mission import DemoMission, MissionAbort, ALIGN_ATTEMPTS


class StalledGround:
    land_folded_on_line = DemoMission.land_folded_on_line

    def __init__(self, recover):
        self.recover = recover
        self.yaw = 0.1
        self.t = 0.0
        self.landings = []
        self.commands = []

    def now(self):
        self.t += 1.0
        return self.t

    def fly_to(self, position, yaw):
        self.landings.append(yaw)
        # A repeatable fold offset can be cancelled by pre-compensating in flight.
        self.yaw = yaw + 0.1 if self.recover else 0.1
        yield

    def land(self):
        yield

    fold_into_wheels = land
    take_off = land

    def pause(self, seconds):
        yield

    def set_mode(self, mode):
        yield

    def send_cmd(self, speed, rate):
        self.commands.append((speed, rate))

    def line_error(self, start, end):
        return self.yaw, 0.0

    def get_logger(self):
        return self

    def info(self, message):
        pass

    warn = info


def test_stalled_ground_alignment_retries_and_compensates_in_flight():
    mission = StalledGround(recover=True)
    list(mission.land_folded_on_line((0, 0), (4, 0)))
    assert len(mission.landings) == 2
    assert math.isclose(mission.landings[1], -0.1)
    assert all(speed == 0.0 for speed, _ in mission.commands)
    assert mission.commands[-1] == (0.0, 0.0)


def test_unrecoverable_alignment_stops_after_bounded_attempts():
    mission = StalledGround(recover=False)
    with pytest.raises(MissionAbort, match='could not line up'):
        list(mission.land_folded_on_line((0, 0), (4, 0)))
    assert len(mission.landings) == ALIGN_ATTEMPTS
    assert mission.commands[-1] == (0.0, 0.0)


def test_fly_to_waits_for_heading_before_landing_can_begin():
    import numpy as np

    class AtPosition:
        fly_to = DemoMission.fly_to
        pos = np.array([0., 0., 0.8])
        vel = np.zeros(3)
        goal = None
        yaw = 0.2

        def now(self):
            return 0.0

        def send_goal(self, position, yaw):
            pass

    mission = AtPosition()
    flight = mission.fly_to((0., 0., 0.8), yaw=0.0)
    next(flight)  # Already at the position, but still facing the wrong way.
    mission.yaw = 0.0
    with pytest.raises(StopIteration):
        next(flight)
