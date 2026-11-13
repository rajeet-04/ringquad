"""Exercise actual mission stop behavior with measured-joint faults."""
import math
import numpy as np
import pytest
pytest.importorskip('rclpy')
from ringquad_control import params as p
from ringquad_control.mission import DemoMission, MissionAbort


class CorridorFault:
    drive_line = DemoMission.drive_line
    active_steering = True
    contact_track = .32
    ring_offset = .08
    yaw = .1
    yaw_rate_measured = 0.
    ground_tilt = 0.
    joint_time = 0.
    minimum_margin = float('inf')
    vel = np.zeros(3)
    pos = np.array([6., .025, .22])
    hinges = {'hinge_'+n: math.pi/2 for n in p.ROTOR_NAMES}
    steering_angles = {'steer_fl': math.radians(25), 'steer_fr': math.radians(25)}

    def __init__(self):
        self.commands=[]

    def now(self):
        return 0.

    def send_cmd(self, speed, rate):
        self.commands.append((speed,rate))

    def get_logger(self):
        return self

    def info(self, message):
        pass

    def line_error(self, *args):
        return self.yaw, self.pos[1]


def test_stuck_steering_stops_even_within_fixed_cross_track_limit():
    mission=CorridorFault()
    with pytest.raises(MissionAbort, match='footprint margin'):
        next(mission.drive_line((3.8,0.),(9.4,0.)))
    assert mission.commands == [(0.,0.)]


@pytest.mark.parametrize('field,value', [('joint_time',-1.),('ground_tilt',math.radians(4)),('steering_angles',{})])
def test_untrusted_footprint_stops(field,value):
    mission=CorridorFault();setattr(mission,field,value)
    with pytest.raises(MissionAbort, match='unavailable or excessive ground tilt'):
        next(mission.drive_line((3.8,0.),(9.4,0.)))
    assert mission.commands == [(0.,0.)]
