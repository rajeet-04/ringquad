"""Ground-mode yaw-rate loop for skid steering.

With the tread friction set up as in the xacro (lower sideways than forward grip, to stand in
for a real tyre's friction circle), turning on the spot follows the kinematic command closely.
Feedforward does most of the work; PI on the IMU yaw rate removes the remaining scrub loss and
holds a straight line when no turn is requested.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class YawRateGains:
    k_ff: float = 1.0       # kinematic yaw command per rad/s requested
    kp: float = 2.0
    ki: float = 5.0
    i_max: float = 2.0
    cmd_max: float = 6.0    # kinematic yaw command limit (rad/s)


class YawRateController:
    def __init__(self, gains=None):
        self.g = gains or YawRateGains()
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, rate_sp, rate_meas, dt):
        """Returns the kinematic yaw-rate command to feed mixer.drive_commands()."""
        g = self.g
        err = rate_sp - rate_meas
        self.integral = float(np.clip(self.integral + g.ki * err * dt, -g.i_max, g.i_max))
        return float(np.clip(g.k_ff * rate_sp + g.kp * err + self.integral, -g.cmd_max, g.cmd_max))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class LineFollowGains:
    k_cross: float = 2.0     # Stanley cross-track gain
    k_heading: float = 2.5   # yaw rate per rad of heading error
    max_yaw_rate: float = 1.5
    slow_zone: float = 0.5   # m before the end: slow down linearly
    min_speed: float = 0.1


def follow_line(pos, yaw, start, end, speed, gains=None):
    """Stanley-style line following for driving: returns (speed, yaw_rate, remaining_m, cross_track_m).

    Steers toward the line's heading, corrected by atan(k * cross_track / speed),
    and slows down over the last slow_zone metres.
    """
    g = gains or LineFollowGains()
    start, end, pos = np.asarray(start, float), np.asarray(end, float), np.asarray(pos, float)[:2]
    length = np.linalg.norm(end - start)
    t = (end - start) / length
    n = np.array([-t[1], t[0]])
    cross = float((pos - start) @ n)          # + = left of the line
    remaining = float(length - (pos - start) @ t)
    v = float(np.clip(speed * remaining / g.slow_zone, g.min_speed, speed)) if remaining > 0 else 0.0
    line_yaw = math.atan2(t[1], t[0])
    heading_sp = line_yaw - math.atan2(g.k_cross * cross, max(v, g.min_speed))
    yaw_rate = float(np.clip(g.k_heading * wrap(heading_sp - yaw), -g.max_yaw_rate, g.max_yaw_rate))
    return v, yaw_rate, remaining, cross
