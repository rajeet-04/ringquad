"""Ground-mode yaw-rate loop for skid steering, and line following.

With ideal motors, turning on the spot followed the kinematic command closely. With
realistic motor torque (0.33-0.4 Nm, direct drive) the wheels cannot push hard enough to
scrub the tyres sideways: the most turning moment they can make (~1.5-1.8 Nm) is less than
the sideways friction resists with (~1.6 Nm at mu 0.6, more on real rubber). So on grippy
ground this footprint can barely turn, and a large left/right speed difference just makes
the wheels fight and stall the robot. The loop is therefore limited to small corrections,
with anti-windup, so it never stalls the drive; real turns are done in the air (see
mission.py).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class YawRateGains:
    k_ff: float = 1.0       # kinematic yaw command per rad/s requested
    kp: float = 1.2
    ki: float = 2.5
    rate_filter_tau: float = 0.05  # s; suppress gyro noise before the low-speed PI loop
    i_max: float = 0.35
    cmd_max: float = 1.0    # kinematic yaw command limit (rad/s): small, so the wheels never fight


class YawRateController:
    def __init__(self, gains=None):
        self.g = gains or YawRateGains()
        self.integral = 0.0
        self.filtered_rate = 0.0

    def reset(self):
        self.integral = 0.0
        self.filtered_rate = 0.0

    def update(self, rate_sp, rate_meas, dt):
        """Returns the kinematic yaw-rate command to feed mixer.drive_commands()."""
        g = self.g
        self.filtered_rate += dt / (g.rate_filter_tau + dt) * (rate_meas - self.filtered_rate)
        err = rate_sp - self.filtered_rate
        unclipped = g.k_ff * rate_sp + g.kp * err + self.integral
        if abs(unclipped) < g.cmd_max or np.sign(err) != np.sign(unclipped):
            # anti-windup: stop integrating while saturated in the direction of the error
            self.integral = float(np.clip(self.integral + g.ki * err * dt, -g.i_max, g.i_max))
        return float(np.clip(g.k_ff * rate_sp + g.kp * err + self.integral, -g.cmd_max, g.cmd_max))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class LineFollowGains:
    k_cross: float = 1.6      # Stanley cross-track gain
    k_heading: float = 2.2    # yaw rate per rad of heading error
    v_soft: float = 0.15      # m/s softening term to prevent low-speed gain blow-up
    axle_offset: float = 0.0  # m along body +x from pos to steering reference point
    max_yaw_rate: float = 1.0
    slow_zone: float = 0.5    # m before the end: slow down linearly
    min_speed: float = 0.1


def follow_line(pos, yaw, start, end, speed, gains=None):
    """Stanley-style line following for driving: returns (speed, yaw_rate, remaining_m, cross_track_m).

    Steers toward the line's heading, corrected by atan(k * cross_track / (v_soft + speed)),
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
    ctrl_cross = cross + g.axle_offset * math.sin(wrap(yaw - line_yaw))
    heading_sp = line_yaw - math.atan2(g.k_cross * ctrl_cross, g.v_soft + max(v, g.min_speed))
    yaw_rate = float(np.clip(g.k_heading * wrap(heading_sp - yaw), -g.max_yaw_rate, g.max_yaw_rate))
    return v, yaw_rate, remaining, cross
