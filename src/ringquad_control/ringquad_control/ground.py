"""Ground-mode yaw-rate loop for skid steering.

With the tread friction set up as in the xacro (lower sideways than forward grip, to stand in
for a real tyre's friction circle), turning on the spot follows the kinematic command closely.
Feedforward does most of the work; PI on the IMU yaw rate removes the remaining scrub loss and
holds a straight line when no turn is requested.
"""
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
