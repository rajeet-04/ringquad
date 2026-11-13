"""Rigid full-width rim envelope, independent of the contact approximation."""
import math
import numpy as np
from . import params as p


def lateral_bounds(yaw, folds, steering, ring_offset):
    """Bounds relative to chassis origin in the corridor lateral direction.

    Uses measured fold/steer joint angles (not nominal wheel heading). The full
    270 x 30 mm rotating cylinder encloses the annular rim and swept blades.
    Ground motion is planar; the caller must reject excessive roll/pitch.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    lo, hi = -(.08*abs(cy)+.08*abs(sy)), .08*abs(cy)+.08*abs(sy)
    for name in p.ROTOR_NAMES:
        side = 1 if name.endswith('l') else -1
        front = 1 if name.startswith('f') else -1
        fold = folds['hinge_'+name]
        sf, cf = math.sin(fold), math.cos(fold)
        a = steering.get('steer_'+name, 0.)
        sa, ca = math.sin(a), math.cos(a)
        # Rx(-side*fold) @ Ry(-side*steer) @ unit_z
        nx, ny = -side*sa, side*sf*ca
        normal_y = sy*nx+cy*ny
        centre_x = front*p.ARM_XY
        centre_y = side*(.08+.12/math.sqrt(2)*cf+ring_offset*sf)
        centre = sy*centre_x+cy*centre_y - .010*normal_y
        extent = p.WHEEL_RADIUS*math.sqrt(max(0., 1-normal_y**2))+.025*abs(normal_y)
        lo, hi = min(lo, centre-extent), max(hi, centre+extent)
    return lo, hi


def corridor_margin(cross, yaw, folds, steering, ring_offset, buffer=.01):
    lo, hi = lateral_bounds(yaw, folds, steering, ring_offset)
    return min(.25-(cross+hi), .25+(cross+lo))-buffer
