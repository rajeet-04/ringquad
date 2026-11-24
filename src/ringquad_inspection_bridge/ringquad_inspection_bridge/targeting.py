"""Pixel to ground target for the down-looking scouting camera (pure geometry, no ROS).

Camera frame is Gazebo's: +x forward (here: body down), +y left, +z up. Mounted at
CAMERA_OFFSET in the body frame with the optical axis pointing straight down, so
image-up is body-forward and image-right is body -y. Keep in sync with camera_joint in
ringquad.urdf.xacro.
"""
import math
from dataclasses import dataclass

CAMERA_OFFSET = (0.0, 0.0, -0.04)
MAX_TILT_RAD = math.radians(20.0)


@dataclass(frozen=True)
class Calibration:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_k(cls, k, width, height):
        return cls(k[0], k[4], k[2], k[5], width, height)


@dataclass(frozen=True)
class Pose:
    position: tuple
    orientation: tuple  # x, y, z, w
    timestamp_ns: int
    frame_id: str


@dataclass(frozen=True)
class GroundPlane:
    z: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class TargetPose:
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str


def _rotation(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0 or not math.isfinite(n):
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _apply(r, v):
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def resolve_target(bbox, pose, camera, plane, view_altitude):
    """World target above the ground point under the bbox centre, or None if it cannot be trusted."""
    if pose is None or camera is None or plane is None:
        return None
    rot = _rotation(pose.orientation)
    if rot is None or not all(math.isfinite(v) for v in pose.position):
        return None
    if math.acos(max(-1.0, min(1.0, rot[2][2]))) > MAX_TILT_RAD:
        return None
    u, v = (bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2
    # sensor-frame ray -> body frame: sensor x -> body -z, y -> y, z -> x
    sx, sy, sz = 1.0, -(u - camera.cx) / camera.fx, -(v - camera.cy) / camera.fy
    d_world = _apply(rot, (sz, sy, -sx))
    if d_world[2] >= -1e-6:
        return None
    off = _apply(rot, CAMERA_OFFSET)
    origin = tuple(pose.position[i] + off[i] for i in range(3))
    t = (plane.z - origin[2]) / d_world[2]
    if t <= 0:
        return None
    x, y = origin[0] + t * d_world[0], origin[1] + t * d_world[1]
    if not plane.contains(x, y):
        return None
    yaw = math.atan2(rot[1][0], rot[0][0])
    return TargetPose(x, y, view_altitude, yaw, pose.frame_id)
