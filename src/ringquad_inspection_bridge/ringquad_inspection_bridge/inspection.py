"""One-request-at-a-time inspection controller (pure logic; the ROS node supplies the Io).

States: accepted -> running -> completed | failed, or rejected. Rejected requests never
publish anything. Setpoints are a ramped carrot toward the target, published only while a
request is running, and they stop the moment the request reaches a terminal state.
"""
import math
import threading
from dataclasses import dataclass

NS = 1_000_000_000


@dataclass(frozen=True)
class InspectionRequest:
    request_id: str
    observation_id: str
    evidence_ids: tuple
    requested_view: str
    target: tuple          # x, y, z in frame_id
    yaw: float
    frame_id: str
    pose_timestamp_ns: int
    pose_position: tuple = None  # vehicle position when the evidence was captured


@dataclass(frozen=True)
class InspectionStatus:
    request_id: str
    state: str
    timestamp_ns: int
    reason: str = None

    def to_dict(self):
        return {'request_id': self.request_id, 'state': self.state,
                'timestamp_ns': self.timestamp_ns, 'reason': self.reason}


@dataclass(frozen=True)
class InspectionConfig:
    frame_id: str = 'odom'
    max_pose_age_ns: int = 2 * NS
    max_request_age_ns: int = 30 * NS
    max_move_m: float = 0.3
    feedback_timeout_s: float = 2.0
    run_timeout_s: float = 60.0
    speed: float = 0.5
    tolerance_m: float = 0.15
    settle_speed: float = 0.2
    min_altitude: float = 0.3


class InspectionController:
    def __init__(self, io, config=None):
        self.io = io
        self.cfg = config or InspectionConfig()
        self._lock = threading.Lock()
        self._traces = {}
        self._requests = {}
        self._active = None
        self._carrot = None
        self._last_tick_ns = None
        self._started_ns = None

    # ---- queries ----
    def status(self, request_id):
        with self._lock:
            trace = self._traces.get(request_id)
            return trace[-1] if trace else None

    def trace(self, request_id):
        with self._lock:
            return list(self._traces.get(request_id, ()))

    # ---- commands ----
    def submit(self, request):
        with self._lock:
            if request.request_id in self._traces:
                return self._traces[request.request_id][-1]
            reason = self._reject_reason(request)
            if reason:
                return self._record(request.request_id, 'rejected', reason)
            self._requests[request.request_id] = request
            self._active = request.request_id
            self._carrot = None
            self._started_ns = self.io.now()
            return self._record(request.request_id, 'accepted')

    def reject(self, request_id, reason):
        """Record a rejection decided before the request could be built (e.g. unresolved target)."""
        with self._lock:
            if request_id in self._traces:
                return self._traces[request_id][-1]
            return self._record(request_id, 'rejected', reason)

    def tick(self):
        with self._lock:
            if self._active is None:
                return
            request = self._requests[self._active]
            rid, now = self._active, self.io.now()
            pose = self.io.current_pose()
            if self.io.lock_fault():
                return self._finish(rid, 'failed', 'steering latch fault during inspection')
            if now - self._started_ns > self.cfg.run_timeout_s * NS:
                return self._finish(rid, 'failed', 'timed out before reaching the target')
            if pose is None or now - pose.timestamp_ns > self.cfg.feedback_timeout_s * NS:
                return self._finish(rid, 'failed', 'lost odometry feedback')
            if self.io.mode() != 'fly':
                return self._finish(rid, 'failed', f'controller left fly mode ({self.io.mode()})')
            if self._traces[rid][-1].state == 'accepted':
                self._record(rid, 'running')
                self._carrot = tuple(pose.position)
            dt = 0.05 if self._last_tick_ns is None else min(max((now - self._last_tick_ns) / NS, 0.0), 0.2)
            self._last_tick_ns = now
            step = self.cfg.speed * dt
            delta = [request.target[i] - self._carrot[i] for i in range(3)]
            dist = math.sqrt(sum(d * d for d in delta))
            self._carrot = tuple(request.target) if dist <= step else tuple(
                self._carrot[i] + delta[i] / dist * step for i in range(3))
            self.io.publish_goal(*self._carrot, request.yaw)
            arrived = (self._carrot == tuple(request.target)
                       and math.dist(pose.position, request.target) < self.cfg.tolerance_m
                       and self.io.current_speed() < self.cfg.settle_speed)
            if arrived:
                self._finish(rid, 'completed')

    # ---- internals ----
    def _reject_reason(self, r):
        c, now = self.cfg, self.io.now()
        if self._active is not None:
            return 'busy: another inspection is active'
        if r.frame_id != c.frame_id:
            return f'unsupported target frame {r.frame_id!r}'
        if len(r.target) != 3 or not all(math.isfinite(v) for v in r.target) or r.target[2] < c.min_altitude:
            return 'invalid target'
        if self.io.lock_fault():
            return 'steering latch fault'
        if self.io.mode() != 'fly':
            return f'controller not flying ({self.io.mode()})'
        pose = self.io.current_pose()
        if pose is None or now - pose.timestamp_ns > c.max_pose_age_ns:
            return 'stale or missing vehicle pose'
        if now - r.pose_timestamp_ns > c.max_request_age_ns:
            return 'request pose is too old'
        if r.pose_position is not None and math.dist(pose.position, r.pose_position) > c.max_move_m:
            return 'vehicle moved since the evidence was captured'
        return None

    def _record(self, rid, state, reason=None):
        status = InspectionStatus(rid, state, self.io.now(), reason)
        self._traces.setdefault(rid, []).append(status)
        return status

    def _finish(self, rid, state, reason=None):
        self._record(rid, state, reason)
        self._active = None
        self._carrot = None
        self._last_tick_ns = None
