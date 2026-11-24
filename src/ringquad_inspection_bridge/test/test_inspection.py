import math

from ringquad_inspection_bridge.inspection import InspectionConfig, InspectionController, InspectionRequest
from ringquad_inspection_bridge.targeting import Calibration, GroundPlane, Pose, resolve_target

NS = 1_000_000_000
CAM = Calibration(fx=400.0, fy=400.0, cx=320.0, cy=240.0, width=640, height=480)
PLANE = GroundPlane(z=0.0, x_min=-1.0, x_max=4.0, y_min=-2.0, y_max=2.0)
LEVEL = (0.0, 0.0, 0.0, 1.0)


def pose(x=0.0, y=0.0, z=1.0, q=LEVEL, t=0):
    return Pose(position=(x, y, z), orientation=q, timestamp_ns=t, frame_id='odom')


def test_image_centre_hits_the_point_below_the_camera():
    # camera is 4 cm under the chassis and looks straight down at 1 m height
    target = resolve_target((310, 230, 330, 250), pose(), CAM, PLANE, view_altitude=0.5)
    assert target is not None and abs(target.x) < 1e-6 and abs(target.y) < 1e-6
    assert target.z == 0.5 and target.frame_id == 'odom'


def test_known_pixel_offsets_map_to_body_axes():
    # image up is body forward, image right is body -y; at 0.96 m camera height, 100 px = 0.24 m
    up = resolve_target((310, 130, 330, 150), pose(), CAM, PLANE, view_altitude=0.5)
    assert math.isclose(up.x, 0.24, abs_tol=1e-3) and abs(up.y) < 1e-6
    right = resolve_target((410, 230, 430, 250), pose(), CAM, PLANE, view_altitude=0.5)
    assert math.isclose(right.y, -0.24, abs_tol=1e-3) and abs(right.x) < 1e-6


def test_yaw_rotates_the_result_into_world():
    s = math.sin(math.pi / 4)
    yawed = pose(q=(0.0, 0.0, s, s))  # +90 deg about z
    target = resolve_target((310, 130, 330, 150), yawed, CAM, PLANE, view_altitude=0.5)
    assert math.isclose(target.y, 0.24, abs_tol=1e-3) and abs(target.x) < 1e-3


def test_out_of_bounds_and_unresolved_targets_are_rejected():
    assert resolve_target((310, 130, 330, 150), pose(x=3.9), CAM, PLANE, view_altitude=0.5) is None
    # on the ground the camera sees nothing: ray does not reach the plane
    assert resolve_target((310, 230, 330, 250), pose(z=-0.5), CAM, PLANE, view_altitude=0.5) is None
    # strongly tilted vehicle: ground point is unreliable
    tilt = math.radians(40)
    assert resolve_target((310, 230, 330, 250), pose(q=(math.sin(tilt / 2), 0, 0, math.cos(tilt / 2))),
                          CAM, PLANE, view_altitude=0.5) is None
    assert resolve_target((310, 230, 330, 250), None, CAM, PLANE, view_altitude=0.5) is None


class FakeIo:
    def __init__(self):
        self.now_ns = 100 * NS
        self.pose_ns = 100 * NS
        self.position = (0.0, 0.0, 1.0)
        self.speed = 0.0
        self.flying = True
        self.fault = False
        self.goals = []
        self.modes = []

    def now(self):
        return self.now_ns

    def current_pose(self):
        return None if self.pose_ns is None else Pose(self.position, LEVEL, self.pose_ns, 'odom')

    def current_speed(self):
        return self.speed

    def mode(self):
        return 'fly' if self.flying else 'idle'

    def lock_fault(self):
        return self.fault

    def publish_goal(self, x, y, z, yaw):
        self.goals.append((x, y, z, yaw))

    def request_mode(self, mode):
        self.modes.append(mode)


def request(rid='r1', **kw):
    base = dict(request_id=rid, observation_id='o1', evidence_ids=('f1',), requested_view='closer view',
                target=(0.5, 0.0, 0.5), yaw=0.0, frame_id='odom', pose_timestamp_ns=100 * NS,
                pose_position=(0.0, 0.0, 1.0))
    base.update(kw)
    return InspectionRequest(**base)


def controller(io=None, **cfg):
    io = io or FakeIo()
    return io, InspectionController(io, InspectionConfig(**cfg))


def test_rejections_never_publish():
    cases = {
        'stale': lambda io: setattr(io, 'pose_ns', 50 * NS),
        'no pose': lambda io: setattr(io, 'pose_ns', None),
        'fault': lambda io: setattr(io, 'fault', True),
        'mode': lambda io: setattr(io, 'flying', False),
    }
    for name, mutate in cases.items():
        io, ctl = controller()
        mutate(io)
        status = ctl.submit(request())
        assert status.state == 'rejected', name
        ctl.tick()
        assert io.goals == [] and io.modes == [], name


def test_stale_request_pose_and_wrong_frame_are_rejected():
    io, ctl = controller()
    assert ctl.submit(request(pose_timestamp_ns=10 * NS)).state == 'rejected'
    assert ctl.submit(request('r2', frame_id='map')).state == 'rejected'
    assert io.goals == []


def test_vehicle_moved_since_capture_is_rejected():
    io, ctl = controller()
    io.position = (2.0, 0.0, 1.0)
    status = ctl.submit(request())
    assert status.state == 'rejected' and 'moved' in status.reason


def test_accepted_request_ramps_to_target_and_completes():
    io, ctl = controller()
    assert ctl.submit(request()).state == 'accepted'
    ctl.tick()
    assert ctl.status('r1').state == 'running'
    for _ in range(400):
        io.now_ns += NS // 20
        io.pose_ns = io.now_ns
        if io.goals:
            x, y, z, _ = io.goals[-1]
            io.position = (x, y, z)
        ctl.tick()
        if ctl.status('r1').state == 'completed':
            break
    assert [s.state for s in ctl.trace('r1')] == ['accepted', 'running', 'completed']
    # setpoints move gradually, never jump to the target
    first = io.goals[0]
    assert math.dist(first[:3], (0.0, 0.0, 1.0)) < 0.2


def test_duplicate_id_does_not_repeat_the_command_sequence():
    io, ctl = controller()
    ctl.submit(request())
    ctl.tick()
    published = len(io.goals)
    again = ctl.submit(request())
    ctl.tick()
    assert again.state in ('accepted', 'running')
    assert [s.state for s in ctl.trace('r1')].count('accepted') == 1
    assert len(io.goals) <= published + 1


def test_second_request_while_busy_is_rejected():
    io, ctl = controller()
    ctl.submit(request())
    status = ctl.submit(request('r2'))
    assert status.state == 'rejected' and 'busy' in status.reason


def test_lost_feedback_ends_in_failed_and_stops_setpoints():
    io, ctl = controller(feedback_timeout_s=1.0)
    ctl.submit(request())
    ctl.tick()
    io.pose_ns = io.now_ns  # last good feedback
    for _ in range(100):
        io.now_ns += NS // 20  # odometry silent from here on
        ctl.tick()
    assert ctl.status('r1').state == 'failed' and 'feedback' in ctl.status('r1').reason
    count = len(io.goals)
    for _ in range(20):
        io.now_ns += NS // 20
        ctl.tick()
    assert len(io.goals) == count


def test_fault_while_running_fails_the_request():
    io, ctl = controller()
    ctl.submit(request())
    ctl.tick()
    io.fault = True
    io.now_ns += NS // 20
    io.pose_ns = io.now_ns
    ctl.tick()
    assert ctl.status('r1').state == 'failed'


def test_overall_timeout_fails():
    io, ctl = controller(run_timeout_s=2.0)
    ctl.submit(request(target=(3.0, 0.0, 0.5)))
    for _ in range(80):
        io.now_ns += NS // 20
        io.pose_ns = io.now_ns
        ctl.tick()
    assert ctl.status('r1').state == 'failed' and 'timed out' in ctl.status('r1').reason


def test_reject_records_once_and_never_publishes():
    io, ctl = controller()
    first = ctl.reject('r9', 'target unresolved')
    assert first.state == 'rejected' and ctl.reject('r9', 'other').reason == 'target unresolved'
    ctl.tick()
    assert io.goals == []
