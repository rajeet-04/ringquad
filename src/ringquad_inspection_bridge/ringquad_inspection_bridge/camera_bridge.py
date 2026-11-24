"""ROS 2 node: serve camera frames, odometry and controller status to the host harness over HTTP.

GET /stream  -> endless framed records (see streaming.py); GET /health -> JSON counters.
Bind to 0.0.0.0 inside the container and publish the port on host loopback only.
"""
import dataclasses
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from .inspection import InspectionConfig, InspectionController, InspectionRequest
from .streaming import Broadcaster, encode_jpeg, encode_record, stamp_ns
from .targeting import Calibration, GroundPlane, Pose, resolve_target

CLOCK_DOMAIN = 'sim_time'


class _RosIo:
    """InspectionController's view of the vehicle, backed by the node's latest messages."""

    def __init__(self, node):
        self.node = node

    def now(self):
        return self.node.get_clock().now().nanoseconds

    def current_pose(self):
        return self.node.latest_pose

    def current_speed(self):
        return self.node.latest_speed

    def mode(self):
        state = self.node.latest_state
        return state.split()[0].removeprefix('mode=') if state else ''

    def lock_fault(self):
        return self.node.latest_fault

    def publish_goal(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node.goal_pub.publish(msg)


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        host = self.declare_parameter('host', '0.0.0.0').value
        port = int(self.declare_parameter('port', 8765).value)
        self.quality = int(self.declare_parameter('jpeg_quality', 85).value)
        self.camera_id = self.declare_parameter('camera_id', 'camera-0').value
        self.broadcaster = Broadcaster(int(self.declare_parameter('client_queue', 8).value))
        self.calibration = None
        self.calibration_id = None
        self.sequence = 0
        self.frames_out = 0
        self.encode_errors = 0
        self.latest_pose = None
        self.latest_speed = 0.0
        self.latest_state = ''
        self.latest_fault = False
        self.plane = GroundPlane(
            float(self.declare_parameter('plane_z', 0.0).value),
            *[float(v) for v in self.declare_parameter('plane_bounds', [-1.0, 4.0, -2.0, 2.0]).value])
        self.declare_parameter('inspection_speed', 0.5)  # m/s setpoint ramp, read on each request
        self.view_altitude = float(self.declare_parameter('view_altitude', 0.8).value)
        self.goal_pub = self.create_publisher(PoseStamped, '/ringquad/goal', 10)
        self.inspection = InspectionController(_RosIo(self), InspectionConfig())
        self.create_timer(0.05, self.inspection.tick)

        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, sensor_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/ringquad/state', self.on_state, 10)
        self.create_subscription(Bool, '/ringquad/lock_fault', self.on_fault, 10)

        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                if self.path == '/health':
                    body = json.dumps({'frames': bridge.frames_out, 'dropped': bridge.broadcaster.total_dropped(),
                                       'encode_errors': bridge.encode_errors,
                                       'calibrated': bridge.calibration is not None}).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/octet-stream')
                    self.end_headers()
                    client = bridge.broadcaster.add()
                    try:
                        while not client.closed:
                            blob = client.pop(1.0)
                            if blob:
                                self.wfile.write(blob)
                                self.wfile.flush()
                    except OSError:
                        pass
                    finally:
                        bridge.broadcaster.remove(client)
                elif self.path.startswith('/inspection-requests/'):
                    rid = self.path.rsplit('/', 1)[1]
                    trace = bridge.inspection.trace(rid)
                    if not trace:
                        return self.send_error(404)
                    self._json(200, {'status': trace[-1].to_dict(), 'trace': [s.to_dict() for s in trace]})
                else:
                    self.send_error(404)

            def do_POST(self):  # noqa: N802
                if self.path != '/inspection-requests':
                    return self.send_error(404)
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 1 << 20:
                        raise ValueError('bad Content-Length')
                    status = bridge.submit_inspection(json.loads(self.rfile.read(length)))
                except (ValueError, KeyError, TypeError) as exc:
                    return self._json(400, {'error': f'{type(exc).__name__}: {exc}'})
                self._json(200, {'status': status.to_dict()})

            def _json(self, code, body):
                raw = json.dumps(body).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def log_message(self, *_):
                pass

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'camera bridge serving http://{host}:{port}/stream')

    def submit_inspection(self, body):
        """Resolve the observation's ground target and hand the request to the controller."""
        rid = str(body['request_id'])
        self.inspection.cfg = dataclasses.replace(
            self.inspection.cfg, speed=float(self.get_parameter('inspection_speed').value))
        if self.inspection.trace(rid):
            return self.inspection.status(rid)
        p, cal = body['pose'], body['calibration']
        pose = Pose(tuple(float(v) for v in p['position']), tuple(float(v) for v in p['orientation']),
                    int(p['timestamp_ns']), str(p['frame_id']))
        if p.get('clock_domain') != CLOCK_DOMAIN:
            return self.inspection.reject(rid, f"pose clock domain {p.get('clock_domain')!r} is not {CLOCK_DOMAIN}")
        camera = Calibration.from_k(cal['k'], int(cal['width']), int(cal['height']))
        target = resolve_target(tuple(float(v) for v in body['bbox']), pose, camera, self.plane, self.view_altitude)
        if target is None:
            return self.inspection.reject(rid, 'target unresolved or outside the scouting area')
        return self.inspection.submit(InspectionRequest(
            rid, str(body['observation_id']), tuple(body.get('evidence_ids', ())), str(body['requested_view']),
            (target.x, target.y, target.z), target.yaw, target.frame_id, pose.timestamp_ns, pose.position))

    def on_info(self, msg):
        k = list(msg.k)
        self.calibration = {'width': msg.width, 'height': msg.height, 'k': k, 'd': list(msg.d),
                            'distortion_model': msg.distortion_model}
        self.calibration_id = f'{msg.header.frame_id}:{msg.width}x{msg.height}:fx{k[0]:.3f}'

    def on_image(self, msg):
        try:
            jpeg = encode_jpeg(msg.width, msg.height, msg.encoding, msg.data, self.quality)
        except ValueError as exc:
            self.encode_errors += 1
            self.get_logger().warn(f'dropping frame: {exc}', throttle_duration_sec=5.0)
            return
        meta = {'kind': 'frame', 'camera_id': self.camera_id, 'sequence': self.sequence,
                'capture_ns': stamp_ns(msg.header.stamp), 'clock_domain': CLOCK_DOMAIN,
                'calibration_id': self.calibration_id, 'calibration': self.calibration,
                'frame_id': msg.header.frame_id, 'dropped_total': self.broadcaster.total_dropped()}
        self.sequence += 1
        self.frames_out += 1
        self.broadcaster.publish(encode_record(meta, jpeg), True)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self.latest_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.latest_pose = Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), stamp_ns(msg.header.stamp),
                                msg.header.frame_id or 'odom')
        meta = {'kind': 'pose', 'timestamp_ns': stamp_ns(msg.header.stamp), 'clock_domain': CLOCK_DOMAIN,
                'frame_id': msg.header.frame_id or 'odom', 'position': [p.x, p.y, p.z],
                'orientation': [q.x, q.y, q.z, q.w], 'source': 'odom'}
        self.broadcaster.publish(encode_record(meta), False)

    def on_state(self, msg):
        self.latest_state = msg.data
        self.broadcaster.publish(encode_record({'kind': 'status', 'state': msg.data}), False)

    def on_fault(self, msg):
        self.latest_fault = self.latest_fault or bool(msg.data)
        self.broadcaster.publish(encode_record({'kind': 'status', 'lock_fault': bool(msg.data)}), False)


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
