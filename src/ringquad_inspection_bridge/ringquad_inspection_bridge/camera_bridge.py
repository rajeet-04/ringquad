"""ROS 2 node: serve camera frames, odometry and controller status to the host harness over HTTP.

GET /stream  -> endless framed records (see streaming.py); GET /health -> JSON counters.
Bind to 0.0.0.0 inside the container and publish the port on host loopback only.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .streaming import Broadcaster, encode_jpeg, encode_record, stamp_ns

CLOCK_DOMAIN = 'sim_time'


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
                else:
                    self.send_error(404)

            def log_message(self, *_):
                pass

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'camera bridge serving http://{host}:{port}/stream')

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
        meta = {'kind': 'pose', 'timestamp_ns': stamp_ns(msg.header.stamp), 'clock_domain': CLOCK_DOMAIN,
                'frame_id': msg.header.frame_id or 'odom', 'position': [p.x, p.y, p.z],
                'orientation': [q.x, q.y, q.z, q.w], 'source': 'odom'}
        self.broadcaster.publish(encode_record(meta), False)

    def on_state(self, msg):
        self.broadcaster.publish(encode_record({'kind': 'status', 'state': msg.data}), False)

    def on_fault(self, msg):
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
