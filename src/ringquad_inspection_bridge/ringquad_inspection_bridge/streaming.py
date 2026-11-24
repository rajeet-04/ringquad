"""ROS-free pieces of the camera bridge: record framing, JPEG encoding, bounded client queues.

Record: 4-byte big-endian JSON length, 4-byte big-endian payload length, JSON, payload.
The host reader lives in harness/src/ringquad_harness/live.py.
"""
import io
import json
import struct
import threading
from collections import deque

from PIL import Image


def encode_record(meta, payload=b''):
    raw = json.dumps(meta, separators=(',', ':')).encode()
    return struct.pack('>II', len(raw), len(payload)) + raw + payload


def encode_jpeg(width, height, encoding, data, quality=85):
    """JPEG bytes for a sensor_msgs/Image buffer (rgb8, bgr8, mono8)."""
    if encoding == 'rgb8':
        img = Image.frombytes('RGB', (width, height), bytes(data))
    elif encoding == 'bgr8':
        img = Image.frombytes('RGB', (width, height), bytes(data), 'raw', 'BGR')
    elif encoding in ('mono8', '8UC1'):
        img = Image.frombytes('L', (width, height), bytes(data)).convert('RGB')
    else:
        raise ValueError(f'unsupported image encoding {encoding!r}')
    out = io.BytesIO()
    img.save(out, format='JPEG', quality=quality)
    return out.getvalue()


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ClientQueue:
    """Bounded per-client queue: when the host is slow the oldest *frame* is dropped and counted."""

    def __init__(self, maxlen=8):
        self.maxlen = maxlen
        self._items = deque()
        self._cond = threading.Condition()
        self.closed = False
        self.dropped = 0

    def push(self, blob, droppable):
        with self._cond:
            if len(self._items) >= self.maxlen:
                for i, (_, d) in enumerate(self._items):
                    if d:
                        del self._items[i]
                        self.dropped += 1
                        break
                else:
                    self._items.popleft()
            self._items.append((blob, droppable))
            self._cond.notify()

    def pop(self, timeout=1.0):
        with self._cond:
            if not self._items and not self.closed:
                self._cond.wait(timeout)
            return self._items.popleft()[0] if self._items else None

    def close(self):
        with self._cond:
            self.closed = True
            self._cond.notify_all()


class Broadcaster:
    def __init__(self, maxlen=8):
        self.maxlen = maxlen
        self._clients = set()
        self._lock = threading.Lock()

    def add(self):
        client = ClientQueue(self.maxlen)
        with self._lock:
            self._clients.add(client)
        return client

    def remove(self, client):
        client.close()
        with self._lock:
            self._clients.discard(client)

    def publish(self, blob, droppable):
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            client.push(blob, droppable)

    def total_dropped(self):
        with self._lock:
            return sum(c.dropped for c in self._clients)
