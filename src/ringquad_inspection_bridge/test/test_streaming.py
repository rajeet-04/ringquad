import io
import struct

from PIL import Image

from ringquad_inspection_bridge.streaming import (
    Broadcaster, ClientQueue, encode_jpeg, encode_record, stamp_ns)


class Stamp:
    sec, nanosec = 3, 500


def test_record_layout():
    blob = encode_record({'kind': 'pose'}, b'xyz')
    meta_len, payload_len = struct.unpack('>II', blob[:8])
    assert payload_len == 3 and blob[8:8 + meta_len] == b'{"kind":"pose"}' and blob.endswith(b'xyz')


def test_jpeg_encodings_keep_color():
    red_rgb = bytes([255, 0, 0] * 4)
    red_from_bgr = bytes([0, 0, 255] * 4)
    for enc, data in (('rgb8', red_rgb), ('bgr8', red_from_bgr)):
        px = Image.open(io.BytesIO(encode_jpeg(2, 2, enc, data))).getpixel((0, 0))
        assert px[0] > 200 and px[1] < 60 and px[2] < 60


def test_unsupported_encoding_raises():
    try:
        encode_jpeg(1, 1, '32FC1', b'\0\0\0\0')
    except ValueError:
        return
    raise AssertionError('expected ValueError')


def test_stamp_ns():
    assert stamp_ns(Stamp) == 3_000_000_500


def test_slow_client_drops_oldest_frame_and_keeps_pose():
    q = ClientQueue(maxlen=3)
    q.push(b'pose', False)
    q.push(b'f1', True)
    q.push(b'f2', True)
    q.push(b'f3', True)
    assert q.dropped == 1
    assert [q.pop(0), q.pop(0), q.pop(0)] == [b'pose', b'f2', b'f3']
    assert q.pop(0) is None


def test_broadcast_counts_drops_per_client():
    b = Broadcaster(maxlen=1)
    slow = b.add()
    for i in range(3):
        b.publish(b'f', True)
    assert slow.dropped == 2 and b.total_dropped() == 2
    b.remove(slow)
    assert b.total_dropped() == 0
