"""Flatten a PointCloud2 to a small top-down point list for the map overlay.

Pure numpy, no ROS imports, so it can be tested without a ROS install. The
result is for display only.
"""
import time

import numpy as np

FLOAT32 = 7  # sensor_msgs/PointField.FLOAT32


def cloud_to_xy_cm(msg, max_points=3000, min_range_m=0.3, max_range_m=100.0):
    """Return a flat [x0, y0, x1, y1, ...] list in centimetres, sensor frame.

    Reads the x/y/z fields by their declared offsets. Non-finite points and
    points outside the range window are dropped, then the rest is thinned
    evenly to at most max_points.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(n in fields and fields[n].datatype == FLOAT32 for n in ('x', 'y', 'z')):
        raise ValueError('Cloud needs float32 x, y and z fields')
    step = int(msg.point_step)
    if step < 12 or any(fields[n].offset + 4 > step for n in ('x', 'y', 'z')):
        raise ValueError('Invalid point layout')
    data = bytes(msg.data)
    count = min(int(msg.width) * int(msg.height), len(data) // step)
    if count < 1:
        return []
    order = '>' if msg.is_bigendian else '<'
    layout = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [order + 'f4'] * 3,
                       'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                       'itemsize': step})
    points = np.frombuffer(data, dtype=layout, count=count)
    x, y, z = points['x'], points['y'], points['z']
    with np.errstate(invalid='ignore', over='ignore'):
        horizontal = np.hypot(x, y)
        keep = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                & (horizontal >= min_range_m) & (horizontal <= max_range_m))
    x, y = x[keep], y[keep]
    if x.size > max_points:
        stride = -(-x.size // max_points)
        x, y = x[::stride], y[::stride]
    flat = np.empty(x.size * 2, dtype=np.int32)
    flat[0::2] = np.round(x * 100.0)
    flat[1::2] = np.round(y * 100.0)
    return flat.tolist()


class LatestScan:
    """Keeps the newest cloud and converts it only when a viewer asks."""

    def __init__(self, stale_sec=2.0, clock=time.monotonic):
        self.stale_sec, self.clock = stale_sec, clock
        self.msg = None
        self.received = None
        self.converted_for = None
        self.xy = []
        self.error = ''

    def put(self, msg):
        self.msg = msg
        self.received = self.clock()

    def snapshot(self):
        msg, received = self.msg, self.received
        age = None if received is None else self.clock() - received
        if msg is None or age > self.stale_sec:
            return {'fresh': False, 'age_s': age, 'count': 0, 'xy': [], 'error': ''}
        if self.converted_for is not msg:
            try:
                self.xy, self.error = cloud_to_xy_cm(msg), ''
            except ValueError as exc:
                self.xy, self.error = [], str(exc)
            self.converted_for = msg
        return {'fresh': not self.error, 'age_s': round(age, 2),
                'frame': msg.header.frame_id, 'count': len(self.xy) // 2,
                'xy': self.xy, 'error': self.error}
