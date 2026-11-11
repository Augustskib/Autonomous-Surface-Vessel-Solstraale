"""Bounded binary PointCloud2 sampling without changing field layout or clock."""
import copy
import math


def preview_cloud(msg, budget=10000, max_bytes=64000000):
    if budget < 1 or msg.width < 1 or msg.height < 1 or msg.point_step < 1:
        raise ValueError('Invalid point count or budget')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        raise ValueError('Invalid cloud stride/data size')
    if len(msg.data) > max_bytes:
        raise ValueError('Cloud exceeds input byte limit')
    fields = {field.name for field in msg.fields}
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('Cloud has no XYZ fields')
    total = msg.width * msg.height
    stride = max(1, math.ceil(total / budget))
    data = bytearray()
    for index in range(0, total, stride):
        row, col = divmod(index, msg.width)
        start = row * msg.row_step + col * msg.point_step
        data.extend(msg.data[start:start+msg.point_step])
    out = copy.copy(msg)
    out.height = 1
    out.width = len(data) // msg.point_step
    out.row_step = len(data)
    out.data = bytes(data)
    return out
