"""Map LiDAR layer: PointCloud2 flattening, without ROS."""
import struct
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/boat_web_ui'))
from boat_web_ui.lidar_view import LatestScan, cloud_to_xy_cm


def cloud(points, step=32, offsets=(0, 4, 8)):
    data = bytearray(step * len(points))
    for i, p in enumerate(points):
        for value, off in zip(p, offsets):
            struct.pack_into('<f', data, i * step + off, value)
    fields = [NS(name=n, offset=o, datatype=7) for n, o in zip('xyz', offsets)]
    return NS(fields=fields, point_step=step, width=len(points), height=1,
              is_bigendian=False, data=bytes(data), header=NS(frame_id='lidar_frame'))


def test_xy_in_centimetres_with_padded_point_layout():
    assert cloud_to_xy_cm(cloud([(1.0, -2.5, 0.3), (10.0, 0.0, 1.0)])) == [100, -250, 1000, 0]


def test_field_offsets_are_read_not_assumed():
    assert cloud_to_xy_cm(cloud([(3.0, 4.0, 0.0)], offsets=(16, 20, 4))) == [300, 400]


def test_nonfinite_near_and_far_points_are_dropped():
    pts = [(float('nan'), 1, 0), (float('inf'), 1, 0), (0.05, 0.05, 0), (500.0, 0, 0), (2.0, 2.0, 0)]
    assert cloud_to_xy_cm(cloud(pts)) == [200, 200]


def test_large_clouds_are_thinned_to_the_budget():
    out = cloud_to_xy_cm(cloud([(1.0 + i * 0.001, 0, 0) for i in range(10000)]), max_points=3000)
    assert 2000 <= len(out) // 2 <= 3000


def test_cloud_without_xyz_is_rejected():
    msg = cloud([(1.0, 1.0, 0.0)])
    msg.fields = msg.fields[:2]
    with pytest.raises(ValueError):
        cloud_to_xy_cm(msg)


def test_scan_goes_stale_and_never_shows_an_old_cloud_as_fresh():
    now = [0.0]
    scan = LatestScan(stale_sec=2.0, clock=lambda: now[0])
    assert scan.snapshot()['fresh'] is False
    scan.put(cloud([(1.0, 0.0, 0.0)]))
    now[0] = 0.5
    snap = scan.snapshot()
    assert snap['fresh'] is True and snap['xy'] == [100, 0] and snap['count'] == 1
    now[0] = 3.0
    snap = scan.snapshot()
    assert snap['fresh'] is False and snap['xy'] == []


def test_bad_cloud_reports_error_instead_of_points():
    scan = LatestScan(clock=lambda: 0.0)
    msg = cloud([(1.0, 0.0, 0.0)])
    msg.fields = []
    scan.put(msg)
    snap = scan.snapshot()
    assert snap['fresh'] is False and snap['error']
