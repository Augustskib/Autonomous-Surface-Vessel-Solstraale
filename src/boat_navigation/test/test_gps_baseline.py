"""GpsBaseline: GPS jitter must not become speed or course. Plain Python."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from boat_navigation.heading import GpsBaseline  # noqa: E402

LAT0, LON0 = 28.5404, -81.3790
M_PER_DEG_LAT = 111320.0


def offset(north_m, east_m):
    return (LAT0 + north_m / M_PER_DEG_LAT,
            LON0 + east_m / (M_PER_DEG_LAT * math.cos(math.radians(LAT0))))


def test_stationary_jitter_gives_no_course_and_zero_speed():
    rng = random.Random(1)
    b = GpsBaseline()
    for t in range(600):
        lat, lon = offset(rng.gauss(0, 2.5), rng.gauss(0, 2.5))
        speed, cog = b.update(lat, lon, float(t), sigma_m=8.7)
        assert cog is None and speed == 0.0


def test_real_motion_gives_course_and_speed():
    b = GpsBaseline()
    out = [b.update(*offset(1.0 * t, 0.0), float(t), sigma_m=3.0) for t in range(20)]
    courses = [(s, c) for s, c in out if c is not None]
    assert courses
    for speed, cog in courses:
        assert abs(speed - 1.0) < 0.05
        assert min(cog, 360 - cog) < 1.0  # north


def test_east_motion_course_is_90():
    b = GpsBaseline()
    out = [b.update(*offset(0.0, 2.0 * t), float(t)) for t in range(10)]
    cog = next(c for s, c in out if c is not None)
    assert abs(cog - 90.0) < 1.0


def test_speed_is_held_while_moving_and_drops_after_stopping():
    b = GpsBaseline()
    out = [b.update(*offset(1.0 * t, 0.0), float(t), sigma_m=3.0) for t in range(30)]
    assert all(abs(s - 1.0) < 0.05 for s, _ in out[7:])
    for t in range(30, 60):
        speed, cog = b.update(*offset(29.0, 0.0), float(t), sigma_m=3.0)
        assert cog is None
    assert speed == 0.0


def test_threshold_follows_reported_noise():
    b = GpsBaseline(min_baseline_m=3.0)
    assert b.threshold_m(None) == 3.0
    assert b.threshold_m(1.0) == 3.0
    assert b.threshold_m(10.0) == 20.0
    # A 5 m step is motion with a clear sky, noise with a poor one.
    b.update(*offset(0, 0), 0.0, sigma_m=1.0)
    assert b.update(*offset(5, 0), 5.0, sigma_m=1.0)[1] is not None
    b.reset()
    b.update(*offset(0, 0), 0.0, sigma_m=8.0)
    assert b.update(*offset(5, 0), 5.0, sigma_m=8.0)[1] is None


def test_slow_drift_inside_window_never_accumulates():
    b = GpsBaseline(min_baseline_m=3.0, window_sec=20.0)
    # 0.1 m/s creep: re-anchors every 20 s, never reaching 3 m.
    for t in range(200):
        assert b.update(*offset(0.1 * t, 0), float(t))[1] is None


def _node():
    """heading_estimator's GPS path with fake ROS, as the node runs it."""
    import ast
    import types
    from pathlib import Path
    from unittest.mock import Mock
    import boat_navigation.heading as heading
    path = Path(__file__).resolve().parents[1] / 'boat_navigation/heading_estimator.py'
    tree = ast.parse(path.read_text())
    keep = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef))
            or (isinstance(n, ast.ClassDef) and n.name == 'HeadingEstimatorNode')]
    for n in keep:
        if isinstance(n, ast.ClassDef):
            n.bases = []
    scope = dict(math=math, GpsBaseline=GpsBaseline, Node=object,
                 Imu=object, NavSatFix=object, HeadingEstimate=object)
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(path), 'exec'), scope)
    obj = object.__new__(scope['HeadingEstimatorNode'])
    obj.get_logger = Mock(return_value=Mock())
    obj.filter = heading.HeadingEstimator()
    obj.filter.on_cog = Mock()
    obj._prev_fix = obj._prev_fix_time = None
    obj._speed_mps = 0.0
    obj.baseline = GpsBaseline()
    clock = types.SimpleNamespace(t=0.0)
    obj._now = lambda: clock.t
    return obj, clock


def fix(lat, lon, sigma):
    import types
    return types.SimpleNamespace(status=types.SimpleNamespace(status=0), latitude=lat,
                                 longitude=lon, position_covariance_type=1,
                                 position_covariance=[sigma ** 2, 0, 0, 0, sigma ** 2, 0, 0, 0, 1])


def test_node_gives_no_course_from_balcony_jitter():
    rng = random.Random(7)
    node, clock = _node()
    for t in range(300):
        clock.t = float(t)
        node.gps_callback(fix(*offset(rng.gauss(0, 3), rng.gauss(0, 3)), sigma=8.7))
    node.filter.on_cog.assert_not_called()
    assert node._speed_mps == 0.0


def test_node_gives_course_when_moving_on_open_water():
    node, clock = _node()
    for t in range(30):
        clock.t = float(t)
        node.gps_callback(fix(*offset(0.0, 1.5 * t), sigma=1.5))
    assert node.filter.on_cog.called
    cog = node.filter.on_cog.call_args.args[0]
    assert abs(cog - 90.0) < 1.0
