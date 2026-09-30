"""Unit tests for waypoint_follower's geometry helpers: haversine_distance_m,
bearing_deg, normalize_angle_deg, yaw_from_quaternion and
compass_heading_from_yaw_deg. The node itself needs rclpy and is not
tested here."""
import math

import pytest

from boat_navigation.waypoint_follower import (
    EARTH_RADIUS_M,
    bearing_deg,
    compass_heading_from_yaw_deg,
    haversine_distance_m,
    normalize_angle_deg,
    yaw_from_quaternion,
)


# ---- haversine_distance_m ----

def test_haversine_same_point_is_zero():
    assert haversine_distance_m(28.5, -81.4, 28.5, -81.4) == pytest.approx(0.0, abs=1e-6)


def test_haversine_one_degree_latitude():
    # One degree of latitude is (pi/180) * EARTH_RADIUS_M on a sphere.
    expected = math.radians(1.0) * EARTH_RADIUS_M
    actual = haversine_distance_m(0.0, 0.0, 1.0, 0.0)
    assert actual == pytest.approx(expected, rel=1e-9)


def test_haversine_is_symmetric():
    d_forward = haversine_distance_m(28.5, -81.4, 28.6, -81.3)
    d_backward = haversine_distance_m(28.6, -81.3, 28.5, -81.4)
    assert d_forward == pytest.approx(d_backward, rel=1e-12)


def test_haversine_never_negative():
    assert haversine_distance_m(10.0, 20.0, -10.0, -20.0) > 0.0


# ---- bearing_deg ----

def test_bearing_due_north():
    assert bearing_deg(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_bearing_due_east():
    assert bearing_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=1e-6)


def test_bearing_due_south():
    assert bearing_deg(0.0, 0.0, -1.0, 0.0) == pytest.approx(180.0, abs=1e-6)


def test_bearing_due_west():
    assert bearing_deg(0.0, 0.0, 0.0, -1.0) == pytest.approx(270.0, abs=1e-6)


def test_bearing_always_in_0_360_range():
    # Includes directions outside [0, 360) before normalization.
    for dlat, dlon in [(1, 1), (1, -1), (-1, 1), (-1, -1), (0.001, -0.001)]:
        b = bearing_deg(10.0, 20.0, 10.0 + dlat, 20.0 + dlon)
        assert 0.0 <= b < 360.0


# ---- normalize_angle_deg ----

@pytest.mark.parametrize('angle,expected', [
    (0.0, 0.0),
    (180.0, 180.0),
    (181.0, -179.0),
    (-181.0, 179.0),
    (360.0, 0.0),
    (-360.0, 0.0),
    (359.0, -1.0),
    (-359.0, 1.0),
])
def test_normalize_angle_deg(angle, expected):
    assert normalize_angle_deg(angle) == pytest.approx(expected, abs=1e-9)


def test_normalize_angle_deg_always_in_range():
    for angle in range(-720, 721, 17):  # odd step to avoid always hitting boundaries
        result = normalize_angle_deg(float(angle))
        assert -180.0 < result <= 180.0


# ---- yaw_from_quaternion ----

def test_yaw_from_identity_quaternion_is_zero():
    # Identity quaternion is yaw 0 (facing east, ENU).
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_yaw_from_quaternion_positive_90_degrees():
    # 90 degree rotation about Z: (x=0, y=0, z=sin(45deg), w=cos(45deg))
    half = math.radians(45.0)
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert yaw == pytest.approx(math.pi / 2, abs=1e-9)


def test_yaw_from_quaternion_negative_90_degrees():
    half = math.radians(-45.0)
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert yaw == pytest.approx(-math.pi / 2, abs=1e-9)


def test_yaw_from_quaternion_180_degrees_either_sign_is_equivalent():
    # +-180 is the same orientation; atan2 may return either sign.
    yaw = yaw_from_quaternion(0.0, 0.0, 1.0, 0.0)
    assert abs(yaw) == pytest.approx(math.pi, abs=1e-9)


# ---- compass_heading_from_yaw_deg ----

def test_compass_heading_yaw_zero_is_east():
    assert compass_heading_from_yaw_deg(0.0) == pytest.approx(90.0, abs=1e-9)


def test_compass_heading_yaw_90_is_north():
    assert compass_heading_from_yaw_deg(math.pi / 2) == pytest.approx(0.0, abs=1e-9)


def test_compass_heading_yaw_negative_90_is_south():
    assert compass_heading_from_yaw_deg(-math.pi / 2) == pytest.approx(180.0, abs=1e-9)


def test_compass_heading_yaw_pi_is_west():
    assert compass_heading_from_yaw_deg(math.pi) == pytest.approx(270.0, abs=1e-9)


def test_compass_heading_always_in_0_360_range():
    for deg in range(-720, 721, 13):
        heading = compass_heading_from_yaw_deg(math.radians(float(deg)))
        assert 0.0 <= heading < 360.0


# ---- integration between bearing_deg/compass heading and steering ----

def test_heading_error_zero_when_facing_target():
    # Heading equal to the bearing must give zero heading error.
    target_bearing = bearing_deg(0.0, 0.0, 1.0, 1.0)
    heading_error = normalize_angle_deg(target_bearing - target_bearing)
    assert heading_error == pytest.approx(0.0, abs=1e-9)
