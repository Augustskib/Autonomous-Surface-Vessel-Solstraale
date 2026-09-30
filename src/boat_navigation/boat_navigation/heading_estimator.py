"""Heading estimator: publishes the one heading estimate everything else uses.

ROS adapter over heading.py (the filter).

  in   boat/imu       (sensor_msgs/Imu)           yaw rate, and absolute orientation in sim
  in   boat/gps/fix   (sensor_msgs/NavSatFix)     position, for course and speed
  out  boat/heading   (boat_msgs/HeadingEstimate) 20 Hz

Unit and sign: Imu.angular_velocity.z is rad/s counter-clockwise (ENU),
headings here are degrees clockwise from north, so
    rate_dps = -degrees(msg.angular_velocity.z)
A wrong sign makes the heading lag every turn and snap back afterwards.
test_heading_logic.py covers it.

If a driver leaves angular_velocity at zero but the orientation changes,
the rate is derived from the quaternion yaw instead (logged once).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

from boat_msgs.msg import HeadingEstimate

from boat_navigation.heading import (
    DEFAULT_ALPHA,
    DEFAULT_ASSUMED_DRIFT_DPS,
    DEFAULT_COG_MIN_SPEED_MPS,
    DEFAULT_FUSED_TIMEOUT_SEC,
    DEFAULT_MAX_COAST_SEC,
    DEFAULT_STATIONARY_SPEED_MPS,
    HeadingEstimator as HeadingFilter,
    course_over_ground_deg,
    normalize_angle_deg,
)

PUBLISH_RATE_HZ = 20.0

EARTH_RADIUS_M = 6371000.0

# Same speed guards as waypoint_follower: duplicate timestamps and duplicate
# publishers produce absurd speeds.
MIN_FIX_DT_SEC = 0.01
MAX_FIX_DT_SEC = 5.0
MAX_PLAUSIBLE_SPEED_MPS = 15.0

# Minimum movement between fixes for their course to mean anything. Extra
# check on top of the filter's speed gate, for long gaps between fixes.
MIN_FIX_STEP_M = 0.3

# Ground speed EMA weight, same as waypoint_follower so both show the same speed.
SPEED_EMA_ALPHA = 0.25

ZERO_RATE_EPSILON = 1e-9


def haversine_distance_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def yaw_from_quaternion(x, y, z, w):
    """Yaw (radians) from an ENU quaternion. 0 = east, counter-clockwise positive."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compass_heading_from_yaw_deg(yaw_rad):
    """ENU yaw to compass heading (0 = north, clockwise)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class HeadingEstimatorNode(Node):
    def __init__(self):
        super().__init__('heading_estimator')

        # Filter constants are parameters for bench tuning, e.g.
        #   ros2 run boat_navigation heading_estimator --ros-args -p cog_min_speed_mps:=0.8
        self.declare_parameter('alpha', DEFAULT_ALPHA)
        self.declare_parameter('cog_min_speed_mps', DEFAULT_COG_MIN_SPEED_MPS)
        self.declare_parameter('stationary_speed_mps', DEFAULT_STATIONARY_SPEED_MPS)
        self.declare_parameter('fused_timeout_sec', DEFAULT_FUSED_TIMEOUT_SEC)
        self.declare_parameter('max_coast_sec', DEFAULT_MAX_COAST_SEC)
        self.declare_parameter('assumed_drift_dps', DEFAULT_ASSUMED_DRIFT_DPS)

        # Simulation only. Gazebo's IMU orientation is absolute; the real
        # Modulino's is gyro-integrated from power-on, so using it as a
        # reference would give a confident, wrong heading. The sim launch
        # file sets this True; bringup sets it False explicitly.
        self.declare_parameter('imu_orientation_is_absolute', False)

        self.filter = HeadingFilter(
            alpha=float(self.get_parameter('alpha').value),
            cog_min_speed_mps=float(self.get_parameter('cog_min_speed_mps').value),
            stationary_speed_mps=float(self.get_parameter('stationary_speed_mps').value),
            fused_timeout_sec=float(self.get_parameter('fused_timeout_sec').value),
            max_coast_sec=float(self.get_parameter('max_coast_sec').value),
            assumed_drift_dps=float(self.get_parameter('assumed_drift_dps').value),
        )

        self.heading_pub = self.create_publisher(
            HeadingEstimate, 'boat/heading', 10)

        self.create_subscription(Imu, 'boat/imu', self.imu_callback, 10)
        self.create_subscription(
            NavSatFix, 'boat/gps/fix', self.gps_callback, 10)

        self._prev_fix = None
        self._prev_fix_time = None
        self._speed_mps = 0.0

        self._prev_imu_yaw_deg = None
        self._prev_imu_time = None
        self._warned_derived_rate = False
        self._last_quality = None

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_heading)

        self.get_logger().info(
            'Heading estimator started -- publishing boat/heading at %.0f Hz. '
            'COG accepted above %.2f m/s; below that the estimate coasts on '
            'the gyro and is marked DEAD_RECKONING. IMU orientation treated '
            'as absolute: %s. NOTE: this is COURSE, not bow heading -- the '
            'two diverge in a current (see heading.py).'
            % (PUBLISH_RATE_HZ,
               self.filter.cog_min_speed_mps,
               self.get_parameter('imu_orientation_is_absolute').value))

    # Subscriptions

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg: Imu):
        now = self._now()

        # rad/s counter-clockwise to deg/s clockwise: scale and negate.
        rate_dps = -math.degrees(msg.angular_velocity.z)

        yaw_deg = compass_heading_from_yaw_deg(yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w))

        if abs(msg.angular_velocity.z) < ZERO_RATE_EPSILON:
            derived = self._rate_from_orientation(yaw_deg, now)
            if derived is not None:
                rate_dps = derived
                if not self._warned_derived_rate:
                    self.get_logger().warn(
                        'boat/imu reports zero angular velocity while its '
                        'orientation quaternion is changing -- deriving the '
                        'yaw rate by differentiating the quaternion instead. '
                        'That is noisier than reading the gyro directly; '
                        'check whether the IMU driver is actually populating '
                        'angular_velocity.')
                    self._warned_derived_rate = True

        self._prev_imu_yaw_deg = yaw_deg
        self._prev_imu_time = now

        self.filter.on_gyro_rate(rate_dps, now)

        if self.get_parameter('imu_orientation_is_absolute').value:
            # Simulation (or a future magnetometer AHRS).
            self.filter.on_absolute_reference(yaw_deg, now, source='SIM')

    def _rate_from_orientation(self, yaw_deg, now):
        if self._prev_imu_yaw_deg is None or self._prev_imu_time is None:
            return None
        dt = now - self._prev_imu_time
        if not (1e-4 < dt < 1.0):
            return None
        return normalize_angle_deg(yaw_deg - self._prev_imu_yaw_deg) / dt

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:            # STATUS_NO_FIX
            return
        # Ignore (0, 0) fixes, emitted before the GPS has a real solution.
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        now = self._now()
        fix = (msg.latitude, msg.longitude)

        if self._prev_fix is None or self._prev_fix_time is None:
            self._prev_fix, self._prev_fix_time = fix, now
            return

        dt = now - self._prev_fix_time
        if not (MIN_FIX_DT_SEC < dt < MAX_FIX_DT_SEC):
            self._prev_fix, self._prev_fix_time = fix, now
            return

        step_m = haversine_distance_m(
            self._prev_fix[0], self._prev_fix[1], fix[0], fix[1])
        inst_speed = step_m / dt

        if inst_speed > MAX_PLAUSIBLE_SPEED_MPS:
            self.get_logger().warn(
                'Implausible GPS jump: %.1f m in %.2f s (%.1f m/s) -- '
                'discarding' % (step_m, dt, inst_speed),
                throttle_duration_sec=5.0)
            self._prev_fix, self._prev_fix_time = fix, now
            return

        self._speed_mps = (SPEED_EMA_ALPHA * inst_speed
                           + (1.0 - SPEED_EMA_ALPHA) * self._speed_mps)
        self.filter.on_ground_speed(self._speed_mps)

        # Only offer a course when the boat moved far enough between fixes.
        if step_m >= MIN_FIX_STEP_M:
            cog = course_over_ground_deg(
                self._prev_fix[0], self._prev_fix[1], fix[0], fix[1])
            self.filter.on_cog(cog, self._speed_mps, now)

        self._prev_fix, self._prev_fix_time = fix, now

    # Output

    def publish_heading(self):
        solution = self.filter.evaluate(self._now())

        msg = HeadingEstimate()
        msg.quality = solution.quality
        msg.heading_deg = (float(solution.heading_deg)
                           if solution.heading_deg is not None else math.nan)
        msg.source = solution.source
        msg.confidence = float(solution.confidence)
        msg.age_since_reference_sec = float(solution.age_since_reference_sec)
        msg.gyro_bias_dps = float(solution.gyro_bias_dps)
        msg.cog_valid = bool(solution.cog_valid)
        msg.ground_speed_mps = float(solution.ground_speed_mps)
        msg.cog_deg = float(solution.cog_deg)
        msg.absolute_reference_available = bool(
            solution.absolute_reference_available)
        msg.detail = solution.detail
        self.heading_pub.publish(msg)

        # Log quality transitions, not every tick.
        if solution.quality != self._last_quality:
            log = (self.get_logger().info if solution.quality >= 2
                   else self.get_logger().warn)
            log('Heading quality -> %s%s'
                % (solution.quality_name,
                   (' (%s)' % solution.detail) if solution.detail else ''))
            self._last_quality = solution.quality


def main(args=None):
    rclpy.init(args=args)
    node = HeadingEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
