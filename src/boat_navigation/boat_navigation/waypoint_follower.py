"""Waypoint follower: steers the boat along the active mission's waypoints.

Reads GPS position and the accepted route (boat/mission_active), and
publishes differential thrust on boat/cmd/waypoint for control_arbiter.

Heading comes from boat/heading (heading_estimator). The control loop only
steers on a fresh estimate with quality DEAD_RECKONING or better, and holds
zero thrust otherwise. With no magnetometer that means the operator drives
manually first until COG gives a heading. COG is the direction of travel,
which is fine for waypoint following but not for docking.

steer_gain, base_speed_pct and waypoint_radius_m are parameters, not tuned yet:
    ros2 run boat_navigation waypoint_follower --ros-args -p steer_gain:=0.9
"""

import math

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from sensor_msgs.msg import NavSatFix, Imu
from foxglove_msgs.msg import GeoJSON
from boat_msgs.msg import (
    HeadingEstimate, MissionCommand, NavStatus, ThrustCommand)

EARTH_RADIUS_M = 6371000.0

# boat/heading older than this is not used (it publishes at 20 Hz).
HEADING_STALE_SEC = 1.0

# Trail points kept for boat/mission_geojson (over 30 min at 1 Hz).
MAX_TRAIL_POINTS = 2000

# A waypoint counts as reached within this distance. It must be larger than
# the boat's turning radius at approach speed (speed / yaw rate), or an
# overshoot can never be corrected and the boat orbits. Slowing down on
# approach (~0.75 m/s, ~1.2 m radius) is what makes 2.5 m work. Check that
# again if you change it (parameter waypoint_radius_m).
DEFAULT_WAYPOINT_RADIUS_M = 2.5

# Start slowing down this far from the target, which shrinks the turning
# circle and avoids overshooting.
APPROACH_SLOWDOWN_M = 15.0

# Minimum approach throttle as a fraction of base speed. Steering comes from
# differential thrust, so the boat needs some way on to turn.
MIN_APPROACH_SPEED_FRACTION = 0.35

# Fly-by detection: once within this distance, if the distance grows again
# past the hysteresis, the waypoint counts as reached instead of being chased
# in circles.
WAYPOINT_APPROACH_M = 25.0
WAYPOINT_PASS_HYSTERESIS_M = 2.0

# Steering gain: turn thrust = gain * heading error (deg), clamped to +/-100.
# Parameter steer_gain.
DEFAULT_STEER_GAIN = 100.0 / 150.0  # full turn authority at 150 degrees of error

# Base forward thrust in percent. Parameter base_speed_pct.
DEFAULT_BASE_SPEED_PCT = 60.0

# Minimum forward component at large heading error, so the boat always
# translates instead of pivoting in place.
MIN_FORWARD_FRACTION = 0.35

# Stop and abort if no usable fix for this long during a mission. This is
# wall-clock time: the stack doesn't use sim time, and the simulator on WSL2
# has stalled for nearly 4 s, so 3 s tripped falsely. Switching to sim time
# is the proper fix.
GPS_LOSS_TIMEOUT_SEC = 10.0

# Ground speed EMA weight. Speed comes from consecutive GPS fixes, so it is
# noisy; this smooths the dashboard readout and ETA.
SPEED_EMA_ALPHA = 0.25

# Below this speed there is no ETA.
MIN_SPEED_FOR_ETA_MPS = 0.05

STATE_EXECUTE = 'EXECUTE'
STATE_ABORTED = 'ABORTED'

# Position jumps implying more than this speed are treated as GPS glitches
# and dropped (e.g. two simulators publishing to one topic).
MAX_PLAUSIBLE_SPEED_MPS = 15.0


def haversine_distance_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees, 0=North, clockwise) from point 1 to point 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    theta = math.atan2(x, y)
    return (math.degrees(theta) + 360.0) % 360.0


def normalize_angle_deg(angle):
    """Normalize an angle (degrees) to the range (-180, 180]."""
    result = angle % 360.0
    if result > 180.0:
        result -= 360.0
    return result


def yaw_from_quaternion(x, y, z, w):
    """Yaw (radians) from an ENU quaternion. 0 = east, counter-clockwise positive."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compass_heading_from_yaw_deg(yaw_rad):
    """ENU yaw (0 = east, CCW) to compass heading (0 = north, clockwise)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Tuning parameters, read every tick so `ros2 param set` applies live.
        self.declare_parameter('steer_gain', DEFAULT_STEER_GAIN)
        self.declare_parameter('base_speed_pct', DEFAULT_BASE_SPEED_PCT)
        self.declare_parameter('waypoint_radius_m', DEFAULT_WAYPOINT_RADIUS_M)

        self.current_fix = None       # (lat, lon) of most recent GPS fix
        self.last_fix_time = None     # time of the last usable fix
        self._gps_lost_reported = False

        # Heading comes from boat/heading. See the module docstring.
        self._heading_msg = None          # latest HeadingEstimate
        self._heading_msg_time = None     # float seconds
        self._imu_fallback_heading_deg = None
        self._warned_imu_fallback = False
        self._last_heading_quality = None

        # Ground speed from GPS deltas. Display only (speed readout and ETA).
        self.speed_mps = 0.0
        self._prev_fix = None
        self._prev_fix_time = None

        self.waypoints = []           # list of (lat, lon)
        self.current_waypoint_idx = 0
        self.mission_active = False
        # Whether EXECUTE has been seen for the current mission (see control_loop).
        self._seen_execute = False
        # Closest approach to the current waypoint, for fly-by detection.
        self._closest_dist = float('inf')

        self.boat_state = None

        # [lon, lat] pairs (GeoJSON order), oldest first, for the map trail.
        self.trail_coords = []

        self.gps_sub = self.create_subscription(
            NavSatFix, 'boat/gps/fix', self.gps_callback, 10)
        # The heading estimate.
        self.heading_sub = self.create_subscription(
            HeadingEstimate, 'boat/heading', self.heading_callback, 10)
        # Raw IMU yaw, kept for reference only; not used for steering.
        self.imu_sub = self.create_subscription(
            Imu, 'boat/imu', self.imu_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'boat/state', self.state_callback, 10)
        # The route mission_manager accepted. Not boat/mission_command, so a
        # rejected route never gets flown.
        self.mission_sub = self.create_subscription(
            MissionCommand, 'boat/mission_active', self.mission_callback, 10)
        # Operator cancel: stop immediately instead of waiting for the next
        # state message.
        self.cancel_sub = self.create_subscription(
            Empty, 'boat/mission_cancel', self.cancel_callback, 10)

        # control_arbiter owns boat/thrust_command.
        self.thrust_pub = self.create_publisher(ThrustCommand, 'boat/cmd/waypoint', 10)
        self.feedback_pub = self.create_publisher(String, 'boat/mission_feedback', 10)
        # Trail and current target as GeoJSON for the Foxglove Map panel.
        self.geojson_pub = self.create_publisher(GeoJSON, 'boat/mission_geojson', 10)
        # Navigation status for the dashboard and Foxglove plots.
        self.nav_status_pub = self.create_publisher(NavStatus, 'boat/nav_status', 10)

        self.control_timer = self.create_timer(0.2, self.control_loop)  # 5 Hz
        self.geojson_timer = self.create_timer(1.0, self._publish_geojson)  # 1 Hz is plenty for viz
        self.nav_status_timer = self.create_timer(0.5, self._publish_nav_status)  # 2 Hz

        self.get_logger().info('Waypoint follower started')

    def gps_callback(self, msg: NavSatFix):
        if (msg.status.status < 0 or not math.isfinite(msg.latitude)
                or not math.isfinite(msg.longitude) or not -90 <= msg.latitude <= 90
                or not -180 <= msg.longitude <= 180):
            self.current_fix = None
            self._publish_thrust(0.0, 0.0)
            return
        now = self.get_clock().now()
        new_fix = (msg.latitude, msg.longitude)

        # Ground speed from the position delta, smoothed. Guarded against zero
        # or absurd dt, which would poison the average.
        if self._prev_fix is not None and self._prev_fix_time is not None:
            dt = (now - self._prev_fix_time).nanoseconds / 1e9
            if 0.01 < dt < 5.0:
                step_m = haversine_distance_m(
                    self._prev_fix[0], self._prev_fix[1], new_fix[0], new_fix[1])
                inst_speed = step_m / dt
                if inst_speed > MAX_PLAUSIBLE_SPEED_MPS:
                    self.get_logger().warn(
                        'Implausible GPS jump: %.1fm in %.2fs (%.1f m/s) -- '
                        'discarding sample' % (step_m, dt, inst_speed),
                        throttle_duration_sec=5.0)
                    return
                else:
                    self.speed_mps = (SPEED_EMA_ALPHA * inst_speed
                                      + (1.0 - SPEED_EMA_ALPHA) * self.speed_mps)
        self._prev_fix = new_fix
        self._prev_fix_time = now

        self.current_fix = new_fix
        self.last_fix_time = now
        self._gps_lost_reported = False  # fix is back

        self.trail_coords.append([msg.longitude, msg.latitude])
        if len(self.trail_coords) > MAX_TRAIL_POINTS:
            self.trail_coords.pop(0)

    def heading_callback(self, msg: HeadingEstimate):
        self._heading_msg = msg
        self._heading_msg_time = self.get_clock().now().nanoseconds / 1e9
        if msg.quality != self._last_heading_quality:
            log = (self.get_logger().info if msg.quality >= HeadingEstimate.FUSED
                   else self.get_logger().warn)
            log('Heading quality is now %d%s'
                % (msg.quality, (' -- %s' % msg.detail) if msg.detail else ''))
            self._last_heading_quality = msg.quality

    def imu_callback(self, msg: Imu):
        """Store raw IMU yaw. Not used for steering: it is gyro-integrated
        with no absolute reference and drifts."""
        q = msg.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._imu_fallback_heading_deg = compass_heading_from_yaw_deg(yaw)

    def current_heading_deg(self):
        """Use only a fresh, usable heading estimate; cached raw yaw is unsafe."""
        now = self.get_clock().now().nanoseconds / 1e9
        msg = self._heading_msg
        fresh = (msg is not None and self._heading_msg_time is not None
                 and (now - self._heading_msg_time) <= HEADING_STALE_SEC)

        if fresh:
            if msg.quality >= HeadingEstimate.DEAD_RECKONING and math.isfinite(msg.heading_deg):
                return msg.heading_deg, msg.quality < HeadingEstimate.FUSED
            # Fresh message that says the heading is unknown: don't steer.
            self.get_logger().warn(
                'Heading estimate is unusable (%s) -- steering suspended'
                % (msg.detail or 'quality NONE'),
                throttle_duration_sec=5.0)
            return None, True

        self.get_logger().warn(
            'boat/heading is absent or stale -- stopping until a usable heading returns',
            throttle_duration_sec=5.0)
        return None, True

    def state_callback(self, msg: String):
        self.boat_state = msg.data

    def mission_callback(self, msg: MissionCommand):
        if len(msg.latitudes) != len(msg.longitudes):
            self.get_logger().error(
                'MissionCommand has mismatched latitudes/longitudes array lengths, ignoring')
            return
        self.waypoints = list(zip(msg.latitudes, msg.longitudes))
        self.current_waypoint_idx = 0
        self.mission_active = len(self.waypoints) > 0
        self._gps_lost_reported = False
        # New mission: EXECUTE not seen yet (see control_loop).
        self._seen_execute = False
        self._closest_dist = float('inf')
        self.get_logger().info(f'New mission accepted: {len(self.waypoints)} waypoints')

    def cancel_callback(self, _msg: Empty):
        """Operator cancelled. Drop the route and stop driving immediately."""
        if not self.mission_active and not self.waypoints:
            return
        self.get_logger().info('Mission cancelled by operator -- clearing route')
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.mission_active = False
        self._seen_execute = False
        self._closest_dist = float('inf')
        self._publish_thrust(0.0, 0.0)

    def control_loop(self):
        if not self.mission_active:
            return
        if self.boat_state != STATE_EXECUTE:
            # Two different cases:
            #  (a) a mission was just accepted and boat/state hasn't reached
            #      EXECUTE here yet. Tearing it down would cancel it
            #      immediately and leave mission_manager stuck in EXECUTE.
            #  (b) a running mission was stopped elsewhere (abort, e-stop,
            #      timeout). Cut thrust now.
            # _seen_execute tells them apart: (a) waits, (b) stops.
            if self.mission_active and (self._seen_execute
                                        or self.boat_state == STATE_ABORTED):
                self._publish_thrust(0.0, 0.0)
                self.mission_active = False
                self._seen_execute = False
            return

        self._seen_execute = True
        if self.current_fix is None:
            self._publish_thrust(0.0, 0.0)
            self.get_logger().warn('No GPS fix yet, holding thrust at zero', throttle_duration_sec=5.0)
            return

        if self.last_fix_time is not None:
            elapsed = (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9
            if elapsed > GPS_LOSS_TIMEOUT_SEC:
                self._publish_thrust(0.0, 0.0)
                if not self._gps_lost_reported:
                    self.get_logger().error(
                        f'GPS fix lost for {elapsed:.1f}s -> stopping and aborting mission')
                    self.feedback_pub.publish(String(data='MISSION_ABORTED GPS_LOST'))
                    self.mission_active = False
                    self._gps_lost_reported = True
                return

        if self.current_waypoint_idx >= len(self.waypoints):
            self._finish_mission()
            return

        waypoint_radius_m = float(self.get_parameter('waypoint_radius_m').value)

        target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
        cur_lat, cur_lon = self.current_fix

        distance = haversine_distance_m(cur_lat, cur_lon, target_lat, target_lon)

        # Closest approach to the current waypoint, for fly-by detection.
        if distance < self._closest_dist:
            self._closest_dist = distance

        inside = distance <= waypoint_radius_m
        # Got close and now moving away: count it as reached.
        flew_by = (self._closest_dist <= WAYPOINT_APPROACH_M
                   and distance > self._closest_dist + WAYPOINT_PASS_HYSTERESIS_M)

        if inside or flew_by:
            self.get_logger().info(
                'Reached waypoint %d (%.6f, %.6f) -- %s, closest approach %.1fm'
                % (self.current_waypoint_idx, target_lat, target_lon,
                   'inside capture radius' if inside else 'passed it (fly-by)',
                   self._closest_dist))
            self.feedback_pub.publish(String(data=f'WAYPOINT_REACHED {self.current_waypoint_idx}'))
            self.current_waypoint_idx += 1
            self._closest_dist = float('inf')
            if self.current_waypoint_idx >= len(self.waypoints):
                self._finish_mission()
            return

        base_speed_pct = float(self.get_parameter('base_speed_pct').value)

        heading_deg, heading_degraded = self.current_heading_deg()
        if heading_deg is None:
            self.get_logger().warn('No usable heading -- holding thrust at zero',
                                   throttle_duration_sec=5.0)
            self._publish_thrust(0.0, 0.0)
            return

        steer_gain = float(self.get_parameter('steer_gain').value)
        if heading_degraded:
            # Gyro-only heading is drifting, so halve the steering authority.
            steer_gain *= 0.5

        target_bearing = bearing_deg(cur_lat, cur_lon, target_lat, target_lon)
        heading_error = normalize_angle_deg(target_bearing - heading_deg)

        turn = max(-100.0, min(100.0, steer_gain * heading_error))
        # Less forward thrust when off course, but never zero.
        forward = base_speed_pct * max(MIN_FORWARD_FRACTION, math.cos(math.radians(heading_error)))

        # Slow down on approach (turning radius = speed / yaw rate). Linear
        # from full speed at APPROACH_SLOWDOWN_M to MIN_APPROACH_SPEED_FRACTION.
        if distance < APPROACH_SLOWDOWN_M:
            forward *= max(MIN_APPROACH_SPEED_FRACTION, distance / APPROACH_SLOWDOWN_M)

        # Positive heading error (target to the right) makes the LEFT
        # thruster stronger, which swings the bow right.
        left = forward + turn
        right = forward - turn
        self._publish_thrust(left, right)

    def _publish_nav_status(self):
        """Publish NavStatus on boat/nav_status, computed fresh each time so
        values stay current when control_loop returns early. Fields that
        don't apply are NaN."""
        status = NavStatus()
        status.active = self.mission_active
        status.waypoint_index = self.current_waypoint_idx
        status.waypoint_count = len(self.waypoints)
        status.speed_mps = round(self.speed_mps, 3)
        status.target_lat = math.nan
        status.target_lon = math.nan
        status.distance_m = math.nan
        status.remaining_m = math.nan
        status.bearing_deg = math.nan
        status.heading_error_deg = math.nan
        status.eta_sec = math.nan
        status.gps_age_sec = math.nan

        if self.last_fix_time is not None:
            status.gps_age_sec = round(
                (self.get_clock().now() - self.last_fix_time).nanoseconds / 1e9, 2)

        have_target = (self.mission_active
                       and 0 <= self.current_waypoint_idx < len(self.waypoints))

        if have_target:
            target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
            status.target_lat = target_lat
            status.target_lon = target_lon

            if self.current_fix is not None:
                cur_lat, cur_lon = self.current_fix
                distance = haversine_distance_m(cur_lat, cur_lon, target_lat, target_lon)
                status.distance_m = round(distance, 2)

                bearing = bearing_deg(cur_lat, cur_lon, target_lat, target_lon)
                status.bearing_deg = round(bearing, 1)
                # Same heading the control loop steers on.
                heading_deg, _degraded = self.current_heading_deg()
                if heading_deg is not None:
                    status.heading_error_deg = round(
                        normalize_angle_deg(bearing - heading_deg), 1)

                # Remaining distance: current leg plus all queued legs.
                remaining = distance
                for i in range(self.current_waypoint_idx, len(self.waypoints) - 1):
                    a_lat, a_lon = self.waypoints[i]
                    b_lat, b_lon = self.waypoints[i + 1]
                    remaining += haversine_distance_m(a_lat, a_lon, b_lat, b_lon)
                status.remaining_m = round(remaining, 2)

                if self.speed_mps > MIN_SPEED_FOR_ETA_MPS:
                    status.eta_sec = round(remaining / self.speed_mps, 1)

        self.nav_status_pub.publish(status)

    def _publish_geojson(self):
        """Publish the trail and current target as a GeoJSON FeatureCollection
        on boat/mission_geojson, for the Foxglove Map panel."""
        features = []

        if len(self.trail_coords) >= 2:
            features.append({
                'type': 'Feature',
                'properties': {'stroke': '#2196f3', 'stroke-width': 3},
                'geometry': {'type': 'LineString', 'coordinates': self.trail_coords},
            })

        if self.mission_active and self.current_waypoint_idx < len(self.waypoints):
            target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
            features.append({
                'type': 'Feature',
                'properties': {'marker-color': '#e53935', 'marker-symbol': 'marker'},
                'geometry': {'type': 'Point', 'coordinates': [target_lon, target_lat]},
            })

        if not features:
            return

        msg = GeoJSON()
        msg.geojson = json.dumps({'type': 'FeatureCollection', 'features': features})
        self.geojson_pub.publish(msg)

    def _publish_thrust(self, left, right):
        msg = ThrustCommand()
        msg.left = max(-100.0, min(100.0, left))
        msg.right = max(-100.0, min(100.0, right))
        self.thrust_pub.publish(msg)

    def _finish_mission(self):
        self.get_logger().info('Mission complete -- all waypoints reached')
        self.feedback_pub.publish(String(data='MISSION_COMPLETE'))
        self.mission_active = False
        self._publish_thrust(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
