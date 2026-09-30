"""Safety supervisor: geofence and ground-control link.

Runs as its own process, separate from mission_manager and
waypoint_follower, so a navigation crash can't take the safety checks down.
ROS adapter over failsafe.py, which holds the logic.

Outputs:
  boat/failsafe_state    level and reason, for the dashboard and the bag
  boat/cmd/failsafe      thrust while a level is driving (RETURN_TO_LAUNCH),
                         a third input to control_arbiter
  boat/mission_feedback  one MISSION_ABORTED on escalation, so the waypoint
                         follower stops competing for the boat

A geofence breach or link loss raises a failsafe level instead of the
e-stop, so the boat can drive itself home and the operator can still drive.
An operator holding the manual lease outranks the failsafe. The e-stop is
still an absolute override and only the dashboard publishes it.

Link loss: POWERED_STOP at first (usually transient), RETURN_TO_LAUNCH
after gcs_rtl_after_sec.

The fence defaults to the simulation origin (Lake Eola). Set it for the
real site from the dashboard, or with boat/geofence_command:

    ros2 topic pub --once boat/geofence_command std_msgs/msg/String \\
      '{data: "{\\"action\\": \\"set_here\\", \\"radius_m\\": 120.0}"}'

Every change is recorded on boat/data_events. Disabling the fence requires
a reason, which stays visible in SafetyStatus.fence_disabled_reason.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import NavSatFix
from boat_msgs.msg import (
    FailsafeState,
    HeadingEstimate,
    SafetyStatus,
    ThrustCommand,
)

from boat_control.failsafe import (
    DEFAULT_BREACH_SAMPLES,
    DEFAULT_GCS_RTL_AFTER_SEC,
    DEFAULT_RTL_ARRIVAL_RADIUS_M,
    DEFAULT_RTL_SPEED_PCT,
    FailsafeSupervisor,
    GeofenceConfig,
    LEVEL_NAMES,
    LEVEL_NOMINAL,
    LEVEL_RETURN_TO_LAUNCH,
    QUALITY_USABLE_MIN,
)

STATE_EXECUTE = 'EXECUTE'

# boat/heading older than this counts as absent (it publishes at 20 Hz).
# Freshness is checked separately from the estimate's own quality.
HEADING_STALE_SEC = 1.0
GPS_STALE_SEC = 2.0


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Geofence. Defaults match the simulation world origin (Lake Eola)
        # with a small 150 m radius. Wrong for any real site: set it from
        # the dashboard before running on the water.
        self.declare_parameter('geofence_enabled', True)
        self.declare_parameter('geofence_center_lat', 28.543611)
        self.declare_parameter('geofence_center_lon', -81.372778)
        self.declare_parameter('geofence_radius_m', 150.0)
        # Warning ring as a fraction of the radius. Published as a radius
        # so the map draws the same ring.
        self.declare_parameter('geofence_warn_fraction', 0.9)
        self.declare_parameter('geofence_breach_samples', DEFAULT_BREACH_SAMPLES)

        # Launch point for RETURN_TO_LAUNCH. NaN means not set; the first
        # valid GPS fix captures it.
        self.declare_parameter('launch_lat', math.nan)
        self.declare_parameter('launch_lon', math.nan)

        # Failsafe behaviour
        self.declare_parameter('gcs_rtl_after_sec', DEFAULT_GCS_RTL_AFTER_SEC)
        self.declare_parameter('rtl_speed_pct', DEFAULT_RTL_SPEED_PCT)
        self.declare_parameter('rtl_arrival_radius_m', DEFAULT_RTL_ARRIVAL_RADIUS_M)
        # Turns off the acting half of the failsafe while keeping the
        # reporting. For debugging only; it is logged at startup.
        self.declare_parameter('failsafe_actions_enabled', True)

        # Ground-control link
        self.declare_parameter('gcs_failsafe_enabled', True)
        # The dashboard polls about once a second, so 10 s is ten missed beats.
        self.declare_parameter('gcs_timeout_sec', 10.0)

        self.logic = FailsafeSupervisor(
            fence=self._fence_from_params(),
            breach_samples=int(self.get_parameter('geofence_breach_samples').value),
            gcs_rtl_after_sec=float(self.get_parameter('gcs_rtl_after_sec').value),
            rtl_speed_pct=float(self.get_parameter('rtl_speed_pct').value),
            rtl_arrival_radius_m=float(
                self.get_parameter('rtl_arrival_radius_m').value),
        )

        # Publishers. This node does not publish boat/estop; the dashboard
        # is its only publisher. It still subscribes to it.
        self.status_pub = self.create_publisher(
            SafetyStatus, 'boat/safety_status', 10)
        self.failsafe_pub = self.create_publisher(
            FailsafeState, 'boat/failsafe_state', 10)
        # Third arbiter input.
        self.failsafe_cmd_pub = self.create_publisher(
            ThrustCommand, 'boat/cmd/failsafe', 10)
        # MISSION_ABORTED on the existing feedback topic; mission_manager
        # moves to ABORTED.
        self.feedback_pub = self.create_publisher(
            String, 'boat/mission_feedback', 10)
        # Geofence changes are recorded in the bag too.
        self.events_pub = self.create_publisher(String, 'boat/data_events', 10)

        # Subscriptions
        self.create_subscription(NavSatFix, 'boat/gps/fix', self.gps_callback, 10)
        self.create_subscription(String, 'boat/state', self.state_callback, 10)
        self.create_subscription(
            Empty, 'boat/gcs_heartbeat', self.heartbeat_callback, 10)
        self.create_subscription(Bool, 'boat/estop', self.estop_callback, 10)
        # Heading decides whether the boat can return to launch or only
        # stop. Without heading_estimator running, every RTL degrades to
        # POWERED_STOP with the reason stated.
        self.create_subscription(
            HeadingEstimate, 'boat/heading', self.heading_callback, 10)
        self.create_subscription(
            String, 'boat/geofence_command', self.geofence_command_callback, 10)
        self.create_subscription(
            String, 'boat/failsafe_clear', self.failsafe_clear_callback, 10)

        self.current_fix = None
        self._last_fix_time = None
        self.boat_state = None
        self.last_heartbeat_time = None
        self._gcs_armed = False
        self._last_heading_time = None
        self._last_heading = None
        self._last_level = LEVEL_NOMINAL
        self._last_decision = None
        self._abort_sent_for_level = LEVEL_NOMINAL
        self._launch_auto_captured = False

        launch = self._launch_from_params()
        if launch is not None:
            self.logic.set_launch_point(*launch)

        self.create_timer(0.5, self.check)           # 2 Hz safety checks
        self.create_timer(1.0, self.publish_status)  # 1 Hz status for the UI

        fence = self.logic.fence
        self.get_logger().info(
            'Safety monitor started -- geofence %s (%.1fm radius, %.1fm warning '
            'ring, around %.6f, %.6f), GCS failsafe %s (%.1fs timeout, RTL '
            'after %.0fs). Failsafe ACTIONS %s. Breach is level-triggered and '
            'responds with RETURN_TO_LAUNCH when heading is usable, '
            'POWERED_STOP when it is not.'
            % ('ON' if fence.enabled else 'OFF',
               fence.radius_m, fence.warn_radius_m,
               fence.center_lat, fence.center_lon,
               'ON' if self.get_parameter('gcs_failsafe_enabled').value else 'OFF',
               float(self.get_parameter('gcs_timeout_sec').value),
               self.logic.gcs_rtl_after_sec,
               'ENABLED' if self.get_parameter('failsafe_actions_enabled').value
               else 'DISABLED (reporting only -- the boat will NOT be steered '
                    'by the failsafe)'))

    # Parameter helpers

    def _fence_from_params(self):
        p = self.get_parameter
        return GeofenceConfig(
            enabled=bool(p('geofence_enabled').value),
            center_lat=float(p('geofence_center_lat').value),
            center_lon=float(p('geofence_center_lon').value),
            radius_m=float(p('geofence_radius_m').value),
            warn_fraction=float(p('geofence_warn_fraction').value),
            disabled_reason=getattr(self, '_fence_disabled_reason', ''),
        )

    def _launch_from_params(self):
        lat = float(self.get_parameter('launch_lat').value)
        lon = float(self.get_parameter('launch_lon').value)
        if math.isnan(lat) or math.isnan(lon):
            return None
        return (lat, lon)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # Subscriptions

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:  # STATUS_NO_FIX
            self.current_fix = None
            return
        # Ignore (0, 0). GPS drivers emit it briefly before a real fix, and
        # it used to trip the geofence at startup.
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().warn(
                'Ignoring (0, 0) GPS fix -- treating as not-yet-valid',
                throttle_duration_sec=10.0)
            return
        self._last_fix_time = self._now()
        self.current_fix = (msg.latitude, msg.longitude)
        self.logic.on_position(msg.latitude, msg.longitude)

        if self.logic.launch_point is None and not self._launch_auto_captured:
            self.logic.set_launch_point(msg.latitude, msg.longitude)
            self._launch_auto_captured = True
            self.get_logger().info(
                'Launch point auto-set to the first valid fix: %.6f, %.6f. '
                'RETURN_TO_LAUNCH will steer here. Move it from the dashboard '
                'if the boat was powered on somewhere other than the launch '
                'site.' % (msg.latitude, msg.longitude))

    def state_callback(self, msg: String):
        self.boat_state = msg.data

    def heartbeat_callback(self, _msg: Empty):
        self.last_heartbeat_time = self.get_clock().now()

    def estop_callback(self, msg: Bool):
        self.logic.on_estop(bool(msg.data))

    def heading_callback(self, msg: HeadingEstimate):
        self._last_heading = msg
        self._last_heading_time = self._now()

    def geofence_command_callback(self, msg: String):
        """Runtime fence changes from the dashboard.

        Actions: set, set_here, enable, disable, set_launch, set_launch_here.
        """
        try:
            cmd = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn('Malformed boat/geofence_command (not JSON)')
            return

        if not isinstance(cmd, dict):
            self.get_logger().warn('geofence_command must be a JSON object')
            return
        action = cmd.get('action')
        try:
            handled = self._apply_geofence_command(action, cmd)
        except Exception as exc:  # noqa: BLE001 - a bad command must not
            # take the safety node down.
            self.get_logger().error('geofence_command %r failed: %s'
                                    % (action, exc))
            return
        if not handled:
            self.get_logger().warn(
                'Unknown geofence_command action: %r' % action)

    def _apply_geofence_command(self, action, cmd):
        p = self.get_parameter
        set_params = []

        if action in ('set', 'set_here'):
            if action == 'set_here':
                if self.current_fix is None:
                    self.get_logger().error(
                        'geofence set_here refused: no valid GPS fix yet')
                    return True
                lat, lon = self.current_fix
            else:
                lat = float(cmd.get('center_lat', p('geofence_center_lat').value))
                lon = float(cmd.get('center_lon', p('geofence_center_lon').value))
            radius = float(cmd.get('radius_m', p('geofence_radius_m').value))
            warn = float(cmd.get('warn_fraction', p('geofence_warn_fraction').value))
            if not (math.isfinite(lat) and math.isfinite(lon)
                    and math.isfinite(radius) and radius > 1.0):
                self.get_logger().error(
                    'geofence set refused: non-finite or absurd values '
                    '(lat=%r lon=%r radius=%r)' % (lat, lon, radius))
                return True
            set_params = [
                rclpy.parameter.Parameter(
                    'geofence_center_lat',
                    rclpy.Parameter.Type.DOUBLE, lat),
                rclpy.parameter.Parameter(
                    'geofence_center_lon',
                    rclpy.Parameter.Type.DOUBLE, lon),
                rclpy.parameter.Parameter(
                    'geofence_radius_m',
                    rclpy.Parameter.Type.DOUBLE, radius),
                rclpy.parameter.Parameter(
                    'geofence_warn_fraction',
                    rclpy.Parameter.Type.DOUBLE, warn),
                rclpy.parameter.Parameter(
                    'geofence_enabled', rclpy.Parameter.Type.BOOL, True),
            ]
            self._fence_disabled_reason = ''
            self.get_logger().info(
                'Geofence set: %.1fm radius (warning ring at %.1fm) around '
                '%.6f, %.6f' % (radius, radius * warn, lat, lon))
            self._emit_event({'event': 'geofence_set', 'center_lat': lat,
                              'center_lon': lon, 'radius_m': radius,
                              'warn_fraction': warn})

        elif action == 'enable':
            set_params = [rclpy.parameter.Parameter(
                'geofence_enabled', rclpy.Parameter.Type.BOOL, True)]
            self._fence_disabled_reason = ''
            self.get_logger().info('Geofence ENABLED')
            self._emit_event({'event': 'geofence_enabled'})

        elif action == 'disable':
            reason = str(cmd.get('reason', '')).strip()
            if not reason:
                self.get_logger().error(
                    'geofence disable refused: a reason is required. '
                    'A geofence disabled without a recorded reason is one '
                    'nobody remembers disabling.')
                return True
            set_params = [rclpy.parameter.Parameter(
                'geofence_enabled', rclpy.Parameter.Type.BOOL, False)]
            self._fence_disabled_reason = reason
            self.get_logger().error(
                'GEOFENCE DISABLED by operator: %s -- the boat now has NO '
                'boundary. This is reported continuously on '
                'boat/safety_status and banner-flagged on the dashboard.'
                % reason)
            self._emit_event({'event': 'geofence_disabled', 'reason': reason})

        elif action in ('set_launch', 'set_launch_here'):
            if action == 'set_launch_here':
                if self.current_fix is None:
                    self.get_logger().error(
                        'launch set_here refused: no valid GPS fix yet')
                    return True
                lat, lon = self.current_fix
            else:
                lat = float(cmd.get('lat'))
                lon = float(cmd.get('lon'))
            if not (math.isfinite(lat) and math.isfinite(lon)):
                self.get_logger().error('launch point refused: non-finite')
                return True
            set_params = [
                rclpy.parameter.Parameter(
                    'launch_lat', rclpy.Parameter.Type.DOUBLE, lat),
                rclpy.parameter.Parameter(
                    'launch_lon', rclpy.Parameter.Type.DOUBLE, lon),
            ]
            self.logic.set_launch_point(lat, lon)
            self._launch_auto_captured = True
            self.get_logger().info(
                'Launch point set to %.6f, %.6f -- RETURN_TO_LAUNCH will '
                'steer here' % (lat, lon))
            self._emit_event({'event': 'launch_point_set',
                              'lat': lat, 'lon': lon})
        else:
            return False

        if set_params:
            self.set_parameters(set_params)
        self.logic.set_fence(self._fence_from_params())
        return True

    def failsafe_clear_callback(self, msg: String):
        """Operator clear. Payload is JSON:

            {"force": false}                     normal clear
            {"force": true, "reason": "..."}     override, reason required

        A normal clear is refused while a condition is still active.
        """
        try:
            cmd = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, TypeError):
            cmd = {}
        if not isinstance(cmd, dict):
            self.get_logger().warn('failsafe_clear must be a JSON object')
            return
        now = self._now()

        if cmd.get('force'):
            reason = str(cmd.get('reason', '')).strip()
            if not reason:
                self.get_logger().error(
                    'Forced failsafe clear refused: a typed reason is '
                    'required, and is recorded into the bag.')
                return
            ok, why = self.logic.force_clear(now)
            if ok:
                self.get_logger().error(
                    'FAILSAFE FORCE-CLEARED by operator with a condition still '
                    'active. Reason: %s' % reason)
                self._emit_event({'event': 'failsafe_force_cleared',
                                  'reason': reason})
            else:
                self.get_logger().warn('Forced clear refused: %s' % why)
            return

        ok, why = self.logic.clear(now)
        if ok:
            self.get_logger().info('Failsafe cleared by operator')
            self._emit_event({'event': 'failsafe_cleared'})
            self._abort_sent_for_level = LEVEL_NOMINAL
        else:
            self.get_logger().warn('Failsafe clear refused: %s' % why)

    # Checks

    def check(self):
        """The 2 Hz tick. One evaluate(), then act on what it said."""
        now = self._now()

        self._feed_heading(now)
        self.logic.on_gcs_link(self._gcs_link_lost(), now)
        if self._last_fix_time is None or now - self._last_fix_time > GPS_STALE_SEC:
            self.current_fix = None
        if self.current_fix is None:
            self.logic.on_position_lost()

        decision = self.logic.evaluate(now)
        self._last_decision = decision

        actions_enabled = bool(
            self.get_parameter('failsafe_actions_enabled').value)

        # Announce level changes
        if decision.level != self._last_level:
            log = (self.get_logger().error if decision.level > LEVEL_NOMINAL
                   else self.get_logger().info)
            log('FAILSAFE %s -> %s (%s)%s'
                % (LEVEL_NAMES.get(self._last_level, '?'),
                   decision.level_name,
                   decision.reason or 'conditions clear',
                   (' [DEGRADED: %s]' % decision.degraded_reason)
                   if decision.degraded_reason else ''))
            if decision.degraded_reason:
                # Degradation also logged on its own at ERROR.
                self.get_logger().error(
                    'FAILSAFE DEGRADED: asked for %s, doing %s instead -- %s'
                    % (LEVEL_NAMES.get(decision.requested_level, '?'),
                       decision.level_name, decision.degraded_reason))
            self._emit_event({'event': 'failsafe_level',
                              'level': decision.level_name,
                              'reason': decision.reason,
                              'degraded': decision.degraded_reason})
            self._last_level = decision.level

        # Abort the running mission once per escalation, so the waypoint
        # follower stops publishing while the failsafe drives.
        if (actions_enabled
                and decision.level > LEVEL_NOMINAL
                and self._abort_sent_for_level == LEVEL_NOMINAL
                and self.boat_state == STATE_EXECUTE):
            self.feedback_pub.publish(String(
                data='MISSION_ABORTED FAILSAFE_%s' % (decision.reason or 'UNKNOWN')))
            self.get_logger().warn(
                'Aborting the running mission because the failsafe escalated '
                'to %s' % decision.level_name)
            self._abort_sent_for_level = decision.level
        elif decision.level == LEVEL_NOMINAL:
            self._abort_sent_for_level = LEVEL_NOMINAL

        # Drive if the level says to
        if actions_enabled and decision.commanding_thrust:
            cmd = ThrustCommand()
            cmd.left = float(decision.left)
            cmd.right = float(decision.right)
            self.failsafe_cmd_pub.publish(cmd)
            self.get_logger().warn(
                'RETURN TO LAUNCH: %.0fm to go, bearing %.0f deg, thrust '
                '%.0f/%.0f' % (decision.distance_to_launch_m,
                               decision.bearing_to_launch_deg,
                               decision.left, decision.right),
                throttle_duration_sec=5.0)
        elif actions_enabled and decision.level == LEVEL_RETURN_TO_LAUNCH:
            # Arrived but latched at RTL: keep publishing zero so the
            # failsafe source stays fresh and keeps outranking waypoints.
            self.failsafe_cmd_pub.publish(ThrustCommand(left=0.0, right=0.0))

        self._publish_failsafe_state(decision)

    def _feed_heading(self, now):
        """Turn the latest boat/heading into heading_usable plus a reason.

        A stale message means the estimator is gone. A fresh message with
        quality NONE means it is running but has no heading. Both are
        unusable, reported differently.
        """
        msg = self._last_heading
        if msg is None or self._last_heading_time is None:
            self.logic.on_heading_lost()
            return
        age = now - self._last_heading_time
        if age > HEADING_STALE_SEC:
            self.logic.on_heading(None, False, 0,
                                  'boat/heading is %.1fs stale -- is '
                                  'heading_estimator running?' % age)
            return
        usable = msg.quality >= QUALITY_USABLE_MIN
        self.logic.on_heading(
            msg.heading_deg if math.isfinite(msg.heading_deg) else None,
            usable, msg.quality,
            msg.detail or ('heading quality %d' % msg.quality))

    def _gcs_link_lost(self):
        if not self.get_parameter('gcs_failsafe_enabled').value:
            self._gcs_armed = False
            return False

        # Link-loss check arms when a mission runs and a heartbeat has been
        # seen, and stays armed until the link is healthy. So a CLI run
        # without a dashboard is never tripped, and the check doesn't
        # disarm itself when the abort moves the state out of EXECUTE.
        if self.boat_state == STATE_EXECUTE and self.last_heartbeat_time is not None:
            self._gcs_armed = True

        if not self._gcs_armed or self.last_heartbeat_time is None:
            return False

        age = (self.get_clock().now() - self.last_heartbeat_time).nanoseconds / 1e9
        lost = age > float(self.get_parameter('gcs_timeout_sec').value)

        # Healthy link and no mission: disarm, so closing the browser while
        # idle is not a trip.
        if not lost and self.boat_state != STATE_EXECUTE:
            self._gcs_armed = False
        return lost

    # Status

    def _publish_failsafe_state(self, decision):
        msg = FailsafeState()
        msg.level = decision.level
        msg.level_name = decision.level_name
        msg.reason = decision.reason
        msg.active_reasons = list(decision.active_reasons)
        msg.requested_level = decision.requested_level
        msg.degraded_reason = decision.degraded_reason
        msg.commanding_thrust = decision.commanding_thrust
        msg.commanded_left = float(decision.left)
        msg.commanded_right = float(decision.right)

        launch = self.logic.launch_point
        msg.launch_point_valid = launch is not None
        msg.launch_lat = launch[0] if launch else math.nan
        msg.launch_lon = launch[1] if launch else math.nan
        msg.distance_to_launch_m = float(decision.distance_to_launch_m)
        msg.bearing_to_launch_deg = float(decision.bearing_to_launch_deg)

        msg.position_valid = self.logic.position is not None
        msg.heading_usable = self.logic.heading_usable
        msg.heading_quality = int(self.logic.heading_quality)
        msg.heading_deg = (float(self.logic.heading_deg)
                           if self.logic.heading_deg is not None else math.nan)
        msg.latched = decision.latched
        msg.seconds_at_level = float(decision.seconds_at_level)
        self.failsafe_pub.publish(msg)

    def publish_status(self):
        fence = self.logic.fence
        ev = self.logic.last_fence
        decision = self._last_decision

        heartbeat_age = math.nan
        if self.last_heartbeat_time is not None:
            heartbeat_age = round(
                (self.get_clock().now() - self.last_heartbeat_time).nanoseconds / 1e9, 1)

        tripped = bool(decision and decision.level > LEVEL_NOMINAL)
        reason = decision.reason if decision else ''

        status = SafetyStatus()
        # One summary for every consumer. ERROR: failsafe active. WARN:
        # inside the warning ring or fence disabled. OK: clear.
        status.level = (SafetyStatus.ERROR if tripped
                        else SafetyStatus.WARN
                        if (ev.warning or not fence.enabled)
                        else SafetyStatus.OK)
        status.tripped = tripped
        status.reason = reason or ''
        status.fence_enabled = fence.enabled
        status.fence_center_lat = fence.center_lat
        status.fence_center_lon = fence.center_lon
        status.fence_radius_m = fence.radius_m
        status.fence_warn_radius_m = fence.warn_radius_m
        status.fence_warn_fraction = fence.warn_fraction
        status.fence_distance_m = math.nan
        status.fence_margin_m = math.nan
        if ev.evaluated:
            status.fence_distance_m = round(ev.distance_m, 1)
            # Room left before the boundary.
            status.fence_margin_m = round(ev.margin_m, 1)
        status.fence_breached = ev.breached
        status.fence_warning = ev.warning
        status.fence_outside_sec = (round(ev.outside_sec, 1) if ev.evaluated
                                    else math.nan)
        status.fence_disabled_reason = fence.disabled_reason
        status.gcs_lost = self.logic.gcs_lost
        status.gcs_heartbeat_age_sec = heartbeat_age
        self.status_pub.publish(status)

        if ev.warning:
            self.get_logger().warn(
                'Approaching geofence: %.1fm from centre, warning ring at '
                '%.1fm, limit %.1fm'
                % (ev.distance_m, fence.warn_radius_m, fence.radius_m),
                throttle_duration_sec=3.0)
        if not fence.enabled:
            self.get_logger().warn(
                'GEOFENCE IS DISABLED%s -- the boat has no boundary'
                % ((': ' + fence.disabled_reason) if fence.disabled_reason else ''),
                throttle_duration_sec=15.0)

    def _emit_event(self, payload):
        """Record a safety-relevant change on boat/data_events, which the
        recorder captures in the bag."""
        self.events_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
