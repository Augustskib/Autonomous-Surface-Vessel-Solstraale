"""Control arbiter: the only publisher of boat/thrust_command.

Manual control (boat/cmd/manual), the waypoint follower (boat/cmd/waypoint)
and the failsafe (boat/cmd/failsafe) each publish to their own topic. This
node picks one every tick and forwards it. motor_driver and thrust_splitter
only see boat/thrust_command.

The decision logic lives in arbitration.py so it can be tested without ROS.
This node converts ROS messages and clocks into calls on that class.

Priority: e-stop > manual lease HELD > failsafe > lease HOLD > waypoint > zero

Publishes boat/mode (IDLE, MANUAL, WAYPOINT, HOLD, FAILSAFE, ESTOP). If the
boat won't drive, check `ros2 topic echo boat/mode` first.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from boat_msgs.msg import FailsafeState, ThrustCommand

from boat_control.arbitration import (
    ControlArbitration,
    DEFAULT_FAILSAFE_TIMEOUT_SEC,
    DEFAULT_LEASE_TIMEOUT_SEC,
    DEFAULT_SOURCE_TIMEOUT_SEC,
)

# Re-evaluation rate. Also bounds how quickly a lease timeout is noticed.
ARBITER_RATE_HZ = 20.0


class ControlArbiter(Node):
    def __init__(self):
        super().__init__('control_arbiter')

        # Timeouts are parameters so they can be tuned on the bench, e.g.
        #   ros2 run boat_control control_arbiter --ros-args -p lease_timeout_sec:=2.0
        self.declare_parameter('source_timeout_sec', DEFAULT_SOURCE_TIMEOUT_SEC)
        self.declare_parameter('lease_timeout_sec', DEFAULT_LEASE_TIMEOUT_SEC)
        self.declare_parameter('failsafe_timeout_sec', DEFAULT_FAILSAFE_TIMEOUT_SEC)

        self.logic = ControlArbitration(
            source_timeout_sec=float(self.get_parameter('source_timeout_sec').value),
            lease_timeout_sec=float(self.get_parameter('lease_timeout_sec').value),
            failsafe_timeout_sec=float(
                self.get_parameter('failsafe_timeout_sec').value),
        )

        self.thrust_pub = self.create_publisher(ThrustCommand, 'boat/thrust_command', 10)
        self.mode_pub = self.create_publisher(String, 'boat/mode', 10)

        self.create_subscription(
            ThrustCommand, 'boat/cmd/manual', self._manual_callback, 10)
        self.create_subscription(
            ThrustCommand, 'boat/cmd/waypoint', self._waypoint_callback, 10)
        self.create_subscription(Bool, 'boat/estop', self._estop_callback, 10)
        # The manual lease: MANUAL / RELEASE / RESUME.
        self.create_subscription(
            String, 'boat/cmd/control_mode', self._control_mode_callback, 10)
        # Failsafe level and failsafe thrust are separate topics, since the
        # stop levels have authority but publish no thrust.
        self.create_subscription(
            ThrustCommand, 'boat/cmd/failsafe', self._failsafe_thrust_callback, 10)
        self.create_subscription(
            FailsafeState, 'boat/failsafe_state', self._failsafe_state_callback, 10)

        self._last_mode = None       # for change-logging only
        self._last_lease_state = None

        self.create_timer(1.0 / ARBITER_RATE_HZ, self._arbitrate)

        self.get_logger().info(
            'Control arbiter started -- sole publisher of boat/thrust_command. '
            'Priority: estop > manual-lease > failsafe > hold > waypoint > '
            'zero. Source timeout %.2fs, lease timeout %.2fs, failsafe level '
            'timeout %.2fs. Manual requires an explicit lease on '
            'boat/cmd/control_mode; thrust traffic alone no longer counts as '
            'manual control.'
            % (self.logic.source_timeout_sec, self.logic.lease_timeout_sec,
               self.logic.failsafe_timeout_sec))

    # Helpers

    def _now(self):
        """rclpy Time to float seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # Subscriptions

    def _manual_callback(self, msg: ThrustCommand):
        if not self.logic.on_manual_thrust(msg.left, msg.right, self._now()):
            self.get_logger().warn(
                'Discarded non-finite manual thrust command (left=%r right=%r)'
                % (msg.left, msg.right), throttle_duration_sec=5.0)

    def _waypoint_callback(self, msg: ThrustCommand):
        if not self.logic.on_waypoint_thrust(msg.left, msg.right, self._now()):
            self.get_logger().warn(
                'Discarded non-finite waypoint thrust command (left=%r right=%r)'
                % (msg.left, msg.right), throttle_duration_sec=5.0)

    def _failsafe_thrust_callback(self, msg: ThrustCommand):
        if not self.logic.on_failsafe_thrust(msg.left, msg.right, self._now()):
            self.get_logger().warn(
                'Discarded non-finite failsafe thrust command '
                '(left=%r right=%r)' % (msg.left, msg.right),
                throttle_duration_sec=5.0)

    def _failsafe_state_callback(self, msg: FailsafeState):
        self.logic.on_failsafe_level(msg.level, self._now())

    def _estop_callback(self, msg: Bool):
        self.logic.on_estop(msg.data)

    def _control_mode_callback(self, msg: String):
        command = (msg.data or '').strip().upper()
        if not self.logic.on_control_mode(command, self._now()):
            self.get_logger().warn(
                'Ignoring unrecognized control_mode command: %r '
                '(expected MANUAL, RELEASE or RESUME)' % msg.data,
                throttle_duration_sec=5.0)

    # Arbitration

    def _arbitrate(self):
        before = self.logic.lease_state
        before_failsafe = self.logic.failsafe_level
        left, right, mode = self.logic.evaluate(self._now())

        # A stale failsafe level means safety_monitor stopped publishing.
        # The arbiter fails open, which is worth a warning.
        if before_failsafe and not self.logic.failsafe_level:
            self.get_logger().error(
                'Failsafe level went STALE (no boat/failsafe_state for >%.1fs) '
                '-- falling back to normal arbitration. Is safety_monitor '
                'still running?' % self.logic.failsafe_timeout_sec)

        # Lease expiry: the operator's link dropped while driving. The boat
        # latches to zero and will not resume autonomy by itself.
        if before == 'HELD' and self.logic.lease_state == 'HOLD':
            self.get_logger().warn(
                'Manual lease EXPIRED (no heartbeat for >%.2fs) -- holding zero '
                'thrust. Autonomy will NOT resume by itself; press Resume Auto '
                'on the dashboard (or publish RESUME to boat/cmd/control_mode).'
                % self.logic.lease_timeout_sec)

        msg = ThrustCommand()
        msg.left = float(left)
        msg.right = float(right)
        self.thrust_pub.publish(msg)
        self.mode_pub.publish(String(data=mode))

        if mode != self._last_mode:
            self.get_logger().info('Control source -> %s' % mode)
            self._last_mode = mode
        if self.logic.lease_state != self._last_lease_state:
            self.get_logger().info('Manual lease -> %s' % self.logic.lease_state)
            self._last_lease_state = self.logic.lease_state


def main(args=None):
    rclpy.init(args=args)
    node = ControlArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
