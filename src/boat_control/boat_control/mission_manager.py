"""Mission manager: ROS adapter over mission_state.MissionStateMachine.

  in   boat/mission_command   (boat_msgs/MissionCommand) route requests
  in   boat/mission_feedback  (std_msgs/String)          from waypoint_follower
  in   boat/mission_cancel    (std_msgs/Empty)           operator cancel
  in   boat/estop             (std_msgs/Bool)
  out  boat/state             (std_msgs/String)          INIT/IDLE/EXECUTE/ABORTED
  out  boat/mission_active    (boat_msgs/MissionCommand) the accepted route

waypoint_follower listens to boat/mission_active, not boat/mission_command,
so only routes this node accepted are flown.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty
from boat_msgs.msg import MissionCommand

from boat_control.mission_state import (
    DEFAULT_MISSION_TIMEOUT_SEC,
    MissionStateMachine,
    REASON_BAD_STATE,
    REASON_EMPTY,
    REASON_ESTOP,
)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('mission_timeout_sec', DEFAULT_MISSION_TIMEOUT_SEC)
        self.logic = MissionStateMachine(
            timeout_sec=float(self.get_parameter('mission_timeout_sec').value))
        self.logic.set_paused(True, self._now())
        self.last_mode_at = None
        self.create_subscription(String, 'boat/mode', self.mode_callback, 10)

        self.state_pub = self.create_publisher(String, 'boat/state', 10)
        # The accepted route. waypoint_follower subscribes to this.
        self.active_mission_pub = self.create_publisher(
            MissionCommand, 'boat/mission_active', 10)

        self.mission_sub = self.create_subscription(
            MissionCommand, 'boat/mission_command', self.mission_callback, 10)
        self.feedback_sub = self.create_subscription(
            String, 'boat/mission_feedback', self.feedback_callback, 10)
        self.cancel_sub = self.create_subscription(
            Empty, 'boat/mission_cancel', self.cancel_callback, 10)
        # E-stop True aborts a running mission. False only clears the latch
        # so a new mission can be accepted; it resumes nothing.
        self.estop_sub = self.create_subscription(
            Bool, 'boat/estop', self.estop_callback, 10)

        self.timer = self.create_timer(1.0, self.publish_state)
        self.init_timer = self.create_timer(2.0, self.finish_init)

        self.get_logger().info(
            'Mission manager started in state: %s (mission timeout %.0fs)'
            % (self.logic.state, self.logic.timeout_sec))

    # Helpers

    def _now(self):
        return time.monotonic()

    def mode_callback(self, msg):
        self.last_mode_at = self._now()
        self.logic.set_paused(msg.data not in ('IDLE', 'WAYPOINT'), self.last_mode_at)

    def _publish_state(self):
        """Publish state on every transition, not only on the 1 Hz timer.

        waypoint_follower gates on boat/state, so this avoids up to a
        second where an accepted mission still looks idle to it.
        """
        self.state_pub.publish(String(data=self.logic.state))

    # Subscriptions

    def finish_init(self):
        if self.logic.finish_init():
            self.get_logger().info('INIT checks complete -> transitioning to IDLE')
            self._publish_state()
        # One-shot timer.
        self.init_timer.cancel()

    def mission_callback(self, msg: MissionCommand):
        # Message shape is checked here, mission logic in the state machine.
        if len(msg.latitudes) != len(msg.longitudes):
            self.get_logger().error(
                'Rejecting mission command: mismatched latitude/longitude '
                'array lengths (%d vs %d)'
                % (len(msg.latitudes), len(msg.longitudes)))
            return

        if (not all(math.isfinite(x) and -90 <= x <= 90 for x in msg.latitudes)
                or not all(math.isfinite(x) and -180 <= x <= 180 for x in msg.longitudes)):
            self.get_logger().error('Rejecting mission: invalid waypoint coordinates')
            return
        num_points = len(msg.latitudes)
        accepted, reason = self.logic.request_mission(num_points, self._now())

        if not accepted:
            if reason == REASON_ESTOP:
                self.get_logger().warn(
                    'Rejecting mission command: e-stop is active -- clear it first')
            elif reason == REASON_BAD_STATE:
                self.get_logger().warn(
                    'Rejecting mission command: state is %s, must be IDLE or '
                    'ABORTED. Cancel the running mission (boat/mission_cancel, '
                    'or the Cancel button) to get back to IDLE.' % self.logic.state)
            elif reason == REASON_EMPTY:
                self.get_logger().error(
                    'Rejecting mission command: empty waypoint list')
            return

        self.get_logger().info(
            'Mission accepted: %d waypoints -> EXECUTE' % num_points)
        # State before route, so waypoint_follower sees EXECUTE before it
        # loads the waypoints.
        self._publish_state()
        self.active_mission_pub.publish(msg)

    def feedback_callback(self, msg: String):
        changed, note = self.logic.on_feedback(msg.data)
        if note == 'complete':
            self.get_logger().info(
                'Navigation reports mission complete -> IDLE')
        elif note == 'aborted':
            self.get_logger().warn(
                'Navigation reports mission aborted (%s) -> ABORTED' % msg.data)
        elif note == 'progress':
            self.get_logger().info('Navigation feedback: %s' % msg.data)
        elif note == 'unrecognized':
            self.get_logger().warn('Unrecognized mission feedback: %s' % msg.data)
        if changed:
            self._publish_state()

    def cancel_callback(self, _msg: Empty):
        was = self.logic.state
        accepted, _reason = self.logic.cancel(self._now())
        if not accepted:
            self.get_logger().warn(
                'Ignoring cancel: still in %s' % self.logic.state)
            return
        self.get_logger().info('Mission cancelled by operator (%s -> IDLE)' % was)
        self._publish_state()

    def estop_callback(self, msg: Bool):
        changed = self.logic.on_estop(msg.data)
        if changed:
            self.get_logger().warn('E-stop triggered -> ABORTED')
            self._publish_state()
        elif not self.logic.estop_active:
            self.get_logger().info('E-stop cleared')

    # Periodic

    def publish_state(self):
        if self.last_mode_at is None or self._now() - self.last_mode_at > 1.0:
            self.logic.set_paused(True, self._now())
        elapsed = self.logic.tick(self._now())
        if elapsed is not None:
            self.get_logger().warn(
                'Mission has been EXECUTE-ing for %.0fs with no completion '
                '-> timing out to ABORTED' % elapsed)
        self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
