"""Mission manager: ROS adapter over mission_state.MissionStateMachine.

  in   boat/mission_command   (boat_msgs/MissionCommand) route requests
  in   boat/mission_feedback  (std_msgs/String)          from waypoint_follower
  in   boat/mission_cancel    (std_msgs/String JSON)     identified operator cancel
  in   boat/estop             (std_msgs/Bool)
  out  boat/state             (std_msgs/String)          INIT/IDLE/EXECUTE/ABORTED
  out  boat/mission_active    (boat_msgs/MissionCommand) the accepted route

waypoint_follower listens to boat/mission_active, not boat/mission_command,
so only routes this node accepted are flown.
"""

import json
import uuid
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Bool
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
        self.status_pub = self.create_publisher(String, 'boat/mission_status', 10)
        # The accepted route. waypoint_follower subscribes to this.
        self.active_mission_pub = self.create_publisher(
            MissionCommand, 'boat/mission_active', 10)

        self.mission_sub = self.create_subscription(
            MissionCommand, 'boat/mission_command', self.mission_callback, 10)
        self.feedback_sub = self.create_subscription(
            String, 'boat/mission_feedback', self.feedback_callback, 10)
        self.cancel_sub = self.create_subscription(
            String, 'boat/mission_cancel', self.cancel_callback, 10)
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

    def _publish_state(self, request_id=None, accepted=None, reason=""):
        """Publish state on every transition, not only on the 1 Hz timer.

        waypoint_follower gates on boat/mission_status, so this avoids up to a
        second where an accepted mission still looks idle to it.
        """
        self.state_pub.publish(String(data=self.logic.state))
        self.status_pub.publish(String(data=json.dumps({
            'manager_id': self.logic.manager_id, 'generation': self.logic.generation,
            'execution_id': self.logic.execution_id, 'state': self.logic.state,
            'request_id': request_id, 'accepted': accepted, 'reason': reason})))

    # Subscriptions

    def finish_init(self):
        if self.logic.finish_init():
            self.get_logger().info('INIT checks complete -> transitioning to IDLE')
            self._publish_state()
        # One-shot timer.
        self.init_timer.cancel()

    def mission_callback(self, msg: MissionCommand):
        if (not msg.execution_id or len(msg.execution_id) > 80 or
                msg.manager_id != self.logic.manager_id or msg.generation != self.logic.generation):
            self._publish_state(msg.execution_id, False, 'Mission request superseded; refresh status')
            return
        if (not msg.latitudes or len(msg.latitudes) != len(msg.longitudes) or
                len(msg.latitudes) > 1000 or
                not all(math.isfinite(x) and -90 <= x <= 90 for x in msg.latitudes) or
                not all(math.isfinite(x) and -180 <= x <= 180 for x in msg.longitudes)):
            self._publish_state(msg.execution_id, False, 'Invalid waypoints')
            return
        accepted, reason = self.logic.request_mission(len(msg.latitudes), self._now(), uuid.uuid4().hex)
        self._publish_state(msg.execution_id, accepted, reason)
        if accepted:
            # Accepted route carries the new generation, not the request's old one.
            msg.generation = self.logic.generation
            msg.execution_id = self.logic.execution_id
            self.active_mission_pub.publish(msg)

    def feedback_callback(self, msg: String):
        try:
            feedback = json.loads(msg.data)
            if not isinstance(feedback, dict):
                return
        except (ValueError, TypeError):
            return
        changed, note = self.logic.on_feedback(feedback.get('event'), feedback.get('execution_id'))
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

    def cancel_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        if (data.get('manager_id') != self.logic.manager_id or
                data.get('generation') != self.logic.generation or
                not data.get('execution_id')):
            self._publish_state(data.get('request_id'), False, 'Cancel superseded; refresh status')
            return
        accepted, reason = self.logic.cancel(self._now(), data['execution_id'])
        self._publish_state(data.get('request_id'), accepted, reason)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
