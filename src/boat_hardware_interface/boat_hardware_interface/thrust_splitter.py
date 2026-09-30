"""Simulation only: converts boat/thrust_command (percent) into per-propeller
force in newtons for the Gazebo Thruster plugins.

motor_driver does this job on the real boat.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from boat_msgs.msg import ThrustCommand

# Force in newtons at 100% thrust, per propeller. Against the surge drag in
# boat_world.sdf (8 linear + 12 quadratic across both pontoons) this gives a
# top speed of about 1.9 m/s (7 km/h). Drag lives in boat_simulation, so
# retune both together. Overridable with the max_thrust_n parameter.
DEFAULT_MAX_THRUST_N = 30.0

# Same stop-on-silence timeout as motor_driver.
COMMAND_TIMEOUT_SEC = 0.5

# Maximum force change per second. Jumping straight to full thrust while the
# sim boat was still settling after spawn crashed Gazebo, so commands ramp.
MAX_RAMP_N_PER_S = 30.0

# Ramp step period, independent of how often commands arrive.
RAMP_PERIOD_SEC = 0.05


class ThrustSplitter(Node):
    def __init__(self):
        super().__init__('thrust_splitter')

        self.declare_parameter('max_thrust_n', DEFAULT_MAX_THRUST_N)

        self.left_thrust_pub = self.create_publisher(Float64, 'boat/sim/left_thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, 'boat/sim/right_thrust', 10)

        self.subscription = self.create_subscription(
            ThrustCommand, 'boat/thrust_command', self.thrust_callback, 10)
        # E-stop cuts thrust immediately, without the watchdog delay.
        self.estop_sub = self.create_subscription(
            Bool, 'boat/estop', self.estop_callback, 10)

        self.estop_active = False
        self.last_command_time = None
        self._stopped_logged = True  # no zero-thrust log at startup

        # target = last command (N), current = published value ramping toward it.
        self.target_left = 0.0
        self.target_right = 0.0
        self.current_left = 0.0
        self.current_right = 0.0

        self.watchdog_timer = self.create_timer(0.1, self.check_watchdog)
        self.ramp_timer = self.create_timer(RAMP_PERIOD_SEC, self._ramp_step)

        self.get_logger().info(
            'Thrust splitter started (sim-only: converts boat/thrust_command '
            'percent -> ramped per-propeller Newtons for Gazebo Thruster plugins)')

    def thrust_callback(self, msg: ThrustCommand):
        self.last_command_time = self.get_clock().now()
        self._stopped_logged = False

        if self.estop_active:
            # Hold zero while e-stopped.
            self.target_left = 0.0
            self.target_right = 0.0
            return

        if not (math.isfinite(msg.left) and math.isfinite(msg.right)):
            self.target_left = self.target_right = 0.0
            self.current_left = self.current_right = 0.0
            self._publish_forces(0.0, 0.0)
            self.get_logger().error('Non-finite thrust command -> forcing stop')
            return
        max_thrust_n = float(self.get_parameter('max_thrust_n').value)
        left = max(-100.0, min(100.0, msg.left)) / 100.0
        right = max(-100.0, min(100.0, msg.right)) / 100.0
        self.target_left = left * max_thrust_n
        self.target_right = right * max_thrust_n

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn('E-stop active -> forcing stop')
            # E-stop skips the ramp.
            self.target_left = 0.0
            self.target_right = 0.0
            self.current_left = 0.0
            self.current_right = 0.0
            self._publish_forces(0.0, 0.0)

    def check_watchdog(self):
        if self.last_command_time is None or self._stopped_logged:
            return
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if elapsed > COMMAND_TIMEOUT_SEC:
            self.get_logger().warn(
                f'No thrust command received for {elapsed:.1f}s -> forcing stop')
            # A lost command source is a safety stop, so skip the ramp.
            self.target_left = 0.0
            self.target_right = 0.0
            self.current_left = 0.0
            self.current_right = 0.0
            self._publish_forces(0.0, 0.0)
            self._stopped_logged = True

    def _ramp_step(self):
        max_delta = MAX_RAMP_N_PER_S * RAMP_PERIOD_SEC
        self.current_left = self._step_toward(self.current_left, self.target_left, max_delta)
        self.current_right = self._step_toward(self.current_right, self.target_right, max_delta)
        self._publish_forces(self.current_left, self.current_right)

    @staticmethod
    def _step_toward(current, target, max_delta):
        if current < target:
            return min(current + max_delta, target)
        elif current > target:
            return max(current - max_delta, target)
        return current

    def _publish_forces(self, left_n, right_n):
        self.left_thrust_pub.publish(Float64(data=left_n))
        self.right_thrust_pub.publish(Float64(data=right_n))


def main(args=None):
    rclpy.init(args=args)
    node = ThrustSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
