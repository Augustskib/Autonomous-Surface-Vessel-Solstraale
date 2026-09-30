"""Motor driver: forwards boat/thrust_command to the Arduino over serial.

Also parses the IMU lines the Arduino streams back and publishes boat/imu,
since this node owns the only handle on the serial port. See
firmware/boat_motor_controller for the protocol.
"""

import math
import re

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from boat_msgs.msg import ThrustCommand

from boat_hardware_interface.imu_publisher import euler_to_quaternion, GRAVITY_MPS2

# Stop if no boat/thrust_command arrives for this long. The Arduino has its
# own 500 ms watchdog for the case where the Pi or the serial link dies.
COMMAND_TIMEOUT_SEC = 0.5

# Thrust is percent, -100..100 (see ThrustCommand.msg).
THRUST_MIN = -100.0
THRUST_MAX = 100.0

# Must match SERIAL_BAUD in the firmware.
DEFAULT_BAUD = 115200

# Retry period when the serial port is missing or lost.
RECONNECT_PERIOD_SEC = 5.0

# The Arduino sends an IMU line every 20 ms.
SERIAL_READ_PERIOD_SEC = 0.02

# Matches the firmware IMU line. Accel in g, gyro rates in deg/s.
IMU_LINE_RE = re.compile(
    r'^IMU AX:(?P<ax>-?\d+\.?\d*) AY:(?P<ay>-?\d+\.?\d*) AZ:(?P<az>-?\d+\.?\d*) '
    r'GR:(?P<gr>-?\d+\.?\d*) GP:(?P<gp>-?\d+\.?\d*) GY:(?P<gy>-?\d+\.?\d*)$'
)

IMU_FRAME_ID = 'imu_frame'

# Complementary filter weight, same as imu_publisher's MPU6050 path.
# Yaw is gyro integration only and drifts.
IMU_COMPLEMENTARY_ALPHA = 0.98


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # With more than one USB-serial device attached, use a stable path
        # from /dev/serial/by-id/ instead of /dev/ttyACM0.
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', DEFAULT_BAUD)
        # mock: log commands instead of opening a serial port.
        self.declare_parameter('mock', False)
        self.declare_parameter('max_thrust_pct', 100.0)
        self.declare_parameter('start_estopped', False)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.mock = bool(self.get_parameter('mock').value)

        self.subscription = self.create_subscription(
            ThrustCommand, 'boat/thrust_command', self.thrust_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, 'boat/estop', self.estop_callback, 10)

        self.max_thrust_pct = float(self.get_parameter('max_thrust_pct').value)
        if not math.isfinite(self.max_thrust_pct) or not 0 <= self.max_thrust_pct <= 100:
            raise ValueError('max_thrust_pct must be between 0 and 100')
        self.estop_active = bool(self.get_parameter('start_estopped').value)
        self.last_command_time = None
        self.watchdog_timer = self.create_timer(0.1, self.check_watchdog)
        self._stopped_logged = True  # no zero-thrust log at startup

        # IMU relayed from the Arduino (Modulino Movement on its Qwiic port).
        self.imu_pub = self.create_publisher(Imu, 'boat/imu', 10)
        self._imu_roll = 0.0
        self._imu_pitch = 0.0
        self._imu_yaw = 0.0
        self._imu_last_time = None
        self._serial_rx_buf = b''

        self.ser = None
        self._last_reconnect_attempt = None
        if not self.mock:
            self._open_serial()
            self.reconnect_timer = self.create_timer(
                RECONNECT_PERIOD_SEC, self._maybe_reconnect)
            # No serial link in mock mode, so no boat/imu either.
            self.serial_read_timer = self.create_timer(
                SERIAL_READ_PERIOD_SEC, self._read_serial)

        if self.mock:
            self.get_logger().info(
                'Motor driver started in MOCK mode (mock:=true) -- logging '
                'commands instead of writing to a real Arduino.')
        else:
            self.get_logger().info(
                f'Motor driver started, targeting Arduino on {self.port} '
                f'@ {self.baud} baud')

    # Serial

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1, exclusive=True)
            self._serial_rx_buf = b''
            self.get_logger().info(f'Opened motor controller serial port {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(
                f'Failed to open motor controller serial port {self.port}: {e} '
                f'-- will retry every {RECONNECT_PERIOD_SEC:.0f}s. Commands will '
                f'be accepted and clamped but NOT sent to any motor until this '
                f'succeeds.')
            self.ser = None

    def _maybe_reconnect(self):
        if self.ser is not None:
            return
        self.get_logger().warn('Motor controller serial port not open, retrying...')
        self._open_serial()

    def _write_serial(self, left, right):
        """Write the command to the Arduino. Never raises: on failure the
        port is closed and the reconnect timer reopens it."""
        if self.mock or self.ser is None:
            return
        line = f'L:{int(round(left))} R:{int(round(right))}\n'
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Serial write to motor controller failed: {e} -- '
                f'closing port, will attempt to reopen')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ROS callbacks

    def estop_callback(self, msg):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn('E-stop active -> forcing stop')
            self._write_serial(0.0, 0.0)

    def thrust_callback(self, msg):
        self.last_command_time = self.get_clock().now()

        if self.estop_active:
            self._stopped_logged = True
            self._write_serial(0.0, 0.0)
            return

        if not (math.isfinite(msg.left) and math.isfinite(msg.right)):
            self.get_logger().error('Non-finite thrust command -> forcing stop')
            self._write_serial(0.0, 0.0)
            self._stopped_logged = True
            return

        limit = self.max_thrust_pct
        left = max(-limit, min(limit, msg.left))
        right = max(-limit, min(limit, msg.right))
        if left != msg.left or right != msg.right:
            self.get_logger().warn(
                f'Thrust command out of range, clamped to left={left:.0f}, right={right:.0f}')

        self._stopped_logged = False
        if self.mock:
            self.get_logger().info(f'[MOCK] Would send to Arduino -> left={left:.0f}, right={right:.0f}')
        else:
            if self.ser is None:
                self.get_logger().warn(
                    'Motor controller serial port not open -- command NOT sent to hardware',
                    throttle_duration_sec=5.0)
            self._write_serial(left, right)

    def check_watchdog(self):
        if self.last_command_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if elapsed > COMMAND_TIMEOUT_SEC and not self._stopped_logged:
            self.get_logger().warn(
                f'No thrust command received for {elapsed:.1f}s -> forcing stop (left=0, right=0)')
            self._write_serial(0.0, 0.0)
            self._stopped_logged = True

    # IMU relayed from the Arduino

    def _read_serial(self):
        """Read whatever serial bytes are buffered and handle complete lines."""
        if self.mock or self.ser is None:
            return
        try:
            # Bounded read so a continuous IMU stream can't starve the thrust
            # callback or the watchdog. Partial lines are kept for next time.
            count = min(self.ser.in_waiting, 4096)
            if count:
                self._serial_rx_buf += self.ser.read(count)
                lines = self._serial_rx_buf.split(b'\n')
                self._serial_rx_buf = lines.pop()
                for line in lines:
                    self._handle_serial_line(line)
                if len(self._serial_rx_buf) > 4096:
                    self._serial_rx_buf = b''
                    self.get_logger().warn('Discarded oversized serial telemetry line')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Serial read from motor controller failed: {e} -- '
                f'closing port, will attempt to reopen')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _handle_serial_line(self, raw_line):
        try:
            line = raw_line.decode('ascii', errors='ignore').strip()
        except Exception:
            return
        if line.startswith('IMU '):
            self._handle_imu_line(line)
        # STATUS lines are ignored for now. Current comes from power_monitor.

    def _handle_imu_line(self, line):
        match = IMU_LINE_RE.match(line)
        if match is None:
            self.get_logger().warn(
                f'Unparseable IMU line from motor controller: {line!r}',
                throttle_duration_sec=5.0)
            return

        ax = float(match.group('ax'))
        ay = float(match.group('ay'))
        az = float(match.group('az'))
        # deg/s to rad/s (sensor_msgs/Imu convention).
        gx = math.radians(float(match.group('gr')))
        gy = math.radians(float(match.group('gp')))
        gz = math.radians(float(match.group('gy')))

        now = self.get_clock().now()
        if self._imu_last_time is None:
            dt = 0.0
        else:
            dt = max(0.0, min(0.5, (now - self._imu_last_time).nanoseconds / 1e9))
        self._imu_last_time = now

        # Complementary filter for roll/pitch, gyro integration for yaw.
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if dt > 0.0:
            gyro_roll = self._imu_roll + gx * dt
            gyro_pitch = self._imu_pitch + gy * dt
            self._imu_roll = (IMU_COMPLEMENTARY_ALPHA * gyro_roll
                               + (1 - IMU_COMPLEMENTARY_ALPHA) * accel_roll)
            self._imu_pitch = (IMU_COMPLEMENTARY_ALPHA * gyro_pitch
                                + (1 - IMU_COMPLEMENTARY_ALPHA) * accel_pitch)
            self._imu_yaw += gz * dt
        else:
            self._imu_roll, self._imu_pitch = accel_roll, accel_pitch

        orientation = euler_to_quaternion(self._imu_roll, self._imu_pitch, self._imu_yaw)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = IMU_FRAME_ID
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orientation
        # Derived orientation, so a loose covariance. Not measured.
        msg.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.3]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.linear_acceleration.x = ax * GRAVITY_MPS2
        msg.linear_acceleration.y = ay * GRAVITY_MPS2
        msg.linear_acceleration.z = az * GRAVITY_MPS2
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_pub.publish(msg)

    def destroy_node(self):
        if self.ser is not None:
            try:
                self._write_serial(0.0, 0.0)
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
