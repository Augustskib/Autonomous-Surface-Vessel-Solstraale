"""GPS node: reads NMEA GGA sentences from the serial GPS and publishes
sensor_msgs/NavSatFix on boat/gps/fix."""

import serial
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.port = port
        self.baud = baud
        self.ser = None
        self._last_reconnect_attempt = None

        self.publisher_ = self.create_publisher(NavSatFix, 'boat/gps/fix', 10)

        self._open_serial()

        self.timer = self.create_timer(0.2, self.read_gps)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f'Opened GPS serial port {self.port} at {self.baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

    def read_gps(self):
        if self.ser is None:
            # Retry opening the port about every 5 s.
            now = self.get_clock().now()
            if (self._last_reconnect_attempt is None or
                    (now - self._last_reconnect_attempt).nanoseconds > 5e9):
                self._last_reconnect_attempt = now
                self.get_logger().warn('GPS serial port not open, retrying...')
                self._open_serial()
            return
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}, will attempt to reopen port')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return

        if not line.startswith('$GPGGA') and not line.startswith('$GNGGA'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_frame'

        if msg.gps_qual and int(msg.gps_qual) > 0:
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude = msg.latitude
            fix.longitude = msg.longitude
            fix.altitude = float(msg.altitude) if msg.altitude else 0.0
            self.get_logger().info(
                f'GPS fix: lat={fix.latitude:.6f}, lon={fix.longitude:.6f}')
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX

        self.publisher_.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
