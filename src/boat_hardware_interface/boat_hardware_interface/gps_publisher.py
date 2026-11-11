"""GPS node: reads NMEA GGA sentences from the serial GPS and publishes
sensor_msgs/NavSatFix on boat/gps/fix."""

import math
import time
from datetime import datetime, timezone

import serial
import pynmea2
import rclpy
from rclpy.executors import ExternalShutdownException
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
        self._rx_buf = b''
        self._discard_line = False
        self.ser = None
        self._last_reconnect_attempt = None

        self.publisher_ = self.create_publisher(NavSatFix, 'boat/gps/fix', 10)

        self._open_serial()

        self.timer = self.create_timer(0.2, self.read_gps)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.ser.reset_input_buffer()
            self._rx_buf = b''
            self._discard_line = False
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
            # Drain a bounded snapshot, not one sentence per timer tick. Discard
            # overload rather than stamp an old queue as live navigation.
            count = self.ser.in_waiting
            if count > 16384:
                self.ser.reset_input_buffer()
                self._rx_buf = b''
                self._discard_line = True
                self.get_logger().warn('GPS backlog discarded; waiting for fresh fix')
                return
            chunk = self.ser.read(count) if count else b''
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}, will attempt to reopen port')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._rx_buf = b''
            return

        if self._discard_line:
            if b'\n' not in chunk:
                return
            chunk = chunk.split(b'\n', 1)[1]
            self._discard_line = False
        lines = (self._rx_buf + chunk).split(b'\n')
        self._rx_buf = lines.pop()
        if len(self._rx_buf) > 256:
            self._rx_buf = b''
            self._discard_line = True
        latest = None
        for raw in lines:
            if len(raw) > 256:
                continue
            try:
                line = raw.decode('ascii').strip()
                if not line.startswith(('$GPGGA,', '$GNGGA,')):
                    continue
                msg = pynmea2.parse(line, check=True)
                # GGA supplies UTC time-of-day, not date. Compare with the
                # nearest host UTC day; host clock must be synchronized.
                stamp = msg.timestamp
                utc = datetime.fromtimestamp(time.time(), timezone.utc)
                host_seconds = (utc.hour * 3600 + utc.minute * 60 + utc.second
                                + utc.microsecond / 1e6)
                gps_seconds = (stamp.hour * 3600 + stamp.minute * 60 + stamp.second
                               + stamp.microsecond / 1e6)
                age = (host_seconds - gps_seconds + 43200) % 86400 - 43200
                if age < -0.5 or age > 2.0:
                    self.get_logger().warn(
                        'GPS UTC timestamp is stale or ahead of host; check Pi clock synchronization',
                        throttle_duration_sec=5.0)
                    continue
                quality = int(msg.gps_qual or 0)
                lat, lon = float(msg.latitude), float(msg.longitude)
                alt = float(msg.altitude or 0.0)
                if quality > 0 and (not all(map(math.isfinite, (lat, lon, alt)))
                                   or not -90 <= lat <= 90 or not -180 <= lon <= 180):
                    continue
                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                # Publish acquisition age, not the age of this callback.
                ns = fix.header.stamp.sec * 1000000000 + fix.header.stamp.nanosec
                ns -= int(max(0.0, age) * 1000000000)
                fix.header.stamp.sec, fix.header.stamp.nanosec = divmod(ns, 1000000000)
                fix.header.frame_id = 'gps_frame'
                fix.status.status = (NavSatStatus.STATUS_FIX if quality > 0
                                     else NavSatStatus.STATUS_NO_FIX)
                if quality > 0:
                    fix.latitude, fix.longitude, fix.altitude = lat, lon, alt
                latest = fix
            except (pynmea2.ParseError, ValueError, TypeError, AttributeError, UnicodeError):
                continue
        if latest is not None:
            self.publisher_.publish(latest)


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
