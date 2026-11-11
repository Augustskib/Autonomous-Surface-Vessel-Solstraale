"""Rate-limited LiDAR preview and independently expiring health for the UI."""
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from boat_perception.lidar import preview_cloud


class LidarPreview(Node):
    def __init__(self):
        super().__init__('lidar_preview')
        self.budget = self.declare_parameter('point_budget', 10000).value
        if not 1 <= self.budget <= 100000:
            raise ValueError('point_budget must be 1..100000')
        self.latest = None
        self.received = None
        self.driver = {}
        self.driver_received = None
        self.error = ''
        self.pub = self.create_publisher(PointCloud2, '/boat/lidar/points_preview', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/boat/lidar/status', 10)
        self.create_subscription(PointCloud2, '/boat/lidar/points_raw', self.cloud, qos_profile_sensor_data)
        self.create_subscription(String, '/boat/lidar/driver_status', self.driver_status, 10)
        self.create_timer(0.2, self.tick)

    def driver_status(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.driver = value
                self.driver_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def cloud(self, msg):
        try:
            self.latest = preview_cloud(msg, self.budget)
            self.received = time.monotonic()
            self.error = ''
        except ValueError as exc:
            self.latest = None
            self.error = str(exc)

    def tick(self):
        now = time.monotonic()
        age = None if self.received is None else now - self.received
        fresh = age is not None and age < 1.0 and not self.error
        driver_fresh = self.driver_received is not None and now-self.driver_received < 2.5
        if fresh and self.latest is not None:
            self.pub.publish(self.latest)
            self.latest = None  # Never republish a frozen cloud as a new scan.
        self.status.publish(String(data=json.dumps({
            'fresh': fresh, 'age_sec': age, 'error': self.error,
            'timestamp_mode': self.driver.get('timestamp_mode', 'unknown'),
            'clock_verified': driver_fresh and self.driver.get('clock_verified', False),
            'driver_status_fresh': driver_fresh,
            'point_budget': self.budget, 'avoidance_enabled': False})))


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
