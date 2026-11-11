"""Validated inputs for an experimental position/yaw-rate EKF, never control."""
import copy
import json
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from boat_navigation.geodesy import enu


class FusionInputs(Node):
    def __init__(self):
        super().__init__('fusion_inputs')
        if not self.declare_parameter('calibration_verified', False).value:
            raise ValueError('Measured sensor transforms and noise settings must be verified first')
        self.datum = tuple(self.declare_parameter('datum', [float('nan')]*3).value)
        enu(*self.datum, self.datum)
        self.gps_var = self.declare_parameter('gps_variance_m2', 25.0).value
        self.gyro_var = self.declare_parameter('gyro_variance_rad2_s2', 0.01).value
        if not all(math.isfinite(v) and v > 0 for v in (self.gps_var, self.gyro_var)):
            raise ValueError('Noise variances must be finite and positive')
        self.gps_pub = self.create_publisher(Odometry, '/boat/localization/gps_enu', 10)
        self.imu_pub = self.create_publisher(Imu, '/boat/localization/imu', 10)
        self.status_pub = self.create_publisher(String, '/boat/localization/status', 10)
        self.last = {}
        self.last_stamp = {}
        self.create_subscription(NavSatFix, '/boat/gps/fix', self.gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/boat/imu', self.imu, qos_profile_sensor_data)
        self.create_timer(0.5, self.status)

    def fresh(self, kind, msg, limit):
        ns = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-ns)/1e9
        if not 0 <= age <= limit or ns <= self.last_stamp.get(kind, -1):
            return False
        self.last_stamp[kind] = ns
        self.last[kind] = time.monotonic()
        return True

    def gps(self, msg):
        if msg.status.status < 0 or msg.header.frame_id != 'gps_frame':
            return
        try:
            x,y,z = enu(msg.latitude, msg.longitude, self.datum[2], self.datum)
        except ValueError:
            return
        if not self.fresh('gps', msg, 2.0):
            return
        out = Odometry()
        out.header = copy.deepcopy(msg.header); out.header.frame_id = 'map'
        out.child_frame_id = 'gps_frame'
        out.pose.pose.position.x = x; out.pose.pose.position.y = y; out.pose.pose.position.z = z
        out.pose.pose.orientation.w = 1.0  # Unobserved: orientation is NOT fused.
        for i in (0,7):
            out.pose.covariance[i] = self.gps_var
        for i in (14,21,28,35):
            out.pose.covariance[i] = 1e6
        self.gps_pub.publish(out)

    def imu(self, msg):
        if msg.header.frame_id != 'imu_frame' or not all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            return
        if msg.angular_velocity_covariance[0] == -1 or not self.fresh('imu', msg, 0.5):
            return
        out = copy.deepcopy(msg)
        out.orientation_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        out.angular_velocity_covariance = [self.gyro_var,0.,0.,0.,self.gyro_var,0.,0.,0.,self.gyro_var]
        self.imu_pub.publish(out)

    def status(self):
        now = time.monotonic()
        self.status_pub.publish(String(data=json.dumps({
            'experimental': True, 'control_ready': False, 'absolute_heading_valid': False,
            'position_reference': 'gps_antenna',
            'gps_fresh': now-self.last.get('gps', -1e12) < 2.0,
            'imu_fresh': now-self.last.get('imu', -1e12) < 0.5,
            'datum': self.datum})))


def main(args=None):
    rclpy.init(args=args); node = FusionInputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
