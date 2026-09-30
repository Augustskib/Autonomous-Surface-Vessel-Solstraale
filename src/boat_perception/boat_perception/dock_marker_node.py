"""Dock marker detection: finds a printed ArUco marker and publishes its pose
relative to the camera. Not yet tested with a real marker.

Print the marker with scripts/generate_dock_marker.py and put it on the dock.

  in   boat/camera/image_raw     (sensor_msgs/Image)
  in   boat/camera/camera_info   (sensor_msgs/CameraInfo) needed for a 3D pose
  out  boat/dock/marker_detected (std_msgs/Bool)
  out  boat/dock/marker_pose     (geometry_msgs/PoseStamped) camera frame
                                 (x right, y down, z forward, metres)
  out  boat/perception/dock_image (sensor_msgs/Image) annotated

Single square marker pose has a known ambiguity near head-on: roll and pitch
can flip between frames. Distance and lateral offset are not affected.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

# Supported ArUco dictionaries. 4X4_50 is the default and matches
# generate_dock_marker.py; small dictionaries detect best at range.
ARUCO_DICTIONARIES = {
    '4X4_50': cv2.aruco.DICT_4X4_50,
    '4X4_100': cv2.aruco.DICT_4X4_100,
    '5X5_50': cv2.aruco.DICT_5X5_50,
    '5X5_100': cv2.aruco.DICT_5X5_100,
    '6X6_50': cv2.aruco.DICT_6X6_50,
    '6X6_100': cv2.aruco.DICT_6X6_100,
}


class DockMarkerNode(Node):
    def __init__(self):
        super().__init__('dock_marker_node')

        self.declare_parameter('marker_dictionary', '4X4_50')
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('marker_length_m', 0.15)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('max_infer_hz', 5.0)

        dict_name = self.get_parameter('marker_dictionary').value
        if dict_name not in ARUCO_DICTIONARIES:
            self.get_logger().error(
                f"Unknown marker_dictionary '{dict_name}', falling back to "
                f"4X4_50. Valid options: {sorted(ARUCO_DICTIONARIES)}")
            dict_name = '4X4_50'
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dict_name])
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        self.target_id = self.get_parameter('target_marker_id').value
        self.marker_length_m = float(self.get_parameter('marker_length_m').value)
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.min_interval_s = 1.0 / max(self.get_parameter('max_infer_hz').value, 0.1)

        # Marker corners in the marker frame (Z = 0, centred), in cv2.aruco
        # order: top-left, top-right, bottom-right, bottom-left.
        half = self.marker_length_m / 2.0
        self.object_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self._last_infer_time = 0.0
        self.camera_matrix = None
        self.dist_coeffs = None
        self._warned_no_camera_info = False

        self.image_sub = self.create_subscription(
            Image, 'boat/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'boat/camera/camera_info', self.camera_info_callback, 10)

        self.detected_pub = self.create_publisher(Bool, 'boat/dock/marker_detected', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'boat/dock/marker_pose', 10)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, 'boat/perception/dock_image', 10)

        self.get_logger().info(
            f'Dock marker node started. Watching for marker id={self.target_id} '
            f'from dictionary {dict_name}, physical size {self.marker_length_m * 100:.1f}cm. '
            f'Print it with scripts/generate_dock_marker.py and measure the printed '
            f'square carefully -- marker_length_m must match the real printout or '
            f'distance estimates will be wrong by the same ratio.')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)

    def image_callback(self, msg: Image):
        import time
        now = time.monotonic()
        if now - self._last_infer_time < self.min_interval_s:
            return
        self._last_infer_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _rejected = self.detector.detectMarkers(frame)

        found = False
        marker_corners = None
        if ids is not None:
            for c, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) == self.target_id:
                    found = True
                    marker_corners = c
                    break

        self.detected_pub.publish(Bool(data=found))

        annotated = frame
        if found:
            annotated = frame.copy() if self.publish_annotated else frame
            if self.publish_annotated:
                cv2.aruco.drawDetectedMarkers(annotated, [marker_corners], np.array([[self.target_id]]))

            if self.camera_matrix is not None:
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points, marker_corners[0], self.camera_matrix,
                    self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    self._publish_pose(msg, rvec, tvec)
                    if self.publish_annotated:
                        self._draw_overlay(annotated, rvec, tvec)
            elif not self._warned_no_camera_info:
                self._warned_no_camera_info = True
                self.get_logger().warn(
                    'Marker detected but no camera_info received yet -- cannot '
                    'compute real-world distance/pose until camera_publisher '
                    'starts publishing boat/camera/camera_info.')

        if self.publish_annotated:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_pub.publish(out_msg)

    def _publish_pose(self, source_msg: Image, rvec, tvec):
        pose = PoseStamped()
        pose.header = source_msg.header
        x, y, z = tvec.ravel().tolist()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        R, _ = cv2.Rodrigues(rvec)
        qw, qx, qy, qz = self._rotation_matrix_to_quaternion(R)
        pose.pose.orientation.w = qw
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        self.pose_pub.publish(pose)

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Rotation matrix to quaternion (w, x, y, z)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return w, x, y, z

    def _draw_overlay(self, annotated, rvec, tvec):
        tvec_flat = tvec.ravel()
        distance_m = float(np.linalg.norm(tvec_flat))
        lateral_m = float(tvec_flat[0])
        text = f'dock: {distance_m:.2f}m  lateral: {lateral_m:+.2f}m'
        cv2.putText(annotated, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        try:
            cv2.drawFrameAxes(annotated, self.camera_matrix, self.dist_coeffs,
                               rvec, tvec, self.marker_length_m * 0.5)
        except cv2.error:
            pass  # axis drawing is cosmetic, never break detection


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
