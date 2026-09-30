"""Object detection on the camera feed (YOLO). Disabled by default.

  in   boat/camera/image_raw               (sensor_msgs/Image)
  in   boat/perception/detect_enable       (std_msgs/Bool) live on/off
  out  boat/perception/detections_image    (sensor_msgs/Image) annotated
  out  boat/perception/detections          (vision_msgs/Detection2DArray) if available

Nothing downstream uses detections yet, so it starts disabled. While
disabled the node doesn't touch frames at all (the enable check comes
before any cv_bridge conversion), and the model loads lazily on the first
frame that needs it.

Turn it on with the dashboard DETECT button, or:
    ros2 topic pub --once /boat/perception/detect_enable std_msgs/Bool "{data: true}"

The default model (yolov8n.pt, COCO classes) is a placeholder, not a
maritime detector. Point model_path at trained weights later.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

try:
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    HAVE_VISION_MSGS = True
except ImportError:
    HAVE_VISION_MSGS = False

try:
    from ultralytics import YOLO
    HAVE_ULTRALYTICS = True
except ImportError:
    HAVE_ULTRALYTICS = False


# COCO classes kept as a rough stand-in for things that matter on the water.
COCO_CLASSES_OF_INTEREST = {
    'person', 'boat', 'bird', 'bottle', 'surfboard', 'kite',
}


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('filter_to_classes_of_interest', True)
        self.declare_parameter('max_infer_hz', 5.0)
        self.declare_parameter('detect_enabled', False)

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.filter_classes = self.get_parameter('filter_to_classes_of_interest').value
        self.min_interval_s = 1.0 / max(self.get_parameter('max_infer_hz').value, 0.1)
        # Whether inference runs. publish_annotated controls whether the
        # image topic exists at all.
        self.detect_enabled = bool(self.get_parameter('detect_enabled').value)

        self.bridge = CvBridge()
        self._last_infer_time = 0.0
        self.model = None

        if not HAVE_ULTRALYTICS:
            self.get_logger().error(
                "ultralytics is not installed -- 'pip install ultralytics'. "
                "This node will subscribe and log but cannot run inference "
                "until that dependency is present.")
        self._model_load_failed = False

        if not HAVE_VISION_MSGS:
            self.get_logger().warn(
                'vision_msgs not installed -- structured Detection2DArray '
                'output disabled, only the annotated image topic will '
                'publish. `sudo apt install ros-$ROS_DISTRO-vision-msgs` '
                'to enable it.')

        self.image_sub = self.create_subscription(
            Image, 'boat/camera/image_raw', self.image_callback, 10)
        self.detect_enable_sub = self.create_subscription(
            Bool, 'boat/perception/detect_enable', self.detect_enable_callback, 10)

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image, 'boat/perception/detections_image', 10)

        if HAVE_VISION_MSGS:
            self.detections_pub = self.create_publisher(
                Detection2DArray, 'boat/perception/detections', 10)

        self.get_logger().info(
            'Detector node started, subscribed to boat/camera/image_raw, '
            f'max {1.0 / self.min_interval_s:.1f} Hz inference '
            f'(confidence >= {self.confidence_threshold}), '
            f'detection {"ENABLED" if self.detect_enabled else "DISABLED"} at start.')

    def detect_enable_callback(self, msg: Bool):
        self.detect_enabled = bool(msg.data)
        self.get_logger().info(
            f'Detection {"ENABLED" if self.detect_enabled else "DISABLED"} '
            '(via boat/perception/detect_enable)')

    def _ensure_model(self):
        """Load the YOLO weights on first use. Returns True if usable. A failed
        load is remembered so it logs once."""
        if self.model is not None:
            return True
        if not HAVE_ULTRALYTICS or self._model_load_failed:
            return False
        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'Loading YOLO model: {model_path} ...')
        try:
            self.model = YOLO(model_path)
        except Exception as exc:  # noqa: BLE001 -- log and degrade, don't die
            self._model_load_failed = True
            self.get_logger().error(
                f'Failed to load YOLO model {model_path!r}: {exc}. '
                'Detection stays off; the rest of the stack is unaffected.')
            return False
        self.get_logger().info(
            'Model loaded. NOTE: this is a general-purpose COCO model, '
            'not a maritime-trained detector -- placeholder to validate '
            'the pipeline. Do not treat its output as real obstacle '
            'detection.')
        return True

    def image_callback(self, msg: Image):
        # Enable check first, before any frame conversion. Disabled costs one
        # boolean check per frame.
        if not self.detect_enabled:
            return

        now = time.monotonic()
        if now - self._last_infer_time < self.min_interval_s:
            # Skip frames rather than fall behind.
            return
        self._last_infer_time = now

        if not self._ensure_model():
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            # An unexpected encoding must not kill the node.
            self.get_logger().error(
                f'cv_bridge failed to convert frame ({msg.encoding}): {exc}',
                throttle_duration_sec=5.0)
            return

        results = self.model.predict(
            frame, conf=self.confidence_threshold, verbose=False)[0]

        boxes = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            if self.filter_classes and cls_name not in COCO_CLASSES_OF_INTEREST:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            boxes.append((cls_name, conf, x1, y1, x2, y2))

        if self.publish_annotated:
            annotated = frame.copy()
            for cls_name, conf, x1, y1, x2, y2 in boxes:
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(annotated, p1, p2, (0, 220, 0), 2)
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(annotated, label, (p1[0], max(p1[1] - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_pub.publish(out_msg)

        if HAVE_VISION_MSGS:
            det_array = Detection2DArray()
            det_array.header = msg.header
            for cls_name, conf, x1, y1, x2, y2 in boxes:
                det = Detection2D()
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                det_array.detections.append(det)
            self.detections_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
