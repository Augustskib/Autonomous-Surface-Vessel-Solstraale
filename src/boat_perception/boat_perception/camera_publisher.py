"""Camera node: publishes frames from rpicam-vid, CameraInfo, and camera health.

If rpicam-vid can't start, the node publishes a scrolling test pattern so
the rest of the stack can run off-hardware. boat/camera/status
(CameraStatus, 1 Hz) says whether frames are real, how many arrive per
second, and whether the intrinsics are calibrated, so the recorder and
dashboard can tell a test pattern from real footage.

A real camera that stops delivering frames is marked unhealthy after a run
of short reads. It never switches to the test pattern mid-run.
"""

import math
import os
import subprocess
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from boat_msgs.msg import CameraStatus
from cv_bridge import CvBridge
import cv2

# Horizontal FOV for nominal intrinsics when there is no calibration file.
# 62.2 deg is the Camera Module v2. The OV5647 (v1) on the boat is about
# 54 deg, so set horizontal_fov_deg or calibrate.
DEFAULT_HFOV_DEG = 62.2

# Consecutive short reads before the node reports unhealthy (1 s at 10 fps).
UNHEALTHY_AFTER_SHORT_READS = 10

CONFIGURED_FPS = 10.0


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.width = 640
        self.height = 480
        self.frame_size = self.width * self.height * 3 // 2  # YUV420 size

        self.declare_parameter('mock', False)
        force_mock = self.get_parameter('mock').value

        # ROS camera_calibration YAML. Empty means nominal intrinsics.
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('horizontal_fov_deg', DEFAULT_HFOV_DEG)
        self.declare_parameter('frame_id', 'camera_frame')
        self.frame_id = self.get_parameter('frame_id').value

        self.publisher_ = self.create_publisher(Image, 'boat/camera/image_raw', 10)
        # CameraInfo next to image_raw, as ROS tools expect.
        self.info_publisher_ = self.create_publisher(
            CameraInfo, 'boat/camera/camera_info', 10)
        # Frame source and health (see module docstring).
        self.status_publisher_ = self.create_publisher(
            CameraStatus, 'boat/camera/status', 10)

        # Set by _build_camera_info() so the status matches what was loaded.
        self.calibration_measured = False
        self.calibration_source = ''

        self.camera_info = self._build_camera_info()
        self.bridge = CvBridge()
        self.proc = None
        self.mock_mode = False
        self.mock_forced = bool(force_mock)
        self._mock_frame_count = 0

        # Health and rate bookkeeping for boat/camera/status.
        self._frame_buffer = bytearray()
        self._last_frame_at = time.monotonic()
        self._frames_published = 0
        self._read_errors = 0
        self._consecutive_short_reads = 0
        self._status_detail = ''
        self._frames_at_last_status = 0
        self._last_status_at = time.monotonic()
        self._measured_fps = 0.0
        self._unhealthy_reported = False

        cmd = [
            'rpicam-vid',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', '10',
            '--codec', 'yuv420',
            '--timeout', '0',
            '--nopreview',
            '-o', '-'
        ]

        if not force_mock:
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=self.frame_size * 2)
                os.set_blocking(self.proc.stdout.fileno(), False)
            except FileNotFoundError:
                self.get_logger().warn(
                    'rpicam-vid not found on this system (not a Raspberry Pi / no camera stack '
                    'installed) -- falling back to a synthetic test-pattern image so the rest of '
                    'the stack (web UI, recording, etc.) can still be developed/tested off-hardware.')
                self.mock_mode = True
                self._status_detail = (
                    'rpicam-vid not found on PATH -- publishing a synthetic '
                    'test pattern, NOT the world')
        else:
            self.get_logger().info('Camera mock mode forced via "mock" parameter.')
            self.mock_mode = True
            self._status_detail = (
                'mock mode forced via the "mock" parameter -- publishing a '
                'synthetic test pattern, NOT the world')

        timer_period = 1.0 / CONFIGURED_FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Separate 1 Hz timer so status keeps publishing if frames stop.
        self.status_timer = self.create_timer(1.0, self.publish_status)

        if self.mock_mode:
            self.get_logger().warn(
                'CAMERA IS IN MOCK MODE. Publishing a synthetic test pattern '
                'to "boat/camera/image_raw" -- any run recorded now contains '
                'a scrolling test card, not the world. This is reported '
                'continuously on boat/camera/status, recorded into the bag, '
                'and shown on the dashboard.')
        else:
            self.get_logger().info(
                'Camera publisher started (via rpicam-vid), publishing to "boat/camera/image_raw"')

    def timer_callback(self):
        if self.mock_mode:
            bgr = self._generate_mock_frame()
        else:
            raw = self._read_frame()
            if raw is None:
                return
            if len(raw) != self.frame_size:
                # Count short reads and go unhealthy if they continue.
                # Never switch to the test pattern here.
                self._read_errors += 1
                self._consecutive_short_reads += 1
                if (self._consecutive_short_reads >= UNHEALTHY_AFTER_SHORT_READS
                        and not self._unhealthy_reported):
                    self.get_logger().error(
                        'CAMERA UNHEALTHY: %d consecutive short reads from '
                        'rpicam-vid. No frames are being published. The '
                        'camera is NOT being replaced by a test pattern -- '
                        'boat/camera/status now reports healthy=false so '
                        'nothing downstream mistakes this for working video.'
                        % self._consecutive_short_reads)
                    self._unhealthy_reported = True
                self._status_detail = (
                    '%d consecutive short reads from rpicam-vid'
                    % self._consecutive_short_reads)
                self.get_logger().warn('Incomplete frame read from rpicam-vid',
                                       throttle_duration_sec=5.0)
                return
            if self._consecutive_short_reads:
                self.get_logger().info(
                    'Camera recovered after %d short reads'
                    % self._consecutive_short_reads)
                self._consecutive_short_reads = 0
                self._unhealthy_reported = False
                self._status_detail = ''
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)
        self._frames_published += 1
        self._last_frame_at = time.monotonic()

        # Same stamp and frame_id as the image, so the pair can be matched.
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.info_publisher_.publish(self.camera_info)

    def _read_frame(self):
        """Accumulate partial pipe reads without blocking ROS health timers."""
        for _ in range(128):
            try:
                chunk = os.read(self.proc.stdout.fileno(),
                                self.frame_size - len(self._frame_buffer))
            except BlockingIOError:
                return None
            if not chunk:
                raw = bytes(self._frame_buffer)
                self._frame_buffer.clear()
                return raw
            self._frame_buffer.extend(chunk)
            if len(self._frame_buffer) == self.frame_size:
                raw = bytes(self._frame_buffer)
                self._frame_buffer.clear()
                return raw
        return None

    def publish_status(self):
        """Publish boat/camera/status at 1 Hz. measured_fps is frames actually
        published in the last second, so throttling shows up here."""
        now = time.monotonic()
        dt = now - self._last_status_at
        if dt > 0.1:
            self._measured_fps = (
                (self._frames_published - self._frames_at_last_status) / dt)
            self._frames_at_last_status = self._frames_published
            self._last_status_at = now

        msg = CameraStatus()
        if self.mock_mode:
            msg.source = CameraStatus.SOURCE_MOCK
        elif self.proc is not None:
            msg.source = CameraStatus.SOURCE_CAMERA
        else:
            msg.source = CameraStatus.SOURCE_UNKNOWN
        msg.mock_mode = bool(self.mock_mode)

        # Real but unhealthy is possible, so these are two separate fields.
        if self.mock_mode:
            # A test pattern is never usable imagery, so never healthy.
            msg.healthy = False
        elif self.proc is None:
            msg.healthy = False
        else:
            msg.healthy = (
                self._consecutive_short_reads < UNHEALTHY_AFTER_SHORT_READS
                and self.proc.poll() is None
                and now - self._last_frame_at <= 2.0)
            if self.proc.poll() is not None and not self._status_detail:
                self._status_detail = (
                    'rpicam-vid exited with code %s' % self.proc.returncode)

        if not self.mock_mode and now - self._last_frame_at > 2.0:
            self._status_detail = 'No complete camera frame for more than 2 seconds'
        msg.detail = self._status_detail
        msg.width = int(self.width)
        msg.height = int(self.height)
        msg.measured_fps = float(round(self._measured_fps, 2))
        msg.configured_fps = float(CONFIGURED_FPS)
        msg.frames_published = int(self._frames_published)
        msg.read_errors = int(self._read_errors)
        msg.calibration_measured = bool(self.calibration_measured)
        msg.calibration_source = self.calibration_source
        self.status_publisher_.publish(msg)

    def _build_camera_info(self):
        """Build the CameraInfo published with every frame.

        Uses a calibration file if given. Otherwise intrinsics come from the
        field of view with zero distortion, which is not a calibration; the
        status topic records which one was used.
        """
        url = self.get_parameter('camera_info_url').value
        if url:
            info = self._load_calibration(url)
            if info is not None:
                # Recorded in boat/camera/status.
                self.calibration_measured = True
                self.calibration_source = 'file:%s' % os.path.expanduser(url)
                return info
            self.get_logger().error(
                'camera_info_url was set to %r but could not be loaded -- '
                'falling back to synthesised intrinsics.' % url)

        hfov = math.radians(float(self.get_parameter('horizontal_fov_deg').value))
        # Pinhole model: f = (width/2) / tan(hfov/2). Square pixels, fy == fx.
        fx = (self.width / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]

        self.calibration_measured = False
        self.calibration_source = 'nominal:%.1fdeg' % math.degrees(hfov)

        self.get_logger().warn(
            'NO CAMERA CALIBRATION LOADED. Publishing NOMINAL intrinsics '
            'synthesised from a %.1f deg horizontal FOV (fx=fy=%.1f px, no '
            'distortion model). Adequate for rough geometry and for testing '
            'the pipeline, but NOT calibration data -- run '
            'camera_calibration and pass the result via the '
            '"camera_info_url" parameter before collecting any footage '
            'intended for a dataset.'
            % (math.degrees(hfov), fx))
        return info

    def _load_calibration(self, path):
        """Load a standard ROS camera_calibration YAML file."""
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().error('Calibration file not found: %s' % path)
            return None
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                calib = yaml.safe_load(handle)
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error('Could not parse calibration %s: %s' % (path, exc))
            return None

        try:
            info = CameraInfo()
            info.width = int(calib['image_width'])
            info.height = int(calib['image_height'])
            info.distortion_model = calib.get('distortion_model', 'plumb_bob')
            info.d = [float(x) for x in calib['distortion_coefficients']['data']]
            info.k = [float(x) for x in calib['camera_matrix']['data']]
            info.r = [float(x) for x in calib.get(
                'rectification_matrix', {'data': [1, 0, 0, 0, 1, 0, 0, 0, 1]})['data']]
            info.p = [float(x) for x in calib['projection_matrix']['data']]
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                'Calibration %s is missing or has malformed fields: %s' % (path, exc))
            return None

        # A calibration from a different resolution doesn't apply. Refuse it.
        if info.width != self.width or info.height != self.height:
            self.get_logger().error(
                'Calibration %s is for %dx%d but the camera is capturing %dx%d. '
                'Intrinsics do not transfer across resolutions -- recalibrate at '
                'the capture resolution.'
                % (path, info.width, info.height, self.width, self.height))
            return None

        self.get_logger().info(
            'Loaded camera calibration from %s (%dx%d, fx=%.1f, fy=%.1f)'
            % (path, info.width, info.height, info.k[0], info.k[4]))
        return info

    def _generate_mock_frame(self):
        """Animated test pattern for when rpicam-vid is unavailable."""
        self._mock_frame_count += 1
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:, :] = (40, 40, 40)
        offset = (self._mock_frame_count * 4) % self.width
        cv2.line(frame, (offset, 0), (offset, self.height), (0, 200, 0), 3)
        cv2.putText(frame, 'MOCK CAMERA - no rpicam-vid', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, f'frame {self._mock_frame_count}', (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return frame

    def destroy_node(self):
        if self.proc is not None:
            self.proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
