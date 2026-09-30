# Camera health, published by camera_publisher on boat/camera/status at 1 Hz.
# Lets the recorder and dashboard see when frames are fake (test pattern)
# or when intrinsics are nominal rather than calibrated.

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_CAMERA=1      # real frames from rpicam-vid
uint8 SOURCE_MOCK=2        # synthetic test pattern

uint8 source
bool mock_mode             # source == SOURCE_MOCK

# False when frames stop arriving or rpicam-vid has exited.
bool healthy

# Reason for mock mode or unhealthy state. Empty when streaming normally.
string detail

uint32 width
uint32 height

# Measured over the last second, not the configured rate.
float64 measured_fps
float64 configured_fps

uint32 frames_published
uint32 read_errors

# True if CameraInfo came from a calibration file, false if it was
# computed from the nominal field of view with no distortion model.
bool calibration_measured
# e.g. "nominal:62.2deg" or "file:/path/to/calibration.yaml"
string calibration_source
