# Heading estimate with a quality flag, published by heading_estimator on
# boat/heading at 20 Hz.
#
# The IMU has no magnetometer, so yaw comes from integrating the gyro and
# drifts. GPS course over ground (COG) corrects it, but only while the boat
# is moving. Note that COG is the direction of travel, not where the bow
# points; the two differ in a cross current.
#
# NONE            no usable estimate. Treat as heading unavailable.
# DEAD_RECKONING  gyro only, drifting. Usable for short periods.
# FUSED           gyro corrected by COG (or a magnetometer) recently.
uint8 NONE=0
uint8 DEAD_RECKONING=1
uint8 FUSED=2

uint8 quality

# Degrees, 0 = north, clockwise, [0, 360). Meaningless when quality is NONE.
float64 heading_deg

# Last input that corrected the estimate: "GYRO", "COG", "MAG" or "".
string source

# 1.0 right after a correction, decaying toward 0.0 as assumed drift grows.
float64 confidence

# Seconds since the last COG/magnetometer correction. NaN if never.
float64 age_since_reference_sec

# Gyro bias (deg/s) learned while stationary.
float64 gyro_bias_dps

# Whether COG was accepted on the last fix (false below the speed threshold).
bool cog_valid
float64 ground_speed_mps     # NaN before two fixes
float64 cog_deg              # raw COG from the last two fixes, NaN if unavailable

# True if an absolute yaw source other than COG is in use (magnetometer,
# or the simulated IMU). False on the real boat.
bool absolute_reference_available

# Explanation when quality is not FUSED. Shown on the dashboard.
string detail
