"""Heading estimation: gyro integration fused with GPS course over ground.

Plain Python, no ROS. heading_estimator.py is the ROS adapter and
test/test_heading_logic.py runs this class against a fake clock.

The IMU (Modulino Movement) has no magnetometer. Roll and pitch come from
gravity, but yaw can only come from integrating the gyro, and gyro bias
makes that drift without bound. GPS course over ground (COG) is absolute
and doesn't drift, but it is slow, noisy, and meaningless when the boat
isn't moving. The two cover each other's weaknesses, so they are fused
with a complementary filter:

    predict:  heading = previous + (gyro_rate - bias) * dt
    correct:  heading = predicted + (1 - alpha) * normalize(cog - predicted)

With alpha = 0.98, each accepted COG sample pulls the estimate 2% of the
way toward it. Angles are never averaged directly: the error is wrapped to
(-180, 180] first, since 359 and 1 degrees are 2 degrees apart.

A Kalman filter has the same predict/correct shape but needs measured
sensor noise, which we don't have yet. alpha_for() is the one place a
computed gain would replace the constant.

COG is the direction of travel, not where the bow points. In a cross
current they differ by a steady offset. That is fine for waypoint
following, not for docking or for projecting camera pixels.

Other absolute sources go through on_absolute_reference(): a magnetometer
(source='MAG') later, or the simulated IMU (source='SIM'), whose
orientation is absolute in Gazebo but not on the real boat.
"""

import math

# Quality levels, mirroring HeadingEstimate.msg. Ordered, so >= works.
QUALITY_NONE = 0            # no usable estimate at all
QUALITY_DEAD_RECKONING = 1  # gyro only, drifting
QUALITY_FUSED = 2           # actively corrected by an absolute reference

QUALITY_NAMES = {
    QUALITY_NONE: 'NONE',
    QUALITY_DEAD_RECKONING: 'DEAD_RECKONING',
    QUALITY_FUSED: 'FUSED',
}

SOURCE_GYRO = 'GYRO'
SOURCE_COG = 'COG'
SOURCE_MAG = 'MAG'

# Below this ground speed COG is just GPS noise. 0.5 m/s is well below
# survey speed. Check it on the water by plotting cog_deg against
# heading_deg at different speeds.
DEFAULT_COG_MIN_SPEED_MPS = 0.5

# Filter weight per accepted correction (2% toward the reference).
DEFAULT_ALPHA = 0.98

# Alpha is per correction, not per second, so a faster GPS pulls harder.

# Below this speed and turn rate the boat counts as stationary and the gyro
# reading is treated as bias.
DEFAULT_STATIONARY_SPEED_MPS = 0.15
DEFAULT_STATIONARY_RATE_DPS = 2.0

# Bias learning rate. Slow, since bias changes with temperature over minutes.
DEFAULT_BIAS_LEARN_RATE = 0.002

# Cap on the learned bias. Anything larger is a fault or a real rotation.
DEFAULT_MAX_BIAS_DPS = 5.0

# Quality drops from FUSED to DEAD_RECKONING after this long without a reference.
DEFAULT_FUSED_TIMEOUT_SEC = 5.0

# After this long without a reference, quality drops to NONE and consumers
# must degrade. Failsafe RTL falls back to POWERED_STOP at this point.
DEFAULT_MAX_COAST_SEC = 120.0

# Assumed residual drift (deg/s), used only for the confidence readout.
# Not measured yet: log heading with the boat still to get the real value.
DEFAULT_ASSUMED_DRIFT_DPS = 0.5

# Confidence reaches 0.0 when accumulated assumed drift hits this many degrees.
CONFIDENCE_FLOOR_ERROR_DEG = 45.0

# Gyro rates above this are corrupt samples (e.g. rad/s read as deg/s).
MAX_PLAUSIBLE_RATE_DPS = 300.0

# Skip integration across gaps longer than this (a stalled node).
MAX_PREDICT_DT_SEC = 1.0
MIN_PREDICT_DT_SEC = 1e-4


def normalize_angle_deg(angle):
    """Normalize an angle (degrees) to (-180, 180]. Same as waypoint_follower's."""
    result = angle % 360.0
    if result > 180.0:
        result -= 360.0
    return result


def wrap_heading_deg(angle):
    """Normalize a compass heading to [0, 360)."""
    return angle % 360.0


def course_over_ground_deg(lat1, lon1, lat2, lon2):
    """Bearing from fix 1 to fix 2 in degrees, 0 = north, clockwise.

    This is course over ground, not bow heading.
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = (math.cos(phi1) * math.sin(phi2)
         - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


class HeadingSolution:
    """One tick's answer. Mirrors boat_msgs/msg/HeadingEstimate.msg."""

    def __init__(self):
        self.quality = QUALITY_NONE
        self.heading_deg = float('nan')
        self.source = ''
        self.confidence = 0.0
        self.age_since_reference_sec = float('nan')
        self.gyro_bias_dps = 0.0
        self.cog_valid = False
        self.ground_speed_mps = float('nan')
        self.cog_deg = float('nan')
        self.absolute_reference_available = False
        self.detail = ''

    @property
    def usable(self):
        """True for FUSED and DEAD_RECKONING. Coasting is degraded, not unusable;
        max_coast_sec is what drops it to NONE."""
        return self.quality >= QUALITY_DEAD_RECKONING

    @property
    def quality_name(self):
        return QUALITY_NAMES.get(self.quality, 'UNKNOWN')


class HeadingEstimator:
    """Complementary filter over a gyro rate and absolute references.

    Every method takes `now` as float seconds, so tests run instantly.
    """

    def __init__(self,
                 alpha=DEFAULT_ALPHA,
                 cog_min_speed_mps=DEFAULT_COG_MIN_SPEED_MPS,
                 stationary_speed_mps=DEFAULT_STATIONARY_SPEED_MPS,
                 stationary_rate_dps=DEFAULT_STATIONARY_RATE_DPS,
                 bias_learn_rate=DEFAULT_BIAS_LEARN_RATE,
                 max_bias_dps=DEFAULT_MAX_BIAS_DPS,
                 fused_timeout_sec=DEFAULT_FUSED_TIMEOUT_SEC,
                 max_coast_sec=DEFAULT_MAX_COAST_SEC,
                 assumed_drift_dps=DEFAULT_ASSUMED_DRIFT_DPS):
        self.alpha = float(alpha)
        self.cog_min_speed_mps = float(cog_min_speed_mps)
        self.stationary_speed_mps = float(stationary_speed_mps)
        self.stationary_rate_dps = float(stationary_rate_dps)
        self.bias_learn_rate = float(bias_learn_rate)
        self.max_bias_dps = float(max_bias_dps)
        self.fused_timeout_sec = float(fused_timeout_sec)
        self.max_coast_sec = float(max_coast_sec)
        self.assumed_drift_dps = float(assumed_drift_dps)

        # Filter state
        self.heading_deg = None          # None until first referenced
        self.gyro_bias_dps = 0.0
        self.last_predict_at = None
        self.last_reference_at = None
        self.last_reference_source = ''
        self.has_absolute_reference = False

        # Latest observations, for reporting
        self.last_rate_dps = 0.0
        self.ground_speed_mps = float('nan')
        self.last_cog_deg = float('nan')
        self.last_cog_accepted = False

        # Diagnostics
        self.rejected_rates = 0
        self.rejected_cog_slow = 0
        self.corrections = 0
        self.bias_samples = 0

    # Filter gain

    def alpha_for(self, source, weight):
        """Blend weight for one correction, in [0, 1).

        A constant, optionally scaled per source by `weight`. A Kalman
        filter would compute this from its current uncertainty instead.
        """
        del source  # unused until per-source gains exist
        weight = max(0.0, min(1.0, float(weight)))
        if weight <= 0.0:
            return 1.0        # ignore this correction
        # weight 1.0 gives self.alpha, 0.5 half as much correction.
        return 1.0 - (1.0 - self.alpha) * weight

    # Inputs

    def on_gyro_rate(self, rate_dps, now):
        """Predict step. Yaw rate in deg/s, positive clockwise.

        sensor_msgs/Imu angular_velocity.z is rad/s counter-clockwise (ENU),
        so the adapter converts and negates. Getting the sign wrong makes the
        filter fight every correction.

        Returns True if the sample was integrated.
        """
        try:
            rate_dps = float(rate_dps)
        except (TypeError, ValueError):
            self.rejected_rates += 1
            return False
        if not math.isfinite(rate_dps) or abs(rate_dps) > MAX_PLAUSIBLE_RATE_DPS:
            # Drop non-finite samples. One NaN integrated makes the heading
            # NaN until restart.
            self.rejected_rates += 1
            return False

        self.last_rate_dps = rate_dps

        if self.last_predict_at is None:
            self.last_predict_at = now
            return False

        dt = now - self.last_predict_at
        self.last_predict_at = now
        if dt < MIN_PREDICT_DT_SEC:
            return False
        if dt > MAX_PREDICT_DT_SEC:
            # Long gap (stalled node): skip it instead of assuming a constant
            # rate. age_since_reference still counts the time.
            return False

        self._learn_bias(rate_dps, dt)

        if self.heading_deg is None:
            # No reference yet. Don't seed 0.0, stay at QUALITY_NONE.
            return False

        self.heading_deg = wrap_heading_deg(
            self.heading_deg + (rate_dps - self.gyro_bias_dps) * dt)
        return True

    def on_ground_speed(self, speed_mps):
        """Latest ground speed, used for the COG gate and bias learning."""
        try:
            speed = float(speed_mps)
        except (TypeError, ValueError):
            return
        if math.isfinite(speed):
            self.ground_speed_mps = speed

    def on_cog(self, cog_deg, speed_mps, now):
        """Correct step from GPS course over ground. Returns True if used.

        Adds the speed gate: below the threshold COG is not a measurement.
        """
        self.on_ground_speed(speed_mps)
        try:
            cog_deg = float(cog_deg)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(cog_deg):
            return False

        self.last_cog_deg = wrap_heading_deg(cog_deg)

        speed = self.ground_speed_mps
        if not math.isfinite(speed) or speed < self.cog_min_speed_mps:
            self.last_cog_accepted = False
            self.rejected_cog_slow += 1
            return False

        self.last_cog_accepted = True
        return self.on_absolute_reference(self.last_cog_deg, now,
                                          source=SOURCE_COG)

    def on_absolute_reference(self, heading_deg, now, source=SOURCE_MAG,
                              weight=1.0):
        """Correct step from any absolute heading source (COG, MAG, SIM).
        Returns True if applied."""
        try:
            heading_deg = float(heading_deg)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(heading_deg):
            return False
        heading_deg = wrap_heading_deg(heading_deg)

        if self.heading_deg is None:
            # First reference: adopt it directly instead of converging slowly
            # from an unknown start.
            self.heading_deg = heading_deg
        else:
            # Wrap the error to the short way round, then move a fraction of it.
            alpha = self.alpha_for(source, weight)
            error = normalize_angle_deg(heading_deg - self.heading_deg)
            self.heading_deg = wrap_heading_deg(
                self.heading_deg + (1.0 - alpha) * error)

        self.last_reference_at = now
        self.last_reference_source = source
        if source != SOURCE_COG:
            self.has_absolute_reference = True
        self.corrections += 1
        return True

    # Bias learning

    def _learn_bias(self, rate_dps, dt):
        """Learn gyro bias while the boat is still.

        Needs both low speed and a low turn rate, since a boat pivoting in
        place has near-zero speed but a real rotation. The learned value
        is capped.
        """
        speed = self.ground_speed_mps
        if not math.isfinite(speed) or speed > self.stationary_speed_mps:
            return
        if abs(rate_dps) > self.stationary_rate_dps:
            return
        # Scale by dt so the rate is independent of sample rate.
        k = min(1.0, self.bias_learn_rate * (dt / 0.05))
        self.gyro_bias_dps += k * (rate_dps - self.gyro_bias_dps)
        self.gyro_bias_dps = max(-self.max_bias_dps,
                                 min(self.max_bias_dps, self.gyro_bias_dps))
        self.bias_samples += 1

    # Output

    def evaluate(self, now):
        """Return the current HeadingSolution. Pure: mutates nothing."""
        s = HeadingSolution()
        s.gyro_bias_dps = self.gyro_bias_dps
        s.ground_speed_mps = self.ground_speed_mps
        s.cog_deg = self.last_cog_deg
        s.cog_valid = self.last_cog_accepted
        s.absolute_reference_available = self.has_absolute_reference

        if self.heading_deg is None or self.last_reference_at is None:
            s.quality = QUALITY_NONE
            s.detail = ('no absolute heading reference has ever been received '
                        '-- the boat does not know which way it is pointing. '
                        'Drive above %.2f m/s to acquire course-over-ground, '
                        'or fit a magnetometer.' % self.cog_min_speed_mps)
            return s

        age = max(0.0, now - self.last_reference_at)
        s.age_since_reference_sec = age
        s.heading_deg = self.heading_deg
        s.source = self.last_reference_source or SOURCE_GYRO

        # Confidence: 1..0 staleness readout from the assumed drift rate.
        # Not a probability.
        drifted_deg = age * self.assumed_drift_dps
        s.confidence = max(0.0, min(
            1.0, 1.0 - drifted_deg / CONFIDENCE_FLOOR_ERROR_DEG))

        if age > self.max_coast_sec:
            s.quality = QUALITY_NONE
            s.detail = (
                'heading has coasted on the gyro for %.0f s with no absolute '
                'reference (limit %.0f s); accumulated drift could be ~%.0f '
                'degrees, which is not steerable. Treat as unavailable.'
                % (age, self.max_coast_sec, drifted_deg))
            # heading_deg is still reported for plots; quality NONE says
            # not to act on it.
            return s

        if age > self.fused_timeout_sec:
            s.quality = QUALITY_DEAD_RECKONING
            s.source = SOURCE_GYRO
            reason = self._why_no_cog()
            s.detail = ('coasting on the gyro for %.1f s -- %s. Accumulated '
                        'drift ~%.1f degrees.' % (age, reason, drifted_deg))
            return s

        s.quality = QUALITY_FUSED
        s.detail = ''
        return s

    def _why_no_cog(self):
        speed = self.ground_speed_mps
        if not math.isfinite(speed):
            return 'no GPS fixes have arrived to derive course from'
        if speed < self.cog_min_speed_mps:
            return ('ground speed %.2f m/s is below the %.2f m/s threshold, so '
                    'course over ground is noise rather than a measurement'
                    % (speed, self.cog_min_speed_mps))
        return ('GPS fixes have stopped arriving despite ground speed %.2f m/s'
                % speed)
