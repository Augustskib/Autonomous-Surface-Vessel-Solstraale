"""Failsafe logic: geofence evaluation and the graded response levels.

Plain Python, no ROS. safety_monitor.py is the ROS adapter and
test/test_failsafe_logic.py drives this class directly with a fake clock.

A boat doesn't stop when thrust is cut; wind and current keep pushing it.
Cutting power on a geofence breach can leave a boat drifting further out
with no way back. So:

  - The fence is level-triggered: evaluate() recomputes the breach from the
    current position every tick, so it keeps asserting while the boat is out.
  - The response depends on what the boat still knows:

      0 NOMINAL           carry on
      1 HOLD_POSITION     station-keep        needs position + heading
      2 RETURN_TO_LAUNCH  drive home          needs position + heading
      3 POWERED_STOP      zero thrust         fallback
      4 HARD_STOP         e-stop              always wins

    Without a usable heading, levels 1 and 2 degrade to POWERED_STOP and
    degraded_reason says why. HOLD_POSITION has no station-keeping node yet,
    so it always degrades.

Engagement latches: once a failsafe fires the boat does not return to
NOMINAL until an operator clears it (boat/failsafe_clear). The response
level is still recomputed each tick, so the boat can move from
POWERED_STOP to RETURN_TO_LAUNCH when a heading comes back.

E-stop is applied last as an output override and is not part of the latch.
"""

import math

# Failsafe levels, ordered by severity. Do not renumber.
LEVEL_NOMINAL = 0
LEVEL_HOLD_POSITION = 1
LEVEL_RETURN_TO_LAUNCH = 2
LEVEL_POWERED_STOP = 3
LEVEL_HARD_STOP = 4

LEVEL_NAMES = {
    LEVEL_NOMINAL: 'NOMINAL',
    LEVEL_HOLD_POSITION: 'HOLD_POSITION',
    LEVEL_RETURN_TO_LAUNCH: 'RETURN_TO_LAUNCH',
    LEVEL_POWERED_STOP: 'POWERED_STOP',
    LEVEL_HARD_STOP: 'HARD_STOP',
}

# Which condition wins when several are active. Not the highest level
# number: POWERED_STOP is a fallback, not a demand. If the boat is outside
# the fence (RTL) and the link has just dropped (POWERED_STOP), returning
# home is the better answer. Lower number wins.
RESPONSE_PRECEDENCE = {
    LEVEL_HARD_STOP: 0,
    LEVEL_RETURN_TO_LAUNCH: 1,
    LEVEL_HOLD_POSITION: 2,
    LEVEL_POWERED_STOP: 3,
    LEVEL_NOMINAL: 4,
}

# Reason codes, as plain strings so they read well in `ros2 topic echo`.
REASON_ESTOP = 'ESTOP'
REASON_GEOFENCE_BREACH = 'GEOFENCE_BREACH'
REASON_GCS_LINK_LOST = 'GCS_LINK_LOST'
REASON_GCS_LINK_LOST_PROLONGED = 'GCS_LINK_LOST_PROLONGED'

EARTH_RADIUS_M = 6371000.0

# Minimum HeadingEstimate.quality usable for levels 1 and 2 (DEAD_RECKONING).
# A literal instead of an import so boat_control doesn't depend on
# boat_navigation; the safety node must survive a navigation fault.
QUALITY_USABLE_MIN = 1

# Consecutive out-of-bounds samples before a breach (1.5 s at 2 Hz). Filters
# single bad GPS samples. Applies to entering a breach only, leaving is immediate.
DEFAULT_BREACH_SAMPLES = 3

# Short link losses are usually transient and stop the boat. Longer ones
# (gcs_rtl_after_sec) send it home.
DEFAULT_GCS_RTL_AFTER_SEC = 30.0

# Return-to-launch controller. Same thrust convention as waypoint_follower:
# positive heading error makes the left thruster stronger. Slower and simpler
# than the waypoint controller on purpose.
DEFAULT_RTL_SPEED_PCT = 45.0
DEFAULT_RTL_STEER_GAIN = 100.0 / 120.0
DEFAULT_RTL_MIN_FORWARD_FRACTION = 0.30

# Stop driving within this distance of the launch point. Wider than the
# waypoint radius, since RTL only needs to get close.
DEFAULT_RTL_ARRIVAL_RADIUS_M = 8.0


def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres.

    Duplicated from waypoint_follower so boat_control has no dependency on
    boat_navigation.
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees, 0=North, clockwise) from point 1 to 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = (math.cos(phi1) * math.sin(phi2)
         - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda))
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def normalize_angle_deg(angle):
    """Normalize an angle (degrees) to (-180, 180]."""
    result = angle % 360.0
    if result > 180.0:
        result -= 360.0
    return result


class GeofenceConfig:
    """Circular geofence. A circle can't be mis-specified and distance to
    the boundary is one subtraction."""

    def __init__(self, enabled=True, center_lat=0.0, center_lon=0.0,
                 radius_m=150.0, warn_fraction=0.9, disabled_reason=''):
        self.enabled = bool(enabled)
        self.center_lat = float(center_lat)
        self.center_lon = float(center_lon)
        self.radius_m = float(radius_m)
        # Clamp so a bad UI value can't disable the warning ring or put it
        # outside the fence.
        self.warn_fraction = max(0.1, min(0.99, float(warn_fraction)))
        self.disabled_reason = str(disabled_reason or '')

    @property
    def warn_radius_m(self):
        return self.radius_m * self.warn_fraction


class GeofenceEvaluation:
    """The result of one level-triggered fence check."""

    def __init__(self):
        self.evaluated = False       # was there anything to evaluate?
        self.distance_m = float('nan')
        self.margin_m = float('nan')
        self.outside = False         # instantaneous, before debounce
        self.breached = False        # debounced, and LEVEL-triggered
        self.warning = False         # inside the hard fence, past the ring
        self.outside_sec = 0.0       # continuous time outside, 0 if inside


class FailsafeDecision:
    """Everything safety_monitor needs to publish and act on for one tick."""

    def __init__(self):
        self.level = LEVEL_NOMINAL
        self.requested_level = LEVEL_NOMINAL
        self.reason = ''
        self.active_reasons = []
        self.degraded_reason = ''
        self.commanding_thrust = False
        self.left = 0.0
        self.right = 0.0
        self.distance_to_launch_m = float('nan')
        self.bearing_to_launch_deg = float('nan')
        self.arrived_at_launch = False
        self.latched = False
        self.conditions_clear = True
        self.seconds_at_level = 0.0

    @property
    def level_name(self):
        return LEVEL_NAMES.get(self.level, 'UNKNOWN')


def rtl_thrust(current_lat, current_lon, launch_lat, launch_lon, heading_deg,
               speed_pct=DEFAULT_RTL_SPEED_PCT,
               steer_gain=DEFAULT_RTL_STEER_GAIN,
               min_forward_fraction=DEFAULT_RTL_MIN_FORWARD_FRACTION,
               arrival_radius_m=DEFAULT_RTL_ARRIVAL_RADIUS_M):
    """Differential thrust toward the launch point.

    Returns (left, right, distance_m, bearing_deg, arrived). Fixed speed
    until inside the arrival radius. Kept here rather than imported from
    boat_navigation so the safety node doesn't depend on it.
    """
    distance = haversine_distance_m(current_lat, current_lon, launch_lat, launch_lon)
    target_bearing = bearing_deg(current_lat, current_lon, launch_lat, launch_lon)

    if distance <= arrival_radius_m:
        return 0.0, 0.0, distance, target_bearing, True

    error = normalize_angle_deg(target_bearing - heading_deg)
    turn = max(-100.0, min(100.0, steer_gain * error))
    # Keep a forward component even at large heading error. A boat pivoting
    # in place has no course over ground, which is the heading estimator's
    # only absolute reference.
    forward = speed_pct * max(min_forward_fraction,
                              math.cos(math.radians(error)))
    return (max(-100.0, min(100.0, forward + turn)),
            max(-100.0, min(100.0, forward - turn)),
            distance, target_bearing, False)


class FailsafeSupervisor:
    """Condition evaluation and response selection.

    Every method takes `now` as float seconds, so timing is testable.
    """

    def __init__(self,
                 fence=None,
                 breach_samples=DEFAULT_BREACH_SAMPLES,
                 gcs_rtl_after_sec=DEFAULT_GCS_RTL_AFTER_SEC,
                 rtl_speed_pct=DEFAULT_RTL_SPEED_PCT,
                 rtl_arrival_radius_m=DEFAULT_RTL_ARRIVAL_RADIUS_M):
        self.fence = fence if fence is not None else GeofenceConfig()
        self.breach_samples = int(breach_samples)
        self.gcs_rtl_after_sec = float(gcs_rtl_after_sec)
        self.rtl_speed_pct = float(rtl_speed_pct)
        self.rtl_arrival_radius_m = float(rtl_arrival_radius_m)

        # Observations
        self.position = None          # (lat, lon) or None
        self.launch_point = None      # (lat, lon) or None
        self.heading_deg = None
        self.heading_usable = False
        self.heading_quality = 0
        self.heading_detail = ''
        self.estop_active = False
        self.gcs_lost = False
        self.gcs_lost_since = None

        # Fence state, recomputed every tick
        self._breach_samples_seen = 0
        self._outside_since = None
        self.last_fence = GeofenceEvaluation()

        # The latch. `engaged` stays true until an operator clears it.
        # `level` is recomputed every tick from live conditions, so the
        # response can change (e.g. POWERED_STOP to RTL once a heading
        # recovers) while the boat stays out of NOMINAL.
        self.engaged = False
        self.level = LEVEL_NOMINAL
        self.max_level_seen = LEVEL_NOMINAL
        # Response to keep while latched after the conditions clear, so a
        # boat back inside the fence doesn't stop and drift out again.
        self._latched_action = LEVEL_NOMINAL
        self._level_entered_at = None
        self._estop_entered_at = None
        self.escalations = 0
        self.clears = 0

    # Observations

    def set_fence(self, fence):
        """Replace the fence at runtime.

        Resets the debounce counter but not the latch, so moving the fence
        can't silently cancel an active failsafe.
        """
        self.fence = fence
        self._breach_samples_seen = 0

    def on_position(self, lat, lon):
        self.position = (float(lat), float(lon))

    def on_position_lost(self):
        """No usable fix. Distinct from 'position is (0,0)'."""
        self.position = None

    def set_launch_point(self, lat, lon):
        self.launch_point = (float(lat), float(lon))

    def on_heading(self, heading_deg, usable, quality=0, detail=''):
        """Latest heading estimate from boat/heading.

        `usable` is decided by the caller (quality and freshness).
        """
        self.heading_deg = None if heading_deg is None else float(heading_deg)
        self.heading_usable = bool(usable) and self.heading_deg is not None
        self.heading_quality = int(quality)
        self.heading_detail = str(detail or '')

    def on_heading_lost(self):
        """No heading estimate at all: heading_estimator is not running or stale."""
        self.heading_deg = None
        self.heading_usable = False
        self.heading_quality = 0
        self.heading_detail = 'no boat/heading message (estimator not running or stale)'

    def on_estop(self, active):
        active = bool(active)
        if active and not self.estop_active:
            self._estop_entered_at = None   # stamped on the next evaluate()
        self.estop_active = active

    def on_gcs_link(self, lost, now):
        lost = bool(lost)
        if lost and not self.gcs_lost:
            self.gcs_lost_since = now
        elif not lost:
            self.gcs_lost_since = None
        self.gcs_lost = lost

    # Fence

    def evaluate_fence(self, now):
        """Recompute the fence from the current position. No edge detection."""
        result = GeofenceEvaluation()

        if not self.fence.enabled or self.position is None:
            self._breach_samples_seen = 0
            self._outside_since = None
            self.last_fence = result
            return result

        result.evaluated = True
        result.distance_m = haversine_distance_m(
            self.fence.center_lat, self.fence.center_lon,
            self.position[0], self.position[1])
        result.margin_m = self.fence.radius_m - result.distance_m
        result.outside = result.distance_m > self.fence.radius_m

        if result.outside:
            self._breach_samples_seen += 1
            if self._outside_since is None:
                self._outside_since = now
            result.outside_sec = max(0.0, now - self._outside_since)
        else:
            self._breach_samples_seen = 0
            self._outside_since = None

        # Debounced on the way in, immediate on the way out.
        result.breached = self._breach_samples_seen >= self.breach_samples
        result.warning = (not result.outside
                          and result.distance_m > self.fence.warn_radius_m)

        self.last_fence = result
        return result

    # Level selection

    def select_level(self, requested, now):
        """Degrade a requested level to one the boat can perform.

        Returns (level, degraded_reason). The reason is empty when nothing
        was degraded.
        """
        if requested <= LEVEL_NOMINAL:
            return LEVEL_NOMINAL, ''
        if requested >= LEVEL_POWERED_STOP:
            # Stop levels need nothing and can't degrade further.
            return requested, ''

        if self.position is None:
            return (LEVEL_POWERED_STOP,
                    'no valid GPS position, so neither holding nor returning '
                    'is possible')

        if not self.heading_usable:
            detail = self.heading_detail or 'heading quality NONE'
            return (LEVEL_POWERED_STOP,
                    'no usable heading estimate (%s) -- a return-to-launch '
                    'controller on a bad heading drives confidently past home, '
                    'which is worse than drifting' % detail)

        if requested == LEVEL_HOLD_POSITION:
            # No station_keeper node yet, so fall back to the next thing
            # the boat can actually do.
            if self.launch_point is not None:
                return (LEVEL_RETURN_TO_LAUNCH,
                        'HOLD_POSITION requested but station-keeping is not '
                        'implemented; returning to launch instead')
            return (LEVEL_POWERED_STOP,
                    'HOLD_POSITION requested but station-keeping is not '
                    'implemented and no launch point is set')

        if requested == LEVEL_RETURN_TO_LAUNCH and self.launch_point is None:
            return (LEVEL_POWERED_STOP,
                    'no launch point set, so there is nowhere to return to -- '
                    'set one from the dashboard before the next run')

        return requested, ''

    # Conditions

    def active_conditions(self, now):
        """Every active condition as (level, reason, text).

        Pure: mutates nothing, so clear() can call it without advancing the
        debounce counter. Uses self.last_fence from the last evaluate().
        """
        conditions = []
        fence = self.last_fence

        if fence.breached:
            conditions.append((
                LEVEL_RETURN_TO_LAUNCH, REASON_GEOFENCE_BREACH,
                'Outside the geofence by %.0f m for %.0f s'
                % (max(0.0, -fence.margin_m), fence.outside_sec)))

        if self.gcs_lost:
            lost_for = (now - self.gcs_lost_since
                        if self.gcs_lost_since is not None else 0.0)
            if lost_for >= self.gcs_rtl_after_sec:
                conditions.append((
                    LEVEL_RETURN_TO_LAUNCH, REASON_GCS_LINK_LOST_PROLONGED,
                    'Ground-control link lost for %.0f s -- returning to launch'
                    % lost_for))
            else:
                # Short loss: stop and wait. HOLD_POSITION would go here
                # once station-keeping exists.
                conditions.append((
                    LEVEL_POWERED_STOP, REASON_GCS_LINK_LOST,
                    'Ground-control link lost for %.0f s (returning to launch '
                    'after %.0f s)' % (lost_for, self.gcs_rtl_after_sec)))

        # Emergencies first, then the most capable response requested.
        conditions.sort(key=lambda c: RESPONSE_PRECEDENCE.get(c[0], 9))
        return conditions

    # Tick

    def evaluate(self, now):
        """Recompute conditions, escalate, and produce this tick's command.

        The only mutating entry point. Call once per tick.
        """
        decision = FailsafeDecision()
        self.evaluate_fence(now)
        conditions = self.active_conditions(now)
        decision.conditions_clear = not conditions

        if conditions:
            requested, reason, _text = conditions[0]
            decision.requested_level = requested
            decision.reason = reason
            decision.active_reasons = [c[2] for c in conditions]
            applied, degraded = self.select_level(requested, now)
            decision.degraded_reason = degraded
        else:
            # Nothing live but latched: keep the last response instead of
            # declaring the emergency over.
            applied = self._latched_action if self.engaged else LEVEL_NOMINAL

        # Engage the latch, then track the response
        if applied > LEVEL_NOMINAL and not self.engaged:
            self.engaged = True
            self.escalations += 1

        if applied != self.level:
            self._level_entered_at = now
        self.level = applied
        if applied > LEVEL_NOMINAL:
            self._latched_action = applied
        self.max_level_seen = max(self.max_level_seen, applied)

        decision.level = self.level
        decision.latched = self.engaged
        decision.seconds_at_level = self._seconds_at_level(now)

        # Latched with nothing live: give a reason for the dashboard.
        if self.engaged and not conditions:
            decision.reason = 'LATCHED'
            decision.active_reasons = [
                'Conditions have cleared, but the failsafe stays latched at %s '
                'until an operator clears it' % LEVEL_NAMES.get(self.level, '?')]

        # Act on the latched level, so a boat back inside the fence keeps
        # driving home until an operator clears it.
        if decision.level == LEVEL_RETURN_TO_LAUNCH:
            self._apply_rtl(decision)
        # HOLD_POSITION, POWERED_STOP and HARD_STOP command no thrust.

        # E-stop: applied last as an override and not stored in
        # self.level, so clearing it restores the previous state without a
        # second clear.
        if self.estop_active:
            if self._estop_entered_at is None:
                self._estop_entered_at = now
            decision.level = LEVEL_HARD_STOP
            decision.requested_level = LEVEL_HARD_STOP
            decision.reason = REASON_ESTOP
            decision.active_reasons = (
                ['Emergency stop engaged'] + decision.active_reasons)
            decision.degraded_reason = ''
            decision.commanding_thrust = False
            decision.left = 0.0
            decision.right = 0.0
            decision.latched = True
            decision.conditions_clear = False
            decision.seconds_at_level = max(0.0, now - self._estop_entered_at)
        else:
            self._estop_entered_at = None

        return decision

    def _apply_rtl(self, decision):
        """Fill in the RTL command, re-checking position and heading.

        The latch can bring us here after either was lost; then fall back
        to POWERED_STOP and say why.
        """
        if self.position is None or self.launch_point is None or not self.heading_usable:
            decision.level = LEVEL_POWERED_STOP
            decision.degraded_reason = decision.degraded_reason or (
                'return-to-launch is latched but %s -- stopping instead'
                % ('there is no valid position' if self.position is None
                   else 'there is no launch point' if self.launch_point is None
                   else 'the heading estimate is not usable (%s)'
                        % (self.heading_detail or 'quality NONE')))
            return

        left, right, dist, brg, arrived = rtl_thrust(
            self.position[0], self.position[1],
            self.launch_point[0], self.launch_point[1],
            self.heading_deg,
            speed_pct=self.rtl_speed_pct,
            arrival_radius_m=self.rtl_arrival_radius_m)

        decision.distance_to_launch_m = dist
        decision.bearing_to_launch_deg = brg
        decision.arrived_at_launch = arrived
        decision.left = left
        decision.right = right
        # Home: stop driving but stay latched at RTL until an operator clears.
        decision.commanding_thrust = not arrived

    # Latch control

    def clear(self, now):
        """Operator clear. Returns (cleared, reason).

        Refuses while a condition is still active and names it.
        """
        if self.estop_active:
            return False, ('the emergency stop is still engaged -- clear that '
                           'first, it is a separate control')
        if not self.engaged:
            return False, 'no failsafe is active'
        # active_conditions() is pure, so this has no side effects.
        conditions = self.active_conditions(now)
        if conditions:
            return False, 'condition still active: %s' % conditions[0][2]
        self._disengage(now)
        return True, ''

    def force_clear(self, now):
        """Clear the latch even with a condition active.

        A failsafe that can't be overridden gets worked around in worse
        ways. The caller must require and record a reason. Still refuses
        under e-stop.
        """
        if self.estop_active:
            return False, 'the emergency stop is still engaged'
        self._disengage(now)
        return True, ''

    # Helpers

    def _disengage(self, now):
        self.engaged = False
        self.level = LEVEL_NOMINAL
        self._latched_action = LEVEL_NOMINAL
        self._level_entered_at = now
        self.clears += 1

    def _seconds_at_level(self, now):
        if self._level_entered_at is None:
            return 0.0
        return max(0.0, now - self._level_entered_at)
