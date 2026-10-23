"""Control arbitration logic. Plain Python, no ROS, so it can be unit tested.

control_arbiter.py wires ROS topics to this class; test/test_control_logic.py
drives it directly.

Two things are tracked separately:
  freshness  has a source published recently (per-source timeout)
  lease      is a human actually driving (explicit, from boat/cmd/control_mode)

Manual thrust traffic alone never counts as a human driving. The dashboard
sends neutral 0/0 heartbeats, and treating those as manual control used to
lock the waypoint follower out whenever a browser tab was open.

Lease states:
  AUTO  no lease. Waypoint source drives if fresh. Startup state, so a
        command-line mission works without a browser.
  HELD  operator holds the lease and manual values drive. Stale manual
        thrust gives zero but keeps the lease.
  HOLD  latched zero after an explicit RELEASE or a lease timeout. Leaves
        only on RESUME (to AUTO) or a new MANUAL acquire (to HELD).
        A dropped connection must not hand the boat back to autonomy.

Priority: e-stop > operator HOLD > safety stop > manual > recovery > waypoint.
Hold always means zero propulsion. Manual can override return-to-launch,
but cannot override a safety stop. Resetting e-stop leaves Hold latched.
After the dashboard acquires control, both Manual and Auto require its
sequenced heartbeat. A lost heartbeat latches Hold until an explicit action.
An established safety-monitor feed going stale also stops propulsion.
"""

import math

# Published on boat/mode so the dashboard shows who has control.
MODE_IDLE = 'IDLE'
MODE_MANUAL = 'MANUAL'
MODE_WAYPOINT = 'WAYPOINT'
MODE_ESTOP = 'ESTOP'
MODE_HOLD = 'HOLD'
# The failsafe is driving or holding zero. Separate from HOLD because the
# operator's response is different.
MODE_FAILSAFE = 'FAILSAFE'

# Internal lease states (not published directly; boat/mode is derived).
LEASE_AUTO = 'AUTO'
LEASE_HELD = 'HELD'
LEASE_HOLD = 'HOLD'

# Values accepted on boat/cmd/control_mode (std_msgs/String).
CMD_MANUAL = 'MANUAL'
CMD_RELEASE = 'RELEASE'
CMD_RESUME = 'RESUME'
VALID_COMMANDS = (CMD_MANUAL, CMD_RELEASE, CMD_RESUME)

# A thrust source goes stale after this. Same as motor_driver and thrust_splitter.
DEFAULT_SOURCE_TIMEOUT_SEC = 0.5

# Lease survives this long without a heartbeat (5 missed 200 ms beats).
# Longer than the thrust timeout, so a short gap stops the boat without
# taking control away from the operator.
DEFAULT_LEASE_TIMEOUT_SEC = 1.0

# The failsafe level is believed for this long. safety_monitor publishes at 2 Hz.
DEFAULT_FAILSAFE_TIMEOUT_SEC = 3.0

# Mirrors boat_msgs/FailsafeState, as literals so this module has no imports.
FAILSAFE_NOMINAL = 0
FAILSAFE_DRIVING_MIN = 1     # levels 1-2 command thrust
FAILSAFE_STOP_MIN = 3        # levels 3-4 command nothing: zero output

THRUST_LIMIT = 100.0


def sanitize_thrust(left, right, limit=THRUST_LIMIT):
    """Reject non-finite values, then clamp to +/-limit.

    Returns (left, right), or None if either value is NaN or infinite.
    The isfinite check comes first because min/max with NaN depends on
    argument order. A NaN thrust also corrupts the Gazebo physics state.
    """
    try:
        left = float(left)
        right = float(right)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(left) and math.isfinite(right)):
        return None
    return (max(-limit, min(limit, left)),
            max(-limit, min(limit, right)))


class Source:
    """Most recent command from one input topic and when it arrived.

    Times are plain float seconds; the ROS adapter converts rclpy Time.
    """

    def __init__(self):
        self.left = 0.0
        self.right = 0.0
        self.last_stamp = None

    def update(self, left, right, now):
        self.left = left
        self.right = right
        self.last_stamp = now

    def is_fresh(self, now, timeout_sec):
        if self.last_stamp is None:
            return False
        return (now - self.last_stamp) <= timeout_sec


class ControlArbitration:
    """Decides who drives. Feed it events, call evaluate() on a tick."""

    def __init__(self,
                 source_timeout_sec=DEFAULT_SOURCE_TIMEOUT_SEC,
                 lease_timeout_sec=DEFAULT_LEASE_TIMEOUT_SEC,
                 failsafe_timeout_sec=DEFAULT_FAILSAFE_TIMEOUT_SEC):
        self.source_timeout_sec = source_timeout_sec
        self.lease_timeout_sec = lease_timeout_sec
        self.failsafe_timeout_sec = failsafe_timeout_sec

        self.manual = Source()
        self.waypoint = Source()
        self.failsafe = Source()
        self.estopped = False

        # Failsafe level and its timestamp, tracked apart from failsafe
        # thrust: POWERED_STOP sends no thrust but still outranks waypoints.
        self.failsafe_level = FAILSAFE_NOMINAL
        self.failsafe_level_stamp = None
        self.failsafe_stale = False

        # Start in AUTO so a mission sent from the command line can run
        # without a browser.
        self.lease_state = LEASE_AUTO
        self.lease_stamp = None

        # Diagnostic counters.
        self.rejected_manual = 0
        self.rejected_waypoint = 0
        self.rejected_failsafe = 0
        self.lease_expiries = 0
        self.failsafe_expiries = 0
        # Dashboard commands carry mode + thrust together on one ordered topic.
        self.operator_session = None
        self.operator_seq = -1
        self.operator_intent = 'HOLD'
        self.operator_stamp = None
        self.operator_expired = False
        self.retired_sessions = set()

    def on_operator_command(self, data, now):
        """Apply a sequenced dashboard command; renewals cannot revive a lost lease."""
        if not isinstance(data, dict):
            return False, 'Invalid command'
        session, seq = data.get('session'), data.get('seq')
        intent, transition = data.get('intent'), data.get('transition') is True
        if (not isinstance(session, str) or not 1 <= len(session) <= 80
                or type(seq) is not int or seq < 0
                or intent not in ('MANUAL', 'AUTO', 'HOLD', 'ESTOP', 'RESET')
                or type(data.get('transition')) is not bool):
            return False, 'Invalid command'
        clean = sanitize_thrust(data.get('left', 0), data.get('right', 0))
        if clean is None:
            return False, 'Invalid thrust'
        if session in self.retired_sessions:
            return False, 'Control belongs to another session; reload to take over'
        if session != self.operator_session:
            if not transition or intent not in ('MANUAL', 'HOLD', 'ESTOP', 'RESET'):
                return False, 'Take Manual or Hold before continuing a mission'
            if self.operator_session:
                self.retired_sessions.add(self.operator_session)
            self.operator_session, self.operator_seq = session, -1
        if seq <= self.operator_seq:
            return False, 'Superseded command'
        self.operator_seq = seq
        self._expire_operator(now)
        if intent in ('ESTOP', 'RESET'):
            if not transition:
                return False, 'Stop/reset requires an explicit action'
            self.on_estop(intent == 'ESTOP')
            intent = 'HOLD'
        if self.estopped and intent != 'HOLD':
            return False, 'E-stop active; reset to Hold first'
        if not transition and (self.operator_expired or intent != self.operator_intent):
            return False, 'Control expired or changed; choose Take Manual or Continue Mission'
        self.operator_intent = intent
        self.operator_stamp = now
        self.operator_expired = False
        self.lease_state = {'MANUAL': LEASE_HELD, 'AUTO': LEASE_AUTO, 'HOLD': LEASE_HOLD}[intent]
        self.lease_stamp = now if intent == 'MANUAL' else None
        self.manual.update(*(clean if intent == 'MANUAL' else (0, 0)), now)
        if transition and intent == 'AUTO':
            # Require a new navigation sample after handing back control.
            self.waypoint.last_stamp = None
        return True, 'Accepted'

    def _expire_operator(self, now):
        if (self.operator_session and self.operator_intent != 'HOLD'
                and self.operator_stamp is not None
                and now - self.operator_stamp > self.lease_timeout_sec):
            self.operator_intent = 'HOLD'
            self.operator_expired = True
            self.lease_state = LEASE_HOLD
            self.manual.update(0, 0, now)

    # Inputs

    def on_manual_thrust(self, left, right, now):
        """A manual thrust command arrived. Returns True if accepted.

        Does not acquire the lease; only boat/cmd/control_mode does.
        """
        if self.operator_session:
            return False
        clean = sanitize_thrust(left, right)
        if clean is None:
            self.rejected_manual += 1
            return False
        self.manual.update(clean[0], clean[1], now)
        return True

    def on_waypoint_thrust(self, left, right, now):
        clean = sanitize_thrust(left, right)
        if clean is None:
            self.rejected_waypoint += 1
            return False
        self.waypoint.update(clean[0], clean[1], now)
        return True

    def on_failsafe_thrust(self, left, right, now):
        """A thrust command from safety_monitor's failsafe (boat/cmd/failsafe)."""
        clean = sanitize_thrust(left, right)
        if clean is None:
            self.rejected_failsafe += 1
            return False
        self.failsafe.update(clean[0], clean[1], now)
        return True

    def on_failsafe_level(self, level, now):
        """Latest boat/failsafe_state level (see FAILSAFE_* above).

        Separate from failsafe thrust because the stop levels publish no
        thrust but still have to outrank the waypoint follower.
        """
        try:
            level = int(level)
        except (TypeError, ValueError):
            return False
        self.failsafe_level = level
        self.failsafe_level_stamp = now
        self.failsafe_stale = False
        return True

    def on_estop(self, active):
        self.estopped = bool(active)
        if active:
            self.lease_state = LEASE_HOLD
            self.operator_intent = 'HOLD'
            self.operator_expired = True
            self.manual.last_stamp = None

    def on_control_mode(self, command, now):
        """Handle boat/cmd/control_mode. Returns True if understood.

        MANUAL is both acquire and heartbeat; the dashboard repeats it every
        200 ms while the operator holds the lease.
        """
        if command not in VALID_COMMANDS:
            return False
        # A legacy heartbeat must not override the sequenced dashboard owner.
        if self.operator_session:
            return False
        if command == CMD_MANUAL:
            self.lease_state = LEASE_HELD
            self.lease_stamp = now
        elif command == CMD_RELEASE:
            # Releasing lands in HOLD, not AUTO. Resuming needs a RESUME.
            self.lease_state = LEASE_HOLD
            self.lease_stamp = None
        elif command == CMD_RESUME:
            self.lease_state = LEASE_AUTO
            self.lease_stamp = None
        return True

    # Decision

    def _expire_lease_if_stale(self, now):
        if self.lease_state != LEASE_HELD:
            return
        if self.lease_stamp is None or (now - self.lease_stamp) > self.lease_timeout_sec:
            self.lease_state = LEASE_HOLD
            self.lease_stamp = None
            self.lease_expiries += 1

    def _expire_failsafe_if_stale(self, now):
        """An established safety feed must not fail open when it disappears."""
        if (not self.failsafe_stale and self.failsafe_level_stamp is not None
                and (now - self.failsafe_level_stamp) > self.failsafe_timeout_sec):
            self.failsafe_level = max(FAILSAFE_STOP_MIN, self.failsafe_level)
            self.failsafe_stale = True
            self.failsafe_expiries += 1

    def evaluate(self, now):
        """Return (left, right, mode) for this tick.

        Priority: estop > HOLD > safety stop > manual > recovery > waypoint > zero
        """
        self._expire_operator(now)
        self._expire_lease_if_stale(now)
        self._expire_failsafe_if_stale(now)

        if self.estopped:
            return 0.0, 0.0, MODE_ESTOP

        # A user's Hold is a propulsion stop, including during RTL recovery.
        if self.lease_state == LEASE_HOLD:
            return 0.0, 0.0, MODE_HOLD

        if self.failsafe_level >= FAILSAFE_STOP_MIN:
            return 0.0, 0.0, MODE_FAILSAFE

        if self.lease_state == LEASE_HELD:
            if self.manual.is_fresh(now, self.source_timeout_sec):
                return self.manual.left, self.manual.right, MODE_MANUAL
            # Lease alive but manual thrust gapped: zero, still MANUAL.
            return 0.0, 0.0, MODE_MANUAL

        # Recovery is below manual, but above the waypoint mission.
        if self.failsafe_level >= FAILSAFE_DRIVING_MIN:
            if self.failsafe.is_fresh(now, self.source_timeout_sec):
                return self.failsafe.left, self.failsafe.right, MODE_FAILSAFE
            # Failsafe is driving but its thrust gapped: hold zero rather
            # than hand the boat back to the mission.
            return 0.0, 0.0, MODE_FAILSAFE

        # LEASE_AUTO
        if self.waypoint.is_fresh(now, self.source_timeout_sec):
            return self.waypoint.left, self.waypoint.right, MODE_WAYPOINT
        return 0.0, 0.0, MODE_IDLE
