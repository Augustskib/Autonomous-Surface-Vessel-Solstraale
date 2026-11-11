"""Arbitration with stopped startup and explicit, acknowledged control.

Production uses boat/operator_command. MANUAL/AUTO transitions require a
one-use ticket and renewals require the current authority generation.
HOLD, heartbeat expiry and e-stop invalidate outstanding authority.

Priority: e-stop > Hold > propulsion inhibit > manual > recovery > waypoint.
Fresh supervision may permit manual recovery from navigation restrictions;
hard stops and missing/stale supervision cannot be overridden. Clearing a
restriction never automatically resumes the retained waypoint mission.
Legacy split mode/thrust inputs are disabled in the ROS adapter.
"""

import math
import json
import uuid

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
                 failsafe_timeout_sec=DEFAULT_FAILSAFE_TIMEOUT_SEC,
                 allow_legacy=False, require_supervisor=True):
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

        # Hardware default: stopped until a fresh explicit transition.
        # Legacy mode exists only for isolated compatibility tests.
        self.lease_state = LEASE_AUTO if allow_legacy else LEASE_HOLD
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
        self.allow_legacy = allow_legacy
        self.require_supervisor = require_supervisor
        self.authority = uuid.uuid4().hex
        self.tickets = {}
        self.manual_recovery_allowed = False
        self.autonomy_interrupted = False

    @staticmethod
    def command_payload(data):
        """Only these immutable fields identify the action a ticket authorizes."""
        return json.dumps({k: data.get(k) for k in
            ('session', 'seq', 'intent', 'transition', 'left', 'right')},
            sort_keys=True, allow_nan=False)

    def _invalidate_authority(self):
        self.authority = uuid.uuid4().hex
        self.tickets.clear()

    def _hold(self, now, expired=True):
        self.lease_state = LEASE_HOLD
        self.operator_intent = 'HOLD'
        self.operator_expired = expired
        self.lease_stamp = None
        self.manual.update(0, 0, now)
        self.waypoint.last_stamp = None
        self._invalidate_authority()

    def prepare_transition(self, data, now):
        """A one-use challenge, checked on the arbiter's own monotonic clock.

        Obtaining a challenge grants no control and sends no thrust.
        Expiry, stop and takeover invalidate all outstanding challenges.
        """
        self.evaluate(now)
        if (not isinstance(data, dict) or data.get('intent') not in ('MANUAL', 'AUTO', 'RESET')
                or data.get('transition') is not True):
            return None
        try:
            payload = self.command_payload(data)
        except (ValueError, TypeError):
            return None
        self.tickets = {k: v for k, v in self.tickets.items() if now < v[0]}
        if len(self.tickets) >= 128:
            return None
        token = uuid.uuid4().hex
        self.tickets[token] = (now + 1.0, self.authority, payload)
        return token

    def _supervisor_fresh(self, now):
        return (not self.require_supervisor or
                (self.failsafe_level_stamp is not None and
                 0 <= now - self.failsafe_level_stamp <= self.failsafe_timeout_sec))

    def on_operator_command(self, data, now):
        """Only fresh, generation-bound commands may obtain or renew motion."""
        self.evaluate(now)
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
        # Stop assertion is independent of drive ownership and sequence history.
        if intent == 'ESTOP':
            if not transition:
                return False, 'Stop requires an explicit action'
            self.on_estop(True)
            return True, 'Emergency stop asserted'
        if session == self.operator_session and seq <= self.operator_seq:
            return False, 'Superseded command'
        if intent == 'HOLD':
            if not transition:
                return False, 'Hold requires an explicit action'
            self._hold(now, expired=False)
        elif transition:
            if not isinstance(data.get('ticket'), str):
                return False, 'Missing transition ticket'
            ticket = self.tickets.pop(data.get('ticket', ''), None)
            if (ticket is None or now >= ticket[0] or ticket[1] != self.authority
                    or ticket[2] != self.command_payload(data)):
                return False, 'Transition expired or superseded; request a new action'
            if intent == 'RESET':
                if self.operator_session not in (None, session):
                    return False, 'Take Hold before resetting emergency stop'
                self.estopped = False
                self._hold(now, expired=False)
            else:
                if self.estopped:
                    return False, 'E-stop active; reset to Hold first'
                if not self._supervisor_fresh(now):
                    return False, 'Fresh safety supervision is required'
                if self.failsafe_level >= 4 or (intent == 'MANUAL' and
                        self.failsafe_level >= 3 and not self.manual_recovery_allowed):
                    return False, 'Propulsion inhibited by safety supervisor'
                if intent == 'AUTO' and self.failsafe_level != 0:
                    return False, 'Clear the autonomy restriction before Continue'
                self._invalidate_authority()
        else:
            if (session != self.operator_session or data.get('authority') != self.authority
                    or self.operator_expired or intent != self.operator_intent):
                return False, 'Control expired or changed; choose an explicit action'
        self.operator_session, self.operator_seq = session, seq
        self.operator_intent = 'HOLD' if intent == 'RESET' else intent
        self.operator_stamp = now
        self.operator_expired = False
        self.lease_state = {'MANUAL': LEASE_HELD, 'AUTO': LEASE_AUTO, 'HOLD': LEASE_HOLD}[self.operator_intent]
        self.lease_stamp = now if intent == 'MANUAL' else None
        self.manual.update(*(clean if intent == 'MANUAL' else (0, 0)), now)
        if transition and intent == 'AUTO':
            self.autonomy_interrupted = False
            self.waypoint.last_stamp = None
        return True, 'Accepted'

    def _expire_operator(self, now):
        if (self.operator_session and self.operator_intent != 'HOLD'
                and self.operator_stamp is not None
                and now - self.operator_stamp > self.lease_timeout_sec):
            self._hold(now)

    # Inputs

    def on_manual_thrust(self, left, right, now):
        """A manual thrust command arrived. Returns True if accepted.

        Does not acquire the lease; only boat/cmd/control_mode does.
        """
        if not self.allow_legacy or self.operator_session:
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

    def on_failsafe_level(self, level, now, manual_recovery_allowed=False):
        """Latest boat/failsafe_state level (see FAILSAFE_* above).

        Separate from failsafe thrust because the stop levels publish no
        thrust but still have to outrank the waypoint follower.
        """
        try:
            level = int(level)
        except (TypeError, ValueError):
            return False
        if level not in (0, 1, 2, 3, 4):
            return False
        if level > 0 and self.operator_intent == 'AUTO':
            self.autonomy_interrupted = True
        if level == 0 and self.autonomy_interrupted:
            self.autonomy_interrupted = False
            if self.operator_intent == 'AUTO':
                self._hold(now)
        self.manual_recovery_allowed = manual_recovery_allowed is True
        self.failsafe_level = level
        self.failsafe_level_stamp = now
        self.failsafe_stale = False
        return True

    def on_estop(self, active):
        # The legacy Bool topic may assert stop, never reset it. Only a fresh
        # sequenced RESET can release the latch; delayed False cannot rearm.
        if active:
            self.estopped = True
            self._hold(0)
        elif self.allow_legacy:
            self.estopped = False

    def on_control_mode(self, command, now):
        """Handle boat/cmd/control_mode. Returns True if understood.

        MANUAL is both acquire and heartbeat; the dashboard repeats it every
        200 ms while the operator holds the lease.
        """
        if command not in VALID_COMMANDS:
            return False
        # A legacy heartbeat must not override the sequenced dashboard owner.
        if not self.allow_legacy or self.operator_session:
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

        if (not self._supervisor_fresh(now) or self.failsafe_level >= 4 or
                (self.failsafe_level >= FAILSAFE_STOP_MIN and
                 not (self.lease_state == LEASE_HELD and self.manual_recovery_allowed))):
            self._hold(now)
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
