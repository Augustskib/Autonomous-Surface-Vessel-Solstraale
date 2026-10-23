"""Mission state machine. Plain Python, no ROS.

mission_manager.py is the ROS adapter; test/test_control_logic.py steps
this class through the operator loop.

States:
  INIT     startup checks. Refuses missions.
  IDLE     ready. Accepts missions.
  EXECUTE  a mission is running.
  ABORTED  stopped by e-stop, a navigation failure or the EXECUTE watchdog.
           Accepts missions; a new mission is the reset.

cancel() is its own transition and lands in IDLE, not ABORTED, since a
deliberate cancel is not a fault. request_mission() returns
(accepted, reason) so the dashboard can tell the operator why.
"""

INIT = 'INIT'
IDLE = 'IDLE'
EXECUTE = 'EXECUTE'
ABORTED = 'ABORTED'

ACCEPTING_STATES = (IDLE, ABORTED)

# Auto-abort a mission stuck in EXECUTE with no completion or abort feedback.
DEFAULT_MISSION_TIMEOUT_SEC = 120.0

# Reason codes for request_mission()/cancel(), used directly in JSON replies.
REASON_OK = 'OK'
REASON_ESTOP = 'ESTOP_ACTIVE'
REASON_BAD_STATE = 'BAD_STATE'
REASON_EMPTY = 'EMPTY_MISSION'


class MissionStateMachine:
    """Feed it events; ask it for state. Times are plain floats (seconds)."""

    def __init__(self, timeout_sec=DEFAULT_MISSION_TIMEOUT_SEC):
        self.timeout_sec = timeout_sec
        self.state = INIT
        self.estop_active = False
        self.execute_started_at = None
        # Point count of the loaded mission.
        self.active_point_count = 0
        self.paused = False
        self.paused_since = None

    # Transitions

    def finish_init(self):
        """Called once, after startup checks. Returns True if it moved."""
        if self.state == INIT:
            self.state = IDLE
            return True
        return False

    def request_mission(self, num_points, now):
        """Try to accept a mission. Returns (accepted, reason).

        E-stop is checked first since it is the most common reason a
        mission is refused.
        """
        if self.estop_active:
            return False, REASON_ESTOP
        if self.state not in ACCEPTING_STATES:
            return False, REASON_BAD_STATE
        if num_points <= 0:
            return False, REASON_EMPTY
        self.state = EXECUTE
        self.execute_started_at = now
        self.paused_since = now if self.paused else None
        self.active_point_count = num_points
        return True, REASON_OK

    def cancel(self, now=None):
        """Operator stop. Returns (accepted, reason). Lands in IDLE."""
        if self.state == INIT:
            return False, REASON_BAD_STATE
        self.state = IDLE
        self.execute_started_at = None
        self.active_point_count = 0
        return True, REASON_OK

    def on_feedback(self, data):
        """Handle boat/mission_feedback from waypoint_follower.

        Returns (state_changed, note). Ignored outside EXECUTE so a late
        message after an abort changes nothing.
        """
        if self.state != EXECUTE:
            return False, 'ignored (not executing)'
        if data == 'MISSION_COMPLETE':
            self.state = IDLE
            self.execute_started_at = None
            self.active_point_count = 0
            return True, 'complete'
        if data.startswith('WAYPOINT_REACHED'):
            return False, 'progress'
        if data.startswith('MISSION_ABORTED'):
            self.state = ABORTED
            self.execute_started_at = None
            self.active_point_count = 0
            return True, 'aborted'
        return False, 'unrecognized'

    def on_estop(self, active):
        """Returns True if this changed the mission state."""
        was_active = self.estop_active
        self.estop_active = bool(active)
        if self.estop_active and not was_active and self.state == EXECUTE:
            self.state = ABORTED
            self.execute_started_at = None
            self.active_point_count = 0
            return True
        # Clearing only allows new missions; it resumes nothing.
        return False

    def tick(self, now):
        """Watchdog. Returns elapsed seconds if it timed out, else None."""
        if self.state != EXECUTE or self.execute_started_at is None:
            return None
        if self.paused:
            return None
        elapsed = now - self.execute_started_at
        if elapsed > self.timeout_sec:
            self.state = ABORTED
            self.execute_started_at = None
            self.active_point_count = 0
            return elapsed
        return None

    def set_paused(self, paused, now):
        """Retain the mission and count only time under autonomous control."""
        paused = bool(paused)
        if paused == self.paused:
            return
        if paused:
            self.paused_since = now if self.state == EXECUTE else None
        else:
            if self.execute_started_at is not None and self.paused_since is not None:
                self.execute_started_at += max(0, now - self.paused_since)
            self.paused_since = None
        self.paused = paused

    # Queries

    def accepts_missions(self):
        return (not self.estop_active) and self.state in ACCEPTING_STATES
