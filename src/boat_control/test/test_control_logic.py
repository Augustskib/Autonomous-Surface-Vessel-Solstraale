"""Logic tests for the control arbiter and the mission state machine.

Plain Python with a fake clock. These test the decision logic only, not ROS
wiring, Gazebo or timing under load; scripts/sim/verify_control_loop.sh
covers the running system.

Run without ROS:
    python3 -m pytest src/boat_control/test/test_control_logic.py -v
    python3 src/boat_control/test/test_control_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from boat_control.arbitration import (  # noqa: E402
    CMD_MANUAL, CMD_RELEASE, CMD_RESUME,
    ControlArbitration,
    MODE_ESTOP, MODE_HOLD, MODE_IDLE, MODE_MANUAL, MODE_WAYPOINT,
    sanitize_thrust,
)
from boat_control.mission_state import (  # noqa: E402
    ABORTED, EXECUTE, IDLE,
    MissionStateMachine,
    REASON_BAD_STATE, REASON_ESTOP,
)


# Neutral manual traffic must not starve autonomy

def test_neutral_manual_heartbeat_does_not_starve_waypoint():
    """Dashboard 0/0 heartbeats without a lease must not block autonomy."""
    a = ControlArbitration()
    t = 0.0
    for _ in range(50):  # 10 seconds of browser heartbeats
        a.on_manual_thrust(0.0, 0.0, t)          # parked browser tab
        a.on_waypoint_thrust(60.0, 40.0, t)      # follower wants to drive
        left, right, mode = a.evaluate(t)
        assert mode == MODE_WAYPOINT, f'starved at t={t}: mode={mode}'
        assert (left, right) == (60.0, 40.0)
        t += 0.2


def test_lease_makes_manual_win():
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(100.0, 100.0, t)
    a.on_waypoint_thrust(60.0, 40.0, t)
    assert a.evaluate(t) == (100.0, 100.0, MODE_MANUAL)


def test_neutral_while_lease_held_means_stop():
    """0/0 with the lease held is a real command, not an absence of one."""
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(0.0, 0.0, t)
    a.on_waypoint_thrust(60.0, 40.0, t)
    left, right, mode = a.evaluate(t)
    assert (left, right, mode) == (0.0, 0.0, MODE_MANUAL)


# Lease expiry must not resume autonomy

def test_lease_expiry_latches_to_zero_and_does_not_resume_autonomy():
    a = ControlArbitration(lease_timeout_sec=1.0)
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(80.0, 80.0, t)
    assert a.evaluate(t)[2] == MODE_MANUAL

    # Operator's link dies. Autonomy is still publishing happily.
    t = 2.0
    a.on_waypoint_thrust(60.0, 40.0, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_HOLD, 'expired lease must latch, not fall through'
    assert (left, right) == (0.0, 0.0)

    # It must STAY held, however long autonomy keeps asking.
    for _ in range(25):
        t += 0.2
        a.on_waypoint_thrust(60.0, 40.0, t)
        assert a.evaluate(t)[2] == MODE_HOLD

    # Only an explicit RESUME hands the boat back.
    a.on_control_mode(CMD_RESUME, t)
    a.on_waypoint_thrust(60.0, 40.0, t)
    assert a.evaluate(t)[2] == MODE_WAYPOINT


def test_explicit_release_also_latches():
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(50.0, 50.0, t)
    assert a.evaluate(t)[2] == MODE_MANUAL
    a.on_control_mode(CMD_RELEASE, t)
    a.on_waypoint_thrust(60.0, 40.0, t)
    assert a.evaluate(t)[2] == MODE_HOLD


def test_estop_beats_everything_and_reset_stays_in_hold():
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(100.0, 100.0, t)
    a.on_estop(True)
    assert a.evaluate(t) == (0.0, 0.0, MODE_ESTOP)
    a.on_estop(False)
    assert a.evaluate(t) == (0.0, 0.0, MODE_HOLD)
    a.on_manual_thrust(100.0, 100.0, t)
    a.on_control_mode(CMD_MANUAL, t)
    assert a.evaluate(t)[2] == MODE_MANUAL


def test_startup_is_auto_so_cli_missions_work_without_a_browser():
    a = ControlArbitration()
    a.on_waypoint_thrust(60.0, 40.0, 0.0)
    assert a.evaluate(0.0)[2] == MODE_WAYPOINT


def test_stale_sources_fall_to_zero():
    a = ControlArbitration()
    a.on_waypoint_thrust(60.0, 40.0, 0.0)
    assert a.evaluate(5.0) == (0.0, 0.0, MODE_IDLE)


# Non-finite rejection

def test_sanitize_rejects_non_finite():
    assert sanitize_thrust(float('nan'), 0.0) is None
    assert sanitize_thrust(0.0, float('inf')) is None
    assert sanitize_thrust(float('-inf'), 0.0) is None
    assert sanitize_thrust('banana', 0.0) is None


def test_sanitize_clamps():
    assert sanitize_thrust(500.0, -500.0) == (100.0, -100.0)


def test_nan_command_is_dropped_not_published():
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    assert a.on_manual_thrust(float('nan'), 0.0, t) is False
    # Dropped entirely, so it doesn't count as a freshness heartbeat.
    left, right, mode = a.evaluate(t)
    assert (left, right) == (0.0, 0.0)
    assert a.rejected_manual == 1


# Mission state machine

def test_mission_lockout_and_cancel_recovery():
    m = MissionStateMachine()
    m.finish_init()
    assert m.state == IDLE

    accepted, _ = m.request_mission(3, 0.0)
    assert accepted and m.state == EXECUTE

    # This is the reported symptom: a second route is refused.
    accepted, reason = m.request_mission(2, 1.0)
    assert not accepted and reason == REASON_BAD_STATE

    # Cancel lands in IDLE, not ABORTED.
    ok, _ = m.cancel(2.0)
    assert ok and m.state == IDLE

    accepted, _ = m.request_mission(2, 3.0)
    assert accepted and m.state == EXECUTE


def test_mission_complete_returns_to_idle():
    m = MissionStateMachine()
    m.finish_init()
    m.request_mission(2, 0.0)
    changed, note = m.on_feedback('WAYPOINT_REACHED 0')
    assert not changed and note == 'progress'
    changed, note = m.on_feedback('MISSION_COMPLETE')
    assert changed and m.state == IDLE
    assert m.request_mission(4, 1.0)[0]


def test_estop_blocks_and_aborts():
    m = MissionStateMachine()
    m.finish_init()
    m.request_mission(2, 0.0)
    assert m.on_estop(True) and m.state == ABORTED
    accepted, reason = m.request_mission(2, 1.0)
    assert not accepted and reason == REASON_ESTOP
    m.on_estop(False)
    assert m.request_mission(2, 2.0)[0]


def test_execute_watchdog():
    m = MissionStateMachine(timeout_sec=120.0)
    m.finish_init()
    m.request_mission(2, 0.0)
    assert m.tick(60.0) is None
    assert m.tick(121.0) is not None
    assert m.state == ABORTED
    assert m.accepts_missions()


# Full operator loop

def test_full_operator_loop():
    """launch, ready, upload, drive, manual, release, resume, second route,
    with no restart. The follower is simulated as publishing waypoint thrust
    while the mission machine is in EXECUTE."""
    a = ControlArbitration(lease_timeout_sec=1.0)
    m = MissionStateMachine()
    t = 0.0
    log = []

    def step(seconds=0.2, follower_drives=None):
        """Advance one arbiter tick. The follower publishes only in EXECUTE."""
        nonlocal t
        t += seconds
        driving = (m.state == EXECUTE) if follower_drives is None else follower_drives
        if driving:
            a.on_waypoint_thrust(60.0, 40.0, t)
        out = a.evaluate(t)
        log.append((round(t, 2), m.state, out[2], out[0], out[1]))
        return out

    # -- launch -> ready
    m.finish_init()
    assert m.state == IDLE
    assert step()[2] == MODE_IDLE

    # -- upload route 1
    assert m.request_mission(3, t)[0]
    assert m.state == EXECUTE

    # -- boat actually drives it
    for _ in range(5):
        left, right, mode = step()
    assert mode == MODE_WAYPOINT and (left, right) == (60.0, 40.0), \
        'route accepted but the arbiter is not selecting the follower'

    # -- operator takes manual control
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(-100.0, 100.0, t)
    left, right, mode = step()
    assert mode == MODE_MANUAL and (left, right) == (-100.0, 100.0)

    # ...holds it for a bit, heartbeating
    for _ in range(5):
        a.on_control_mode(CMD_MANUAL, t)
        a.on_manual_thrust(-100.0, 100.0, t)
        assert step()[2] == MODE_MANUAL

    # -- releases. Autonomy must NOT pick straight back up.
    a.on_control_mode(CMD_RELEASE, t)
    for _ in range(5):
        left, right, mode = step()
        assert mode == MODE_HOLD, 'release silently resumed autonomy'
        assert (left, right) == (0.0, 0.0)

    # -- explicit resume
    a.on_control_mode(CMD_RESUME, t)
    left, right, mode = step()
    assert mode == MODE_WAYPOINT and (left, right) == (60.0, 40.0)

    # Mission 1 finishes. waypoint_follower publishes a final 0/0 before
    # going quiet.
    changed, note = m.on_feedback('MISSION_COMPLETE')
    assert changed and note == 'complete' and m.state == IDLE
    a.on_waypoint_thrust(0.0, 0.0, t)
    left, right, mode = step(follower_drives=False)
    assert (left, right) == (0.0, 0.0)

    # -- upload route 2, no restart of anything
    accepted, reason = m.request_mission(2, t)
    assert accepted, f'second route refused: {reason}'
    assert m.state == EXECUTE
    for _ in range(5):
        left, right, mode = step()
    assert mode == MODE_WAYPOINT and (left, right) == (60.0, 40.0)

    return log


def _run_standalone():
    """Run all tests without pytest and print a trace of the loop."""
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            print('  PASS  %s' % name)
        except AssertionError as exc:
            failures += 1
            print('  FAIL  %s\n        %s' % (name, exc))
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print('  ERROR %s\n        %r' % (name, exc))

    print('\n%d/%d passed\n' % (len(tests) - failures, len(tests)))

    print('Trace of the target loop (time, mission state, arbiter mode, L, R):')
    print('  %-8s %-9s %-9s %7s %7s' % ('t', 'state', 'mode', 'left', 'right'))
    prev = None
    for row in test_full_operator_loop():
        key = (row[1], row[2])
        marker = '  <-- change' if key != prev else ''
        print('  %-8.2f %-9s %-9s %7.1f %7.1f%s'
              % (row[0], row[1], row[2], row[3], row[4], marker))
        prev = key
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(_run_standalone())
