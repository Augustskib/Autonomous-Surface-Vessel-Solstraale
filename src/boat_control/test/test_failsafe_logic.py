"""Logic tests for the geofence and the graded failsafe levels.

Plain Python with a fake clock. These test failsafe.py and the failsafe
priority in arbitration.py, not ROS wiring, Gazebo or timing under load;
scripts/sim/verify_failsafe_and_heading.sh covers the running system.

Run without ROS:
    python3 -m pytest src/boat_control/test/test_failsafe_logic.py -v
    python3 src/boat_control/test/test_failsafe_logic.py   (prints a trace)
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from boat_control.arbitration import (  # noqa: E402
    CMD_MANUAL, CMD_RELEASE,
    ControlArbitration,
    MODE_ESTOP, MODE_FAILSAFE, MODE_HOLD, MODE_MANUAL, MODE_WAYPOINT,
)
from boat_control.failsafe import (  # noqa: E402
    FailsafeSupervisor,
    GeofenceConfig,
    LEVEL_HOLD_POSITION,
    LEVEL_NOMINAL,
    LEVEL_POWERED_STOP,
    LEVEL_RETURN_TO_LAUNCH,
    REASON_GCS_LINK_LOST,
    REASON_GCS_LINK_LOST_PROLONGED,
    REASON_GEOFENCE_BREACH,
    haversine_distance_m,
    normalize_angle_deg,
    rtl_thrust,
)

# A test fence away from the simulation default.
CENTRE_LAT, CENTRE_LON = 60.391300, 5.322054
RADIUS_M = 150.0

# Metres per degree of latitude, near enough for building test positions.
M_PER_DEG_LAT = 111320.0


def at_distance_north(metres, lat=CENTRE_LAT, lon=CENTRE_LON):
    """A position `metres` due north of the fence centre."""
    return (lat + metres / M_PER_DEG_LAT, lon)


def make_supervisor(**kwargs):
    fence = GeofenceConfig(enabled=True, center_lat=CENTRE_LAT,
                           center_lon=CENTRE_LON, radius_m=RADIUS_M,
                           warn_fraction=0.9)
    sup = FailsafeSupervisor(fence=fence, **kwargs)
    sup.set_launch_point(CENTRE_LAT, CENTRE_LON)
    return sup


def good_heading(sup, deg=0.0):
    """A FUSED-quality heading, i.e. one the boat may act on."""
    sup.on_heading(deg, usable=True, quality=2, detail='')


# A breach keeps asserting while the boat is outside

def test_breach_is_level_triggered_not_one_shot():
    """The boat crosses the fence and keeps drifting outward. The failsafe
    must assert at every step, including 400 m out. (An edge-triggered
    fence fires once and never again.)"""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0

    # Comfortably inside: nothing should be happening.
    sup.on_position(*at_distance_north(50.0))
    assert sup.evaluate(t).level == LEVEL_NOMINAL

    # Cross the boundary and keep drifting outward, forever.
    for i, dist in enumerate([160.0, 180.0, 220.0, 300.0, 400.0, 900.0]):
        sup.on_position(*at_distance_north(dist))
        t += 0.5
        d = sup.evaluate(t)
        if i >= 2:      # past the 3-sample debounce
            assert d.level == LEVEL_RETURN_TO_LAUNCH, (
                'fence stopped asserting at %.0f m out: level=%s'
                % (dist, d.level_name))
            assert d.commanding_thrust, (
                'no recovery thrust at %.0f m out -- this is the exact '
                'failure: detected the boat leaving and then did nothing'
                % dist)

    # And the reported time-outside keeps growing, which is the visible
    # evidence on the dashboard that the condition is still live.
    assert sup.last_fence.outside_sec > 2.0
    assert sup.last_fence.breached


def test_breach_debounce_ignores_a_single_bad_sample():
    """A single glitched fix must not trip the failsafe (debounce)."""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(5):
        sup.on_position(*at_distance_north(100.0))
        t += 0.5
        assert sup.evaluate(t).level == LEVEL_NOMINAL

    # One absurd sample.
    sup.on_position(0.0, 0.0)
    t += 0.5
    assert sup.evaluate(t).level == LEVEL_NOMINAL, 'one bad fix tripped the fence'

    # Back to normal.
    sup.on_position(*at_distance_north(100.0))
    t += 0.5
    assert sup.evaluate(t).level == LEVEL_NOMINAL


def test_warning_ring_fires_before_the_hard_fence():
    """The soft ring must warn while there is still room to steer away."""
    sup = make_supervisor()
    good_heading(sup)

    # Inside the warning ring (135 m for a 150 m fence at 0.9).
    sup.on_position(*at_distance_north(120.0))
    sup.evaluate(0.0)
    assert not sup.last_fence.warning
    assert not sup.last_fence.breached

    # Between the warning ring and the fence: warn, but do not act.
    sup.on_position(*at_distance_north(140.0))
    d = sup.evaluate(0.5)
    assert sup.last_fence.warning, 'no warning inside the warning band'
    assert not sup.last_fence.breached
    assert d.level == LEVEL_NOMINAL, 'a warning must not trip a failsafe'

    # The published warning radius follows the configured fraction.
    sup.set_fence(GeofenceConfig(True, CENTRE_LAT, CENTRE_LON, 150.0, 0.5))
    assert abs(sup.fence.warn_radius_m - 75.0) < 1e-6
    sup.on_position(*at_distance_north(100.0))
    sup.evaluate(1.0)
    assert sup.last_fence.warning, 'warn_fraction is not being honoured'


# Response chosen by what the boat still knows

def test_breach_returns_to_launch_when_heading_is_usable():
    sup = make_supervisor()
    good_heading(sup, deg=0.0)     # pointing north, i.e. further out
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)

    assert d.level == LEVEL_RETURN_TO_LAUNCH
    assert d.reason == REASON_GEOFENCE_BREACH
    assert d.commanding_thrust
    assert d.degraded_reason == ''
    # Home is due south with the bow pointing north, so a hard turn.
    # Positive heading error (target clockwise of the bow) makes LEFT stronger.
    assert abs(d.bearing_to_launch_deg - 180.0) < 1.0
    assert d.left != d.right, 'no turn commanded despite a 180 degree error'


def test_breach_degrades_to_powered_stop_without_heading_and_says_why():
    """Without a usable heading, RTL degrades to a powered stop and says why.
    RTL on a drifted heading drives past home and drains the battery."""
    sup = make_supervisor()
    sup.on_heading(None, usable=False, quality=0,
                   detail='no absolute reference has ever been received')
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)

    assert d.level == LEVEL_POWERED_STOP
    assert d.requested_level == LEVEL_RETURN_TO_LAUNCH, (
        'the degradation must record what it WANTED to do')
    assert d.degraded_reason, 'degraded silently -- the whole point is that it does not'
    assert 'heading' in d.degraded_reason.lower()
    assert not d.commanding_thrust
    assert (d.left, d.right) == (0.0, 0.0)


def test_dead_reckoning_heading_is_still_good_enough_to_come_home():
    """Coasting on the gyro is degraded but still usable for RTL. The coast
    timeout in heading.py is what eventually drops quality to NONE."""
    sup = make_supervisor()
    sup.on_heading(10.0, usable=True, quality=1, detail='coasting on gyro')
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH
    assert d.commanding_thrust


def test_no_position_degrades_to_powered_stop():
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        sup.evaluate(t)
    # GPS drops out while the failsafe is latched at RTL.
    sup.on_position_lost()
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == LEVEL_POWERED_STOP
    assert not d.commanding_thrust
    assert 'position' in d.degraded_reason.lower()


def test_regaining_a_heading_upgrades_a_powered_stop_to_return_home():
    """POWERED_STOP must be able to move up to RETURN_TO_LAUNCH once a
    heading recovers. The engagement latches, the response is recomputed
    every tick. Latching the level itself stranded the boat at zero thrust."""
    sup = make_supervisor()
    sup.on_heading(None, usable=False, quality=0, detail='quality NONE')
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.level == LEVEL_POWERED_STOP
    assert sup.engaged

    # The estimator acquires course over ground as the boat drifts.
    good_heading(sup, deg=0.0)
    sup.on_position(*at_distance_north(210.0))
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH, (
        'stayed stopped after regaining a heading -- the boat could have '
        'come home and did not')
    assert d.commanding_thrust
    assert sup.engaged, 'the latch itself must still be engaged'
    # And it still will not clear itself.
    assert not sup.clear(t)[0]


def test_no_launch_point_degrades_and_explains():
    sup = make_supervisor()
    sup.launch_point = None
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.level == LEVEL_POWERED_STOP
    assert 'launch point' in d.degraded_reason.lower()


def test_hold_position_is_honestly_unimplemented():
    """Station-keeping is not implemented, so HOLD_POSITION must degrade
    with a stated reason."""
    sup = make_supervisor()
    good_heading(sup)
    sup.on_position(*at_distance_north(10.0))
    sup.evaluate(0.0)
    level, why = sup.select_level(LEVEL_HOLD_POSITION, 0.0)
    assert level == LEVEL_RETURN_TO_LAUNCH
    assert 'not implemented' in why.lower()


# Recovery, the latch, and clearing

def test_rtl_stops_commanding_on_arrival_but_stays_latched():
    """Getting home is not the same statement as 'safe to carry on'."""
    sup = make_supervisor()
    good_heading(sup, deg=180.0)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH and d.commanding_thrust

    # RTL succeeds: the boat is back at the launch point.
    sup.on_position(CENTRE_LAT, CENTRE_LON)
    t += 0.5
    d = sup.evaluate(t)
    assert d.arrived_at_launch
    assert not d.commanding_thrust, 'still driving after arriving'
    assert d.level == LEVEL_RETURN_TO_LAUNCH, (
        'de-escalated on its own -- escalation is automatic, coming back '
        'down must be an operator decision')
    assert d.latched
    assert d.conditions_clear, 'should report the condition as gone'


def test_operator_clear_refused_while_still_outside_then_accepted():
    """A clear that silently does nothing would be worse than a refusal:
    the operator would believe the boat was back under their control."""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        sup.evaluate(t)

    ok, why = sup.clear(t)
    assert not ok
    assert 'geofence' in why.lower() or 'outside' in why.lower()
    assert sup.level == LEVEL_RETURN_TO_LAUNCH

    # Back inside, and the condition clears -- but the latch holds.
    sup.on_position(*at_distance_north(50.0))
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH, 'latch released itself'
    assert d.conditions_clear

    ok, why = sup.clear(t)
    assert ok, 'clear refused with no condition active: %s' % why
    assert sup.level == LEVEL_NOMINAL

    t += 0.5
    assert sup.evaluate(t).level == LEVEL_NOMINAL


def test_force_clear_requires_nothing_but_is_counted():
    """Forced clear works; the caller must require and record a reason."""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        sup.evaluate(t)
    assert not sup.clear(t)[0]
    ok, _ = sup.force_clear(t)
    assert ok and sup.level == LEVEL_NOMINAL and sup.clears == 1


def test_moving_the_fence_does_not_cancel_an_active_failsafe():
    """Realising the fence was pointed at the wrong country and fixing it
    must not silently clear a failsafe that is currently holding."""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        sup.evaluate(t)
    assert sup.level == LEVEL_RETURN_TO_LAUNCH

    # Widen the fence so the boat is now inside it.
    sup.set_fence(GeofenceConfig(True, CENTRE_LAT, CENTRE_LON, 500.0, 0.9))
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH, 'latch cancelled by a fence change'
    assert d.conditions_clear, 'condition should now read as clear'
    assert sup.clear(t)[0], 'operator should be able to clear it deliberately'


def test_disabled_fence_never_breaches():
    sup = make_supervisor()
    good_heading(sup)
    sup.set_fence(GeofenceConfig(False, CENTRE_LAT, CENTRE_LON, 150.0, 0.9,
                                 disabled_reason='testing in a marina'))
    t = 0.0
    for _ in range(10):
        sup.on_position(*at_distance_north(5000.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.level == LEVEL_NOMINAL
    assert not sup.last_fence.evaluated
    assert sup.fence.disabled_reason == 'testing in a marina'


# E-stop

def test_estop_overrides_everything_and_commands_nothing():
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.commanding_thrust, 'precondition: RTL should be driving'

    sup.on_estop(True)
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == 4                    # HARD_STOP
    assert not d.commanding_thrust
    assert (d.left, d.right) == (0.0, 0.0)
    assert d.reason == 'ESTOP'


def test_clearing_estop_restores_the_previous_situation():
    """E-stop is an output override, not a failsafe state, so clearing it
    doesn't leave a second latch that needs its own clear."""
    sup = make_supervisor()
    good_heading(sup)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        sup.evaluate(t)

    sup.on_estop(True)
    t += 0.5
    assert sup.evaluate(t).level == 4

    sup.on_estop(False)
    t += 0.5
    d = sup.evaluate(t)
    assert d.level == LEVEL_RETURN_TO_LAUNCH, (
        'clearing the e-stop should restore the failsafe situation that '
        'was in effect before it, not leave HARD_STOP latched')
    assert d.commanding_thrust


def test_estop_blocks_even_a_forced_clear():
    sup = make_supervisor()
    sup.on_estop(True)
    sup.evaluate(0.0)
    ok, why = sup.force_clear(0.5)
    assert not ok and 'emergency stop' in why.lower()


# Link loss

def test_link_loss_stops_then_escalates_to_return_home():
    """Short losses are usually transient; prolonged ones are not."""
    sup = make_supervisor(gcs_rtl_after_sec=30.0)
    good_heading(sup)
    sup.on_position(*at_distance_north(50.0))

    sup.on_gcs_link(True, 100.0)
    d = sup.evaluate(101.0)
    assert d.level == LEVEL_POWERED_STOP
    assert d.reason == REASON_GCS_LINK_LOST
    assert not d.commanding_thrust

    # Still gone half a minute later: bring it home rather than let it
    # drift indefinitely.
    d = sup.evaluate(135.0)
    assert d.level == LEVEL_RETURN_TO_LAUNCH
    assert d.reason == REASON_GCS_LINK_LOST_PROLONGED
    assert d.commanding_thrust


def test_geofence_outranks_a_short_link_loss():
    """When several conditions are live, the most severe selects."""
    sup = make_supervisor(gcs_rtl_after_sec=30.0)
    good_heading(sup)
    sup.on_gcs_link(True, 0.0)
    t = 0.0
    for _ in range(4):
        sup.on_position(*at_distance_north(200.0))
        t += 0.5
        d = sup.evaluate(t)
    assert d.reason == REASON_GEOFENCE_BREACH
    assert d.level == LEVEL_RETURN_TO_LAUNCH
    assert len(d.active_reasons) == 2, 'both conditions should be reported'


# RTL controller

def test_rtl_steers_the_short_way_round():
    """Circular-angle handling: heading 350, target 10 is a 20 degree
    RIGHT turn, not a 340 degree left one."""
    lat, lon = at_distance_north(-100.0)     # 100 m SOUTH of home
    left, right, dist, brg, arrived = rtl_thrust(lat, lon, CENTRE_LAT, CENTRE_LON,
                                                 heading_deg=350.0)
    assert not arrived
    assert abs(brg - 0.0) < 1.0 or abs(brg - 360.0) < 1.0
    err = normalize_angle_deg(brg - 350.0)
    assert 5.0 < err < 15.0, 'expected a small right turn, got %.1f' % err
    assert left > right, 'a right turn needs the LEFT thruster stronger'


def test_rtl_stops_inside_the_arrival_radius():
    lat, lon = at_distance_north(3.0)
    left, right, dist, _brg, arrived = rtl_thrust(
        lat, lon, CENTRE_LAT, CENTRE_LON, heading_deg=0.0,
        arrival_radius_m=8.0)
    assert arrived and (left, right) == (0.0, 0.0) and dist < 8.0


def test_rtl_always_keeps_some_forward_component():
    """RTL keeps some forward thrust. Pivoting in place gives no course over
    ground, which is the heading estimator's only absolute reference."""
    lat, lon = at_distance_north(200.0)
    left, right, _d, _b, _a = rtl_thrust(lat, lon, CENTRE_LAT, CENTRE_LON,
                                         heading_deg=0.0)   # 180 deg error
    assert (left + right) / 2.0 > 0.0, 'commanded a pure pivot'


def test_haversine_sanity():
    d = haversine_distance_m(*at_distance_north(0.0), *at_distance_north(100.0))
    assert abs(d - 100.0) < 1.0, 'distance maths is off: %.2f' % d


# Arbiter priority

def test_manual_operator_outranks_the_failsafe():
    """An operator holding the manual lease outranks the RTL controller."""
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_manual_thrust(80.0, 20.0, t)
    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_failsafe_thrust(-50.0, 50.0, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_MANUAL and (left, right) == (80.0, 20.0)


def test_failsafe_outranks_the_hold_latch():
    """The failsafe outranks HOLD: HOLD blocks the mission, not recovery."""
    a = ControlArbitration()
    t = 0.0
    a.on_control_mode(CMD_MANUAL, t)
    a.on_control_mode(CMD_RELEASE, t)        # -> HOLD, latched zero
    assert a.evaluate(t)[2] == MODE_HOLD

    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_failsafe_thrust(45.0, 30.0, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_FAILSAFE and (left, right) == (45.0, 30.0)


def test_failsafe_outranks_the_waypoint_follower():
    a = ControlArbitration()
    t = 0.0
    a.on_waypoint_thrust(60.0, 60.0, t)
    assert a.evaluate(t)[2] == MODE_WAYPOINT
    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_failsafe_thrust(20.0, 40.0, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_FAILSAFE and (left, right) == (20.0, 40.0)


def test_powered_stop_zeroes_rather_than_falling_through_to_autonomy():
    """POWERED_STOP publishes no thrust but must still outrank the waypoint
    follower."""
    a = ControlArbitration()
    t = 0.0
    a.on_waypoint_thrust(60.0, 60.0, t)
    a.on_failsafe_level(LEVEL_POWERED_STOP, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_FAILSAFE and (left, right) == (0.0, 0.0)


def test_estop_outranks_the_failsafe_in_the_arbiter_too():
    a = ControlArbitration()
    t = 0.0
    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_failsafe_thrust(45.0, 30.0, t)
    a.on_estop(True)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_ESTOP and (left, right) == (0.0, 0.0)


def test_stale_failsafe_level_fails_open():
    """A stale failsafe level is forgotten (fail open). The e-stop is the
    mechanism that has to survive a dead safety node."""
    a = ControlArbitration(failsafe_timeout_sec=3.0)
    t = 0.0
    a.on_waypoint_thrust(60.0, 60.0, t)
    a.on_failsafe_level(LEVEL_POWERED_STOP, t)
    assert a.evaluate(t)[2] == MODE_FAILSAFE

    # safety_monitor dies. Keep the waypoint source alive.
    for _ in range(30):
        t += 0.2
        a.on_waypoint_thrust(60.0, 60.0, t)
        mode = a.evaluate(t)[2]
    assert mode == MODE_WAYPOINT, 'a dead supervisor pinned the boat forever'
    assert a.failsafe_expiries == 1


def test_failsafe_holds_zero_if_its_thrust_gaps():
    """The level says the failsafe is driving but its commands have
    gapped. Hold zero rather than hand the boat back mid-recovery."""
    a = ControlArbitration()
    t = 0.0
    a.on_waypoint_thrust(60.0, 60.0, t)
    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_failsafe_thrust(40.0, 40.0, t)
    assert a.evaluate(t)[2] == MODE_FAILSAFE

    # Refresh the LEVEL but not the thrust, past the source timeout.
    t += 1.0
    a.on_failsafe_level(LEVEL_RETURN_TO_LAUNCH, t)
    a.on_waypoint_thrust(60.0, 60.0, t)
    left, right, mode = a.evaluate(t)
    assert mode == MODE_FAILSAFE and (left, right) == (0.0, 0.0)


# Scenario trace for the standalone runner

def scenario_breach_and_recovery():
    """Printable trace: wind pushes the boat out, the fence keeps asserting,
    the boat drives home, the operator clears. Then again without a heading,
    which must stop and say why."""
    log = []
    sup = make_supervisor()
    a = ControlArbitration()
    good_heading(sup, deg=0.0)
    t = 0.0

    def step(dist_m, note=''):
        nonlocal t
        sup.on_position(*at_distance_north(dist_m))
        t += 0.5
        d = sup.evaluate(t)
        a.on_failsafe_level(d.level, t)
        if d.commanding_thrust:
            a.on_failsafe_thrust(d.left, d.right, t)
        a.on_waypoint_thrust(60.0, 60.0, t)     # the mission, still trying
        left, right, mode = a.evaluate(t)
        log.append((t, dist_m, d.level_name, mode, left, right,
                    note or (d.degraded_reason or d.reason)))
        return d

    for dist in (100.0, 130.0, 145.0, 160.0, 170.0, 185.0, 200.0):
        step(dist)
    # RTL works: the boat comes back.
    for dist in (170.0, 140.0, 90.0, 30.0, 4.0):
        step(dist)
    d = step(4.0, 'arrived; still latched')
    assert d.conditions_clear
    ok, _ = sup.clear(t)
    assert ok
    step(4.0, 'operator cleared')
    return log


def scenario_breach_without_heading():
    log = []
    sup = make_supervisor()
    sup.on_heading(None, usable=False, quality=0,
                   detail='no absolute reference has ever been received')
    t = 0.0
    for dist in (100.0, 160.0, 170.0, 185.0, 250.0):
        sup.on_position(*at_distance_north(dist))
        t += 0.5
        d = sup.evaluate(t)
        log.append((t, dist, d.level_name, 'FAILSAFE' if d.level else '-',
                    d.left, d.right, d.degraded_reason or d.reason))
    return log


def _run_standalone():
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

    def trace(title, rows):
        print(title)
        print('  %-6s %8s  %-17s %-9s %7s %7s  %s'
              % ('t', 'dist_m', 'failsafe', 'arbiter', 'left', 'right', 'why'))
        prev = None
        for r in rows:
            key = (r[2], r[3])
            mark = ' <-' if key != prev else ''
            print('  %-6.1f %8.0f  %-17s %-9s %7.1f %7.1f  %s%s'
                  % (r[0], r[1], r[2], r[3], r[4], r[5], r[6][:58], mark))
            prev = key
        print()

    trace('Breach and recovery WITH a usable heading '
          '(fence 150m, warning ring 135m):', scenario_breach_and_recovery())
    trace('The same breach with NO usable heading -- note it stops, and says why:',
          scenario_breach_without_heading())
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(_run_standalone())
