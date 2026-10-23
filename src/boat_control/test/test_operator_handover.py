"""Deterministic operator handover tests; run with Python, no ROS required."""
import ast
import math
from pathlib import Path
import sys
import time
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from boat_control.arbitration import ControlArbitration
from boat_control.mission_state import MissionStateMachine


def command(a, seq, intent, now, transition=True, session='pilot', thrust=0):
    return a.on_operator_command(dict(session=session, seq=seq, intent=intent,
                                     transition=transition, left=thrust, right=thrust), now)


class HandoverTests(unittest.TestCase):
    def test_manual_route_takeover_hold_continue(self):
        a, m = ControlArbitration(), MissionStateMachine(timeout_sec=10)
        m.finish_init()
        self.assertTrue(command(a, 1, 'MANUAL', 0, thrust=30)[0])
        self.assertEqual(a.evaluate(0), (30, 30, 'MANUAL'))
        command(a, 2, 'HOLD', .1)
        m.set_paused(True, .1)
        self.assertTrue(m.request_mission(3, .1)[0])
        command(a, 3, 'AUTO', .2)
        a.on_waypoint_thrust(40, 30, .2)
        m.set_paused(False, .2)
        self.assertEqual(a.evaluate(.2), (40, 30, 'WAYPOINT'))
        command(a, 4, 'MANUAL', .3, thrust=-20)
        m.set_paused(True, .3)
        self.assertEqual(a.evaluate(.3), (-20, -20, 'MANUAL'))
        command(a, 5, 'HOLD', .4)
        self.assertEqual(a.evaluate(.4), (0, 0, 'HOLD'))
        self.assertIsNone(m.tick(100))
        self.assertEqual(m.active_point_count, 3)
        command(a, 6, 'AUTO', 100)
        self.assertEqual(a.evaluate(100), (0, 0, 'IDLE'))
        m.set_paused(False, 100)
        a.on_waypoint_thrust(20, 40, 100)
        self.assertEqual(a.evaluate(100), (20, 40, 'WAYPOINT'))
        self.assertIsNone(m.tick(109))
        self.assertIsNotNone(m.tick(111))

    def test_old_manual_packet_cannot_undo_hold_or_resume(self):
        a = ControlArbitration()
        command(a, 1, 'MANUAL', 0, thrust=50)
        command(a, 3, 'HOLD', .2)
        self.assertFalse(command(a, 2, 'MANUAL', .3, False, thrust=90)[0])
        self.assertEqual(a.evaluate(.3), (0, 0, 'HOLD'))
        command(a, 5, 'AUTO', .4)
        self.assertFalse(command(a, 4, 'HOLD', .5)[0])
        a.on_waypoint_thrust(20, 20, .5)
        self.assertEqual(a.evaluate(.5), (20, 20, 'WAYPOINT'))

    def test_link_loss_in_manual_and_auto_requires_explicit_reentry(self):
        for intent in ('MANUAL', 'AUTO'):
            a = ControlArbitration()
            command(a, 1, 'HOLD', 0)
            command(a, 2, intent, .1, thrust=50)
            a.on_waypoint_thrust(60, 60, 2)
            a.on_failsafe_level(2, 2)
            a.on_failsafe_thrust(30, 30, 2)
            self.assertEqual(a.evaluate(2), (0, 0, 'HOLD'))
            self.assertFalse(command(a, 3, intent, 2, False, thrust=50)[0])
            self.assertEqual(a.evaluate(2), (0, 0, 'HOLD'))
            self.assertTrue(command(a, 4, 'MANUAL', 2, thrust=10)[0])
            self.assertEqual(a.evaluate(2), (10, 10, 'MANUAL'))

    def test_reset_cannot_restore_old_thrust(self):
        a = ControlArbitration()
        command(a, 1, 'MANUAL', 0, thrust=80)
        command(a, 2, 'ESTOP', .1)
        self.assertEqual(a.evaluate(.1), (0, 0, 'ESTOP'))
        command(a, 3, 'RESET', .2)
        a.on_waypoint_thrust(100, 100, .2)
        self.assertEqual(a.evaluate(.2), (0, 0, 'HOLD'))
        self.assertFalse(command(a, 4, 'MANUAL', .3, False, thrust=80)[0])

    def test_old_browser_and_legacy_topic_cannot_reclaim_control(self):
        a = ControlArbitration()
        command(a, 1, 'MANUAL', 0, session='one', thrust=30)
        command(a, 1, 'MANUAL', .1, session='two', thrust=20)
        self.assertFalse(command(a, 100, 'MANUAL', .2, session='one', thrust=90)[0])
        self.assertFalse(a.on_manual_thrust(90, 90, .2))
        self.assertFalse(a.on_control_mode('RESUME', .2))
        self.assertEqual(a.evaluate(.2), (20, 20, 'MANUAL'))

    def test_safety_stop_beats_manual_and_stale_supervisor_stays_stopped(self):
        a = ControlArbitration()
        command(a, 1, 'MANUAL', 0, thrust=50)
        a.on_failsafe_level(3, 0)
        self.assertEqual(a.evaluate(0), (0, 0, 'FAILSAFE'))
        a.on_failsafe_level(0, .1)
        # Even a previously nominal safety feed must not fail open.
        command(a, 2, 'MANUAL', 4, thrust=50)
        self.assertEqual(a.evaluate(4), (0, 0, 'FAILSAFE'))

    def test_invalid_commands_and_duplicate_sequence_do_not_move(self):
        a = ControlArbitration()
        command(a, 1, 'HOLD', 0)
        self.assertFalse(command(a, 1, 'MANUAL', 0, thrust=100)[0])
        self.assertFalse(command(a, 2, 'MANUAL', 0, thrust=float('nan'))[0])
        self.assertFalse(a.on_operator_command([], 0)[0])
        self.assertEqual(a.evaluate(0), (0, 0, 'HOLD'))

    def test_pause_cancel_and_new_mission_do_not_inherit_old_pause_time(self):
        m = MissionStateMachine(timeout_sec=10)
        m.finish_init()
        m.request_mission(2, 0)
        m.set_paused(True, 1)
        m.cancel(2)
        m.request_mission(3, 100)
        m.set_paused(False, 110)
        self.assertIsNone(m.tick(119))
        self.assertIsNotNone(m.tick(121))


def follower_class():
    """Load the actual node methods without ROS imports; transport is not tested."""
    path = Path(__file__).resolve().parents[2] / 'boat_navigation/boat_navigation/waypoint_follower.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    tree.body = [ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)] + [
        n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
    ns = dict(Node=object, math=math, time=time, json=__import__('json'),
              String=lambda **kw: types.SimpleNamespace(**kw))
    exec(compile(ast.fix_missing_locations(tree), str(path), 'exec'), ns)
    return ns['WaypointFollower']


class NavigationPauseTests(unittest.TestCase):
    def make_follower(self):
        cls = follower_class()
        f = cls.__new__(cls)
        f.mission_active = True
        f.boat_state = 'EXECUTE'
        f._seen_execute = True
        f.current_waypoint_idx = 0
        f.waypoints = [(10., 20.), (11., 21.)]
        f.current_fix = (10., 20.)  # exactly on target
        f.last_fix_time = None
        f._closest_dist = .1
        f.control_mode = 'MANUAL'
        f.last_mode_at = time.monotonic()
        f.output, f.feedback = [], []
        f._publish_thrust = lambda l, r: f.output.append((l, r))
        f.get_parameter = lambda name: types.SimpleNamespace(value=5)
        f.get_logger = lambda: types.SimpleNamespace(info=lambda *a, **k: None)
        f.feedback_pub = types.SimpleNamespace(publish=f.feedback.append)
        return f

    def test_manual_hold_failsafe_and_stale_mode_do_not_advance_target(self):
        for mode in ('MANUAL', 'HOLD', 'FAILSAFE', 'ESTOP', None):
            f = self.make_follower()
            f.control_mode = mode
            f.control_loop()
            self.assertEqual(f.current_waypoint_idx, 0)
            self.assertEqual(f.feedback, [])
            self.assertTrue(f.mission_active)
            self.assertEqual(f.output[-1], (0, 0))
        f.control_mode = 'WAYPOINT'
        f.last_mode_at = time.monotonic() - 2
        f.control_loop()
        self.assertEqual(f.current_waypoint_idx, 0)

    def test_continue_resumes_same_target_and_can_then_advance(self):
        f = self.make_follower()
        f.control_loop()
        f.mode_callback(types.SimpleNamespace(data='IDLE'))
        f.control_loop()
        self.assertEqual(f.current_waypoint_idx, 1)
        self.assertEqual(f.feedback[0].data, 'WAYPOINT_REACHED 0')

    def test_abort_while_paused_still_clears_active_mission(self):
        f = self.make_follower()
        f.boat_state = 'ABORTED'
        f.control_loop()
        self.assertFalse(f.mission_active)
        self.assertEqual(f.output[-1], (0, 0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
