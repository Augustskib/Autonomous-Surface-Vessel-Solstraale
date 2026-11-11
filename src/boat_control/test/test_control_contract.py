"""Hardware-default protocol regressions. No ROS or physical outputs."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from boat_control.arbitration import ControlArbitration
from boat_control.mission_state import MissionStateMachine


def packet(a, intent='MANUAL', seq=1, session='pilot', now=0, thrust=30):
    data = dict(session=session, seq=seq, intent=intent, transition=True,
                left=thrust, right=thrust)
    if intent in ('MANUAL', 'AUTO', 'RESET'):
        data['ticket'] = a.prepare_transition(data, now)
    return data


class ContractTests(unittest.TestCase):
    def ready(self):
        a = ControlArbitration()
        a.on_failsafe_level(0, 0)
        return a

    def test_start_and_restart_require_arming(self):
        for a in (ControlArbitration(), self.ready()):
            a.on_waypoint_thrust(40, 40, 0)
            self.assertEqual(a.evaluate(0), (0, 0, 'HOLD'))
            self.assertFalse(a.on_control_mode('RESUME', 0))
            self.assertFalse(a.on_manual_thrust(99, 99, 0))

    def test_no_supervisor_no_motion(self):
        a = ControlArbitration()
        self.assertFalse(a.on_operator_command(packet(a), 0)[0])
        self.assertEqual(a.evaluate(0)[:2], (0, 0))

    def test_previous_browser_can_always_assert_stop(self):
        a = self.ready()
        a.on_operator_command(packet(a, session='A'), 0)
        a.on_operator_command(packet(a, session='B'), 0)
        self.assertTrue(a.on_operator_command(packet(a, 'ESTOP', session='A'), 0)[0])
        self.assertEqual(a.evaluate(0), (0, 0, 'ESTOP'))

    def test_expired_transition_never_revives_motion(self):
        a = self.ready()
        delayed = packet(a)
        self.assertFalse(a.on_operator_command(delayed, 20)[0])
        self.assertEqual(a.evaluate(20)[:2], (0, 0))

    def test_ticket_bound_to_payload_and_one_use(self):
        a = self.ready()
        changed = packet(a)
        changed['left'] = 90
        self.assertFalse(a.on_operator_command(changed, 0)[0])
        good = packet(a)
        self.assertTrue(a.on_operator_command(good, 0)[0])
        good['seq'] = 2
        self.assertFalse(a.on_operator_command(good, 0)[0])

    def test_restart_and_takeover_invalidate_prepared_packets(self):
        a = self.ready()
        old = packet(a, session='A')
        b = self.ready()
        self.assertFalse(b.on_operator_command(old, 0)[0])
        a.on_operator_command(packet(a, session='B'), 0)
        self.assertFalse(a.on_operator_command(old, 0)[0])

    def test_heartbeat_generation_and_expiry(self):
        a = self.ready()
        d = packet(a)
        a.on_operator_command(d, 0)
        d.update(seq=2, transition=False, authority=a.authority)
        self.assertTrue(a.on_operator_command(d, .2)[0])
        a.evaluate(2)
        d['seq'] = 3
        self.assertFalse(a.on_operator_command(d, 2)[0])
        self.assertEqual(a.evaluate(2), (0, 0, 'HOLD'))

    def test_safety_clear_does_not_resume_auto(self):
        a = self.ready()
        a.on_operator_command(packet(a, 'AUTO'), 0)
        a.on_waypoint_thrust(40, 40, .1)
        self.assertEqual(a.evaluate(.1)[2], 'WAYPOINT')
        a.on_failsafe_level(3, .2)
        self.assertEqual(a.evaluate(.2)[:2], (0, 0))
        a.on_failsafe_level(0, .3)
        a.on_waypoint_thrust(40, 40, .3)
        self.assertEqual(a.evaluate(.3), (0, 0, 'HOLD'))

    def test_recovery_clear_also_requires_continue(self):
        a = self.ready()
        a.on_operator_command(packet(a, 'AUTO'), 0)
        a.on_failsafe_level(2, .1)
        a.on_failsafe_thrust(20, 20, .1)
        self.assertEqual(a.evaluate(.1), (20, 20, 'FAILSAFE'))
        a.on_failsafe_level(0, .2)
        a.on_waypoint_thrust(40, 40, .2)
        self.assertEqual(a.evaluate(.2), (0, 0, 'HOLD'))

    def test_manual_recovery_needs_explicit_permission(self):
        for allowed in (False, True):
            a = self.ready()
            a.on_failsafe_level(3, 0, allowed)
            ok, _ = a.on_operator_command(packet(a), 0)
            self.assertEqual(ok, allowed)
            self.assertEqual(a.evaluate(0)[:2], (30, 30) if allowed else (0, 0))

    def test_hard_fault_and_stale_monitor_block_manual_recovery(self):
        a = self.ready()
        a.on_failsafe_level(3, 0, True)
        a.on_operator_command(packet(a), 0)
        a.on_failsafe_level(4, .1, True)
        self.assertEqual(a.evaluate(.1)[:2], (0, 0))
        a.on_failsafe_level(3, .2, True)
        self.assertFalse(a.on_operator_command(packet(a, seq=2, now=4), 4)[0])

    def test_delayed_false_cannot_reset_stop(self):
        a = self.ready()
        a.on_estop(True)
        a.on_estop(False)
        self.assertEqual(a.evaluate(0)[2], 'ESTOP')
        a.on_operator_command(packet(a, 'HOLD'), 0)
        self.assertTrue(a.on_operator_command(packet(a, 'RESET', seq=2), 0)[0])
        self.assertEqual(a.evaluate(0), (0, 0, 'HOLD'))

    def test_old_and_anonymous_feedback_cannot_finish_new_mission(self):
        m = MissionStateMachine()
        m.finish_init()
        m.request_mission(2, 0, 'A')
        m.cancel(1, 'A')
        m.request_mission(3, 2, 'B')
        for identity in ('A', None):
            self.assertFalse(m.on_feedback('MISSION_COMPLETE', identity)[0])
            self.assertEqual(m.state, 'EXECUTE')
        self.assertFalse(m.cancel(3, 'A')[0])
        self.assertTrue(m.on_feedback('MISSION_COMPLETE', 'B')[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
