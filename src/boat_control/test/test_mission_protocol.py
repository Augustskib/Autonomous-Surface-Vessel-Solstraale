"""Exercise actual manager/follower/gateway methods with in-memory transport."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest
import uuid

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'boat_control'))
sys.path.insert(0, str(SRC / 'boat_web_ui/test'))
from boat_control.mission_state import MissionStateMachine
from test_operator_endpoint import methods
import test_operator_handover

Obj = types.SimpleNamespace


class MissionProtocolTests(unittest.TestCase):
    def setUp(self):
        self.f = test_operator_handover.NavigationPauseTests().make_follower()
        self.f.mission_active = False
        self.f.execution_id = self.f.status_execution_id = None
        self.f.status_manager_id = None
        self.f.status_generation = -1
        self.f.control_mode = 'HOLD'
        self.status = []
        manager_ns = dict(json=json, uuid=uuid, String=Obj, math=math, MissionCommand=Obj)
        methods(SRC / 'boat_control/boat_control/mission_manager.py',
                ['_publish_state', 'mission_callback', 'feedback_callback', 'cancel_callback'], manager_ns, 'MissionManager')
        self.m = Obj(logic=MissionStateMachine(), _now=time.monotonic,
                     state_pub=Obj(publish=lambda msg: None),
                     get_logger=lambda: Obj(info=lambda *a: None, warn=lambda *a: None))
        for name in ('_publish_state', 'mission_callback', 'feedback_callback', 'cancel_callback'):
            setattr(self.m, name, types.MethodType(manager_ns[name], self.m))
        self.m.logic.finish_init()
        self.ns = dict(json=json, uuid=uuid, math=math, time=time,
                       telemetry_lock=threading.Lock(), latest_mission={}, latest_mission_time=None,
                       mission_receipts={}, jsonify=lambda **kw: kw,
                       request=Obj(get_json=lambda **kw: self.body), _await=lambda fn: fn(),
                       String=Obj, MissionCommand=Obj)
        web = SRC / 'boat_web_ui/boat_web_ui/web_server.py'
        methods(web, ['mission', 'mission_cancel', '_mission_snapshot', '_mission_receipt'], self.ns)
        methods(web, ['mission_status_callback', 'publish_mission', 'publish_mission_cancel'], self.ns, 'WebServerNode')
        self.ros = Obj(mission_pub=Obj(publish=self.m.mission_callback),
                       mission_cancel_pub=Obj(publish=self.m.cancel_callback))
        self.ros.publish_mission = types.MethodType(self.ns['publish_mission'], self.ros)
        self.ros.publish_mission_cancel = types.MethodType(self.ns['publish_mission_cancel'], self.ros)
        self.ns['ros_node'] = self.ros
        def status(msg):
            self.status.append(json.loads(msg.data))
            self.f.state_callback(msg)
            self.ns['mission_status_callback'](None, msg)
        self.m.status_pub = Obj(publish=status)
        self.m.active_mission_pub = Obj(publish=self.f.mission_callback)
        self.m._publish_state()
        self.body = dict(latitudes=[10., 11.], longitudes=[20., 21.])

    def start(self):
        self.body = dict(latitudes=[10., 11.], longitudes=[20., 21.])
        response, code = self.ns['mission']()
        self.assertEqual(code, 200)
        return response['execution_id']

    def test_route_receipt_matches_real_execution_and_follower(self):
        identity = self.start()
        self.assertEqual(identity, self.m.logic.execution_id)
        self.assertEqual(identity, self.f.execution_id)
        self.f.control_mode = 'WAYPOINT'
        self.f.control_loop()
        feedback = json.loads(self.f.feedback[-1].data)
        self.assertEqual(feedback['execution_id'], identity)

    def test_old_feedback_and_cancel_cannot_terminate_replacement(self):
        old = self.start()
        old_cancel = dict(request_id='cancel-old', execution_id=old,
                          manager_id=self.m.logic.manager_id, generation=self.m.logic.generation)
        self.body = dict(execution_id=old)
        self.assertEqual(self.ns['mission_cancel']()[1], 200)
        current = self.start()
        self.m.feedback_callback(Obj(data=json.dumps(dict(execution_id=old, event='MISSION_COMPLETE'))))
        self.m.cancel_callback(Obj(data=json.dumps(old_cancel)))
        self.assertEqual(self.m.logic.state, 'EXECUTE')
        self.assertEqual(self.f.execution_id, current)
        self.body = dict(execution_id=old)
        self.assertEqual(self.ns['mission_cancel']()[1], 409)

    def test_route_waits_for_matching_status_and_stale_status_stops(self):
        self.start()
        self.f.control_mode = 'WAYPOINT'
        self.f.status_execution_id = 'previous'
        self.f.control_loop()
        self.assertEqual(self.f.current_waypoint_idx, 0)
        self.assertEqual(self.f.output[-1], (0, 0))
        self.f.status_execution_id = self.f.execution_id
        self.f.status_at = time.monotonic() - 3
        self.f.control_loop()
        self.assertEqual(self.f.current_waypoint_idx, 0)

    def test_duplicate_route_does_not_reset_progress(self):
        identity = self.start()
        self.f.current_waypoint_idx = 1
        self.f.mission_callback(Obj(execution_id=identity, manager_id=self.m.logic.manager_id,
                                   generation=self.m.logic.generation, latitudes=[10.,11.], longitudes=[20.,21.]))
        self.assertEqual(self.f.current_waypoint_idx, 1)

    def test_old_status_does_not_restore_cancelled_route(self):
        identity = self.start()
        previous = self.status[-1]
        self.body = dict(execution_id=identity)
        self.ns['mission_cancel']()
        self.f.state_callback(Obj(data=json.dumps(previous)))
        self.assertEqual(self.f.boat_state, 'IDLE')
        self.f.control_loop()
        self.assertEqual(self.f.output[-1], (0, 0))

    def test_missing_delivery_does_not_use_previous_execute_state(self):
        self.start()
        self.ros.mission_pub.publish = lambda msg: None
        self.assertEqual(self.ns['mission']()[1], 504)

    def test_cancel_before_first_navigation_tick_clears_route(self):
        identity = self.start()
        self.assertFalse(self.f._seen_execute)
        self.body = dict(execution_id=identity)
        self.assertEqual(self.ns['mission_cancel']()[1], 200)
        self.assertFalse(self.f.mission_active)
        self.assertEqual(self.f.waypoints, [])
        self.assertEqual(self.f.output[-1], (0, 0))

    def test_old_abort_status_cannot_erase_route_waiting_for_acceptance_status(self):
        self.start()
        self.f.boat_state = 'ABORTED'
        self.f.status_execution_id = 'old-run'
        self.f.control_loop()
        self.assertTrue(self.f.mission_active)
        self.assertEqual(len(self.f.waypoints), 2)
        self.assertEqual(self.f.output[-1], (0, 0))

    def test_manager_restart_rejects_pre_restart_route_request(self):
        old = Obj(execution_id='queued', manager_id=self.m.logic.manager_id,
                  generation=self.m.logic.generation, latitudes=[10.], longitudes=[20.])
        self.m.logic = MissionStateMachine()
        self.m.logic.finish_init()
        self.m.mission_callback(old)
        self.assertFalse(self.status[-1]['accepted'])
        self.assertEqual(self.m.logic.state, 'IDLE')


if __name__ == '__main__':
    unittest.main(verbosity=2)
