"""Exercise real endpoint and adapter methods with in-memory ROS transport.

Run with Python alone. Flask's HTTP stack and DDS transport are not exercised.
"""
import ast
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid
from types import SimpleNamespace as Obj
import unittest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'boat_control'))
from boat_control.arbitration import ControlArbitration


def methods(path, names, ns, class_name=None):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    nodes = tree.body
    if class_name:
        nodes = next(n.body for n in nodes if isinstance(n, ast.ClassDef) and n.name == class_name)
    selected = [n for n in nodes if isinstance(n, ast.FunctionDef) and n.name in names]
    for n in selected:
        n.decorator_list = []
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), ns)


class EndpointTests(unittest.TestCase):
    def setUp(self):
        self.body = dict(session='pilot', seq=1, intent='MANUAL', transition=True, left=90, right=-90)
        self.ns = dict(json=json, math=math, time=time, uuid=uuid, String=Obj,
            request=Obj(path='/operator', get_json=lambda **kw: self.body), jsonify=lambda **kw: kw,
            telemetry_lock=threading.Lock(), operator_receipts={},
            latest_operator={}, latest_operator_time=None, _await=lambda p: p())
        web = SRC / 'boat_web_ui/boat_web_ui/web_server.py'
        methods(web, ['operator_command'], self.ns)
        methods(web, ['operator_callback'], self.ns, 'WebServerNode')
        arb_ns = dict(json=json, String=Obj, Bool=Obj)
        methods(SRC / 'boat_control/boat_control/control_arbiter.py',
                ['_operator_callback', '_publish_operator_status'], arb_ns, 'ControlArbiter')
        self.arb = Obj(logic=ControlArbitration(), _now=lambda: time.monotonic())
        self.arb.logic.on_failsafe_level(0, time.monotonic())
        self.arb.operator_pub = Obj(publish=lambda msg: self.ns['operator_callback'](None, msg))
        self.estops = []
        self.arb.estop_pub = Obj(publish=lambda msg: self.estops.append(msg.data))
        self.arb._publish_operator_status = lambda *args: arb_ns['_publish_operator_status'](self.arb, *args)
        self.ns['ros_node'] = Obj(manual_thrust_limit=35,
            operator_pub=Obj(publish=lambda msg: arb_ns['_operator_callback'](self.arb, msg)))

    def call(self):
        if isinstance(self.body, dict) and self.body.get('intent') in ('MANUAL','AUTO','RESET'):
            prepared = dict(self.body)
            for k in ('left','right'):
                if isinstance(prepared.get(k), (int,float)) and math.isfinite(prepared[k]):
                    prepared[k] = max(-35, min(35, float(prepared[k])))
            self.body['ticket'] = self.arb.logic.prepare_transition(prepared, time.monotonic()) or ''
        return self.ns['operator_command']()

    def test_legacy_endpoint_cannot_report_false_success(self):
        methods(SRC / 'boat_web_ui/boat_web_ui/web_server.py', ['legacy_control'], self.ns)
        reply, code = self.ns['legacy_control']()
        self.assertEqual(code, 410)
        self.assertEqual(reply['status'], 'error')

    def test_acknowledged_command_is_clamped_and_reaches_arbiter(self):
        reply, code = self.call()
        self.assertEqual(code, 200)
        self.assertEqual(reply['operator']['request_seq'], 1)
        self.assertEqual(reply['operator']['mode'], 'MANUAL')
        self.assertEqual(self.arb.logic.evaluate(time.monotonic())[:2], (35, -35))

    def test_reordered_old_command_gets_rejection_not_success(self):
        self.call()
        self.body.update(seq=3, intent='HOLD')
        self.call()
        self.body.update(seq=2, intent='MANUAL')
        reply, code = self.call()
        self.assertEqual(code, 409)
        self.assertFalse(reply['operator']['accepted'])
        self.assertEqual(self.arb.logic.evaluate(time.monotonic()), (0, 0, 'HOLD'))

    def test_no_arbiter_ack_returns_error(self):
        self.ns['ros_node'].operator_pub.publish = lambda msg: None
        reply, code = self.call()
        self.assertEqual(code, 504)
        self.assertEqual(reply['status'], 'error')

    def test_invalid_thrust_and_no_ros_are_errors(self):
        self.body['left'] = float('inf')
        self.assertEqual(self.call()[1], 400)
        self.ns['ros_node'] = None
        self.assertEqual(self.call()[1], 503)

    def test_stop_reset_reaches_estop_topic_and_reset_stays_held(self):
        self.body['intent'] = 'ESTOP'
        self.assertEqual(self.call()[1], 200)
        self.body.update(seq=2, intent='RESET')
        self.assertEqual(self.call()[1], 200)
        self.assertEqual(self.estops, [True, False])
        self.assertEqual(self.arb.logic.evaluate(time.monotonic()), (0, 0, 'HOLD'))

    def test_cached_receipt_cannot_confirm_a_new_transport_attempt(self):
        self.assertEqual(self.call()[1], 200)
        self.ns['ros_node'].operator_pub.publish = lambda msg: None
        self.assertEqual(self.call()[1], 504)

    def test_reusing_sequence_with_changed_payload_is_rejected(self):
        self.assertEqual(self.call()[1], 200)
        self.body['left'] = -10
        self.assertEqual(self.call()[1], 409)

    def test_non_object_json_is_a_client_error(self):
        for value in (None, [], [1], 'command', 123, True):
            self.body = value
            self.assertEqual(self.call()[1], 400)

    def test_prepare_endpoint_returns_arbiter_ticket_without_motion(self):
        self.ns['request'].path = '/operator/prepare'
        reply, code = self.call()
        self.assertEqual(code, 200)
        self.assertTrue(reply['operator']['ticket'])
        self.assertEqual(self.arb.logic.evaluate(time.monotonic()), (0, 0, 'HOLD'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
