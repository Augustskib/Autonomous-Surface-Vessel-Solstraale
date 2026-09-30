"""Hardware-free tests of production callbacks, with ROS I/O replaced by fakes.

Run: python3 -m unittest discover -s tests -v
The AST loader retains real function/class bodies, avoiding a dependency on
built ROS messages. These tests do not claim to exercise DDS or physical pins.
"""
import ast
import json
import math
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1] / 'src'


def load(relative, names, **env):
    path = ROOT / relative
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names]
    for node in nodes:
        node.decorator_list = []
    tree.body = [ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)] + nodes
    scope = dict(Node=object, math=math, json=json, re=re, time=time, **env)
    exec(compile(ast.fix_missing_locations(tree), str(path), 'exec'), scope)
    return NS(**scope)


def instance(cls):
    obj = object.__new__(cls)
    obj.get_logger = Mock(return_value=Mock())
    obj.get_clock = Mock(return_value=NS(now=lambda: Stamp(100)))
    return obj


class Stamp:
    def __init__(self, seconds): self.nanoseconds = int(seconds * 1e9)
    def __sub__(self, other): return Stamp((self.nanoseconds-other.nanoseconds)/1e9)


class Regressions(unittest.TestCase):
    def motor(self):
        mod = load('boat_hardware_interface/boat_hardware_interface/motor_driver.py', ['MotorDriver'],
                   THRUST_MIN=-100., THRUST_MAX=100., serial=NS(SerialException=OSError))
        obj = instance(mod.MotorDriver)
        obj.max_thrust_pct=100.; obj.estop_active=False; obj.mock=False; obj.ser=object()
        obj._write_serial=Mock()
        return obj

    def test_nonfinite_thrust_never_becomes_full_power(self):
        for value in (math.nan, math.inf, -math.inf):
            for left, right in ((value,40),(40,value)):
                obj=self.motor(); obj.thrust_callback(NS(left=left,right=right))
                obj._write_serial.assert_called_once_with(0.,0.)

    def test_normal_thrust_is_clamped(self):
        obj=self.motor(); obj.thrust_callback(NS(left=120,right=-25))
        obj._write_serial.assert_called_once_with(100.,-25)

    def test_serial_partial_lines_survive_between_ticks(self):
        obj=self.motor(); obj._serial_rx_buf=b''; obj._handle_serial_line=Mock()
        obj.ser=Mock(in_waiting=7); obj.ser.read.return_value=b'IMU AX:'
        obj._read_serial();obj._handle_serial_line.assert_not_called()
        obj.ser.read.return_value=b'1\nSTATUS WDT:0\n'
        obj._read_serial()
        self.assertEqual([x.args[0] for x in obj._handle_serial_line.call_args_list],
                         [b'IMU AX:1',b'STATUS WDT:0'])

    def test_continuous_serial_stream_has_bounded_work(self):
        obj=self.motor();obj._serial_rx_buf=b'';obj._handle_serial_line=Mock()
        obj.ser=Mock(in_waiting=100000);obj.ser.read.return_value=b'X'*4096
        obj._read_serial();obj.ser.read.assert_called_once_with(4096)

    def test_dashboard_lock_supports_nested_power_snapshot(self):
        path=ROOT/'boat_web_ui/boat_web_ui/web_server.py'
        tree=ast.parse(path.read_text())
        assignment=next(n for n in tree.body if isinstance(n,ast.Assign)
                        and any(isinstance(t,ast.Name) and t.id=='telemetry_lock' for t in n.targets))
        scope={'threading':threading}
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),str(path),'exec'),scope)
        lock=scope['telemetry_lock']
        with lock:
            self.assertTrue(lock.acquire(blocking=False),'telemetry() deadlocks in _power_snapshot()')
            lock.release()

    def test_safety_discards_stale_gps_before_rtl(self):
        mod=load('boat_control/boat_control/safety_monitor.py',['SafetyMonitor'],
                 GPS_STALE_SEC=2., LEVEL_NOMINAL=0, LEVEL_RETURN_TO_LAUNCH=2)
        obj=instance(mod.SafetyMonitor); obj._now=lambda:100
        obj._feed_heading=Mock();obj._gcs_link_lost=lambda:False
        obj._last_fix_time=90;obj.current_fix=(28.5,-81.4)
        obj.logic=Mock();obj.logic.evaluate.return_value=NS(level=0,commanding_thrust=False)
        obj._last_level=0;obj._abort_sent_for_level=0
        obj.get_parameter=lambda _:NS(value=False);obj._publish_failsafe_state=Mock()
        obj.check()
        obj.logic.on_position_lost.assert_called_once()
        self.assertIsNone(obj.current_fix)

    def follower(self):
        mod=load('boat_navigation/boat_navigation/waypoint_follower.py',
                 ['WaypointFollower','haversine_distance_m'],EARTH_RADIUS_M=6371000.,
                 HEADING_STALE_SEC=1.,HeadingEstimate=NS(DEAD_RECKONING=1,FUSED=2),
                 MAX_PLAUSIBLE_SPEED_MPS=15.,MAX_TRAIL_POINTS=2000,SPEED_EMA_ALPHA=.25)
        obj=instance(mod.WaypointFollower);obj._publish_thrust=Mock()
        return obj

    def test_heading_loss_does_not_reuse_cached_raw_yaw(self):
        obj=self.follower();obj._heading_msg=None;obj._heading_msg_time=None
        obj._imu_fallback_heading_deg=270
        self.assertEqual(obj.current_heading_deg(),(None,True))

    def test_nonfinite_heading_is_unusable(self):
        obj=self.follower();obj._heading_msg=NS(heading_deg=math.nan,quality=2,detail='bad')
        obj._heading_msg_time=100
        self.assertEqual(obj.current_heading_deg(),(None,True))

    def test_gps_jump_does_not_replace_navigation_position(self):
        obj=self.follower();obj._prev_fix=obj.current_fix=(28.5,-81.4)
        obj._prev_fix_time=obj.last_fix_time=Stamp(99);obj.trail_coords=[]
        obj.gps_callback(NS(status=NS(status=0),latitude=40.,longitude=-70.))
        self.assertEqual(obj.current_fix,(28.5,-81.4))
        self.assertEqual(obj.last_fix_time.nanoseconds,99_000_000_000)

    def test_gps_no_fix_immediately_stops(self):
        obj=self.follower();obj.current_fix=(28.5,-81.4)
        obj.gps_callback(NS(status=NS(status=-1),latitude=0.,longitude=0.))
        obj._publish_thrust.assert_called_once_with(0.,0.)
        self.assertIsNone(obj.current_fix)

    def test_camera_pipe_can_pause_mid_frame_without_blocking(self):
        import os
        mod=load('boat_perception/boat_perception/camera_publisher.py',
                 ['CameraPublisher'], os=os)
        obj=instance(mod.CameraPublisher);obj.frame_size=6;obj._frame_buffer=bytearray()
        rd,wr=os.pipe()
        try:
            os.set_blocking(rd,False)
            obj.proc=NS(stdout=NS(fileno=lambda:rd))
            self.assertIsNone(obj._read_frame())
            os.write(wr,b'abc');self.assertIsNone(obj._read_frame())
            os.write(wr,b'def');self.assertEqual(obj._read_frame(),b'abcdef')
        finally:
            os.close(rd);os.close(wr)

    def test_bad_mission_does_not_enter_execute(self):
        mod=load('boat_control/boat_control/mission_manager.py',['MissionManager'])
        obj=instance(mod.MissionManager);obj.logic=Mock()
        obj.mission_callback(NS(latitudes=[math.nan],longitudes=[-81.]))
        obj.logic.request_mission.assert_not_called()

    def test_json_arrays_do_not_crash_ros_command_handlers(self):
        for file,cls,method in [
            ('boat_control/boat_control/safety_monitor.py','SafetyMonitor','geofence_command_callback'),
            ('boat_control/boat_control/safety_monitor.py','SafetyMonitor','failsafe_clear_callback'),
            ('boat_data/boat_data/data_recorder.py','DataRecorder','command_callback')]:
            mod=load(file,[cls]);obj=instance(getattr(mod,cls))
            getattr(obj,method)(NS(data='[1,2,3]'))
            obj.get_logger().warn.assert_called()

if __name__=='__main__': unittest.main()
