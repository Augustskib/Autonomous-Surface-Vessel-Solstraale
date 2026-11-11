"""Further whole-project regression cases, using real methods with fake I/O."""
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from test_regressions import load, instance


@pytest.mark.parametrize('maximum',[float('nan'),float('inf'),-30.])
def test_sim_invalid_force_scale_stops(maximum):
    obj=instance(load('boat_hardware_interface/boat_hardware_interface/thrust_splitter.py',['ThrustSplitter']).ThrustSplitter)
    obj.estop_active=False;obj._publish_forces=Mock()
    obj.get_parameter=lambda _:NS(value=maximum)
    obj.thrust_callback(NS(left=25.,right=25.))
    obj._publish_forces.assert_called_once_with(0.,0.)
    assert obj.target_left==obj.current_left==0


@pytest.mark.parametrize('name,value',[('steer_gain',float('nan')),('waypoint_radius_m',float('inf')),('base_speed_pct',-1)])
def test_navigation_invalid_settings_cannot_advance_route_or_drive(name,value):
    cls=load('boat_navigation/boat_navigation/waypoint_follower.py',['WaypointFollower'],STATE_EXECUTE='EXECUTE',STATE_ABORTED='ABORTED').WaypointFollower
    obj=instance(cls);obj.mission_active=True;obj.status_at=time.monotonic()
    obj.status_execution_id=obj.execution_id='execution'
    obj.status_manager_id=obj.mission_manager_id='manager'
    obj.status_generation=obj.mission_generation=1
    obj.boat_state='EXECUTE';obj.control_mode='WAYPOINT';obj.last_mode_at=time.monotonic()
    obj.current_fix=(28.,-81.);obj.last_fix_time=None
    obj.current_waypoint_idx=0;obj.waypoints=[(28.,-81.)]
    values={'steer_gain':1.,'base_speed_pct':30.,'waypoint_radius_m':3.};values[name]=value
    obj.get_parameter=lambda key:NS(value=values[key]);obj._publish_thrust=Mock()
    obj.control_loop();obj._publish_thrust.assert_called_once_with(0.,0.)
    assert obj.current_waypoint_idx==0 and obj.mission_active


def test_detector_runtime_exception_does_not_escape_callback():
    cls=load('boat_perception/boat_perception/detector_node.py',['DetectorNode']).DetectorNode
    obj=instance(cls);obj.detect_enabled=True;obj._last_infer_time=0;obj.min_interval_s=0
    obj._ensure_model=lambda:True;obj.bridge=Mock();obj.model=Mock();obj.confidence_threshold=.35
    obj.model.predict.side_effect=RuntimeError('inference failed')
    obj.image_callback(NS(encoding='bgr8'))
    obj.get_logger().error.assert_called_once()


def test_duplicate_segment_does_not_orphan_original():
    cls=load('boat_data/boat_data/data_recorder.py',['DataRecorder']).DataRecorder
    obj=instance(cls);obj._recording=lambda _:True
    original={'step_id':'step1','t_end_utc':None};obj.manifest={'segments':[original]}
    obj.open_segments={'step1':original};obj._write_manifest=Mock()
    obj.segment_begin({'step_id':'step1'})
    assert obj.manifest['segments']==[original] and obj.open_segments['step1'] is original
    obj._write_manifest.assert_not_called()

def test_non_string_record_action_rejected_before_dispatch():
    cls=load('boat_data/boat_data/data_recorder.py',['DataRecorder']).DataRecorder
    obj=instance(cls)
    obj.command_callback(NS(data='{"action": []}'))
    obj.get_logger().warn.assert_called_once()


def test_system_disk_does_not_invent_recording_duration():
    from pathlib import Path
    cls=load('boat_hardware_interface/boat_hardware_interface/system_monitor.py',['SystemMonitor'],
             Path=Path,shutil=NS(disk_usage=lambda p:NS(free=24000000000,total=32000000000,used=8000000000))).SystemMonitor
    obj=instance(cls);obj.get_parameter=lambda _:NS(value='/recordings')
    disk=obj._read_disk()
    assert disk['free_gb']==24 and disk['est_imagery_minutes'] is None
    assert disk['fallback'] is False

def test_recorder_event_after_ros_shutdown_does_not_block_finalization():
    cls=load('boat_data/boat_data/data_recorder.py',['DataRecorder'],
             rclpy=NS(ok=lambda:False),utc_now_iso=lambda:'shutdown-time').DataRecorder
    obj=instance(cls);obj.events_pub=Mock()
    obj._emit_event({'event':'run_stop'})
    obj.events_pub.publish.assert_not_called()

# These tests exercise the production callbacks without requiring ROS hardware.
def safety_monitor():
    cls=load('boat_control/boat_control/safety_monitor.py',['SafetyMonitor'],
             GPS_STALE_SEC=2., STATE_EXECUTE='EXECUTE').SafetyMonitor
    obj=instance(cls);obj._now=lambda:100.
    obj.current_fix=(28.,-81.);obj._last_fix_time=99.;obj.logic=Mock()
    obj.get_parameter=lambda _:NS(value=10.)
    return obj

@pytest.mark.parametrize('lat,lon,status',[(float('nan'),0,0),(91,10,0),(20,181,0),(0,0,0),(28,-81,-1)])
def test_invalid_fix_immediately_removes_old_location(lat,lon,status):
    obj=safety_monitor()
    obj.gps_callback(NS(latitude=lat,longitude=lon,status=NS(status=status)))
    assert obj.current_fix is None and obj._last_fix_time is None
    obj.logic.on_position_lost.assert_called_once()
    obj.logic.set_launch_point.assert_not_called()

@pytest.mark.parametrize('action',['set_here','set_launch_here'])
def test_here_command_cannot_capture_stale_position(action):
    obj=safety_monitor();obj._last_fix_time=90.;obj.set_parameters=Mock()
    assert obj._apply_geofence_command(action,{}) is True
    obj.set_parameters.assert_not_called()
    obj.logic.set_launch_point.assert_not_called()

@pytest.mark.parametrize('cmd',[
    {'action':'set','center_lat':91,'center_lon':20,'radius_m':100,'warn_fraction':.9},
    {'action':'set','center_lat':20,'center_lon':20,'radius_m':100,'warn_fraction':float('nan')},
    {'action':'set_launch','lat':20,'lon':181},
])
def test_invalid_runtime_site_configuration_is_rejected(cmd):
    obj=safety_monitor();obj.set_parameters=Mock()
    obj._apply_geofence_command(cmd['action'],cmd)
    obj.set_parameters.assert_not_called()
    obj.logic.set_launch_point.assert_not_called()

def test_gcs_timeout_uses_monotonic_clock_even_if_ros_clock_is_frozen():
    obj=safety_monitor();obj.boat_state='EXECUTE';obj._gcs_armed=False
    obj.heartbeat_callback(None)
    assert obj.last_heartbeat_time==100.
    obj._now=lambda:111.
    assert obj._gcs_link_lost()
    obj.get_clock.assert_not_called()
