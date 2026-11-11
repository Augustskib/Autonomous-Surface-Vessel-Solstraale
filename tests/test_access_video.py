import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'/'boat_web_ui'))
from boat_web_ui.video_stream import SharedVideo
from boat_web_ui.access import authorized, same_origin


def test_shared_video_encodes_once_across_viewers_and_expires():
    now=[0.]; calls=[]
    def encode(frame): calls.append(frame);return bytes(frame)
    stream=SharedVideo(encode,b'stale',clock=lambda:now[0])
    assert stream.get()==b'stale'
    stream.put(b'one')
    for _ in range(20): assert stream.get()==b'one'
    assert calls==[b'one']
    now[0]=3
    assert stream.get()==b'stale'
    stream.put(b'two');assert stream.get()==b'two'


def test_video_rate_cap_and_encoder_failure():
    now=[0.];calls=[]
    def encode(frame):
        calls.append(frame)
        if frame==b'bad':raise ValueError('bad')
        return frame
    stream=SharedVideo(encode,b'stale',clock=lambda:now[0])
    stream.put(b'one');assert stream.get()==b'one'
    now[0]=.01;stream.put(b'two');assert stream.get()==b'one'
    now[0]=.11;assert stream.get()==b'two'
    now[0]=.22;stream.put(b'bad');assert stream.get()==b'stale'
    assert len(calls)==3


def test_auth_and_origin_checks():
    assert not authorized(None,'secret')
    assert not authorized(NS(type='basic',username='operator',password='wrong'),'secret')
    assert authorized(NS(type='basic',username='operator',password='secret'),'secret')
    assert same_origin('http://localhost:5000','http://localhost:5000/')
    assert not same_origin('https://evil.example','http://localhost:5000/')
    assert not same_origin('null','http://localhost:5000/')

def test_splitter_watchdog_uses_steady_elapsed_time():
    from test_regressions import load,instance
    from unittest.mock import Mock
    obj=instance(load('boat_hardware_interface/boat_hardware_interface/thrust_splitter.py',
                      ['ThrustSplitter'],COMMAND_TIMEOUT_SEC=.5).ThrustSplitter)
    import time
    obj.last_command_time=time.monotonic()-1
    obj._stopped_logged=False;obj._publish_forces=Mock()
    obj.get_clock.side_effect=AssertionError('ROS time must not control watchdog')
    obj.check_watchdog();obj._publish_forces.assert_called_once_with(0.,0.)


def test_supervisor_now_independent_of_ros_time():
    from test_regressions import load,instance
    obj=instance(load('boat_control/boat_control/safety_monitor.py',['SafetyMonitor']).SafetyMonitor)
    obj.get_clock.side_effect=AssertionError('ROS time must not control safety')
    assert obj._now()>0


def test_cleanup_signals_only_created_group(monkeypatch):
    import importlib.util
    import signal
    script=Path(__file__).resolve().parents[1]/'scripts/sim/run_sim.py'
    spec=importlib.util.spec_from_file_location('asv_run_sim',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    calls=[]
    def killpg(pid,sig):
        calls.append((pid,sig))
        if sig==0:raise ProcessLookupError
    monkeypatch.setattr(module.os,'killpg',killpg,raising=False)
    monkeypatch.setattr(module, 'signal', NS(SIGINT=2,SIGTERM=15,SIGKILL=9))
    module.stop_group(12345)
    assert calls==[(12345,signal.SIGINT),(12345,0)]
