#!/usr/bin/env python3
"""Real DDS smoke test in validation container only. Never starts hardware drivers."""
import os
import signal
import subprocess
import time


def main():
    if os.environ.get('ASV_VALIDATION_CONTAINER') != '1':
        raise SystemExit('Refusing outside the isolated validation container')
    import rclpy
    from std_msgs.msg import Bool, String
    from boat_msgs.msg import ThrustCommand, FailsafeState
    children=[]
    rclpy.init()
    node=rclpy.create_node('asv_validation_probe')
    seen={'thrust':[], 'safety':[], 'mode':[]}
    node.create_subscription(ThrustCommand,'/boat/thrust_command',lambda m:seen['thrust'].append((m.left,m.right)),10)
    node.create_subscription(FailsafeState,'/boat/failsafe_state',lambda m:seen['safety'].append(m.level),10)
    node.create_subscription(String,'/boat/mode',lambda m:seen['mode'].append(m.data),10)
    stop=node.create_publisher(Bool,'/boat/estop',10)
    try:
        for args in (
            ['boat_control','safety_monitor','--ros-args','-p','geofence_enabled:=false','-p','gcs_failsafe_enabled:=false'],
            ['boat_control','control_arbiter']):
            children.append(subprocess.Popen(['ros2','run',*args],start_new_session=True))
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if any(p.poll() is not None for p in children):raise RuntimeError('Core node exited')
            if seen['thrust'] and seen['safety']:break
        assert seen['thrust'] and seen['safety'], 'DDS outputs did not arrive'
        assert all(pair==(0.,0.) for pair in seen['thrust']), 'Unexpected idle thrust'
        seen['mode'].clear()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            stop.publish(Bool(data=True));rclpy.spin_once(node,timeout_sec=.1)
            if 'ESTOP' in seen['mode']:break
        assert 'ESTOP' in seen['mode'], f'ESTOP not observed: {seen["mode"]}'
        assert all(pair==(0.,0.) for pair in seen['thrust'])
        print('DDS smoke passed: fresh safety, zero idle thrust, ESTOP mode')
    finally:
        for proc in children:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in children:
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        node.destroy_node();rclpy.try_shutdown()
        assert all(p.returncode == 0 for p in children), 'Core nodes failed during shutdown'


if __name__=='__main__':main()
