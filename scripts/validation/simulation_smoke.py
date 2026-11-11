#!/usr/bin/env python3
"""Bounded, isolated Gazebo startup and sensor/heading smoke test."""
import json
import os
import signal
import subprocess
import time


def main():
    if os.environ.get('ASV_VALIDATION_CONTAINER') != '1':
        raise SystemExit('Container-only simulation check')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, Imu, Image
    from rosgraph_msgs.msg import Clock
    from boat_msgs.msg import HeadingEstimate, ThrustCommand
    rclpy.init();node=rclpy.create_node('simulation_probe')
    samples={k:0 for k in ('gps','imu','camera','clock','heading','thrust')}
    qualities=[];nonzero=[]
    def callback(kind):
        def receive(msg):
            samples[kind]+=1
            if kind=='heading':qualities.append(msg.quality)
            if kind=='thrust' and (msg.left != 0 or msg.right != 0):nonzero.append((msg.left,msg.right))
        return receive
    for kind,typ,topic in [('gps',NavSatFix,'/boat/gps/fix'),('imu',Imu,'/boat/imu'),
        ('camera',Image,'/boat/camera/image_raw'),('clock',Clock,'/clock'),
        ('heading',HeadingEstimate,'/boat/heading'),('thrust',ThrustCommand,'/boat/thrust_command')]:
        node.create_subscription(typ,topic,callback(kind),qos_profile_sensor_data)
    with open('/tmp/asv-sim-nodes.log','w') as log:
        proc=subprocess.Popen(['ros2','launch','boat_simulation','boat_simulation.launch.py'],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+35
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if proc.poll() is not None:raise RuntimeError('Launch exited early')
            names=set(node.get_node_names())
            expected={'heading_estimator','waypoint_follower','control_arbiter','safety_monitor','thrust_splitter'}
            assert expected <= names, f'Nodes missing: {expected-names}'
            assert all(samples.values()), f'Missing stream: {samples}'
            assert any(q>=2 for q in qualities), 'No usable simulation heading'
            assert not nonzero, f'Unexpected idle thrust: {nonzero[:3]}'
            print(json.dumps({'samples':samples,'heading_qualities':sorted(set(qualities)),'idle_thrust_zero':True}))
        finally:
            proc.send_signal(signal.SIGINT) # Launch forwards once; avoid group-wide duplicate SIGINT.
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            node.destroy_node();rclpy.try_shutdown()
            print(open('/tmp/asv-sim-nodes.log').read()[-5000:])
            assert proc.returncode==0, f'Launch shutdown code {proc.returncode}'


if __name__=='__main__':main()
