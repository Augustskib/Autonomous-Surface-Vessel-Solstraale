"""Isolated container-only recording acceptance; no hardware or control publishers."""
import os
import tempfile
import time
from pathlib import Path
import json
import hashlib
import rclpy
from std_msgs.msg import String
from boat_data.data_recorder import DataRecorder
from boat_data.library import name_recording, recording_library

if os.environ.get('ASV_VALIDATION_CONTAINER') != '1':
    raise SystemExit('Run only in the isolated validation container')
with tempfile.TemporaryDirectory() as temporary:
    rclpy.init(args=['--ros-args','-p','data_root:='+temporary,
                    '-p','topics:=[/boat/data_events]'])
    node=DataRecorder()
    try:
        node.start_run({'site':'validation','source':'simulation'})
        run_id=node.run_id
        assert node.proc is not None
        end=time.monotonic()+8
        while time.monotonic()<end:
            node.events_pub.publish(String(data=json.dumps({'event':'recording_smoke','at':time.time()})))
            rclpy.spin_once(node,timeout_sec=.1)
        node.stop_run()
        root=Path(temporary)/'runs'
        folder=root/run_id
        manifest=json.loads((folder/'manifest.json').read_text())
        assert manifest['ended_utc'] and manifest['quality']['bytes']>0
        assert not manifest['quality']['forced_shutdown']
        name_recording(root,run_id,'Lake trial acceptance')
        assert recording_library(root)[0]['name']=='Lake trial acceptance'
        for line in (folder/'checksums.sha256').read_text().splitlines():
            digest,name=line.split('  ',1)
            assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest, name
        print('PASS: real ROS MCAP start, write, stop, persistent name and capture checksums',flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
