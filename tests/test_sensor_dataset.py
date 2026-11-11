"""Real MCAP round trips through the offline pipeline; synthetic measurements."""
import json
from pathlib import Path
import sqlite3
import struct
import sys
from types import SimpleNamespace as NS

import pytest
from mcap_ros2.writer import Writer

ROOT = Path(__file__).resolve().parents[1]
for package in ('boat_data','boat_navigation','boat_perception'):
    sys.path.insert(0, str(ROOT/'src'/package))
from boat_data.dataset import index_run, export_images, export_3d, archive_run, sha256, TOPICS
from boat_navigation.geodesy import enu
from boat_perception.lidar import preview_cloud

HEADER = '''
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
DEFS = {
 'camera': 'std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data'+HEADER,
 'gps': 'std_msgs/Header header\ntest_msgs/Status status\nfloat64 latitude\nfloat64 longitude'+HEADER+'\n================================================================================\nMSG: test_msgs/Status\nint8 status\n',
 'imu': 'std_msgs/Header header\ntest_msgs/Vector angular_velocity\nfloat64[9] angular_velocity_covariance'+HEADER+'\n================================================================================\nMSG: test_msgs/Vector\nfloat64 x\nfloat64 y\nfloat64 z\n',
 'thrust': 'float32 left\nfloat32 right',
 'lidar': 'std_msgs/Header header\nuint32 width\nuint32 height\nuint32 row_step\nuint32 point_step\nbool is_bigendian\ntest_msgs/Field[] fields\nuint8[] data'+HEADER+'\n================================================================================\nMSG: test_msgs/Field\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n',
}


def write_bag(path, times, missing=(), nofix=False, verified=True, future_thrust=False):
    with path.open('wb') as stream:
        writer = Writer(stream)
        schemas = {k: writer.register_msgdef('test_msgs/msg/'+k.title(), v) for k,v in DEFS.items()}
        status_schema = writer.register_msgdef('std_msgs/msg/String', 'string data')
        for ns in times:
            writer.write_message('/boat/lidar/driver_status', status_schema,
                                 {'data':json.dumps({'timestamp_mode':'sensor' if verified else 'receive','clock_verified':verified})}, log_time=ns-1)
            header = {'stamp': {'sec': ns//1000000000,'nanosec':ns%1000000000},'frame_id':'test_frame'}
            values = {
                'camera': {'header':header,'height':1,'width':2,'encoding':'bgr8','is_bigendian':0,'step':8,'data':bytes([0,0,255,0,255,0,99,99])},
                'gps': {'header':header,'status':{'status':-1 if nofix else 0},'latitude':28.0,'longitude':-81.0},
                'imu': {'header':header,'angular_velocity':{'x':0.,'y':0.,'z':0.1},'angular_velocity_covariance':[0.]*9},
                'thrust': {'left':10.,'right':20.},
                'lidar': {'header':header,'width':1,'height':1,'row_step':12,'point_step':12,'is_bigendian':False,'fields':[{'name':k,'offset':i*4,'datatype':7,'count':1} for i,k in enumerate(('x','y','z'))],'data':struct.pack('<fff',1.,2.,3.)}}
            for k, msg in values.items():
                if k not in missing:
                    timestamp = ns+100000000 if k == 'thrust' and future_thrust else ns
                    writer.write_message(TOPICS[k],schemas[k],msg,log_time=timestamp)
        writer.finish()


def run(tmp_path):
    (tmp_path/'bag'/'rosbag2').mkdir(parents=True)
    (tmp_path/'manifest.json').write_text(json.dumps({'ended_utc':'2026-11-08T00:00:00Z'}))
    return tmp_path/'bag'/'rosbag2'


def test_all_segments_real_decode_export_and_archive(tmp_path):
    bags = run(tmp_path)
    write_bag(bags/'run_0.mcap',[1000000000])
    write_bag(bags/'run_1.mcap',[2000000000])
    original = sha256(tmp_path/'manifest.json')
    out, report = index_run(tmp_path)
    assert report['pass'] and report['camera_frames'] == 2
    assert sha256(tmp_path/'manifest.json') == original
    images = export_images(tmp_path,out,every=1)
    assert len(list((images/'images').glob('*.png'))) == 2
    from PIL import Image
    assert Image.open(next((images/'images').glob('*.png'))).getpixel((0,0)) == (255,0,0)
    dest = tmp_path.parent/(tmp_path.name+'-archive')
    archive_run(tmp_path,dest)
    assert sha256(dest/'bag/rosbag2/run_1.mcap') == sha256(bags/'run_1.mcap')
    assert (dest/'transfer_receipt.json').is_file()


@pytest.mark.parametrize('missing',[('gps',),('imu',),('thrust',),('lidar',),('camera',)])
def test_required_stream_missing_cannot_pass(tmp_path, missing):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000],missing=missing)
    _, report=index_run(tmp_path)
    assert not report['pass'] and report['missing']


def test_nested_no_fix_rejected(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000],nofix=True)
    _, report=index_run(tmp_path)
    assert report['accepted_frames']==0


def test_sensor_clock_not_verified_is_not_synced(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000],verified=False)
    _, report=index_run(tmp_path)
    assert not report['pass'] and report['accepted_frames']==0


def test_future_thrust_is_not_associated(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000],future_thrust=True)
    out, report=index_run(tmp_path)
    assert report['accepted_frames']==0
    with sqlite3.connect(out/'index.sqlite') as db:
        associations=json.loads(db.execute('SELECT matches FROM sync').fetchone()[0])
        assert associations['thrust'] is None


def test_reversal_survives_index_sorting(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[2000000000,1000000000])
    out, report=index_run(tmp_path)
    assert not report['pass'] and report['timestamp_reversals']['camera']==1
    with pytest.raises(ValueError): export_images(tmp_path,out)


def test_tampered_capture_blocks_export(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000])
    out,_=index_run(tmp_path)
    with (bags/'run_0.mcap').open('ab') as stream: stream.write(b'changed')
    with pytest.raises(ValueError): export_images(tmp_path,out)


def test_preview_keeps_fields_and_skips_row_padding():
    msg=NS(width=2,height=2,point_step=2,row_step=6,data=bytes(range(12)),
           fields=[NS(name=k) for k in ('x','y','z')],header=NS(frame_id='lidar_frame'))
    out=preview_cloud(msg,budget=2)
    assert out.data==bytes([0,1,6,7]) and out.width==2 and out.height==1
    assert out.header is msg.header and out.fields is msg.fields
    assert msg.width==2 and msg.height==2


def test_enu_origin_and_cardinal_directions():
    assert enu(0,0,0,(0,0,0))==pytest.approx((0,0,0))
    east=enu(0,0.001,0,(0,0,0)); north=enu(0.001,0,0,(0,0,0))
    assert east[0]==pytest.approx(111.319,abs=0.01) and abs(east[1])<1e-5
    assert north[1]==pytest.approx(110.574,abs=0.01) and abs(north[0])<1e-5


def test_3d_export_pairs_actual_mcap_cloud_and_camera(tmp_path):
    bags=run(tmp_path); write_bag(bags/'run_0.mcap',[1000000000])
    out,_=index_run(tmp_path)
    exported=export_3d(tmp_path,out,every=1)
    cloud=next(exported.glob('*.pcd'))
    assert 'FIELDS x y z' in cloud.read_text()
    assert '1.0 2.0 3.0' in cloud.read_text()
    assert cloud.with_suffix('.png').exists()


def test_pcd_preserves_big_endian_fields_and_row_padding(tmp_path):
    from boat_data.pcd import write_pcd
    fields=[NS(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
    cloud=NS(width=1,height=2,point_step=12,row_step=16,fields=fields,is_bigendian=True,
             data=struct.pack('>fff',1,2,3)+b'PAD!'+struct.pack('>fff',4,5,6)+b'PAD!')
    write_pcd(cloud,tmp_path/'cloud.pcd')
    assert (tmp_path/'cloud.pcd').read_text().endswith('1.0 2.0 3.0\n4.0 5.0 6.0\n')

def test_export_accepts_documented_sqlite_path(tmp_path):
    bags=run(tmp_path);write_bag(bags/'run_0.mcap',[1000000000])
    out,_=index_run(tmp_path)
    assert list((export_images(tmp_path,out/'index.sqlite',every=1)/'images').glob('*.png'))


def test_optional_unverified_lidar_does_not_fail_camera_profile_but_blocks_3d(tmp_path):
    bags=run(tmp_path);write_bag(bags/'run_0.mcap',[1000000000],verified=False)
    out,report=index_run(tmp_path,required=('camera','gps','imu','thrust'))
    assert report['pass'] and report['accepted_frames']==1
    assert list((export_images(tmp_path,out)/'images').glob('*.png'))
    with pytest.raises(ValueError,match='LiDAR required'):export_3d(tmp_path,out)
