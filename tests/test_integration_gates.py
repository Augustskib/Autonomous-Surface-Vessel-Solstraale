"""Input rejection and command acknowledgements, without claiming ROS integration."""
import copy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from test_regressions import load, instance


def fusion():
    cls=load('boat_navigation/boat_navigation/fusion_inputs.py',['FusionInputs'],copy=copy).FusionInputs
    obj=instance(cls); obj.last={}; obj.last_stamp={}; obj.imu_pub=Mock(); obj.gyro_var=.01
    return obj


def sample(sec=100, x=0., frame='imu_frame'):
    return NS(header=NS(stamp=NS(sec=sec,nanosec=0),frame_id=frame),
              angular_velocity=NS(x=x,y=0.,z=.1),angular_velocity_covariance=[0.]*9,
              orientation_covariance=[0.]*9,linear_acceleration_covariance=[0.]*9)


@pytest.mark.parametrize('msg',[sample(98),sample(101),sample(x=float('nan')),sample(frame='wrong')])
def test_fusion_rejects_invalid_or_stale_input(msg):
    obj=fusion();obj.imu(msg);obj.imu_pub.publish.assert_not_called();assert not obj.last


def test_fusion_duplicate_and_unavailable_rejected():
    obj=fusion();msg=sample();msg.angular_velocity_covariance[0]=-1
    obj.imu(msg);obj.imu_pub.publish.assert_not_called()
    msg.angular_velocity_covariance[0]=0
    obj.imu(msg);obj.imu(msg);obj.imu_pub.publish.assert_called_once()
    out=obj.imu_pub.publish.call_args.args[0]
    assert out.orientation_covariance[0]==-1 and out.linear_acceleration_covariance[0]==-1
    assert msg.orientation_covariance[0]==0


def test_record_command_is_not_confirmation():
    node=NS(publish_data_command=Mock())
    mod=load('boat_web_ui/boat_web_ui/web_server.py',['_queue_data_command'],ros_node=node,jsonify=lambda x:x)
    payload,status=mod._queue_data_command({'action':'start'})
    assert status==202 and payload['confirmed'] is False
    node.publish_data_command.assert_called_once_with({'action':'start'})


def test_record_command_without_ros_fails():
    mod=load('boat_web_ui/boat_web_ui/web_server.py',['_queue_data_command'],ros_node=None,jsonify=lambda x:x)
    payload,status=mod._queue_data_command({'action':'start'})
    assert status==503 and payload['status']=='error'
