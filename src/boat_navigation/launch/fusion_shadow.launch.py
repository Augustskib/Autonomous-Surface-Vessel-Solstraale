"""Opt-in shadow fusion: requires measured calibration; publishes no control TF."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    path = Path(LaunchConfiguration('calibration').perform(context))
    c = json.loads(path.read_text())
    if c.get('verified') is not True or not c.get('calibration_id'):
        raise ValueError('Calibration must be measured, named and verified')
    for key in ('datum',):
        if len(c[key]) != 3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in c[key]):
            raise ValueError('A finite three-element datum is required')
    nodes = []
    for frame in ('gps_frame', 'imu_frame'):
        t = c['transforms'][frame]
        if any(len(t[k]) != 3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in t[k]) for k in ('xyz_m','rpy_rad')):
            raise ValueError(f'Measured transform required: {frame}')
        x,y,z = map(str,t['xyz_m']); roll,pitch,yaw = map(str,t['rpy_rad'])
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='mount_'+frame, arguments=['--x',x,'--y',y,'--z',z,'--roll',roll,'--pitch',pitch,'--yaw',yaw,
                                                        '--frame-id','base_link','--child-frame-id',frame]))
    nodes += [Node(package='boat_navigation', executable='fusion_inputs', parameters=[{
        'calibration_verified': True, 'datum': [float(v) for v in c['datum']],
        'gps_variance_m2': float(c['gps_variance_m2']),
        'gyro_variance_rad2_s2': float(c['gyro_variance_rad2_s2'])}]),
        Node(package='robot_localization', executable='ekf_node', name='asv_ekf',
             parameters=[str(Path(get_package_share_directory('boat_navigation'))/'config'/'ekf_shadow.yaml')],
             remappings=[('odometry/filtered','/boat/localization/odometry')])]
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration'), OpaqueFunction(function=configure)])
