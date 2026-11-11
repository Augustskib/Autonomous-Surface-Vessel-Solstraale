"""Explicit hardware-only launch; no default host or invented mounting transform."""
import json
import math
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def measured_mount(context):
    path = LaunchConfiguration('calibration').perform(context)
    if not path:
        return []  # Standalone lidar_frame view only until mount is measured.
    c = json.loads(Path(path).read_text())
    if c.get('verified') is not True:
        raise ValueError('Measured LiDAR mount must be verified')
    mount = c['transforms']['lidar_frame']
    for key in ('xyz_m','rpy_rad'):
        if len(mount[key]) != 3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in mount[key]):
            raise ValueError('LiDAR mount contains missing/nonfinite values')
    x,y,z=map(str,mount['xyz_m']); r,p,yaw=map(str,mount['rpy_rad'])
    return [Node(package='tf2_ros', executable='static_transform_publisher', name='mount_lidar',
                 arguments=['--x',x,'--y',y,'--z',z,'--roll',r,'--pitch',p,'--yaw',yaw,
                            '--frame-id','base_link','--child-frame-id','lidar_frame'])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host'),
        DeclareLaunchArgument('calibration', default_value=''),
        OpaqueFunction(function=measured_mount),
        DeclareLaunchArgument('settings_file', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value='receive'),
        DeclareLaunchArgument('sensor_clock_verified', default_value='false'),
        Node(package='boat_lidar', executable='quanergy_m8_node', output='screen',
             parameters=[{'host': LaunchConfiguration('host'),
                          'settings_file': LaunchConfiguration('settings_file'),
                          'timestamp_mode': LaunchConfiguration('timestamp_mode'),
                          'sensor_clock_verified': ParameterValue(LaunchConfiguration('sensor_clock_verified'), value_type=bool)}]),
        Node(package='boat_perception', executable='lidar_preview', output='screen'),
    ])
