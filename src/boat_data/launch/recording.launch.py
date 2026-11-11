"""Start recorder ready/idle; recording begins only on an explicit data command."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    qos = str(Path(get_package_share_directory('boat_data')) / 'config' / 'recorder_qos.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('data_root'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        Node(package='boat_data', executable='data_recorder', output='screen', parameters=[{
            'data_root': LaunchConfiguration('data_root'), 'qos_overrides_path': qos,
            'calibration_file': LaunchConfiguration('calibration_file'),
            'max_bag_size_bytes': 1000000000, 'min_free_gb': 5.0, 'stop_free_gb': 2.0}])])
