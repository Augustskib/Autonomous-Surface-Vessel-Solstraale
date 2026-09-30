"""Run the boat stack against Gazebo instead of hardware.

Starts Gazebo, the ros_gz bridge and thrust_splitter in place of the
hardware nodes, plus the hardware-independent nodes. Do not run it together
with boat_bringup, both publish on the same topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    boat_simulation_share = get_package_share_directory('boat_simulation')
    world_path = os.path.join(boat_simulation_share, 'worlds', 'boat_world.sdf')
    bridge_config_path = os.path.join(boat_simulation_share, 'config', 'bridge.yaml')

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # -s: server only, no Gazebo GUI. Use Foxglove to watch.
        launch_arguments={'gz_args': f'-r -s {world_path}'}.items(),
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen',
    )

    return LaunchDescription([
        gz_sim_launch,
        bridge_node,
        Node(
            package='boat_hardware_interface',
            executable='thrust_splitter',
            name='thrust_splitter',
            output='screen',
        ),
        Node(
            package='boat_control',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        ),
        # imu_orientation_is_absolute is true here because the Gazebo IMU
        # gives an absolute orientation. It must stay false on the real
        # boat, where the Modulino quaternion has no north reference.
        Node(
            package='boat_navigation',
            executable='heading_estimator',
            name='heading_estimator',
            output='screen',
            parameters=[{'imu_orientation_is_absolute': True}],
        ),
        Node(
            package='boat_navigation',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
        ),
        # Sole publisher of boat/thrust_command.
        Node(
            package='boat_control',
            executable='control_arbiter',
            name='control_arbiter',
            output='screen',
        ),
        # Safety supervisor (geofence, link loss). Default fence is 150 m
        # around the spawn point.
        Node(
            package='boat_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
        ),
        # detect_enabled is false: nothing in the world is a trained class
        # and inference competes with Gazebo for CPU.
        Node(
            package='boat_perception',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{'detect_enabled': False}],
        ),
        # dock_marker_node is not started, the world has no ArUco marker.
        Node(
            package='boat_web_ui',
            executable='web_server',
            name='web_server',
            output='screen',
        ),
        # Foxglove websocket on ws://<host>:8765.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}],
        ),
    ])
