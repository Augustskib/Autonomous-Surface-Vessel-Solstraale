"""Bring up the full stack on the boat (Raspberry Pi).

    ros2 launch boat_bringup boat_bringup.launch.py
    ros2 launch boat_bringup boat_bringup.launch.py max_thrust_pct:=50
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Thrust cap in percent for motor_driver and the dashboard. 30% while
    # testing.
    max_thrust = ParameterValue(LaunchConfiguration('max_thrust_pct'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('max_thrust_pct', default_value='30.0',
                              description='Thrust cap in percent, 0-100'),
        Node(
            package='boat_perception',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
        ),
        # Inference off by default, turn it on with the DETECT button.
        Node(
            package='boat_perception',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{'detect_enabled': False}],
        ),
        # ArUco dock marker. Publishes only when a marker is in frame.
        Node(
            package='boat_perception',
            executable='dock_marker_node',
            name='dock_marker_node',
            output='screen',
        ),
        Node(
            package='boat_web_ui',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[{'manual_thrust_limit': max_thrust}],
        ),
        Node(
            package='boat_hardware_interface',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[{'max_thrust_pct': max_thrust}],
        ),
        Node(
            package='boat_hardware_interface',
            executable='gps_publisher',
            name='gps_publisher',
            output='screen',
        ),
        # imu_publisher is not launched: the Modulino IMU is on the
        # Arduino and motor_driver publishes boat/imu.
        # power_monitor: INA226 addresses (0x40/0x41/0x44) and the 0.1 ohm
        # shunt are not verified against the boards yet.
        Node(
            package='boat_hardware_interface',
            executable='power_monitor',
            name='power_monitor',
            output='screen',
            parameters=[{'i2c_bus': 1}],
        ),
        # CPU temperature, load, throttling and disk space.
        Node(
            package='boat_hardware_interface',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
        ),
        Node(
            package='boat_control',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        ),
        # Fuses GPS course over ground with the gyro into boat/heading.
        # imu_orientation_is_absolute must stay false: the Modulino has no
        # magnetometer, so its orientation has no north reference.
        Node(
            package='boat_navigation',
            executable='heading_estimator',
            name='heading_estimator',
            output='screen',
            parameters=[{'imu_orientation_is_absolute': False}],
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
        # Safety supervisor: geofence and link loss. The default fence is
        # centred on the sim origin (Lake Eola). Set it to the real site
        # before any water test:
        #   -p geofence_center_lat:=<lat> -p geofence_center_lon:=<lon>
        #   -p geofence_radius_m:=<m>
        Node(
            package='boat_control',
            executable='safety_monitor',
            name='safety_monitor',
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
