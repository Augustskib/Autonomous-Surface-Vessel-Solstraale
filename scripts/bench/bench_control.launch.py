"""Manual-only bench control station at 15% thrust. No autonomy is launched.

    ros2 launch scripts/bench/bench_control.launch.py
"""
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [
        Node(package='boat_hardware_interface', executable='motor_driver',
             output='screen', parameters=[{'max_thrust_pct': 15.0, 'start_estopped': True}]),
        Node(package='boat_control', executable='control_arbiter', output='screen'),
        Node(package='boat_control', executable='mission_manager', output='screen'),
        Node(package='boat_web_ui', executable='web_server', output='screen',
             parameters=[{'manual_thrust_limit': 15.0}]),
    ]
    # If any node exits, shut the whole launch down.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='Bench control node exited'))]))
                for node in nodes]
    return LaunchDescription(handlers + nodes)
