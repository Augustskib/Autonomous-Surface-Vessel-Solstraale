#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHON=/opt/asv-tests/bin/python
# Behavioral tests, using generated ROS messages where imports require them.
"$PYTHON" -m pytest tests src/boat_navigation/test/test_heading_logic.py src/boat_navigation/test/test_waypoint_follower_math.py src/boat_data/test/test_recorder_logic.py src/boat_control/test/test_control_logic.py src/boat_control/test/test_failsafe_logic.py src/boat_control/test/test_operator_handover.py src/boat_control/test/test_control_contract.py src/boat_control/test/test_mission_protocol.py src/boat_web_ui/test/test_operator_endpoint.py -q
node --test src/boat_web_ui/test/operator_controls.test.cjs src/boat_web_ui/test/pilot_page.test.cjs src/boat_web_ui/test/recording_page.test.cjs
# Loads launch descriptions without starting motors or devices.
ros2 launch boat_simulation boat_simulation.launch.py --show-args
ros2 launch boat_bringup boat_bringup.launch.py --show-args
ros2 launch boat_data recording.launch.py --show-args
ros2 launch boat_navigation fusion_shadow.launch.py --show-args
ros2 launch boat_lidar quanergy.launch.py --show-args

python3 scripts/validation/dds_smoke.py

g++ -std=c++17 -Ifirmware/test firmware/test/test.cpp -o /tmp/asv-firmware-test
/tmp/asv-firmware-test

python3 scripts/validation/recording_smoke.py
