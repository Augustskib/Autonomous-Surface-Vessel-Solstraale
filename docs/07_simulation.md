# Simulation

[Back to README](../README.md)

I use Gazebo to develop the dashboard and navigation before trying them on the physical boat. The simulation runs with ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04 through WSL.

## Run

From the repository root, with the ROS 2 and Gazebo dependencies available:

~~~bash
colcon build --symlink-install
source install/setup.bash
ros2 launch boat_simulation boat_simulation.launch.py
~~~

Gazebo starts without its 3D window. I use the dashboard at `http://localhost:5000` and Foxglove at `ws://localhost:8765` to view the camera and telemetry.

In another sourced terminal:

~~~bash
ros2 node list
ros2 topic list
ros2 topic info /boat/thrust_command --verbose
ros2 topic hz /boat/gps/fix
ros2 topic hz /boat/imu
ros2 topic hz /boat/camera/image_raw
~~~

Check that one arbiter is publishing the selected motor command. Do not run the hardware and simulation launches together; they use the same topic names.

## Helper script

`scripts/sim/boatsim.sh` finds the workspace relative to itself, sources `/opt/ros/jazzy/setup.bash`, and turns on software rendering.

It also force-stops processes matching names such as the dashboard and control nodes. That can interfere with other ROS sessions, so check what is running before using it. The direct launch above avoids that cleanup step.

## What I still need to test

I have used simulation for dashboard and waypoint development. I want repeatable tests for manual takeover, HOLD, resume, cancellation, mission completion, and delayed or missing sensor messages.

For each run, I need to keep the configuration, sensor rates, commands, and outcome.

## Differences from the real boat

- The mass, drag, and propulsion settings still need tuning.
- The 30 N simulated propeller limit is an estimate, not a measured thruster rating.
- The simulated IMU has absolute orientation; the real Modulino has no north reference.
- The bridge has no clock bridge yet, so simulation-time handling needs work.
- Heat, sealing, power behavior, and communication range need physical tests.

Once the hull is finished, I can update the model using its dimensions, loaded mass, and measured motor response.
