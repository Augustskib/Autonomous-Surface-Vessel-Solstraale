# Software architecture

[Back to README](../README.md)

I split the software into ROS 2 packages so I can work on the hardware interfaces, control, navigation, and recording separately. The repository root is the colcon workspace.

## Packages

| Package | What it does |
|---|---|
| `boat_msgs` | Custom messages |
| `boat_bringup` | Launches the physical boat |
| `boat_control` | Mission management, command selection, and safety checks |
| `boat_navigation` | Waypoint following and heading estimation |
| `boat_hardware_interface` | Motors, GPS, IMU, power, system monitoring, and simulated thrust |
| `boat_perception` | Camera, YOLO detection, and dock-marker detection |
| `boat_data` | Recording and run metadata |
| `boat_web_ui` | Browser dashboard |
| `boat_description` | Vehicle model |
| `boat_simulation` | Gazebo world and sensor bridge |

Arduino code is in `firmware/boat_motor_controller/`. Bench tools are in `scripts/bench/`, and simulation helpers are in `scripts/sim/`.

## Main control topics

| Topic | Purpose |
|---|---|
| `/boat/cmd/manual` | Manual requests |
| `/boat/cmd/waypoint` | Navigation requests |
| `/boat/cmd/failsafe` | Safety-recovery requests |
| `/boat/thrust_command` | Command selected by the arbiter |
| `/boat/data_command` | Recorder actions |
| `/boat/data_events` | Recording markers and segments |

The sensor topics are described in [system architecture](02_system_architecture.md).

## Dashboard and perception

The dashboard has manual controls, route tools, telemetry, camera view, and recording controls. Manual driving, the RGB feed, IMU feedback, and E-stop are working. I have not started testing recording yet. It runs on port 5000. There is no login or separate controlling-user account yet, so I use it on a controlled local network.

YOLO is off at startup and can be enabled from the dashboard. The default `yolov8n.pt` model is a general-purpose starting point. I still need a model suited to the objects and conditions the boat will encounter.

Dock-marker detection is also implemented. Using it to dock the boat autonomously is future work, as is obstacle avoidance.

## Hardware and simulation

Both launches start the main control and navigation nodes, dashboard, safety monitor, and Foxglove.

The hardware launch adds the motor, GPS, camera, power, and system interfaces. It sets a 30% motor-command cap and treats the IMU orientation as relative.

Simulation uses Gazebo sensors and `thrust_splitter`. It treats the simulated IMU orientation as absolute and does not start the real motor interface or dock-marker node.

The recorder starts separately in both setups:

~~~bash
ros2 run boat_data data_recorder
~~~

## Remaining work

I need more testing of mission transitions, manual takeover, reconnects, and missing sensor data. I also plan to add command IDs and acknowledgements so it is easier to follow a request through the system.

The [test log](09_test_log.md) records the software checks and their limits.
