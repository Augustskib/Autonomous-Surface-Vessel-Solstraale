# Hardware bring-up

[Back to README](../README.md)

The electronics and thrusters are installed, and I can drive the boat from the control station. I have tested manual control, the motors, IMU, RGB camera, and E-stop. GPS still needs a verified fix, and I have not tested connection loss yet.

This is the checklist I am working through as I finish the engineering.

## Setup details to keep

For each test, I want to keep the wiring, software version, flashed Arduino code, sensor models, and settings. That should make it easier to work out what changed when something stops working.

The Pi setup uses Raspberry Pi OS based on Debian 13, with ROS 2 Jazzy through conda. I still need to save the exact environment and dependency versions.

The motor interface defaults to `/dev/ttyACM0` at 115200 baud. GPS uses `/dev/serial0` at 9600 baud. These need checking against the connected devices.

## Order of checks

1. Finish the wiring diagram with both supplies, the 12 A fuse, connectors, and Arduino pins.
2. Measure the supply rails with the motors disconnected, then under a controlled load.
3. Verify an outdoor GPS fix and check IMU axes and units.
4. Measure motor current, temperature, and response under load.
5. Test command loss, serial disconnection, and restart; measure E-stop timing.
6. Calibrate the camera and IMU, and try the thermal feed.
7. Check sealing and record loaded flotation before a supervised water run.

I use the [test template](../evidence/test_record_template.md) to keep the setup and results together.

## Build and launch

From the repository root, with the ROS 2 environment active:

~~~bash
colcon build --symlink-install
source install/setup.bash
~~~

Keep propulsion disconnected during the initial sensor and software checks. This launch starts the real motor interface:

~~~bash
ros2 launch boat_bringup boat_bringup.launch.py
~~~

It sets a 30% command cap. The geofence defaults to the simulation location, so I need to change and check its center and radius before a water trial. Those settings are not exposed as top-level launch arguments yet.

The dashboard is at `http://<boat-ip>:5000`. Foxglove connects through `ws://<boat-ip>:8765`.

## Bench scripts

There are motor-identification, short-run, instrumented-test, and hardware-check scripts in `scripts/bench/`. Some command the motors directly. Check their ports, command levels, and stop behavior before running them.

## Before water testing

Before a water trial, I want the remaining wiring and ratings checked, a usable GPS fix, and stop response measured. Sealing, loaded flotation, communication range, and cooling also need checks with the assembled boat.
