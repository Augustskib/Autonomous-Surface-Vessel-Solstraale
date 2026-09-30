# SOLSTRÅLE: Autonomous Surface Vessel

SOLSTRÅLE is a small twin-hull autonomous surface vessel that I am building
on my own to work through the whole robotics pipeline: hull and electronics,
embedded motor control, ROS 2, navigation, perception, and eventually
synchronized sensor datasets recorded on real water.

## Status

| Area | State |
|---|---|
| ROS 2 workspace (Jazzy) | Built, runs on the Raspberry Pi |
| Motor control (Arduino + 2x BTS7960) | Verified on the bench, both directions, both motors |
| Control station (web dashboard) | Verified on the bench: manual drive, camera, IMU attitude |
| Camera | Verified, uncalibrated |
| IMU (via Arduino) | Verified, not yet calibrated |
| GPS | Wired, needs an outdoor fix test |
| Heading estimation | Built, needs on-water testing |
| Waypoint navigation | Verified in simulation only |
| Gazebo simulation | Verified |
| Data recording | Built, not yet used on a real run |
| Object detection | Placeholder, disabled |

Thrust is capped at 30% by default until the boat has had a proper testing
period on the water.

## Hardware

- Raspberry Pi 4, Raspberry Pi OS (Debian 13), ROS 2 Jazzy through conda
- Arduino UNO R4 WiFi with a Modulino Movement IMU on Qwiic
- 2x BTS7960 (IBT-2) motor drivers, 2x brushed underwater thrusters, 3S LiPo
- NEO-6M GPS on the Pi UART, OV5647 camera on the Pi CSI port

## Repository layout

```
src/        ROS 2 packages (the repo root is the colcon workspace)
firmware/   Arduino motor controller and bench sketches
scripts/    bench and simulation helper scripts
tests/      regression tests that run without ROS
```

## Build and run

```bash
cd ~/Autonomous-Surface-Vessel-Solstraale
colcon build --symlink-install
source install/setup.bash

# on the boat
ros2 launch boat_bringup boat_bringup.launch.py

# in simulation (kills leftover Gazebo processes first)
scripts/sim/boatsim.sh
```

The control station runs on the boat at `http://<boat-ip>:5000`.

Full documentation is in progress. Earlier development history is kept in a
private archive.

## License

MIT
