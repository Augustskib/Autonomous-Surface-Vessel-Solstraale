# Reproducible ROS validation

Target environment: ROS 2 Jazzy on Ubuntu 24.04. The Docker recipe resolves package dependencies with rosdep, builds the workspace, then executes behavior tests against installed ROS interfaces, browser-script tests, launch-description loading, a real DDS core-node smoke test and host firmware tests. It does not connect serial devices or command physical hardware.

From the repository root on a machine with Docker:

```sh
docker build -f scripts/validation/Dockerfile -t asv-validation:jazzy .
docker run --rm --network none asv-validation:jazzy
```

The runtime has no network access beyond its own container loopback and no host devices mounted. The DDS test starts only safety_monitor and control_arbiter. It checks fresh safety publication, zero idle output, and emergency-stop mode. It does not prove manual driving, mission execution, complete simulation, motor fault recovery or physical stop latency.

Optional SDK compilation gate, after the base image succeeds:

```sh
docker build -f scripts/validation/Dockerfile.lidar -t asv-validation:quanergy .
```

This pins Quanergy SDK source revision 83d76e1675d14426f7d8c0dc34682bf8f5c75c29 and adapts its C++ standard for modern PCL. The base workspace intentionally builds without the hardware executable. Passing the base image alone does not prove LiDAR compilation. Neither compilation gate connects a sensor.

GitHub workflow `.github/workflows/ros-validation.yml` runs the base gate on pull requests or manual dispatch after the files are committed/pushed. It has not been triggered merely by creating the file locally. Dependencies are resolved at build time, so this is a reproducible procedure, not a byte-for-byte locked operating-system image; retain image IDs and logs with acceptance evidence.

The existing repository has historical style/lint debt. This gate explicitly runs behavioral suites; it is not a claim that all ament lint suites pass. The waypoint geometry tests now run against installed ROS imports, rather than being omitted due to unavailable rclpy on Windows.


For source-only rechecks without reinstalling unchanged dependencies:

```sh
docker build -f scripts/validation/Dockerfile.recheck -t asv-validation:working .
docker run --rm --network none asv-validation:working
```

Use the full Dockerfile again whenever package dependencies change.

The optional live simulation smoke test receives all six expected streams and checks usable heading and zero idle output:

```sh
docker run --rm --network none -e LIBGL_ALWAYS_SOFTWARE=1 -e ROS_DOMAIN_ID=174 asv-validation:working bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 scripts/validation/simulation_smoke.py'
```

A passing launch exit does not imply every child exited on SIGINT. Inspect its emitted node log for escalation: the current environment required SIGTERM for Gazebo even though Python nodes exited cleanly. This test does not exercise waypoint motion, pause/resume, clock rewind or physical hardware.
