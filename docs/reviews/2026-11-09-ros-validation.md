# ROS validation evidence — 2026-11-09

This report closes the previously missing link from the sensor runbook. These are desktop Linux/container results, not Pi or water acceptance.

## Completed

- Ubuntu 24.04 / ROS 2 Jazzy workspace build succeeded, with optional Quanergy hardware adapter disabled for the base build.
- Recorded base validation: 216 Python tests and 19 JavaScript tests passed; launch descriptions loaded; host firmware harness passed. See `../../evidence/2026-11-09-ros-validation/asv-recheck-tests.log`.
- DDS smoke received fresh safety status and zero idle thrust, then observed ESTOP. Shutdown fixes replaced duplicate shutdown calls with try_shutdown and contained external shutdown exceptions.
- Full simulation exposed and fixed a real rclpy logger error: changing logging severity through one indirect call site crashed heading/safety/navigation callbacks. Calls now use distinct severity-specific sites.
- Simulation received GPS, IMU, camera, simulation clock, usable heading and zero idle thrust. Recorded samples: GPS 98, IMU 391, camera 116, clock 19457, heading 390, thrust 666. See `../../evidence/2026-11-09-ros-validation/asv-simulation-tests.log`.
- Optional Quanergy SDK and ROS adapter compiled in a separate x86-64 image with SDK revision `83d76e1675d14426f7d8c0dc34682bf8f5c75c29`, C++17, visualization disabled and explicit Boost dependencies. Executable loading reached required-host validation. This build was observed in the work session; its full build log was not archived before that session stopped.

## Limits

Gazebo needed SIGTERM after its SIGINT grace period. Simulation results do not establish clean shutdown of every process under every failure, waypoint motion, pause/rewind correctness, or obstacle avoidance. No ARM/Pi build, live LiDAR reception, electrical validation, motor test or water test was performed. A later final test run was started but its output was not retrieved; no result is claimed for that run.

The Nov 11 local regression run passed 200 Python tests (a different selection, excluding ROS-dependent geometry checks), including the new site-coordinate and monotonic-link-timeout cases. One existing pytest warning concerns a test returning a trace list.
