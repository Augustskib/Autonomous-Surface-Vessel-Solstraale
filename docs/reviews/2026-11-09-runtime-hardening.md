# Runtime hardening — 2026-11-09

## Implemented

- Both MJPEG endpoints now use shared rate-limited JPEG caches (raw 10 Hz, detector 5 Hz). Multiple viewers reuse the same encoding. A missing/stale source produces a NO FRESH VIDEO image after two seconds, rather than displaying the last cached image indefinitely. Encoding/conversion errors are contained. Receipt freshness is not proof of acquisition timing; a publisher replaying old frames still requires timestamp validation.
- Dashboard binds only 127.0.0.1:5000. Host and cross-origin POST checks protect the local endpoint against ordinary cross-site browser requests. Optional HTTP Basic authentication accepts username `operator` and the password in ASV_OPERATOR_PASSWORD. It does not encrypt traffic. No password is stored in the repository.
- Hardware and simulation Foxglove launches bind 127.0.0.1:8765 and advertise only connectionGraph capability, removing client publication, parameter changes and service calls through that bridge. Verify installed bridge behavior before deployment. This does not authenticate DDS or protect against other local ROS publishers.
- Simulation launcher no longer runs global pkill. A Linux file lock prevents duplicate managed launches from this checkout. A new process session owns its children; shutdown signals only that group, escalating after bounded waits. Simulation uses ROS domain 42 (override ASV_SIM_DOMAIN_ID) and a workspace-specific Gazebo partition. Lock enforcement applies to this launcher, not manually started ROS stacks. Old unowned orphan processes are intentionally not killed automatically.
- Gazebo /clock is bridged. Heading estimator and waypoint follower use simulation time for physics calculations. Safety-supervisor timers/elapsed time and simulator command-loss watchdog use steady time. Arbiter already had a steady watchdog. Full rewind/reset and pause/resume acceptance remains to be tested in ROS/Gazebo.
- Hardware bring-up requires explicit geofence center/radius rather than silently using Lake Eola. It is a deliberate launch-interface change. Use the measured operating site, not guessed coordinates.

## Connecting from laptop/desktop

The Pi's direct web address will no longer work with these defaults. From the computer that will run the browser, establish an authenticated SSH connection to the Pi with forwarding:

```sh
ssh -N -L 127.0.0.1:5000:127.0.0.1:5000 -L 127.0.0.1:8765:127.0.0.1:8765 YOUR_PI_USER@YOUR_PI_ADDRESS
```

Keep the tunnel running. Open http://127.0.0.1:5000 and connect Foxglove to ws://127.0.0.1:8765. SSH credentials/keys and an actual route to the Pi are required; this does not solve carrier NAT or isolated hotspots. Do not open public router forwarding to these control ports. A VPN or reverse tunnel may be needed after the modem topology is known. No host network settings were changed here.

For an additional dashboard password, set ASV_OPERATOR_PASSWORD in the environment launching the boat stack. Enter username operator in the browser's login prompt. Never use this HTTP password over an unencrypted public connection; the SSH tunnel is the encryption boundary. The password is optional for local-only use, where OS/SSH access remains the boundary.

Hardware launch now requires:

```sh
ros2 launch boat_bringup boat_bringup.launch.py geofence_center_lat:=YOUR_LATITUDE geofence_center_lon:=YOUR_LONGITUDE geofence_radius_m:=YOUR_RADIUS max_thrust_pct:=30
```

Simulation: `scripts/sim/boatsim.sh` or `scripts/sim/boatsim.sh --build`. Other simulation tools must use ROS_DOMAIN_ID=42 to see this stack. Hardware must use a different domain. Web ports remain shared, so do not run both web stacks on one host without explicit port configuration.

## Validation and remaining work

Offline Python/browser tests cover cache sharing/rate/staleness, failure containment, password/origin helpers, steady watchdog callbacks and scoped cleanup signals. Linux process-tree cleanup, SSH, Foxglove capabilities, DDS and live Gazebo pause/resume were not exercised on this Windows host. These remain required commissioning checks.

Still outstanding: measured camera/IMU/GPS/LiDAR transforms and noise, true camera acquisition freshness, camera-to-LiDAR calibration, power-sensor configuration/calibration, independent physical stop timing and Arduino watchdog behavior, full motor-link health propagation, detector health messages, live ROS/C++ builds and Pi storage/performance measurements. No hardware verification is implied by source changes.

Bridge parameter reference: https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md

Final checks for this batch: 188 Python tests passed (one existing pytest return-value warning), 19 JavaScript tests passed, and all Python/YAML sources parsed. The process-cleanup test injects POSIX signal values and fake process-group calls on Windows; it is not a real Linux process lifecycle test.
