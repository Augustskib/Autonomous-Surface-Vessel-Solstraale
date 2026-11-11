# Whole-project engineering review — 7 November 2026

This review covers the current **local working tree** on `fix/operator-handover`,
including the uncommitted UI/control changes. It is not a review of a deployed
Pi image, and no motor, ROS launch or remote system was operated.

The architecture is workable and does not need a wholesale rewrite. The main
remaining weaknesses are freshness and provenance of sensor data, integration
between packages, runtime fault recovery, and confidence in the recording pipeline.
A 3D LiDAR should initially be added as a measured sensor and visualization source,
not silently promoted to an obstacle-avoidance controller.

## Follow-up repairs

The findings below describe the 7 November baseline. Local repairs to R01–R05
are documented in [the 8 November repair batch](2026-11-08-repair-batch.md).
R05 is partial; hardware and ROS validation remain outstanding.

## Scope and evidence

Reviewed runtime paths across control, navigation, hardware interfaces, perception,
web gateway, recording, launch files, simulation bridge/model, robot description,
Arduino firmware, bench tooling, package manifests and current project docs.
Coverage is source-level and targeted behavioral testing, not a claim that every
possible behavior or every ancillary script has been exhaustively verified.

- Earlier control/UI checks: 109 passed.
- Additional checks this pass: 13 regression, 21 heading, 10 recorder checks passed.
- One old regression fixture broke because it lacked the newly required mission
  identity. Updated that fixture to exercise valid identity plus invalid coordinates;
  its rejection assertion now passes. No production behavior changed in this pass.
- Existing host C++ firmware tests passed. Added diagnostic probes reproduced
  oversized-line and trailing-garbage acceptance using the actual firmware source.
- Python probes exercised actual production functions/callbacks with synthetic
  transport. They demonstrate behavior, not real receiver performance.
- Full ROS build, ament lint, DDS, hardware, radio range and real bags remain untested.
  The waypoint math suite could not run normally because pytest and ROS are absent.
- Several recording findings were already listed in `docs/10_known_gaps.md`;
  this review confirms they remain unresolved rather than claiming them as new.

Evidence files are in the desktop work folder:
`outputs/asv-whole-project-review-2026-11-07/` (`review_probes.py`,
`probe-results.json`, `firmware_probe.cpp`, `firmware-results.txt`,
`battery-results.txt`, and `additional-test-results.txt`). Diagnostic probes
intentionally demonstrate bad behavior; they are not acceptance tests.

## Prioritized findings

P1 means resolve before relying on the affected workflow. P2 means a consequential
correctness, performance or maintainability issue. “Confirmed” describes code
behavior or configuration; hardware-dependent consequences remain unmeasured.

### R01 · P1 · Firmware accepts the tail of an oversized command

**Evidence:** Reproduced. [firmware/boat_motor_controller/boat_motor_controller.ino:64](C:/Users/August/Documents/Codex/ASV-operator-handover/firmware/boat_motor_controller/boat_motor_controller.ino:64)

A 32-character noise prefix followed by L:100 R:100 on the same line applies full thrust and refreshes the watchdog. Overflow resets the buffer immediately instead of discarding everything until newline. sscanf also accepts trailing garbage.

**Improvement:** Use a discard-until-newline state, require complete bounded parsing, and add tests for fragmented, oversized and malformed commands. The host C++ probe reproduced both cases without hardware.

### R02 · P1 · Heading remains usable after the gyro stops

**Evidence:** Reproduced. [src/boat_navigation/boat_navigation/heading.py:341](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_navigation/boat_navigation/heading.py:341)

After one reference and one gyro sample, evaluate(30) reports DEAD_RECKONING and source GYRO with no further gyro data. The ROS adapter keeps publishing at 20 Hz, so consumer message freshness cannot detect the frozen underlying measurement.

**Improvement:** Track accepted gyro acquisition freshness independently of reference freshness. Expire the relevant estimate, expose degraded GPS-course-only operation separately, and test disconnected IMU with the estimator still running.

### R03 · P1 · GPS backlog is restamped as current data

**Evidence:** Reproduced conditional input. [src/boat_hardware_interface/boat_hardware_interface/gps_publisher.py:29](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_hardware_interface/boat_hardware_interface/gps_publisher.py:29)

Only one NMEA line is consumed every 0.2 seconds, including ignored sentences. In a fake 10-line/s stream, after ten seconds 50 lines remain queued and the published sample is 4.9 seconds old, yet stamped with the current clock. Actual backlog depends on receiver configuration.

**Improvement:** Use bounded nonblocking draining, retain the latest complete valid fix, preserve acquisition timing and report serial lag. Reject malformed numeric fields and configure receiver output deliberately.

### R04 · P1 · Manual-only bench launch cannot acquire control

**Evidence:** Confirmed integration regression. [scripts/bench/bench_control.launch.py:13](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/bench/bench_control.launch.py:13)

The launch starts the stricter arbiter but no safety_monitor. The arbiter now requires a fresh safety stream before MANUAL. This is a compatibility regression introduced by the recent hardening.

**Improvement:** Provide an explicit supervised bench profile with appropriate site-independent policy and test its complete launch. Do not globally disable supervision to make the bench work.

### R05 · P1 · Motor-link recovery can restore thrust without explicit rearm

**Evidence:** Confirmed behavior; hardware timing unverified. [src/boat_hardware_interface/boat_hardware_interface/motor_driver.py:122](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_hardware_interface/boat_hardware_interface/motor_driver.py:122)

Serial reconnection is automatic; incoming commands continue to be accepted while disconnected. When the port returns, the next command is sent. STATUS watchdog/applied-output reports are ignored, so the arbiter/UI cannot distinguish requested control from motor-link health.

**Improvement:** Publish motor-controller health and applied output, latch Hold on loss/reset, and require explicit rearm after a reconnect. Test actual USB disconnect, Arduino reboot and stale queued commands.

### R06 · P1 · IMU covariance marks real measurements as absent

**Evidence:** Integration contract defect. [src/boat_hardware_interface/boat_hardware_interface/motor_driver.py:280](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_hardware_interface/boat_hardware_interface/motor_driver.py:280)

Gyro and acceleration are populated, but covariance[0] is -1 in both IMU publishers. ROS Imu defines -1 as no estimate; unknown covariance is all zeros. A compliant future EKF can discard these data. Current custom heading logic ignores the covariance, hiding the mismatch.

**Improvement:** Publish measured covariance, or unknown zeros pending calibration; reserve -1 for unavailable components. Add an EKF-facing message contract test. See the linked official Imu definition.

### R07 · P1 · Sensor frames are incomplete and camera axes disagree

**Evidence:** Integration gap. [src/boat_description/urdf/boat.urdf:73](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_description/urdf/boat.urdf:73)

No imu_frame or lidar_frame exists. camera_frame has body-aligned axes, while dock_marker_node reports optical coordinates under the image frame name. Neither runtime launch starts robot_state_publisher. The mesh also points to an old laptop absolute path.

**Improvement:** Use a portable description with measured sensor mounts, camera optical frame, IMU and LiDAR frames; publish static transforms in hardware and simulation. Add dynamic map/odom transforms only from an actual localization source.

### R08 · P2 · Battery display assumes four cells on the documented three-cell boat

**Evidence:** Reproduced; existing documented TODO. [src/boat_web_ui/boat_web_ui/web_server.py:66](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_web_ui/boat_web_ui/web_server.py:66)

The production function classifies 12.6 V as BELOW_RESERVE with zero reserve. Passing cells=3 classifies the same input as OK. This is a display/model error, not a verified battery protection system.

**Improvement:** Make chemistry/cell count site or hardware configuration; label voltage-derived reserve as an estimate and test configured thresholds. Confirm the installed pack before changing deployment settings.

### R09 · P2 · Current sensing depends on unverified hard-coded shunts

**Evidence:** Confirmed configuration limitation. [src/boat_hardware_interface/boat_hardware_interface/power_monitor.py:29](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_hardware_interface/boat_hardware_interface/power_monitor.py:29)

Addresses and shunt resistance are fixed in code and documented as unverified. Current is divided by that value, so a mismatch directly scales every current/power reading. There is no sensor-identity or saturation check.

**Improvement:** Parameterize each channel, verify markings and address, compare readings with known measurements, and mark uncalibrated data explicitly. Do not use it for automatic protection until validated.

### R10 · P1 · Post-processing cannot locate normal recorder output

**Evidence:** Reproduced; previously listed gap. [scripts/postprocess_run.py:61](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/postprocess_run.py:61)

Recorder writes run/bag/rosbag2/*.mcap; finder only glob-searches run/bag/*.mcap. A run path fails discovery. Direct nested-file invocation also selects the wrong run root for manifest merging, and only the first bag file is processed.

**Improvement:** Resolve the archive root from manifest/metadata, discover all segments deterministically and test using an archive produced by the recorder.

### R11 · P1 · Real NavSatFix messages break synchronization processing

**Evidence:** Reproduced; previously listed gap. [scripts/postprocess_run.py:157](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/postprocess_run.py:157)

build_sync_index reads gps_msg.status as though it were an integer; it is a nested NavSatStatus message. Comparing it with zero raises TypeError.

**Improvement:** Read status.status and test real ROS-shaped decoded records, including missing and no-fix samples. Synthetic flattened test data must not define the production schema.

### R12 · P1 · Missing streams can produce a passing quality report

**Evidence:** Reproduced. [scripts/postprocess_run.py:168](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/postprocess_run.py:168)

With camera frames but no GPS or IMU, poor_sync is false and the quality report passes. An empty report also defaults to pass. Drop and sync issues mostly append warnings without failing the result.

**Improvement:** Define required streams per collection profile, distinguish missing/unknown/degraded/fail, require minimum valid coverage, and make the quality criteria explicit.

### R13 · P2 · Sorting removes evidence of out-of-order samples

**Evidence:** Reproduced. [scripts/postprocess_run.py:81](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/postprocess_run.py:81)

The loader sorts by log time before validating ordering. The out-of-order check therefore sees sorted input and reports zero. Synchronization also uses arrival/log time rather than sensor acquisition time.

**Improvement:** Validate original sequence and header timestamps before sorting a separate index; keep both acquisition and receipt clocks, with offset and clock-reset diagnostics.

### R14 · P2 · Post-processing invalidates the capture checksum manifest

**Evidence:** Confirmed. [scripts/postprocess_run.py:250](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/postprocess_run.py:250)

Capture hashes manifest.json, then post-processing overwrites it without regenerating or separating checksums. All decoded messages, including image payloads, are retained in RAM, which scales poorly with long recordings and LiDAR.

**Improvement:** Keep immutable capture provenance/checksums and versioned derived reports. Stream metadata/timestamps rather than retaining every decoded sensor payload.

### R15 · P2 · Recording storage estimates are too optimistic and no reserve stop exists

**Evidence:** Confirmed arithmetic / lifecycle gap. [src/boat_data/boat_data/data_recorder.py:97](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_data/boat_data/data_recorder.py:97)

640x480 bgr8 at 10 Hz is 9.216 MB/s or 33.18 GB/hour before overhead, versus the 7 GB/hour planning text. The start threshold is checked only at start; runtime status reports free space but does not stop before exhaustion. LiDAR adds model-dependent traffic.

**Improvement:** Use observed rate for estimates and a configurable runtime reserve with graceful finalization. Measure actual MCAP storage and include LiDAR profile limits; do not treat the raw calculation as a measured disk rate.

### R16 · P2 · Video encoding is unbounded per client

**Evidence:** Reproduced. [src/boat_web_ui/boat_web_ui/web_server.py:2596](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_web_ui/boat_web_ui/web_server.py:2596)

An unchanged cached frame was JPEG-encoded ten times for ten reads with no pacing. Each connected viewer runs another encoder loop; an old frame continues to look like a streaming feed after capture stops.

**Improvement:** Encode once per new frame, share bytes across clients, bound output FPS/quality and display frame age. Apply the same rule to detection video and future LiDAR previews.

### R17 · P2 · Most subsystem status can remain green after its publisher dies

**Evidence:** Confirmed. [src/boat_web_ui/boat_web_ui/web_server.py:2935](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_web_ui/boat_web_ui/web_server.py:2935)

Operator and mission status have arrival times, but cached camera/power/system/recording/dock-marker state has no general expiry. Recording may keep displaying the last true value after the recorder node disappears; a last detected marker can persist after video loss.

**Improvement:** Give every status source its own timestamp, timeout and unavailable state. Display sensor age separately from transport connectivity. Extend this contract to LiDAR.

### R18 · P2 · Non-control buttons still report success without delivery

**Evidence:** Confirmed. [src/boat_web_ui/boat_web_ui/web_server.py:2516](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_web_ui/boat_web_ui/web_server.py:2516)

Recording and detector endpoints return status ok even when ros_node is absent; they do not require matching subsystem acceptance. Several JSON handlers still assume an object. The corrected operator protocol has not been generalized to these commands.

**Improvement:** Validate input types and use accepted/pending/failed receipts for subsystem actions. Include recorder absent, full disk, model-load failure and no ROS tests.

### R19 · P2 · Hold and powered safety stops ramp down in simulation

**Evidence:** Confirmed simulation mismatch. [src/boat_hardware_interface/boat_hardware_interface/thrust_splitter.py:63](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_hardware_interface/boat_hardware_interface/thrust_splitter.py:63)

An ordinary zero target ramps from 30 N in roughly one second. Only the separate e-stop and watchdog paths bypass the ramp. The real driver writes zero immediately, so simulated Hold behavior differs from the control contract and real path.

**Improvement:** Represent stop intent explicitly or bypass down-ramping for propulsion stops; preserve an intentional acceleration ramp. Test force output, not only arbiter percentages.

### R20 · P2 · Simulation time is not wired through the runtime

**Evidence:** Confirmed configuration gap. [src/boat_simulation/config/bridge.yaml:5](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_simulation/config/bridge.yaml:5)

The bridge has no /clock mapping and the launch does not set use_sim_time. Heading/GPS velocity math uses callback wall-time intervals although Gazebo sensors follow simulation timing. Slow or accelerated simulation changes estimated speed and course gating.

**Improvement:** Use simulation time for sensor/physics calculations and retain monotonic/steady clocks for operator watchdogs. Test pause and changed real-time factor deliberately.

### R21 · P2 · Simulation cleanup can kill unrelated processes and leave duplicates

**Evidence:** Confirmed. [scripts/sim/boatsim.sh:19](C:/Users/August/Documents/Codex/ASV-operator-handover/scripts/sim/boatsim.sh:19)

The script globally SIGKILLs generic names including web_server and safety_monitor, but omits control_arbiter and heading_estimator. It can affect another stack and still leave duplicate controller/heading publishers.

**Improvement:** Manage a launch process group/PID set, shut down gracefully, and verify exactly one expected node per domain. Separate simulation and hardware ROS domains.

### R22 · P1 · Network access is also control authority

**Evidence:** Confirmed exposure; deployment-dependent reachability. [src/boat_web_ui/boat_web_ui/web_server.py:3073](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_web_ui/boat_web_ui/web_server.py:3073)

The server binds all interfaces with no authentication; Foxglove also exposes a separate ROS bridge. Operator session IDs provide ordering, not identity/security. The phone-hotspot/boat-modem network and reachable interfaces are not configured in this repository.

**Improvement:** Define the trusted network boundary, authenticate command access, restrict bridge capabilities and use an encrypted private path when crossing networks. Test routing/reconnect separately from internet access; do not expose raw command ports publicly.

### R23 · P2 · Heading and calibration are not ready for global LiDAR fusion

**Evidence:** Confirmed limitation. [src/boat_navigation/boat_navigation/heading_estimator.py:51](C:/Users/August/Documents/Codex/ASV-operator-handover/src/boat_navigation/boat_navigation/heading_estimator.py:51)

COG is travel direction, not bow yaw. A fixed 0.3 m displacement gate rejects clean 1 m/s motion at 5 Hz (0.2 m per fix); GPS noise can dominate consecutive-fix direction. The filter also accepts invalid speed by retaining an older valid speed.

**Improvement:** Separate course, body attitude and map pose. Accumulate displacement over a suitable window, use receiver velocity/quality where available, reject invalid inputs, and calibrate before choosing an EKF. Start LiDAR visualization in the boat frame.

### R24 · P2 · Launch and documentation no longer describe one consistent system

**Evidence:** Confirmed documentation / launch drift. [docs/10_known_gaps.md:21](C:/Users/August/Documents/Codex/ASV-operator-handover/docs/10_known_gaps.md:21)

The gaps page still says stale safety returns to nominal and command IDs do not exist; the current code differs. The simulation package launches boat_perception without declaring it as a runtime dependency. Recorder remains opt-in, and runtime launch does not include TF.

**Improvement:** Update a single capability matrix with implemented/offline-tested/sim-tested/hardware-tested states. Add all launched package dependencies and launch smoke tests; keep hardware, bench and simulation profiles explicit.

## Additional risks requiring measurement or explicit policy

- The Arduino watchdog is a cooperative check in loop(), not an independent hardware
  watchdog. Serial prints and IMU I2C work execute in the same loop. Blocking behavior
  under USB backpressure or a hung sensor must be measured; host stubs cannot establish
  a real 500 ms worst-case stop. Bound work, make telemetry droppable and evaluate an
  independent watchdog/physical stop path.
- IMU samples have no acquisition timestamp in the serial protocol. Multiple buffered
  samples are stamped at processing time; dt and integrated attitude can be distorted.
- Power and temperature are monitored but not fed into a calibrated electrical fault
  policy. A hard-stop priority in the arbiter does not mean overcurrent protection is
  implemented. INA shunts, current range and the physical disconnect need verification.
- Firmware duty maps linearly to percent. Motor dead zones, asymmetry, reverse thrust,
  battery sag and reversal current are unmeasured; simulation uses estimated forces and
  drag. Neither percentage nor simulated newtons establishes real speed or stopping distance.
- Driver reconnects, estimator restarts and follower restarts need end-to-end scenarios.
  Accepted routes use volatile delivery; a restarted follower has no active route replay
  mechanism. Remaining stopped is preferable to automatic replay, but the UI needs an
  explicit recovery/reload workflow rather than an apparently running stranded mission.
- Runtime steering parameters are read without a coherent validated configuration
  contract. Out-of-range/non-finite values should be rejected before controller arithmetic.
- Camera calibration can be nominal; dock-marker pose is still produced from it. Tag
  calibration validity before presenting docking distances as measured geometry.
- Alternate Pi IMU code falls back to mock data on initialization errors. It is not
  launched today; require explicit mock enable and source health before reusing it.
- Scripts such as hold.py directly drive serial and bypass ROS safety policies. Keep
  them clearly bench-only with exclusive port ownership and independent disconnects.
- Map libraries are loaded from a CDN and map tiles require network service. Bundle
  essential UI assets and provide a clear offline map state for field use.

## What is already good

- Plain-Python control, mission, heading and recorder logic is separated from ROS I/O,
  making regression testing practical. The ROS package boundaries are useful.
- The arbiter centralizes normal propulsion selection. The recent authority/receipt
  and mission-identity work addresses real races instead of hiding them in the UI.
- Motor input rejects non-finite thrust and applies a hardware-path cap. Firmware
  starts disabled, drives enable low when stopped and includes a command watchdog.
- Serial receive work on the Pi is bounded and handles partial lines.
- Navigation rejects invalid fixes and large position jumps; it does not blindly use
  raw IMU yaw as a north reference. Heading quality is explicitly represented.
- Camera mock mode is visibly identified; camera health is distinct from frame source.
- The recorder has run IDs, segmentation, atomic manifest writes and disk-growth
  monitoring. Those are useful foundations once discovery/validation is corrected.
- Documentation openly identifies unmeasured hardware, placeholder perception and
  lack of station keeping. Preserve that honesty while removing stale contradictions.

## Recommended order of work

1. **Restore a trustworthy baseline:** fix bench profile compatibility, include all
   suites in one command/CI, then build all ROS interfaces and launch profiles together.
   Keep the earlier UI/control changes separate from new broad fixes when committing.
2. **Propulsion and sensor integrity:** firmware framing/watchdog bounds, controller
   health/rearm, GPS backlog and acquisition timestamps, heading input freshness and
   IMU covariance. Finish with fault injection, then bench verification.
3. **Recording and performance:** repair archive parsing and quality gates, preserve
   immutable capture hashes, stream processing, cap shared video encoding, add telemetry
   expiry and disk reserve. Record and replay one real short run end to end.
4. **LiDAR foundation:** model-specific driver, measured frames, matching QoS, sensor
   health, local recording and a bounded preview. See [LiDAR plan](../lidar-integration.md).
5. **Navigation development:** separate body yaw from course, quantify sensor noise,
   then select a localization/EKF design. Only afterward develop and validate obstacle
   tracking, clearance policy and automatic avoidance/docking.

No broad runtime fixes were silently bundled into this review. The findings above
remain open unless stated otherwise. The earlier implemented control changes are
still local and uncommitted; this review does not deploy or push them.

## Standards checked

The IMU covariance finding follows the official [Jazzy Imu message definition](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/sensor_msgs/msg/Imu.msg).
Frame recommendations follow [ROS REP 103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst).

## Purchased LiDAR update

The user identified a Quanergy M8-1 Rev B. See [the device-specific integration plan](../lidar-integration.md) for the Ethernet/driver research, compact UI arrangement and required label/cable evidence. The published vendor ROS wrapper is ROS 1; ROS 2 and Pi compatibility remain to be demonstrated. No power pinout or PoE support is assumed.
