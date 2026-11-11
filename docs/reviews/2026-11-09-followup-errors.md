# Whole-project review follow-up — 2026-11-09

This pass inspected navigation, simulation propulsion, perception, hardware/system diagnostics, hardware/simulation launches, recorder lifecycle and commands, offline exports, and existing network/video/cleanup findings. Source review and offline tests are not a complete ROS or hardware acceptance test. Prior local work was preserved.

## Confirmed defects fixed

| ID | Priority | Trigger and previous result | Change and evidence |
|---|---|---|---|
| F01 | P1 | NaN steering gain could be converted by Python min/max clamping into a saturated turn; infinite waypoint radius could mark any destination reached | Navigation validates finite positive radius, speed 0–100 and finite nonnegative gain before waypoint progress or thrust arithmetic. Invalid configuration sends zero and preserves progress. Parameterized regression tests exercise these cases. |
| F02 | P2 | Negative or nonfinite simulator force scale could reverse direction or propagate invalid targets | Splitter explicitly zeros targets/current output when scale is invalid. Three invalid-value tests. |
| F03 | P2 | Dataset instructions supply index.sqlite, but exports treated it as a directory | Both export paths now accept the index directory or its index.sqlite file and validate required index artifacts. Real MCAP export test. |
| F04 | P1 for dataset integrity | Indexing with LiDAR optional still stored nearest LiDAR associations; 3D export checked only their existence, allowing unchecked timing | 3D export now requires an index whose required streams include LiDAR. Optional receive-time LiDAR cannot be promoted to synchronized 3D export. Regression test uses real MCAP. |
| F05 | P2 | Optional LiDAR clock warnings failed an otherwise valid camera-only collection profile | Overall quality/export reversal gates now apply to required streams. Diagnostics for optional streams remain visible. |
| F06 | P2 | System monitor retained the old 7 GB/hour estimate, independently of recorder fixes | Removed invented duration; system monitor reports capacity and fallback-path identity. Recording-duration estimates belong to the measured recorder rate. Regression test. |
| F07 | P2 | Model inference exceptions or empty results escaped the detector callback | Inference failures are logged and empty predictions ignored without killing the node. No fabricated detection output. Fake inference-failure test; real model performance still untested. |
| F08 | P2 | Beginning the same segment ID twice overwrote the open-segment entry, leaving the first manifest segment unclosed | Reject duplicate segment IDs for the run, preserving the original. Regression test. |
| F09 | P2 | A JSON list/object used as recorder action caused dictionary lookup TypeError before the command error handler | Reject non-string actions before dispatch. Regression test. |

F03–F05 were gaps in the newly added integration code and are explicitly included here, rather than attributed to the older project.

## Significant findings still open

- Network access remains sufficient to reach control interfaces; operator ordering IDs are not authentication. Web server and Foxglove need a defined network/access policy before remote operation. No arbitrary password or network topology was imposed in this pass.
- The simulation cleanup script still uses broad process-name SIGKILL matching, which can kill unrelated stacks. Replace it with managed launch ownership/process groups and test Linux shutdown/restart. It was not executed here.
- Simulation sensor/physics timing and wall-clock watchdog timing are not consistently separated. Simply setting use_sim_time everywhere can freeze safety timers on pause, so this needs a deliberate integration test rather than a blanket edit.
- MJPEG handlers still encode cached frames repeatedly per client and do not provide a shared bounded encoder. Camera overlays/health need independent freshness enforcement. Pi load has not been measured.
- Hardware bring-up still has simulation-site geofence defaults, unverified INA226 addresses/shunts and incomplete measured TF. These remain commissioning requirements, not corrected by the new LiDAR launcher.
- Detector inference exceptions now survive, but runtime detection health is not yet exposed as a dedicated status stream. Camera calibration/optical-frame and dock-pose validity remain unverified; nominal calibration must not be treated as measured docking geometry.
- The firmware watchdog remains cooperative, not a proven independent hardware stop. Arduino acquisition timestamps, physical stop latency, calibrated current and motor-link fault propagation still need work.
- ROS 2 compilation, DDS/QoS, complete launch profiles, EKF execution and actual LiDAR SDK operation remain untested on this Windows host.

## Validation

182 Python tests passed after the main changes, including 12 additional regression cases. Existing warning: test_full_operator_loop returns a list rather than None. The prior 19 JavaScript tests remain the latest browser-script result; no browser code changed in this pass. A final targeted dataset rerun covers the required-stream reversal-gate adjustment. These tests use fake ROS/transport where necessary and real synthetic MCAP files; they do not claim physical sensor or control-loop validation.

Nothing was pushed, flashed or deployed. See the integration runbook for hardware preparation and the earlier whole-project review for the full backlog.
