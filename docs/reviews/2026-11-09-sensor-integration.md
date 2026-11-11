# Sensor integration implementation review — 2026-11-09

This supplements the November 7 whole-project review and November 8 repair batch. It does not mark all prior findings closed. Existing local UI/control changes were preserved. No hardware deployment, firmware flashing or remote push was performed.

## Changes and reasons

| Area | Change | Remaining boundary |
|---|---|---|
| Quanergy | Optional ROS 2 C++ adapter around SDK 5, raw cloud publication and explicit clock mode | Not compiled here; sensor power/network/calibration and SDK compatibility need target verification |
| Preview/UI | Bounded cloud sampling, independent freshness status and LiDAR badge | Full 3D visualization remains in Foxglove; no avoidance |
| GPS/IMU fusion | Fixed-datum ENU conversion, rejected stale/duplicate/invalid inputs, measured-transform gate, experimental robot_localization configuration | No validated absolute yaw or control authority; real ROS filter not exercised |
| IMU faults | Removed implicit mock fallback on hardware/dependency failure | Explicit mock remains available for intentional testing |
| IMU covariance | Unknown measurements no longer advertised as unavailable | Noise and mounting calibration still required |
| Motor telemetry | Arduino status exposed with receipt time and explicit ADC-count units | Not force measurement or calibrated current; no acquisition timestamp |
| Recording | Split MCAP, sensor QoS profile, disk-reserve stop, calibration provenance, forced-shutdown flag | Need real disk-pressure/finalization tests on Pi |
| Recorder UI/API | Queued commands distinguished from confirmed results; disconnected ROS returns failure; stale status shown as unknown | Transport acceptance remains separate from recorder completion |
| Dataset | Streaming MCAP decode with SQLite metadata, five-stream associations and quality reports | Association windows are provisional; physical sensor timing not proven |
| Export | PNG/provenance and paired PCD/PNG export; source checksums checked | No annotations, camera-LiDAR calibration, projection or deskew generated |
| Archive | Finalized-run requirement, checksum-verified staging copy and transfer receipt | User must retain redundant verified storage; no automatic upload/deletion |
| Battery display | Default cell count now matches reported 3S pack | Voltage-based reserve is approximate, not a return-distance guarantee |

## Newly confirmed issues addressed

- A hardware IMU failure could silently switch to synthetic data. This could contaminate both a dataset and a future filter. Hardware mode now fails visibly.
- A successful web command response could be mistaken for recording actually starting. Responses now explicitly say queued/unconfirmed; stale recorder state no longer appears as current OFF/RECORDING state.
- Incoming run-site text could create unintended directory components. The recorder now sanitizes it to a bounded name.
- Invalid X/Y gyro values could reach the filter despite a finite Z value, then contaminate rotated measurements. The fusion gate checks all three components.
- Camera-only storage estimates were too optimistic for raw images. The runbook and budget tool use measured aggregate rates and explain the 30–60 minute constraint on a shared 32 GB card.

## Validation

170 Python tests passed, including real synthetic MCAP serialization/decoding, split segments, missing sensors, timestamp reversal, future-command rejection, image export, paired PCD export, source-tampering rejection, verified archive copying, fusion gates and operator endpoints. One existing test returns a list instead of None and produces a pytest warning; it is not a new functional failure.

19 JavaScript tests passed, including execution of composed Pilot-page scripts with fake DOM/map/network. Initial invocation selected the Windows Python launcher and failed before page tests; setting PYTHON to the project's virtual environment resolved the runner issue. This is not a visual browser rendering test.

All repository Python sources parsed; package XML and YAML parsed. Example capacity calculation: 24 GB free, 2 GB reserve, measured 10 MB/s and 25% margin gives approximately 29.3 minutes, so a 60-minute request fails the estimate.

Not tested here: ROS 2/DDS behavior, robot_localization execution, C++ SDK adapter compilation, physical sensor operation, real disk exhaustion, Pi sustained throughput or camera/LiDAR calibration. No claim of hardware readiness or complete project-wide defect closure is made.

## Next acceptance priorities

1. Build/launch the optional adapter and experimental filter on the intended Linux ROS 2 installation.
2. Measure actual onboard storage capacity/rate and choose storage or explicit recording reductions for 30–60 minutes.
3. Verify M8 revision/manual/power/network and sensor clock before powering and collecting.
4. Measure transforms/noise, characterize camera and IMU timing, and replay stationary/moving captures.
5. Continue remaining findings from the whole-project review: network access protection, reconnect/fault propagation, camera workload and end-to-end mission/safety acceptance.

See [the integration runbook](../sensor-integration-runbook.md) for the diagram, launch commands, offline workflow and photo/measurement checklist.
