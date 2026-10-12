# Known gaps and next steps

[Back to README](../README.md)

My main focus is getting the assembled boat ready for repeatable supervised tests.

## Physical build

- Tidy the installed electronics layout and wiring.
- Finish sealing the cable entries.
- Confirm motor voltage limits and add cable and connector specifications.
- Record the Pi setup and flashed Arduino version.
- Measure loaded mass, flotation, current, voltage sag, and temperature.
- Check stopping behavior and finish the physical disconnect arrangement.
- Test communication range and reconnects with the assembled boat.

The propulsion fuse is 12 A. I still need to add its position to the wiring diagram.

## Control and navigation

Manual control currently takes priority over failsafe stop levels, while the separate E-stop takes priority over both. I need to decide whether that behavior is right. Stale failsafe state also returns to nominal, which needs checking.

Mission handover, HOLD/resume, cancellation, delayed feedback, connection loss, and multiple dashboard viewers need more testing.

The heading behavior differs between hardware and simulation. I need outdoor GPS checks, IMU calibration, measured sensor positions, and water tests to understand how well it works.

## Recording

Fix the nested-bag search, read all recording files, handle GPS status properly, and check for missing streams. Keep the original capture intact when generating quality reports.

Recording and data collection have not started yet. After the engineering work, I want to complete one recording and replay test and measure timing before collecting a larger dataset.

## Fixes already in the code

The newer code handles several earlier problems, including invalid motor commands, partial serial messages, stale GPS, GPS jumps, and heading fallback. The regression results are in the [test log](09_test_log.md).

I still need to test those changes with the complete system running.

## Later goals

A full Kalman filter or EKF, active station keeping, obstacle avoidance, autonomous docking, and the annotation workflow are future work.

I also want command IDs and acknowledgements so it is easier to tell whether a request reached the right part of the system. That communication redesign has not been implemented yet.
