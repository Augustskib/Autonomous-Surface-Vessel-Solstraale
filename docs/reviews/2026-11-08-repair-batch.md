# Repair batch — 8 November 2026

These are local code changes following the whole-project review. Nothing has been
flashed, deployed, committed or pushed. Earlier UI/control edits are preserved.

## Changed behavior

| Area | Before | After |
| --- | --- | --- |
| Arduino command framing | Overflow could turn a trailing fragment into full thrust; trailing text accepted | Discard entire malformed frame through newline; exact bounded integer syntax; embedded NUL rejected; CRLF supported |
| Heading | Frozen gyro-backed estimate could remain usable for minutes | No gyro/reference updates for over one second makes heading unusable; missed integration requires a new reference |
| GPS speed gate | Invalid speed could reuse the previous valid speed | Invalid/negative speed invalidates the gate and cannot authorize COG correction |
| GPS input | One sentence per 200 ms tick could accumulate a queue | Nonblocking bounded drain; newest complete valid fresh GGA per tick; oversized backlog discarded |
| GPS time | Callback time hid old fixes | GGA UTC checked against synchronized host UTC; reject over two seconds old or over 0.5 seconds ahead; preserve nonnegative acquisition age in ROS stamp |
| Motor reconnect | Continuing commands could restart thrust when serial returned | Read/write/open failures latch stop; reconnection sends zero; operator must press STOP then RESET after connection returns |
| Bench launch | Required safety supervisor absent | Launch supervisor with geofence disabled for indoor bench; link monitoring and 15% driver limit retained |

Motor numeric values outside -100..100 are rejected by firmware, rather than
clamped. The Pi already clamps valid finite commands to this range. Malformed
frames do not refresh the firmware watchdog.

## How to read the changes

Start at firmware `readSerialCommands()` and `handleLine()`: framing is separated
from command application. Then read `HeadingEstimator.evaluate()` and the gap
handling in `on_gyro_rate()`. `GPSPublisher.read_gps()` contains the bounded drain,
checksum request, time gate and one-publication-per-tick behavior. Finally read
`MotorDriver._latch_link_fault()` and `estop_callback()` for restart gating.

## Verification

- Firmware host suite passed, including new malformed-frame and integer-boundary checks.
- 24 heading checks passed, including missing gyro, late first gyro and invalid speed.
- 22 hardware-free callback/regression checks passed, including nine transport checks.
- 31 standalone failsafe checks passed.
- Control unittest discovery executed 33 checks successfully but could not import
  three ament lint modules; the overall discovery command therefore failed.
- Changed Python files parse; whitespace/diff check completed.

These tests use fake serial/ROS I/O. They do not verify the Arduino toolchain,
physical pins, pyserial behavior on the Pi, real NMEA parsing, ROS launch/DDS,
or propulsion response. Full ROS build and hardware validation remain required.

## Remaining limitations and next work

- GPS time-of-day requires a synchronized Pi UTC clock. GGA has no date, so this
  cannot detect a replay delayed by exactly whole days. Test real receiver timing,
  checksum failures and clock offsets before outdoor navigation.
- Heading uses callback arrival times for gyro freshness. Acquisition timestamps,
  buffered IMU timing and covariance corrections remain separate work.
- Reconnect gating is local to the motor driver. Surface a stamped motor-link fault
  in the arbiter/UI, add controller acknowledgement and detect silent controller
  reboot/unplug cases that do not raise a serial exception. R05 remains partial.
- STOP/RESET gating prevents automatic reconnection restart but is not authenticated
  transport: ROS network access control remains open under R22.
- Bench supervisor wiring is source-reviewed, not launch-tested. Validate manual
  entry, heartbeat loss, supervisor exit and STOP/RESET with propulsion disconnected.
- Next batch: recorded bag discovery, GPS status parsing and missing-stream quality
  checks. LiDAR power/driver validation remains pending label and cable evidence.
