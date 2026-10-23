# Operator handover: Manual, Hold and Continue

Implemented on `fix/operator-handover`, based on `106f83d` (12 October 2026).
Validated offline; not deployed to the Pi or tested on live ROS/hardware.

## What the operator can expect

| Action | Propulsion/control | Existing mission |
| --- | --- | --- |
| Take Manual, or press WASD | Manual control after acknowledgement; releasing keys requests neutral | Paused at the same target |
| Start Mission | Hold first, accept route, then explicitly hand over to navigation | Starts a new route; refuses to replace an existing route |
| Hold | Zero commanded thrust, including during return-to-launch recovery | Retained |
| Continue Mission | Explicitly returns control to autonomy, subject to safety stops | Continues the remaining route |
| Cancel Mission | Hold first, then cancel | Removed |
| Engine Stop | Emergency stop requested; mission aborts | Aborted, not resumable |
| Reset to Hold | Clears emergency stop but leaves propulsion held | Does not restart anything |
| Lost operator heartbeat | Hold latches; restored heartbeats alone cannot resume | Retained |
| Leave/hide/blur the Pilot page | Requests Hold; watchdog is the fallback | Retained |

Hold is **zero thrust**, not station keeping. Wind and current can move the boat.
The motor bars show requested manual thrust, not measured motor output.
“Hide Route Drawing” only hides a map overlay; “Cancel Mission” cancels navigation.
Return Dock starts a new one-point route and therefore requires cancelling an
existing mission first. It does not silently discard the retained route.

## Why both bugs and logic needed fixing

The previous browser code sent thrust and control mode independently. Resume
could send RESUME before a previous asynchronous RELEASE finished; that late
RELEASE then put the boat back in Hold. Failures on manual heartbeats were
ignored, and the UI optimistically claimed control before the arbiter confirmed it.

The navigation loop could progress waypoints during manual control, and the
mission timeout kept running during detours. Hold also allowed recovery thrust.
Native browser dialogs blocked drive timers. These were inconsistent meanings
of pause, stop and ownership, not just cosmetic UI problems.

## Implementation

- `POST /operator` sends one `boat/operator_command` String JSON message containing
  `session`, increasing `seq`, `intent`, `transition`, `left` and `right`.
  `boat/operator_status` carries the exact request receipt and actual arbiter mode.
  The server returns success only after a matching arbiter acknowledgement.
- Sequence numbers reject older commands within a session. Browser revisions also
  prevent old responses from changing newer UI decisions. Explicit takeover retires
  the previous session so its outstanding requests cannot reclaim ownership.
- MANUAL and AUTO heartbeats renew authority every 200 ms, with one renewal in
  flight. Missing renewals latch Hold after the configured lease timeout (default
  1 s, detected on a 20 Hz steady-clock timer). Manual thrust has an additional
  0.5 s freshness check. These are software thresholds, not measured motor-stop times.
- A renewal cannot re-arm an expired lease. The operator must explicitly select
  Manual or Continue again. A new AUTO transition requires a fresh navigation sample.
- Waypoint progress and fly-by detection pause outside fresh IDLE/WAYPOINT mode.
  Manual/Hold time is excluded from the existing mission timeout. The remaining
  route and target index are kept in memory.
- Priority is e-stop, operator Hold, safety stop, manual, recovery, waypoint, zero.
  Manual can override return-to-launch steering but cannot override a safety stop.
  An established safety-monitor stream going stale now stops instead of failing open.
- The Pilot page uses inline notices. Its errors and safety override reason input
  no longer open blocking browser dialogs. Manual controls remain available while
  route requests are pending; duplicate route submissions are suppressed.

## Compatibility and limits

Update `boat_control`, `boat_navigation` and `boat_web_ui` together, rebuild, restart
their nodes and reload the dashboard. The JS file is included as package data.
An old web server cannot use the new protocol, and a new web server will report
missing acknowledgements against an old arbiter.

Legacy `boat/cmd/manual` and `boat/cmd/control_mode` inputs still work before a
dashboard session acquires authority. They are rejected afterward so old clients
cannot override that owner. Existing command-line startup remains AUTO for
compatibility: arbiter restart and retained publishers need a separate live
restart test before deployment. This change does not provide persistent ownership
across process restarts, authentication, hardware emergency isolation, or mission
IDs for correlating arbitrary delayed route feedback. Mission acceptance timeout
is reported as uncertain; stay held and inspect actual mission state before retrying.

The dashboard link-loss policy is now Hold, including during AUTO. That deliberately
blocks the recovery controller from moving the boat after an operator link loss.
Autonomous return on link loss would need a separate, explicit policy.

## Offline verification

77 checks passed on 22 October 2026: 16 existing control checks, 31 failsafe
checks (including the intentionally revised Hold/stale-supervisor expectations),
11 handover/navigation checks, 5 endpoint/adapter checks and 14 JavaScript checks.
All changed Python files also passed syntax parsing; `git diff --check` passed.

From the repository root, with Python 3 and Node available:

```sh
python3 src/boat_control/test/test_control_logic.py
python3 src/boat_control/test/test_failsafe_logic.py
python3 src/boat_control/test/test_operator_handover.py
python3 src/boat_web_ui/test/test_operator_endpoint.py
node --test src/boat_web_ui/test/operator_controls.test.cjs src/boat_web_ui/test/pilot_page.test.cjs
```

Set `PYTHON` if the page test needs a different Python executable. These tests
cover pure arbitration, pause timing, actual follower methods with transport
stubbed, endpoint/adapter acknowledgement handling with in-memory transport,
and actual composed page JavaScript with a fake DOM, map and network.
They do not test DDS delivery, Flask HTTP serving, browser rendering, the installed
ROS package, Wi-Fi latency or motor firmware.

## Next verification before propulsion

Run the updated control/mission/navigation/web nodes against simulated GPS and
heading, without motor_driver or any hardware connection. Observe actual
`boat/thrust_command`, `boat/mode`, `boat/state` and `boat/nav_status` through the
complete Manual → Start Mission → Manual → Hold → Continue → Cancel sequence.
Verify that the target index does not advance during a manual detour, and that
late/reordered packets, closing the tab, Wi-Fi interruption, node restarts,
e-stop/reset and stale sensor feeds produce the stated behavior.
Then check the motor controller's independent watchdog and stop path on the bench.
Offline tests passing is not water-test clearance.
