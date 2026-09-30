#!/usr/bin/env bash
#
# verify_failsafe_and_heading.sh: step-by-step check of the failsafe,
# geofence, heading estimator and recording status on a running sim.
# Run verify_control_loop.sh first. Each step prints EXPECT, asks you to
# act where needed, then prints what it OBSERVED.
#
# Usage (from the repo root):
#   colcon build --symlink-install
#   source install/setup.bash
#   ros2 launch boat_simulation boat_simulation.launch.py   # terminal 1
#   bash scripts/sim/verify_failsafe_and_heading.sh       # terminal 2
#
# Keep http://localhost:5000 open. The link-loss steps close it.

set -u

BOLD=$'\033[1m'; DIM=$'\033[2m'; GRN=$'\033[32m'; RED=$'\033[31m'
YEL=$'\033[33m'; RST=$'\033[0m'

SAMPLE_SEC="${SAMPLE_SEC:-3}"

step()   { printf '\n%s=== STEP %s ===%s\n' "$BOLD" "$*" "$RST"; }
expect() { printf '%sEXPECT :%s %s\n' "$GRN" "$RST" "$*"; }
note()   { printf '%s        %s%s\n' "$DIM" "$*" "$RST"; }
act()    { printf '%sDO     :%s %s\n' "$YEL" "$RST" "$*"; }
warn()   { printf '%sWARNING:%s %s\n' "$RED" "$RST" "$*"; }
pause()  { printf '\n%s[press Enter when done]%s ' "$DIM" "$RST"; read -r _; }

sample() {
  local topic="$1" field="${2:-data}" label="${3:-$1}"
  local out
  out=$(timeout "$SAMPLE_SEC" ros2 topic echo "$topic" --field "$field" 2>/dev/null \
        | grep -v '^---$' | sed '/^$/d' | sort | uniq -c | sort -rn | head -5)
  if [ -z "$out" ]; then
    printf '%sOBSERVED:%s %s -> %sNOTHING PUBLISHED in %ss%s\n' \
      "$BOLD" "$RST" "$label" "$RED" "$SAMPLE_SEC" "$RST"
  else
    printf '%sOBSERVED:%s %s ->\n' "$BOLD" "$RST" "$label"
    printf '%s\n' "$out" | sed 's/^/            /'
  fi
}

# Print one whole message.
show() {
  local topic="$1" label="${2:-$1}"
  local out
  out=$(timeout "$SAMPLE_SEC" ros2 topic echo "$topic" --once 2>/dev/null)
  if [ -z "$out" ]; then
    printf '%sOBSERVED:%s %s -> %sNOTHING PUBLISHED in %ss%s\n' \
      "$BOLD" "$RST" "$label" "$RED" "$SAMPLE_SEC" "$RST"
  else
    printf '%sOBSERVED:%s %s ->\n' "$BOLD" "$RST" "$label"
    printf '%s\n' "$out" | sed 's/^/            /'
  fi
}

cat <<'BANNER'
=====================================================================
 ASV failsafe + heading verification
 Covers: the geofence being level-triggered, the graded response,
         explicit degradation without a heading, e-stop still winning,
         runtime fence configuration, heading quality transitions,
         honest recording status, and camera mock reporting.
=====================================================================
BANNER

# ---------------------------------------------------------------------
step "0  Preflight: did the rebuild actually happen?"
# ---------------------------------------------------------------------
expect "boat_msgs contains HeadingEstimate, FailsafeState and CameraStatus"
note "these are NEW message types. If they are missing, colcon did not"
note "rebuild boat_msgs and everything below will fail confusingly."
for m in HeadingEstimate FailsafeState CameraStatus SafetyStatus; do
  if ros2 interface show "boat_msgs/msg/$m" >/dev/null 2>&1; then
    printf '            %s%-20s present%s\n' "$GRN" "$m" "$RST"
  else
    printf '            %s%-20s MISSING -- colcon build --symlink-install, then re-source%s\n' \
      "$RED" "$m" "$RST"
  fi
done
echo
expect "SafetyStatus has the new fence_warn_radius_m / fence_outside_sec fields"
note "if it does not, boat_msgs was rebuilt from a stale copy -- check the"
note "workspace was actually updated before building"
printf '%sOBSERVED:%s ' "$BOLD" "$RST"
ros2 interface show boat_msgs/msg/SafetyStatus 2>/dev/null \
  | grep -E 'fence_warn_radius_m|fence_outside_sec|fence_disabled_reason' \
  | sed 's/^/            /' || printf '%s(none found)%s\n' "$RED" "$RST"
echo
expect "heading_estimator and system_monitor are running"
printf '%sOBSERVED:%s running nodes ->\n' "$BOLD" "$RST"
ros2 node list 2>/dev/null | sed 's/^/            /'
echo
expect "the new topics exist"
for t in /boat/heading /boat/failsafe_state /boat/cmd/failsafe \
         /boat/geofence_command /boat/failsafe_clear /boat/camera/status \
         /boat/system_status; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    printf '            %s%-28s present%s\n' "$GRN" "$t" "$RST"
  else
    printf '            %s%-28s MISSING%s\n' "$RED" "$t" "$RST"
  fi
done
pause

# ---------------------------------------------------------------------
step "1  Heading: does it acquire, and does it say how good it is?"
# ---------------------------------------------------------------------
note "THE KEY IDEA: boat/heading is not a bare number. quality is"
note "  0 = NONE           the boat does not know which way it points"
note "  1 = DEAD_RECKONING gyro only, drifting"
note "  2 = FUSED          corrected by course over ground"
echo
expect "sitting still at startup: quality 0 or 1, and a detail string saying why"
note "in SIMULATION this may go straight to 2, because the sim launch file"
note "passes imu_orientation_is_absolute:=true (Gazebo's IMU really is"
note "absolute). On the REAL boat that parameter is false and you should"
note "see 0 until the boat is moving."
show /boat/heading "boat/heading (at rest)"
pause

act "drive the boat forward above 0.5 m/s (hold W on the dashboard for ~15s)"
pause
expect "quality 2 (FUSED), source COG, cog_valid true, ground_speed_mps > 0.5"
show /boat/heading "boat/heading (under way)"
echo
expect "gyro_bias_dps is small and settled, not wandering"
note "if it is at the +/-5.0 cap, the bias learner is absorbing something"
note "that is not bias -- check the IMU is really still when it thinks it is"
sample /boat/heading gyro_bias_dps "boat/heading gyro_bias_dps"
pause

act "now STOP the boat and let it sit for 30 seconds"
pause
expect "quality falls 2 -> 1 within ~5s, and detail explains the speed threshold"
note "this is the degradation that matters: below the COG speed threshold"
note "the estimate is coasting on the gyro and is drifting. It should say so."
show /boat/heading "boat/heading (stopped)"
echo
expect "the dashboard Vessel box shows 'DEAD RECKONING' in amber, not a bare number"
pause

# ---------------------------------------------------------------------
step "2  Failsafe baseline: NOMINAL with everything healthy"
# ---------------------------------------------------------------------
expect "boat/failsafe_state level 0, level_name NOMINAL, latched false"
expect "position_valid true, heading_usable true (if you are moving)"
show /boat/failsafe_state "boat/failsafe_state"
echo
expect "the banner at the top of BOTH pages is green and says NOMINAL"
note "check /data too -- the banner and nav bar are on both pages now"
pause

# ---------------------------------------------------------------------
step "3  Set the geofence from the dashboard (the field-readiness fix)"
# ---------------------------------------------------------------------
note "THE DEFAULT FENCE CENTRE IS LAKE EOLA, FLORIDA. At any real site the"
note "fence is breached the instant a fix arrives. Being able to fix that"
note "from the dashboard instead of an SSH session is the point of F4."
act "on the Pilot page, set Radius to 40, then click 'Set Fence Here'"
pause
expect "SafetyStatus fence_radius_m 40, fence_warn_radius_m 36 (0.9 x 40)"
note "fence_warn_radius_m is NEW. The map used to draw the warning ring at"
note "a hardcoded radius*0.9 regardless of the actual warn_fraction -- so"
note "the ring you saw was a lie if you ever changed that parameter."
show /boat/safety_status "boat/safety_status"
echo
expect "TWO dashed circles on the chart, centred on the boat: red at 40m, amber at 36m"
expect "a geofence_set entry appears on boat/data_events"
sample /boat/data_events data "boat/data_events"
pause

expect "changing the warn fraction moves the AMBER ring, not just the number"
act "set 'Warn ring (frac)' to 0.5 and click 'Set Fence Here' again"
pause
note "the amber ring should now be at 20m, i.e. visibly halfway in."
note "if it stays at 36m you are running a stale web_server."
show /boat/safety_status "boat/safety_status"
pause

# ---------------------------------------------------------------------
step "4  THE BIG ONE: breach must keep asserting, and must drive home"
# ---------------------------------------------------------------------
warn "Do this in SIMULATION first. Do not learn this behaviour on water."
note "Set the fence small (step 3) so you can leave it deliberately."
act "click 'Set Launch Here', then drive the boat OUT past the red ring and KEEP GOING"
pause
expect "fence_warning true as you cross the amber ring, before any failsafe"
expect "then failsafe level 2 RETURN_TO_LAUNCH, reason GEOFENCE_BREACH"
expect "boat/mode becomes FAILSAFE"
expect "boat/cmd/failsafe carries non-zero thrust"
expect "the boat TURNS AROUND AND COMES BACK -- this is the whole point"
show /boat/failsafe_state "boat/failsafe_state"
sample /boat/mode data "boat/mode"
sample /boat/cmd/failsafe left "boat/cmd/failsafe (left)"
echo
expect "fence_outside_sec KEEPS COUNTING UP while outside"
note "THIS IS THE REGRESSION CHECK. On the old build the breach was"
note "edge-triggered: it fired an e-stop once, cut all thrust, and the"
note "boat drifted away on the same wind that pushed it out, with the"
note "fence unable to ever fire again. If this number is stuck at 0 or"
note "the boat sits still and drifts, you are running the old build."
sample /boat/safety_status fence_outside_sec "fence_outside_sec"
echo
expect "boat/estop is FALSE throughout -- breach no longer asserts an e-stop"
note "that is deliberate: an e-stop removes the boat's ability to come"
note "back, which is exactly what it needs at that moment"
sample /boat/estop data "boat/estop"
pause

expect "the mission (if one was running) went to ABORTED, not IDLE"
note "safety_monitor publishes MISSION_ABORTED FAILSAFE_<reason> so the"
note "waypoint follower stops fighting the failsafe for the boat"
sample /boat/state data "boat/state"
pause

# ---------------------------------------------------------------------
step "5  Recovery and the latch"
# ---------------------------------------------------------------------
expect "once back inside, the failsafe STAYS at RETURN_TO_LAUNCH"
expect "reason becomes LATCHED and active_reasons explains the latch"
note "escalation is automatic; coming back down is a human decision. A"
note "boat blown out of its fence and driven back in is sitting where the"
note "wind is still trying to push it out."
show /boat/failsafe_state "boat/failsafe_state (back inside)"
echo
act "press 'Clear Failsafe' in the banner"
pause
expect "level returns to 0 NOMINAL and boat/mode leaves FAILSAFE"
show /boat/failsafe_state "boat/failsafe_state (after clear)"
echo
expect "clearing WHILE STILL OUTSIDE is refused with a named reason"
act "drive back out past the fence, then press 'Clear Failsafe'"
pause
note "you should get an error dialog naming the live condition, NOT a"
note "silent no-op. An operator must never believe they cleared a"
note "failsafe that is still holding."
act "now press 'Override', type a reason, and confirm it clears"
pause
expect "a failsafe_force_cleared entry with your reason on boat/data_events"
sample /boat/data_events data "boat/data_events"
pause

# ---------------------------------------------------------------------
step "6  Degradation must be VISIBLE, not silent"
# ---------------------------------------------------------------------
note "Levels 1 and 2 need a usable heading. Without one, a"
note "return-to-launch controller drives confidently PAST home and keeps"
note "going, which is worse than drifting. So it must fall back to a"
note "powered stop AND SAY WHY."
act "stop heading_estimator:  ros2 lifecycle/kill it, or in its terminal Ctrl-C"
note "(simplest: run the stack, then kill the heading_estimator process)"
pause
expect "failsafe_state heading_usable false within ~1s"
act "now drive out past the fence again"
pause
expect "level 3 POWERED_STOP, requested_level 2, and a NON-EMPTY degraded_reason"
expect "the banner reads POWERED STOP and names the heading as the cause"
show /boat/failsafe_state "boat/failsafe_state (no heading)"
note "if degraded_reason is empty here, the degradation is silent and that"
note "is a bug -- the whole design rests on this field being populated."
pause

# ---------------------------------------------------------------------
step "7  E-STOP MUST STILL WIN. This is the regression that matters most."
# ---------------------------------------------------------------------
act "restart heading_estimator, drive out past the fence so RTL engages"
pause
expect "boat/mode FAILSAFE, boat driving home"
sample /boat/mode data "boat/mode"
act "now press ENGINE STOP on the dashboard"
pause
expect "boat/mode becomes ESTOP immediately and thrust is 0/0"
expect "failsafe_state level 4 HARD_STOP, reason ESTOP"
note "e-stop outranks the failsafe, the manual lease, and everything else."
sample /boat/mode data "boat/mode"
show /boat/failsafe_state "boat/failsafe_state (e-stopped)"
sample /boat/thrust_command left "boat/thrust_command (left)"
echo
act "press RESET / IDLE to clear the e-stop"
pause
expect "the failsafe level returns to what it was BEFORE the e-stop (2 or 3),"
expect "NOT stuck at 4"
note "e-stop is an output override, not a state the failsafe enters --"
note "otherwise clearing it would need a second, separate clear and the"
note "boat would look broken."
show /boat/failsafe_state "boat/failsafe_state (e-stop cleared)"
pause

# ---------------------------------------------------------------------
step "8  A HUMAN OUTRANKS THE FAILSAFE"
# ---------------------------------------------------------------------
act "with the failsafe driving home (RTL active), press and hold W"
pause
expect "boat/mode becomes MANUAL and YOUR keys drive the boat"
note "someone who has grabbed the controls to drive back inside the fence"
note "must not have to fight a return-to-launch controller for the throttle"
sample /boat/mode data "boat/mode"
act "release the keys and wait ~6 seconds for the lease to lapse"
pause
expect "boat/mode returns to FAILSAFE (not HOLD) and the boat resumes coming home"
note "HOLD withholds the MISSION, not the boat's ability to save itself"
sample /boat/mode data "boat/mode"
pause

# ---------------------------------------------------------------------
step "9  Disabling the fence must be deliberate and loud"
# ---------------------------------------------------------------------
act "click 'Disable Fence' and type a reason"
pause
expect "an amber banner: GEOFENCE DISABLED, with your reason"
expect "both rings disappear from the chart"
expect "SafetyStatus fence_enabled false, fence_disabled_reason = your text"
expect "a geofence_disabled entry on boat/data_events"
note "a silently-disabled geofence is worse than none, because you will"
note "believe you have one"
show /boat/safety_status "boat/safety_status (fence off)"
act "click 'Enable Fence' again before continuing"
pause

# ---------------------------------------------------------------------
step "10 Camera honesty"
# ---------------------------------------------------------------------
expect "boat/camera/status exists and reports the real source"
note "source: 0 UNKNOWN, 1 REAL CAMERA, 2 MOCK TEST PATTERN"
note "IN SIMULATION there is no rpicam-vid, so camera_publisher is not"
note "launched at all -- frames come from the Gazebo bridge. Expect this"
note "topic to be absent in sim and present on the Pi."
show /boat/camera/status "boat/camera/status"
echo
expect "ON THE PI: source 1, mock_mode false, healthy true, measured_fps ~10"
expect "the /data page Capture Integrity box is NOT red"
note "TO TEST THE MOCK PATH DELIBERATELY (do this once, on the Pi):"
note "  ros2 run boat_perception camera_publisher --ros-args -p mock:=true"
note "the /data page should turn red and say the video is a test pattern."
note "That red box is the thing standing between you and an afternoon of"
note "recorded test card."
pause

# ---------------------------------------------------------------------
step "11 Recording status must mean 'bytes reaching disk'"
# ---------------------------------------------------------------------
act "start data_recorder:  ros2 run boat_data data_recorder"
act "then press 'Start Data Run' on the /data page"
pause
expect "Recording YES *and* 'Writing To Disk' YES, Bag Size climbing"
expect "Write Rate non-zero, Time Remaining shows '(measured)'"
show /boat/recording_status "boat/recording_status"
echo
expect "NOW THE INTERESTING TEST: make it record topics that do not exist"
act "stop the run. Restart the recorder with a bogus topic list:"
note "  ros2 run boat_data data_recorder --ros-args -p topics:='[/nope/nothing]'"
act "start a run from the dashboard and watch for ~10 seconds"
pause
expect "Recording YES but 'Writing To Disk' NO, with a stall count"
expect "the /data page goes RED with 'RECORDING BUT NOT WRITING'"
expect "an ERROR line in the recorder log saying the bag has not grown"
note "THIS IS THE OLD LIE. Before this change the dashboard would have"
note "shown a happy green REC for the whole run and you would have found"
note "out when you opened the bag."
show /boat/recording_status "boat/recording_status (bogus topics)"
pause

expect "the recorded topic set now makes thrust attributable"
act "stop the bogus run, restart the recorder normally, record ~20s, stop"
note "then check the bag contains the command provenance topics:"
note "  ros2 bag info <run_dir>/bag/rosbag2"
expect "/boat/mode, /boat/cmd/manual, /boat/cmd/waypoint, /boat/cmd/failsafe present"
note "without these, a thrust value in the bag has no attributable source"
note "and cannot support imitation learning -- which is the use this"
note "recorder's own docstring claims."
pause

# ---------------------------------------------------------------------
step "12 Battery and host telemetry"
# ---------------------------------------------------------------------
note "IN SIMULATION power_monitor is not launched, so expect NO MONITOR."
note "On the Pi it should show per-channel V/A/W."
expect "the Pilot page Power & Host box shows CPU temp and free storage"
expect "a channel whose I2C read FAILED is greyed with 'read fail',"
expect "NOT shown as a healthy 0.0 A"
show /boat/system_status "boat/system_status"
show /boat/power_status "boat/power_status"
echo
expect "on the Pi: CPU temp is a real number and rises under load"
note "a Pi in a sealed enclosure in sun throttles, and a throttled Pi"
note "drops camera frames -- which you would otherwise misdiagnose as a"
note "camera fault. Watch 'Throttling' for under-voltage flags too: the"
note "Pi shares a pack with the motors, so acceleration can brown it out."
pause

# ---------------------------------------------------------------------
step "13 Navigation bar"
# ---------------------------------------------------------------------
expect "both pages have the same nav bar at the very top"
expect "the current page is marked by an amber underline AND a background,"
expect "not by colour alone"
expect "one click switches between Pilot and Data Collection"
expect "the right-hand summary shows REC / CAMERA: MOCK / temp / free GB"
pause

cat <<'DONE'

=====================================================================
 Done. Report what you actually saw, especially anything that did not
 match.

 The three things most worth reporting, in order:

   1. Step 4 -- did the boat actually TURN AROUND AND COME BACK, or did
      it stop and drift? That is the entire point of this change and it
      is the one thing no test here can prove.
   2. Step 7 -- did e-stop still cut everything instantly, and did
      clearing it restore the previous situation rather than leaving
      the boat stuck?
   3. Step 6 -- was degraded_reason populated, in words, when the
      heading was unavailable? A silent degradation is the failure mode
      this whole design is built to prevent.

 Also worth noting: the heading filter's constants (COG speed
 threshold, alpha, assumed drift rate) have NEVER been measured against
 real hardware. They are reasoned defaults. Step 1 is where you start
 finding out what they should actually be.
=====================================================================
DONE
