#!/usr/bin/env bash
#
# verify_control_loop.sh: step-by-step check of the control loop on a
# running sim. The logic tests cover the decisions; this covers topic names,
# launched nodes and QoS. Each step prints EXPECT, asks you to act in the
# browser where needed, then prints what it OBSERVED.
#
# Usage (from the repo root):
#   source install/setup.bash
#   ros2 launch boat_simulation boat_simulation.launch.py   # terminal 1
#   bash scripts/sim/verify_control_loop.sh               # terminal 2
#
# Keep http://localhost:5000 open in a browser throughout.

set -u

BOLD=$'\033[1m'; DIM=$'\033[2m'; GRN=$'\033[32m'; RED=$'\033[31m'
YEL=$'\033[33m'; RST=$'\033[0m'

SAMPLE_SEC="${SAMPLE_SEC:-3}"

step()   { printf '\n%s=== STEP %s ===%s\n' "$BOLD" "$*" "$RST"; }
expect() { printf '%sEXPECT :%s %s\n' "$GRN" "$RST" "$*"; }
note()   { printf '%s        %s%s\n' "$DIM" "$*" "$RST"; }
act()    { printf '%sDO     :%s %s\n' "$YEL" "$RST" "$*"; }
pause()  { printf '\n%s[press Enter when done]%s ' "$DIM" "$RST"; read -r _; }

# Sample a topic field for SAMPLE_SEC and print the distinct values seen.
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

sample_thrust() {
  local out
  out=$(timeout "$SAMPLE_SEC" ros2 topic echo /boat/thrust_command 2>/dev/null \
        | grep -E '^(left|right):' | paste - - | sort | uniq -c | sort -rn | head -5)
  if [ -z "$out" ]; then
    printf '%sOBSERVED:%s /boat/thrust_command -> %sNOTHING PUBLISHED%s\n' \
      "$BOLD" "$RST" "$RED" "$RST"
  else
    printf '%sOBSERVED:%s /boat/thrust_command ->\n' "$BOLD" "$RST"
    printf '%s\n' "$out" | sed 's/^/            /'
  fi
}

cat <<'BANNER'
=====================================================================
 ASV control-loop verification
 Target loop: launch -> ready -> upload route -> boat drives it ->
              take manual -> release -> explicit resume ->
              upload a second route, with no restarts.
=====================================================================
BANNER

# ---------------------------------------------------------------------
step "0  Preflight: is the stack actually up?"
# ---------------------------------------------------------------------
expect "control_arbiter, mission_manager, waypoint_follower, web_server all listed"
printf '%sOBSERVED:%s running nodes ->\n' "$BOLD" "$RST"
ros2 node list 2>/dev/null | sed 's/^/            /'
echo
expect "/boat/mode, /boat/cmd/control_mode, /boat/mission_active, /boat/mission_cancel present"
note "these four are new in this change -- if any is missing, the build is stale"
printf '%sOBSERVED:%s new topics ->\n' "$BOLD" "$RST"
for t in /boat/mode /boat/cmd/control_mode /boat/mission_active /boat/mission_cancel; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    printf '            %s%-28s present%s\n' "$GRN" "$t" "$RST"
  else
    printf '            %s%-28s MISSING -- rebuild (colcon build) and re-source%s\n' "$RED" "$t" "$RST"
  fi
done
pause

# ---------------------------------------------------------------------
step "1  Idle baseline, WITH the dashboard tab open"
# ---------------------------------------------------------------------
act "make sure http://localhost:5000 is open. Do not touch WASD."
expect "/boat/mode is IDLE"
note "THIS IS THE PRE-FIX REGRESSION CHECK. On the OLD code this reads"
note "MANUAL, because the browser published a neutral 0/0 thrust command"
note "every 200ms and the arbiter counted that as 'the operator is"
note "driving'. If you see MANUAL here, you are running the old build."
sample /boat/mode data "/boat/mode"
sample /boat/state data "/boat/state"
sample_thrust
pause

# ---------------------------------------------------------------------
step "2  Upload route 1 and confirm the boat actually drives"
# ---------------------------------------------------------------------
act "click 3 points on the dashboard map, then press Send Mission"
note "or from the CLI, roughly 40m north of the Lake Eola spawn point:"
note "  ros2 topic pub --once /boat/mission_command boat_msgs/msg/MissionCommand \\"
note "    '{latitudes: [28.543971], longitudes: [-81.372778]}'"
pause
expect "/boat/state EXECUTE   AND   /boat/mode WAYPOINT   AND non-zero thrust"
note "EXECUTE with mode MANUAL or HOLD is the original bug: the mission is"
note "loaded but something else owns the throttle and the boat will sit still."
sample /boat/state data "/boat/state"
sample /boat/mode data "/boat/mode"
sample_thrust
echo
expect "GPS position changing -- the boat is physically moving"
note "watch latitude change across these samples; if it is frozen the"
note "thrust is being published but Gazebo is not applying it"
for i in 1 2 3; do
  printf '            fix %d: ' "$i"
  timeout 2 ros2 topic echo /boat/gps/fix --field latitude 2>/dev/null | head -1
  sleep 1
done
echo
expect "waypoint follower is publishing its own commands"
sample /boat/cmd/waypoint left "/boat/cmd/waypoint (left)"
pause

# ---------------------------------------------------------------------
step "3  Take manual control"
# ---------------------------------------------------------------------
act "press and HOLD W on the dashboard (or click 'Take Manual')"
expect "/boat/mode switches to MANUAL, thrust follows your keys"
note "the lease is acquired by the keypress itself -- you should not need"
note "to click anything first"
sample /boat/mode data "/boat/mode"
sample_thrust
pause

# ---------------------------------------------------------------------
step "4  Release -- and confirm autonomy does NOT silently resume"
# ---------------------------------------------------------------------
act "click 'Release' (or just stop touching the keys for 5+ seconds)"
expect "/boat/mode is HOLD, thrust latched at 0/0, and it STAYS there"
note "This is the safety property. The mission is still EXECUTE and the"
note "follower is still publishing to /boat/cmd/waypoint -- the arbiter"
note "must ignore it until you explicitly resume. If this flips back to"
note "WAYPOINT on its own, the latch is broken."
sample /boat/mode data "/boat/mode"
sample_thrust
echo
note "sampling again after a further 5s to confirm the latch holds..."
sleep 5
sample /boat/mode data "/boat/mode (5s later)"
pause

# ---------------------------------------------------------------------
step "5  Explicit resume"
# ---------------------------------------------------------------------
act "click 'Resume Auto'"
expect "/boat/mode returns to WAYPOINT and the boat starts moving again"
sample /boat/mode data "/boat/mode"
sample_thrust
pause

# ---------------------------------------------------------------------
step "6  Cancel back to IDLE"
# ---------------------------------------------------------------------
act "click 'Cancel'"
expect "/boat/state is IDLE (not ABORTED -- a deliberate cancel is not a fault)"
note "the old build had no real cancel: it pulsed the e-stop true/false"
note "300ms apart, which raced and logged a routine cancel as an emergency"
sample /boat/state data "/boat/state"
sample /boat/mode data "/boat/mode"
pause

# ---------------------------------------------------------------------
step "7  Upload route 2 -- the reported symptom"
# ---------------------------------------------------------------------
act "plot new points and press Send Mission again. Restart NOTHING."
expect "accepted: /boat/state EXECUTE, /boat/mode WAYPOINT, boat drives"
note "the reported failure was 'the boat isn't in the correct state and"
note "won't accept a new waypoint' -- a 409 here means it is still not"
note "returning to an accepting state"
sample /boat/state data "/boat/state"
sample /boat/mode data "/boat/mode"
sample_thrust
pause

# ---------------------------------------------------------------------
step "8  Lease expiry (the failure case that matters most)"
# ---------------------------------------------------------------------
act "hold W to take manual control, then CLOSE THE BROWSER TAB while holding it"
expect "/boat/mode goes to HOLD within ~1s and stays there. NOT WAYPOINT."
note "this simulates the operator's link dying mid-manoeuvre. The boat"
note "must stop and stay stopped, not hand itself back to autonomy."
note "control_arbiter should log: 'Manual lease EXPIRED'"
pause
sample /boat/mode data "/boat/mode"
sample_thrust
pause

# ---------------------------------------------------------------------
step "9  Detector is off and quiet"
# ---------------------------------------------------------------------
expect "no YOLO model load in the detector_node log, no cv_bridge errors"
expect "/boat/perception/detections publishes NOTHING"
note "disabled now means the node returns before touching frame data at"
note "all -- previously cv_bridge ran on every frame regardless"
sample /boat/perception/detections detections "/boat/perception/detections"
note "'NOTHING PUBLISHED' above is the PASS condition for this step"
echo
expect "the camera feed still works -- /boat/camera/image_raw is live"
printf '%sOBSERVED:%s ' "$BOLD" "$RST"
timeout 5 ros2 topic hz /boat/camera/image_raw 2>&1 | grep -m1 'average rate' \
  || echo "no rate reported -- check camera/bridge"

cat <<'DONE'

=====================================================================
 Done. Report what you actually saw, including anything that did not
 match. A step that failed here is more useful than one that passed:
 the logic tests already cover the decisions, so anything broken at
 this level is wiring, QoS, launch config or physics -- exactly the
 half those tests cannot reach.
=====================================================================
DONE
