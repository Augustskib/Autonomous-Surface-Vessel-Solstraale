# Failsafe level, published by safety_monitor on boat/failsafe_state at 2 Hz.
#
# A boat does not stop when thrust is cut, it drifts. So the response to a
# fault depends on what the boat still knows (position, heading), not only
# on what tripped:
#
#   NOMINAL           no fault
#   HOLD_POSITION     station-keep. Not implemented yet, falls back to POWERED_STOP
#   RETURN_TO_LAUNCH  drive back to the launch point. Needs position and a usable heading
#   POWERED_STOP      zero thrust. Used when holding or returning is not possible
#   HARD_STOP         e-stop, overrides everything including manual control
uint8 NOMINAL=0
uint8 HOLD_POSITION=1
uint8 RETURN_TO_LAUNCH=2
uint8 POWERED_STOP=3
uint8 HARD_STOP=4

uint8 level
string level_name                 # e.g. "RETURN_TO_LAUNCH"

# Condition that selected the level, e.g. "GEOFENCE_BREACH". Empty at NOMINAL.
string reason
# All active conditions, most severe first.
string[] active_reasons

# Level the condition asked for, and why a less capable level was applied
# instead (e.g. RETURN_TO_LAUNCH requested, POWERED_STOP applied because
# there is no usable heading).
uint8 requested_level
string degraded_reason

# Thrust the failsafe is sending on boat/cmd/failsafe. False at
# NOMINAL, POWERED_STOP and HARD_STOP.
bool commanding_thrust
float32 commanded_left
float32 commanded_right

# Launch point. Distance and bearing are NaN without a fix.
float64 launch_lat
float64 launch_lon
float64 distance_to_launch_m
float64 bearing_to_launch_deg
bool launch_point_valid

# Inputs the decision was made from.
bool position_valid
bool heading_usable
uint8 heading_quality        # mirrors HeadingEstimate.quality
float64 heading_deg          # NaN when not usable

# Escalation is automatic, de-escalation needs an operator clear on
# boat/failsafe_clear.
bool latched
float64 seconds_at_level
