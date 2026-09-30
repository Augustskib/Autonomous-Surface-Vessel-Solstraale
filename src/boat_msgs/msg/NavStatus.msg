# Navigation status, published by waypoint_follower on boat/nav_status at 2 Hz.
# Fields without a value yet (no target, no fix) are NaN.

bool active
int32 waypoint_index
int32 waypoint_count
float64 speed_mps
float64 target_lat
float64 target_lon
float64 distance_m
float64 remaining_m
float64 bearing_deg
float64 heading_error_deg
float64 eta_sec
float64 gps_age_sec
