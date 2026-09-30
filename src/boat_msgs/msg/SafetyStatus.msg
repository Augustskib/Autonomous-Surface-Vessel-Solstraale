# Geofence and link status, published by safety_monitor on boat/safety_status.
# Optional numeric fields are NaN when there is no value.

# Summary for the dashboard, computed once in safety_monitor.
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2

uint8 level
bool tripped
string reason
bool fence_enabled
float64 fence_center_lat
float64 fence_center_lon
float64 fence_radius_m
float64 fence_distance_m
float64 fence_margin_m
bool fence_breached
bool fence_warning
bool gcs_lost
float64 gcs_heartbeat_age_sec

# Warning ring inside the fence (radius * warn_fraction). The radius is
# published so the dashboard draws the same ring safety_monitor uses.
float64 fence_warn_radius_m
float64 fence_warn_fraction

# Seconds continuously outside the fence. 0.0 inside, NaN without a fix.
float64 fence_outside_sec

# Operator's reason, set only while the geofence is disabled.
string fence_disabled_reason
