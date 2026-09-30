#!/usr/bin/env bash
#
# Launch the simulation stack after killing anything left from a previous
# run. Gazebo often survives Ctrl+C or a closed terminal, and an orphan keeps
# publishing on the same topics, so the boat appears to jump between
# positions and waypoint following fails.
#
# Usage:
#   scripts/sim/boatsim.sh          launch
#   scripts/sim/boatsim.sh --build  colcon build first, then launch
#
# No set -u (ROS setup.bash uses unset variables) and no set -e (pkill
# returns non-zero when nothing matches). Errors are checked explicitly.

WORKSPACE="$(cd "$(dirname "$0")/../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

# Processes started by the sim launch, matched by command line.
STALE_PATTERNS=(
    "gz sim"
    "gz_tools_vendor"
    "parameter_bridge"
    "web_server"
    "mission_manager"
    "waypoint_follower"
    "thrust_splitter"
    "foxglove_bridge"
    "safety_monitor"
)

echo "== Clearing any leftover simulation processes =="
killed_any=0
for pattern in "${STALE_PATTERNS[@]}"; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
        echo "   killing: ${pattern}"
        pkill -9 -f "${pattern}" >/dev/null 2>&1
        killed_any=1
    fi
done
if [ "${killed_any}" -eq 0 ]; then
    echo "   nothing stale found"
fi

sleep 1

# Refuse to launch if Gazebo is still running.
if pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "ERROR: a 'gz sim' process is STILL running after being killed:" >&2
    pgrep -fa "gz sim" >&2
    echo "Investigate before launching -- results would be meaningless." >&2
    exit 1
fi

cd "${WORKSPACE}" || { echo "ERROR: no workspace at ${WORKSPACE}" >&2; exit 1; }

# shellcheck disable=SC1090
source "${ROS_SETUP}"

if [ "${1:-}" = "--build" ]; then
    echo "== colcon build =="
    colcon build || { echo "ERROR: build failed, not launching." >&2; exit 1; }
fi

if [ ! -f install/setup.bash ]; then
    echo "ERROR: install/setup.bash missing -- run '$0 --build' first." >&2
    exit 1
fi

# shellcheck disable=SC1091
source install/setup.bash

# WSL2 has no GPU for Ogre2. Without software rendering the camera is grey.
export LIBGL_ALWAYS_SOFTWARE=1

echo "== Launching (one dashboard tab only: http://localhost:5000) =="
exec ros2 launch boat_simulation boat_simulation.launch.py
