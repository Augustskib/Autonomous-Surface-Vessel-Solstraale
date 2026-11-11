#!/usr/bin/env bash
# Managed simulation, isolated from the hardware ROS domain.
WORKSPACE="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$WORKSPACE" || exit 1
source /opt/ros/jazzy/setup.bash || exit 1
if [ "${1:-}" = "--build" ]; then
    colcon build || exit 1
elif [ -n "${1:-}" ]; then
    echo 'Usage: boatsim.sh [--build]' >&2
    exit 2
fi
if [ ! -f install/setup.bash ]; then
    echo 'Build first with boatsim.sh --build' >&2
    exit 1
fi
source install/setup.bash || exit 1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
exec python3 scripts/sim/run_sim.py
