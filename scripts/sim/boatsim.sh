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
if pgrep -f "gz sim" >/dev/null 2>&1; then
    echo 'WARNING: a gz sim process is already running. If the boat jumps between' >&2
    echo 'positions, stop it first: pkill -f "gz sim"' >&2
fi
echo "Simulation uses ROS_DOMAIN_ID=${ASV_SIM_DOMAIN_ID:-42}. In other terminals run:"
echo "  export ROS_DOMAIN_ID=${ASV_SIM_DOMAIN_ID:-42}"
exec python3 scripts/sim/run_sim.py
