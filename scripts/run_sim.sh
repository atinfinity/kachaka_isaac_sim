#!/usr/bin/env bash
# Launch the Kachaka simulator with Isaac Sim's bundled ROS 2 Jazzy libraries.
#   scripts/run_sim.sh [--headless] [--config config/robot.yaml] ...
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/.venv/bin/python"
ROS_LIB="$ROOT/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROS_LIB"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"

SCRIPT="${KACHAKA_SIM_SCRIPT:-$ROOT/scripts/run_sim.py}"
exec "$PY" "$SCRIPT" "$@"
