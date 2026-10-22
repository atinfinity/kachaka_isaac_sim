#!/usr/bin/env bash
# Run the kachaka-api demo notebooks headless against the simulator (#64):
# tests/demo_notebooks_check.py, with the simulator in the default room and
# Nav2 on a saved map of it (switch_map and set_robot_pose need one).
#
#   scripts/demo_notebooks.sh                  # all of them
#   scripts/demo_notebooks.sh speak robot_pose # some
#
# 1. The simulator starts with SLAM; its map is saved twice into maps/ as
#    0_demo_notebooks and 0_demo_notebooks_2 (removed at the end).
# 2. Nav2 restarts on the saved map and the robot is localized on it.
# 3. The notebooks run (build/demo_notebooks/ has the executed copies).
#
# Needs the ROS image (docker compose -f docker/compose.yaml build) and uv.
# About 10 min.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
LOG=build/demo_notebooks_sim.log
NAV_LOG=build/demo_notebooks_nav.log
NAV_CONTAINER=kachaka-demo-notebooks-nav
MAP=0_demo_notebooks
PY=(uv run --no-project --with kachaka-api==3.17.5.0 --with nbclient --with ipykernel
    --with numpy --with pyyaml --with pillow --with opencv-python-headless --with matplotlib --with plotly
    --with ipywidgets --with anywidget --with ipympl --with fastapi --with uvicorn --with nest_asyncio --with requests
    --with onnx --with onnxruntime
    python tests/demo_notebooks_check.py)

cleanup() {
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
    rm -f maps/"${MAP:?}".yaml maps/"${MAP:?}".pgm maps/"${MAP:?}"_2.yaml maps/"${MAP:?}"_2.pgm
}
trap cleanup EXIT

if [[ -e maps/$MAP.yaml ]]; then
    echo "[notebooks] maps/$MAP.yaml exists: another run? remove it first"
    exit 1
fi

echo "[notebooks] starting the simulator (log: $LOG)"
scripts/run_sim.sh --headless > "$LOG" 2>&1 &
SIM=$!
until grep -q "\[kachaka\] running" "$LOG"; do
    kill -0 "$SIM" 2>/dev/null || { echo "[notebooks] simulator exited; see $LOG"; exit 1; }
    sleep 2
done

# launch file and arguments; waits until its lifecycle manager is active
start_nav() {
    echo "[notebooks] starting $* (log: $NAV_LOG)"
    docker run --rm --network host --ipc host --name "$NAV_CONTAINER" \
        -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" -v "$PWD/maps:/maps" \
        kachaka-isaac-sim-ros:jazzy ros2 launch kachaka_sim_bringup "$@" rviz:=false > "$NAV_LOG" 2>&1 &
    local pid=$! manager=lifecycle_manager_navigation
    [[ "$1" == navigation.launch.py ]] && manager=lifecycle_manager_localization
    for _ in $(seq 120); do
        grep -q "\[$manager\]: .*Managed nodes are active" "$NAV_LOG" && return 0
        kill -0 "$pid" 2>/dev/null || { echo "[notebooks] $1 exited; see $NAV_LOG"; exit 1; }
        sleep 1
    done
    echo "[notebooks] $1 did not become active; see $NAV_LOG"
    exit 1
}

start_nav slam.launch.py nav:=true
"${PY[@]}" --save-map "$MAP"
docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true
start_nav navigation.launch.py map:=/maps/$MAP.yaml
"${PY[@]}" --localize "$@"
