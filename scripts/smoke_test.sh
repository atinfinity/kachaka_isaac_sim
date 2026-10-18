#!/usr/bin/env bash
# Headless smoke test: start the simulator in the default room and run
# tests/ros_smoke_check.py in the ROS 2 container. Exit code 0 = all passed.
#
#   scripts/smoke_test.sh
#
# Needs the ROS image: docker compose -f docker/compose.yaml build
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
LOG=build/smoke_sim.log

cleanup() { pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true; }
trap cleanup EXIT

echo "[smoke] starting the simulator (log: $LOG)"
scripts/run_sim.sh --headless > "$LOG" 2>&1 &
SIM=$!
until grep -q "\[kachaka\] running" "$LOG"; do
    kill -0 "$SIM" 2>/dev/null || { echo "[smoke] simulator exited; see $LOG"; exit 1; }
    sleep 2
done

echo "[smoke] running checks"
docker run --rm --network host --ipc host \
    -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
    -v "$PWD/tests:/tests:ro" kachaka-isaac-sim-ros:jazzy \
    python3 /tests/ros_smoke_check.py
