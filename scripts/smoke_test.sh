#!/usr/bin/env bash
# Headless smoke test: start the simulator in the default room and run
# tests/ros_smoke_check.py in the ROS 2 container. Exit code 0 = all passed.
#
#   scripts/smoke_test.sh          # topics, TF, driving
#   scripts/smoke_test.sh --nav    # also SLAM + Nav2: the navigation, docking,
#                                  # shelf and command checks, one simulator run;
#                                  # each check starts with sim/reset
#
# Both end with tests/ros_hazard_check.py (step sensors, collisions) in a
# second simulator run with a drop in the floor. With --nav, collisions seen
# during the Nav2 checks are logged to build/smoke_collisions.log, and
# tests/ros_door_check.py runs in a third simulator run with a door.
#
# Needs the ROS image: docker compose -f docker/compose.yaml build
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
LOG=build/smoke_sim.log
NAV_LOG=build/smoke_nav.log
NAV_CONTAINER=kachaka-smoke-nav
COLLISION_CONTAINER=kachaka-smoke-collisions
COLLISION_LOG=build/smoke_collisions.log
NAV=false
[[ "${1:-}" == "--nav" ]] && NAV=true

cleanup() {
    docker rm -f "$NAV_CONTAINER" "$COLLISION_CONTAINER" >/dev/null 2>&1 || true
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT

ros() {
    docker run --rm --network host --ipc host \
        -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
        -v "$PWD/tests:/tests:ro" "$@"
}

start_sim() {
    echo "[smoke] starting the simulator $* (log: $LOG)"
    scripts/run_sim.sh --headless "$@" > "$LOG" 2>&1 &
    SIM=$!
    until grep -q "\[kachaka\] running" "$LOG"; do
        kill -0 "$SIM" 2>/dev/null || { echo "[smoke] simulator exited; see $LOG"; exit 1; }
        sleep 2
    done
}

start_sim

FAILED=()
echo "[smoke] running checks"
ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_smoke_check.py

start_nav() {
    echo "[smoke] starting SLAM + Nav2 (log: $NAV_LOG)"
    ros --name "$NAV_CONTAINER" kachaka-isaac-sim-ros:jazzy \
        ros2 launch kachaka_sim_bringup slam.launch.py nav:=true rviz:=false > "$NAV_LOG" 2>&1 &
    NAV_PID=$!
    for _ in $(seq 120); do
        grep -q "\[lifecycle_manager_navigation\]: .*Managed nodes are active" "$NAV_LOG" && break
        kill -0 "$NAV_PID" 2>/dev/null || { echo "[smoke] Nav2 exited; see $NAV_LOG"; exit 1; }
        sleep 1
    done
    grep -q "\[lifecycle_manager_navigation\]: .*Managed nodes are active" "$NAV_LOG" \
        || { echo "[smoke] Nav2 did not become active; see $NAV_LOG"; exit 1; }
}

if $NAV; then
    start_nav
    # Each check resets the simulator first, so one failure does not spoil the
    # next: run them all and report the failures at the end.
    ros --name "$COLLISION_CONTAINER" kachaka-isaac-sim-ros:jazzy \
        ros2 topic echo /kachaka/sim/collision std_msgs/msg/String > "$COLLISION_LOG" 2>&1 &
    for check in ros_nav_check ros_dock_check ros_shelf_check ros_command_check ros_detection_check; do
        echo "[smoke] running $check"
        ros kachaka-isaac-sim-ros:jazzy python3 "/tests/$check.py" || FAILED+=("$check")
    done
    docker rm -f "$NAV_CONTAINER" "$COLLISION_CONTAINER" >/dev/null 2>&1 || true
    echo "[smoke] collisions during the Nav2 checks: $(grep -c '^data:' "$COLLISION_LOG" || true) (see $COLLISION_LOG)"
fi

stop_sim() {
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
    wait "$SIM" 2>/dev/null || true
}

if $NAV; then
    # A partition wall with a door (tests/ros_door_check.py).
    stop_sim
    LOG=build/smoke_sim_door.log
    start_sim --layout layouts/room_door.yaml
    start_nav
    echo "[smoke] running ros_door_check"
    ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_door_check.py || FAILED+=(ros_door_check)
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true
fi

# Step sensors and collisions need a drop in the floor: restart the simulator.
stop_sim
LOG=build/smoke_sim_drop.log
start_sim --drop 1.0 0.0 0.4 0.8
echo "[smoke] running ros_hazard_check"
ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_hazard_check.py || FAILED+=(ros_hazard_check)
if ((${#FAILED[@]})); then
    echo "[smoke] failed: ${FAILED[*]}"
    exit 1
fi
