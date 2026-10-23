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
# simulator run with a drop in the floor, then tests/ros_degradation_check.py
# in one with degraded sensors (config/degraded.yaml). With --nav, collisions seen
# during the Nav2 checks are logged to build/smoke_collisions.log, and
# tests/ros_door_check.py runs in a third simulator run with a door,
# tests/ros_auto_door_check.py in a fourth with an automatic door,
# tests/ros_nav_check.py --no-collisions in a fifth with people and a cart
# moving across the way (layouts/room_movers.yaml), and
# tests/ros_elevator_check.py in a sixth with two floors and an elevator
# (layouts/elevator.yaml), on saved maps per floor, and again with the floors
# stacked (layouts/elevator_vertical.yaml).
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
        -v "$PWD/tests:/tests:ro" -v "$PWD/maps:/maps" "$@"
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

# SLAM + Nav2, or Nav2 on a saved map: start_nav [launch file and arguments]
start_nav() {
    local launch=("${@:-slam.launch.py}")
    ((${#launch[@]} > 1)) || [[ "${launch[0]}" != slam.launch.py ]] || launch+=(nav:=true)
    echo "[smoke] starting ${launch[*]} (log: $NAV_LOG)"
    ros --name "$NAV_CONTAINER" kachaka-isaac-sim-ros:jazzy \
        ros2 launch kachaka_sim_bringup "${launch[@]}" rviz:=false > "$NAV_LOG" 2>&1 &
    NAV_PID=$!
    # On a saved map Nav2 waits for AMCL's first pose (the check gives it):
    # wait for localization then.
    local manager=lifecycle_manager_navigation
    [[ "${launch[0]}" == navigation.launch.py ]] && manager=lifecycle_manager_localization
    for _ in $(seq 120); do
        grep -q "\[$manager\]: .*Managed nodes are active" "$NAV_LOG" && break
        kill -0 "$NAV_PID" 2>/dev/null || { echo "[smoke] Nav2 exited; see $NAV_LOG"; exit 1; }
        sleep 1
    done
    grep -q "\[$manager\]: .*Managed nodes are active" "$NAV_LOG" \
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

    # An automatic door, never commanded (tests/ros_auto_door_check.py).
    stop_sim
    LOG=build/smoke_sim_auto_door.log
    start_sim --layout layouts/room_auto_door.yaml
    start_nav
    echo "[smoke] running ros_auto_door_check"
    ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_auto_door_check.py || FAILED+=(ros_auto_door_check)
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true

    # People and a cart moving across the robot's way: reach the goals
    # without touching them.
    stop_sim
    LOG=build/smoke_sim_movers.log
    start_sim --layout layouts/room_movers.yaml
    start_nav
    echo "[smoke] running ros_nav_check --no-collisions (moving people)"
    ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_nav_check.py --no-collisions || FAILED+=(ros_nav_check_movers)
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true

    # Two floors and an elevator, on saved maps drawn from the layout.
    stop_sim
    docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/repo:ro" -v "$PWD/maps:/repo/maps" \
        kachaka-isaac-sim-ros:jazzy python3 /repo/scripts/make_maps.py /repo/layouts/elevator.yaml --out /repo/maps
    LOG=build/smoke_sim_elevator.log
    start_sim --layout layouts/elevator.yaml
    start_nav navigation.launch.py map:=/maps/elevator_1F.yaml
    echo "[smoke] running ros_elevator_check"
    ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_elevator_check.py || FAILED+=(ros_elevator_check)
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true

    # The same with the floors stacked: the car's floor carries the robot up.
    stop_sim
    LOG=build/smoke_sim_elevator_vertical.log
    start_sim --layout layouts/elevator_vertical.yaml
    start_nav navigation.launch.py map:=/maps/elevator_1F.yaml
    echo "[smoke] running ros_elevator_check (stacked floors)"
    ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_elevator_check.py || FAILED+=(ros_elevator_check_vertical)
    docker rm -f "$NAV_CONTAINER" >/dev/null 2>&1 || true
fi

# Step sensors and collisions need a drop in the floor: restart the simulator.
stop_sim
LOG=build/smoke_sim_drop.log
start_sim --drop 1.0 0.0 0.4 0.8
echo "[smoke] running ros_hazard_check"
ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_hazard_check.py || FAILED+=(ros_hazard_check)

# Degraded sensors (config/degraded.yaml), a person to the robot's left.
stop_sim
LOG=build/smoke_sim_degraded.log
start_sim --config config/degraded.yaml --person 0.0 1.0 0.0 --seed 1
echo "[smoke] running ros_degradation_check"
ros kachaka-isaac-sim-ros:jazzy python3 /tests/ros_degradation_check.py || FAILED+=(ros_degradation_check)
if ((${#FAILED[@]})); then
    echo "[smoke] failed: ${FAILED[*]}"
    exit 1
fi
