#!/usr/bin/env bash
# Record the README demos: the simulator's view next to RViz.
#
#   scripts/record_demo.sh        # Nav2 goals during SLAM -> docs/demo.gif
#   scripts/record_demo.sh dock   # RETURN_HOME onto the charger -> docs/dock.gif
#   scripts/record_demo.sh shelf  # dock, tow, turn and leave a shelf -> docs/shelf.gif
#   scripts/record_demo.sh scene office  # Nav2 route in an Isaac Sim scene -> build/scene_office.mp4
#                                        # (both views follow the robot)
#
# Needs the ROS image (docker compose -f docker/compose.yaml build) and
# ffmpeg on the host. Runs headless: RViz draws into Xvfb inside the container.
set -euo pipefail
cd "$(dirname "$0")/.."
DEMO=${1:-nav}
SIM_ARGS=()
RVIZ_SED=""  # sed script applied to demo.rviz
LABEL="RViz: slam_toolbox + Nav2"
COMPOSE_ARGS=()
case "$DEMO" in
    nav)
        GIF=docs/demo.gif
        VIEW=(0.0 -2.4 7.5 0.0 -0.1 0.0)
        RUN="python3 /tests/ros_nav_check.py" ;;
    dock)
        GIF=docs/dock.gif
        VIEW=(-2.1 -1.7 2.0 -1.7 0.0 0.0)  # from the south like RViz (+x right), over box_b
        RUN="python3 /tests/ros_dock_check.py --demo" ;;
    shelf)
        GIF=docs/shelf.gif
        VIEW=(0.9 -1.6 5.8 0.9 0.3 0.0)  # inside the south wall, looking down (+x right like RViz)
        RUN="python3 /tests/ros_shelf_check.py --demo" ;;
    scene)
        WORLD=${2:?usage: $0 scene <office|hospital|warehouse|simple_room>}
        DEMO=scene_$WORLD
        GIF=build/$DEMO.mp4
        # Straight down from 7.5 m, near clip at 5.8 m: walls and racks are cut
        # at 1.7 m, ceilings vanish. Turned so the spawn heading points right,
        # like the SLAM map frame in RViz.
        case "$WORLD" in
            office|warehouse) VIEW=(0.0 0.0 7.5 -0.01 0.0 0.0) ;;
            *) VIEW=(0.0 0.0 7.5 0.0 0.01 0.0) ;;
        esac
        SIM_ARGS=(--world "$WORLD" --record-follow --record-near 5.8)
        RVIZ_SED="s/Scale: 165/Scale: 45/; /Name: Current View/a\\      Target Frame: base_footprint"
        LABEL="RViz: slam_toolbox + Nav2 ($WORLD)"
        RVIZ_CROP=${RVIZ_CROP:-576 106 720 720}  # square around the robot
        COMPOSE_ARGS=(--height 540 --mark-robot 45)
        RUN="python3 /tests/ros_nav_check.py --world $WORLD" ;;
    *) echo "usage: $0 [nav|dock|shelf|scene <world>]" >&2; exit 2 ;;
esac
OUT=build/$DEMO
DISPLAY_NUM=99
COMPOSE=(docker compose -f docker/compose.yaml)
rm -rf "$OUT" && mkdir -p "$OUT"

cleanup() {
    [[ -n "${FFMPEG_PID:-}" ]] && kill -INT "$FFMPEG_PID" 2>/dev/null || true
    docker logs kachaka-demo-ros > "$OUT/ros.log" 2>&1 || true
    docker rm -f kachaka-demo-ros >/dev/null 2>&1 || true
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT

echo "[demo] starting the simulator"
scripts/run_sim.sh --headless --no-cameras --record "$OUT/sim" --record-view "${VIEW[@]}" "${SIM_ARGS[@]}" > "$OUT/sim.log" 2>&1 &
until grep -q "\[kachaka\] running" "$OUT/sim.log"; do sleep 2; done

echo "[demo] starting SLAM, Nav2 and RViz (Xvfb :$DISPLAY_NUM)"
docker run -d --rm --name kachaka-demo-ros --network host --ipc host \
    -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
    -e DISPLAY=":$DISPLAY_NUM" -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$PWD/tests:/tests:ro" \
    kachaka-isaac-sim-ros:jazzy bash -c "
        Xvfb :$DISPLAY_NUM -screen 0 1440x900x24 -nolisten tcp -ac &
        sleep 2
        sed '$RVIZ_SED' /ws/install/kachaka_sim_bringup/share/kachaka_sim_bringup/config/demo.rviz > /tmp/demo.rviz
        ros2 launch kachaka_sim_bringup slam.launch.py nav:=true rviz_config:=/tmp/demo.rviz" > /dev/null
until [[ -S /tmp/.X11-unix/X$DISPLAY_NUM ]]; do sleep 1; done
sleep 25  # let Nav2 come up and RViz draw the first map

echo "[demo] recording"
date +%s.%N > "$OUT/rviz_start.txt"
ffmpeg -hide_banner -loglevel error -f x11grab -draw_mouse 0 -framerate 10 -video_size 1440x900 -i ":$DISPLAY_NUM" \
    -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT/rviz.mp4" &
FFMPEG_PID=$!
docker exec kachaka-demo-ros bash -c "source /ws/install/setup.bash && $RUN" | tee "$OUT/run.log"
date +%s.%N > "$OUT/end.txt"
sleep 1
kill -INT "$FFMPEG_PID"; wait "$FFMPEG_PID" || true
FFMPEG_PID=

# Stop the simulator so the frame writer finishes the last PNGs.
pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill -INT
for _ in $(seq 60); do pgrep -f "^\S*/bin/python \S*run_sim.py" >/dev/null || break; sleep 1; done

echo "[demo] composing $GIF"
.venv/bin/python scripts/make_demo_gif.py "$OUT" "$GIF" --speed ${SPEED:-2} --rviz-crop ${RVIZ_CROP:-470 90 920 750} \
    --rviz-label "$LABEL" "${COMPOSE_ARGS[@]}"
