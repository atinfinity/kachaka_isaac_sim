#!/usr/bin/env bash
# Record the README demo: the simulator's bird's-eye view next to RViz while
# Nav2 drives the robot through goals during SLAM. Produces docs/demo.gif.
#
#   scripts/record_demo.sh
#
# Needs the ROS image (docker compose -f docker/compose.yaml build) and
# ffmpeg on the host. Runs headless: RViz draws into Xvfb inside the container.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=build/demo
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
scripts/run_sim.sh --headless --no-cameras --record "$OUT/sim" > "$OUT/sim.log" 2>&1 &
until grep -q "\[kachaka\] running" "$OUT/sim.log"; do sleep 2; done

echo "[demo] starting SLAM, Nav2 and RViz (Xvfb :$DISPLAY_NUM)"
docker run -d --rm --name kachaka-demo-ros --network host --ipc host \
    -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
    -e DISPLAY=":$DISPLAY_NUM" -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$PWD/tests:/tests:ro" \
    kachaka-isaac-sim-ros:jazzy bash -c "
        Xvfb :$DISPLAY_NUM -screen 0 1440x900x24 -nolisten tcp -ac &
        sleep 2
        ros2 launch kachaka_sim_bringup slam.launch.py nav:=true \
            rviz_config:=/ws/install/kachaka_sim_bringup/share/kachaka_sim_bringup/config/demo.rviz" > /dev/null
until [[ -S /tmp/.X11-unix/X$DISPLAY_NUM ]]; do sleep 1; done
sleep 25  # let Nav2 come up and RViz draw the first map

echo "[demo] recording"
date +%s.%N > "$OUT/rviz_start.txt"
ffmpeg -hide_banner -loglevel error -f x11grab -draw_mouse 0 -framerate 10 -video_size 1440x900 -i ":$DISPLAY_NUM" \
    -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT/rviz.mp4" &
FFMPEG_PID=$!
docker exec kachaka-demo-ros bash -c "source /ws/install/setup.bash && python3 /tests/ros_nav_check.py" \
    | tee "$OUT/nav.log"
date +%s.%N > "$OUT/end.txt"
sleep 1
kill -INT "$FFMPEG_PID"; wait "$FFMPEG_PID" || true
FFMPEG_PID=

echo "[demo] composing docs/demo.gif"
.venv/bin/python scripts/make_demo_gif.py "$OUT" docs/demo.gif --rviz-crop ${RVIZ_CROP:-470 90 920 750}
