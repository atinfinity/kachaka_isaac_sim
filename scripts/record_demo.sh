#!/usr/bin/env bash
# Record the README demos: the simulator's view next to RViz.
#
#   scripts/record_demo.sh        # Nav2 goals during SLAM -> docs/demo.gif
#   scripts/record_demo.sh dock   # RETURN_HOME onto the charger -> docs/dock.gif
#   scripts/record_demo.sh shelf  # dock, tow, turn and leave a shelf -> docs/shelf.gif
#   scripts/record_demo.sh shelf_return  # leave a shelf, then RETURN_SHELF around it -> docs/shelf_return.gif
#   scripts/record_demo.sh door   # Nav2 replans when a door closes and opens -> docs/door.gif
#   scripts/record_demo.sh auto_door  # Nav2 through an automatic door -> docs/auto_door.gif
#   scripts/record_demo.sh movers # Nav2 goals among people and a cart on the move -> docs/movers.gif
#   scripts/record_demo.sh elevator  # Nav2 into the elevator on 1F, ride, out on 2F -> docs/elevator.gif
#   scripts/record_demo.sh elevator_vertical  # the same with the floors stacked -> docs/elevator_vertical.gif
#   scripts/record_demo.sh scene office  # Nav2 route in an Isaac Sim scene -> build/scene_office.mp4
#                                        # (both views follow the robot)
#   scripts/record_demo.sh api    # kachaka-api calls from the host -> build/api.mp4
#   scripts/record_demo.sh notebooks sensors  # kachaka-api demo notebooks in JupyterLab
#   scripts/record_demo.sh notebooks control  #   -> docs/notebooks_<set>.gif (the browser instead of
#   scripts/record_demo.sh notebooks map      #   RViz, or next to RViz for map; needs chromium on the host)
#   scripts/record_demo.sh front_stop  # front obstacle stop on/off at a wall -> build/front_stop.mp4
#   scripts/record_demo.sh cliff  # step sensors at a drop, power button, a collision -> docs/cliff.gif
#   scripts/record_demo.sh speed  # manual vs autonomous top speed, towing and alone -> docs/speed.gif
#
# Needs the ROS image (docker compose -f docker/compose.yaml build) and
# ffmpeg on the host. Runs headless: RViz draws into Xvfb inside the container.
set -euo pipefail
cd "$(dirname "$0")/.."
DEMO=${1:-nav}
SIM_ARGS=()
CAMERAS=(--no-cameras)  # the demos that do not show a camera image skip rendering it
RVIZ_SED=""  # sed script applied to demo.rviz
LABEL="RViz: slam_toolbox + Nav2"
COMPOSE_ARGS=()
RUN=""       # run in the ROS container
EXEC_ARGS=()  # docker exec options for RUN
BROWSER=""   # set: HOST_RUN records the right-hand video itself
LEFT=""      # with BROWSER: "rviz" puts the RViz capture on the left, not the simulator
SAVE_MAP=""  # set: save the SLAM map under this name (twice) first; LAUNCH then uses it
HOST_RUN=""  # or on the host
# ROS side: SLAM + Nav2 with RViz, or e.g. "navigation.launch.py map:=/maps/x.yaml" (RViz started separately)
LAUNCH="slam.launch.py nav:=true"
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
    shelf_return)
        GIF=docs/shelf_return.gif
        VIEW=(0.9 -1.6 5.8 0.9 0.3 0.0)
        SPEED=${SPEED:-6}
        RUN="python3 /tests/ros_shelf_check.py --demo --return" ;;
    door)
        GIF=docs/door.gif
        VIEW=(0.3 -2.0 5.5 0.3 0.0 0.0)  # from the south (+x right like RViz), along the partition
        SIM_ARGS=(--layout layouts/room_door.yaml)
        LABEL="RViz: Nav2 and a door"
        SPEED=${SPEED:-4}
        RUN="python3 /tests/ros_door_check.py" ;;
    auto_door)
        GIF=docs/auto_door.gif
        VIEW=(0.3 -2.0 5.5 0.3 0.0 0.0)  # from the south (+x right like RViz), along the partition
        SIM_ARGS=(--layout layouts/room_auto_door.yaml)
        LABEL="RViz: Nav2 and an automatic door"
        SPEED=${SPEED:-4}
        RUN="python3 /tests/ros_auto_door_check.py" ;;
    movers)
        GIF=docs/movers.gif
        VIEW=(0.0 -3.2 6.0 0.0 -0.2 0.0)  # from the south, a little slanted to show the people walking
        SIM_ARGS=(--layout layouts/room_movers.yaml)
        LABEL="RViz: Nav2 among moving people"
        SPEED=${SPEED:-4}
        RUN="python3 /tests/ros_nav_check.py --no-collisions" ;;
    elevator)
        GIF=docs/elevator.gif
        VIEW=(0.0 -3.0 6.0 0.4 0.0 0.0)  # follows the robot (the floors are 10 m apart), from the south
        SIM_ARGS=(--layout layouts/elevator.yaml --record-follow)
        LAUNCH="navigation.launch.py map:=/maps/elevator_1F.yaml"
        RVIZ_SED="/Name: Current View/a\\      Target Frame: base_footprint"
        LABEL="RViz: Nav2 on each floor's map"
        SPEED=${SPEED:-4}
        docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/repo:ro" -v "$PWD/maps:/repo/maps" \
            kachaka-isaac-sim-ros:jazzy python3 /repo/scripts/make_maps.py /repo/layouts/elevator.yaml --out /repo/maps
        RUN="python3 /tests/ros_elevator_check.py --captions /build/elevator/captions.csv" ;;
    elevator_vertical)
        GIF=docs/elevator_vertical.gif
        # Follows the robot up too; from above, near clip at 4.4 m: what is
        # higher than 2.6 m over the robot's floor (the floor above) is cut.
        VIEW=(0.0 -1.2 7.0 0.3 0.0 0.0)
        SIM_ARGS=(--layout layouts/elevator_vertical.yaml --record-follow --record-near 4.4)
        LAUNCH="navigation.launch.py map:=/maps/elevator_1F.yaml"
        RVIZ_SED="/Name: Current View/a\\      Target Frame: base_footprint"
        LABEL="RViz: Nav2 on each floor's map"
        SPEED=${SPEED:-4}
        docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/repo:ro" -v "$PWD/maps:/repo/maps" \
            kachaka-isaac-sim-ros:jazzy python3 /repo/scripts/make_maps.py /repo/layouts/elevator.yaml --out /repo/maps
        RUN="python3 /tests/ros_elevator_check.py --captions /build/elevator_vertical/captions.csv" ;;
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
    api)
        GIF=build/api.mp4
        VIEW=(0.0 -2.4 7.5 0.0 -0.1 0.0)
        LABEL="RViz: slam_toolbox + Nav2 (driven by kachaka-api)"
        COMPOSE_ARGS=(--height 540)
        HOST_RUN="uv run --no-project --with kachaka-api==3.17.5.0 python scripts/api_demo.py $PWD/build/api/captions.csv" ;;
    notebooks)
        SET=${2:-control}
        GIF=docs/notebooks_$SET.gif
        VIEW=(0.0 -2.4 7.5 0.0 -0.1 0.0)
        LABEL="JupyterLab: kachaka-api demo notebooks"
        RVIZ_CROP="0 0 960 720"
        COMPOSE_ARGS=(--height 450)
        SPEED=${SPEED:-4}
        CAMERAS=()
        BROWSER=1  # record_notebook_demo.py writes the right-hand video, not ffmpeg
        if [[ "$SET" == map ]]; then
            # switch_map needs two maps, and set_robot_pose a map to localize on
            SAVE_MAP=0_notebooks_demo
            LAUNCH="navigation.launch.py map:=/maps/$SAVE_MAP.yaml"
            LEFT=rviz
            COMPOSE_ARGS+=(--left-video left --left-crop 740 95 660 745 --left-label "RViz: Nav2 (AMCL)")
        fi
        HOST_RUN="uv run --no-project --with kachaka-api==3.17.5.0 --with jupyterlab --with ipympl
            --with ipywidgets --with plotly --with anywidget --with fastapi --with uvicorn --with nest_asyncio
            --with opencv-python-headless --with matplotlib --with numpy --with pillow --with pyyaml --with websockets
            --with onnx --with onnxruntime python scripts/record_notebook_demo.py $PWD/build/notebooks_$SET $SET" ;;
    front_stop)
        GIF=build/front_stop.mp4
        VIEW=(1.9 -1.7 0.9 1.9 0.0 0.05)  # from the south, side-on to the path at the east wall
        LABEL="RViz: front obstacle stop"
        COMPOSE_ARGS=(--height 540)
        # Captions go to the host's build/ directory, written as the host user.
        EXEC_ARGS=(-u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log)
        RUN="python3 /tests/ros_front_stop_demo.py /build/front_stop/captions.csv" ;;
    cliff)
        GIF=docs/cliff.gif
        VIEW=(0.6 -1.8 2.2 0.6 0.6 0.0)  # from the south (+x right like RViz), over the drop and the north wall
        SIM_ARGS=(--drop 1.0 0.0 0.4 0.8)
        LABEL="RViz: step sensors and collisions"
        COMPOSE_ARGS=(--height 540)
        EXEC_ARGS=(-u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log)
        RUN="python3 /tests/ros_cliff_demo.py /build/cliff/captions.csv" ;;
    speed)
        GIF=docs/speed.gif
        VIEW=(-0.1 -1.9 4.5 -0.1 0.4 0.0)  # from the south (+x right like RViz), over the whole runway
        SIM_ARGS=(--x -1.8 --shelf -1.8 0 0 --no-charger --no-people)
        LABEL="RViz: speed limits with and without a shelf"
        RVIZ_SED="s/^      X: 0$/      X: 1.2/"  # centre on the runway (map origin = spawn)
        COMPOSE_ARGS=(--height 540)
        EXEC_ARGS=(-u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_log)
        RUN="python3 /tests/ros_speed_demo.py /build/speed/captions.csv" ;;
    *) echo "usage: $0 [nav|dock|shelf|shelf_return|door|auto_door|movers|elevator|elevator_vertical|scene <world>|api|notebooks [sensors|control|map|detection]|front_stop|cliff|speed]" >&2; exit 2 ;;
esac
OUT=build/$DEMO
[[ "$DEMO" == notebooks ]] && OUT=build/notebooks_$SET
DISPLAY_NUM=99
COMPOSE=(docker compose -f docker/compose.yaml)
rm -rf "$OUT" && mkdir -p "$OUT"

cleanup() {
    [[ -n "${FFMPEG_PID:-}" ]] && kill -INT "$FFMPEG_PID" 2>/dev/null || true
    docker logs kachaka-demo-ros > "$OUT/ros.log" 2>&1 || true
    docker rm -f kachaka-demo-ros >/dev/null 2>&1 || true
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
    if [[ -n "$SAVE_MAP" ]]; then
        docker rm -f kachaka-demo-slam >/dev/null 2>&1 || true
        rm -f maps/"${SAVE_MAP:?}".yaml maps/"${SAVE_MAP:?}".pgm maps/"${SAVE_MAP:?}"_2.yaml maps/"${SAVE_MAP:?}"_2.pgm
    fi
}
trap cleanup EXIT

echo "[demo] starting the simulator"
scripts/run_sim.sh --headless "${CAMERAS[@]}" --record "$OUT/sim" --record-view "${VIEW[@]}" "${SIM_ARGS[@]}" > "$OUT/sim.log" 2>&1 &
until grep -q "\[kachaka\] running" "$OUT/sim.log"; do sleep 2; done

if [[ -n "$SAVE_MAP" ]]; then
    [[ -e maps/$SAVE_MAP.yaml ]] && { echo "[demo] maps/$SAVE_MAP.yaml exists: another run? remove it first"; exit 1; }
    echo "[demo] saving the SLAM map as maps/$SAVE_MAP"
    docker run -d --rm --name kachaka-demo-slam --network host --ipc host \
        -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
        kachaka-isaac-sim-ros:jazzy ros2 launch kachaka_sim_bringup slam.launch.py rviz:=false > /dev/null
    uv run --no-project --with kachaka-api==3.17.5.0 --with numpy --with pyyaml --with pillow \
        python tests/demo_notebooks_check.py --save-map "$SAVE_MAP"
    docker rm -f kachaka-demo-slam > /dev/null
fi

echo "[demo] starting SLAM, Nav2 and RViz (Xvfb :$DISPLAY_NUM)"
docker run -d --rm --name kachaka-demo-ros --network host --ipc host \
    -e FASTDDS_BUILTIN_TRANSPORTS="UDPv4?sockets_size=4194304" \
    -e DISPLAY=":$DISPLAY_NUM" -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$PWD/tests:/tests:ro" -v "$PWD/build:/build" -v "$PWD/maps:/maps" \
    kachaka-isaac-sim-ros:jazzy bash -c "
        Xvfb :$DISPLAY_NUM -screen 0 1440x900x24 -nolisten tcp -ac &
        sleep 2
        sed '$RVIZ_SED' /ws/install/kachaka_sim_bringup/share/kachaka_sim_bringup/config/demo.rviz > /tmp/demo.rviz
        if [[ '$LAUNCH' == slam.launch.py* ]]; then
            ros2 launch kachaka_sim_bringup $LAUNCH rviz_config:=/tmp/demo.rviz
        else
            ros2 run rviz2 rviz2 -d /tmp/demo.rviz --ros-args -p use_sim_time:=true &
            ros2 launch kachaka_sim_bringup $LAUNCH rviz:=false
        fi" > /dev/null
until [[ -S /tmp/.X11-unix/X$DISPLAY_NUM ]]; do sleep 1; done
sleep 25  # let Nav2 come up and RViz draw the first map

echo "[demo] recording"
if [[ -z "$BROWSER" || -n "$LEFT" ]]; then
    CAPTURE=rviz
    [[ -n "$LEFT" ]] && CAPTURE=left  # the browser is rviz.mp4 then
    date +%s.%N > "$OUT/${CAPTURE}_start.txt"
    ffmpeg -hide_banner -loglevel error -f x11grab -draw_mouse 0 -framerate 10 -video_size 1440x900 -i ":$DISPLAY_NUM" \
        -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT/$CAPTURE.mp4" &
    FFMPEG_PID=$!
fi
if [[ -n "$HOST_RUN" ]]; then
    $HOST_RUN | tee "$OUT/run.log"
else
    docker exec "${EXEC_ARGS[@]}" kachaka-demo-ros bash -c "source /ws/install/setup.bash && $RUN" | tee "$OUT/run.log"
fi
date +%s.%N > "$OUT/end.txt"
sleep 1
if [[ -n "${FFMPEG_PID:-}" ]]; then
    kill -INT "$FFMPEG_PID"; wait "$FFMPEG_PID" || true
    FFMPEG_PID=
fi

# Stop the simulator so the frame writer finishes the last PNGs.
pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill -INT
for _ in $(seq 60); do pgrep -f "^\S*/bin/python \S*run_sim.py" >/dev/null || break; sleep 1; done

echo "[demo] composing $GIF"
.venv/bin/python scripts/make_demo_gif.py "$OUT" "$GIF" --speed ${SPEED:-2} --rviz-crop ${RVIZ_CROP:-470 90 920 750} \
    --rviz-label "$LABEL" "${COMPOSE_ARGS[@]}"
