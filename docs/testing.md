# Tests

[日本語](ja/testing.md) · [README](../README.md)

## Without the simulator

These run in CI (`.github/workflows/ci.yml`) on every push and pull request:

```bash
uvx ruff@0.17.1 check                                   # lint
uv run --no-project --with pytest --with numpy --with pyyaml --with pillow python -m pytest
                                                        # tests/unit: config, drive, power, IMU, URDF, STL, maps
docker run --rm kachaka-isaac-sim-ros:jazzy bash -c \
    'colcon test --packages-select kachaka_sim_bringup && colcon test-result --verbose'
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/nav2_config_check.py
python3 scripts/build_usd.py --urdf-only                # build/kachaka.urdf, for the next one
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/api_fake_check.py
docker run --rm -v $PWD:/repo kachaka-isaac-sim-ros:jazzy python3 /repo/tests/speed_limit_check.py
uv run --no-project --with pyyaml python scripts/run_scenarios.py scenarios/fake_room.yaml --backend fake
```

The unit tests on the URDF need `scripts/fetch_assets.py` first.
`tests/nav2_config_check.py` starts slam.launch.py (with Nav2) and then
navigation.launch.py on the saved map, with `fake_robot` (a point robot in an
empty room, `ros2 run kachaka_sim_bringup fake_robot`) in place of the
simulator. It checks that every lifecycle manager comes up and that Nav2
goals succeed.

`tests/speed_limit_check.py` measures the top speeds of manual and autonomous
commands with and without a docked shelf in `scripts/fake_sim.py` (add
`--external` to check a running simulator started with `--shelf 0 0 0 --no-people`).

`tests/api_fake_check.py` runs `tests/api_check.py --no-cameras` (the
kachaka-api regression check: move_to_pose, rotate_in_place, return_home,
shelves, error codes, history and more) against `scripts/fake_sim.py`, with
SLAM, Nav2 and the command server as for the simulator. `fake_sim.py` runs the
simulator's own ROS node and gRPC server, and replaces Isaac Sim with a
kinematic robot in a 2D copy of the default room: the LiDAR sees its walls,
boxes, people and shelf legs, the charger and the shelves work, and the robot
stops at whatever it would run into. Add `-e FAKE_SIM_ARGS=--trace` to log the
robot's pose and command once a second (in `fake_sim.log` under `CHECK_LOG_DIR`). It has no physics and no cameras, so
these still need the simulator: camera images and object detection, the
cliff and collision detection, how the robot moves under physics (wheel slip,
bumping into things, pushing a shelf), Isaac scenes, and switching to a saved
map (`tests/api_map_check.py`, `navigation.launch.py`).

CI has no GPU, so the simulator itself is checked locally:
`scripts/smoke_test.sh --nav` starts it headless and runs
`tests/ros_smoke_check.py`, then with SLAM + Nav2 the navigation, docking,
shelf, command and object detection checks one after another in the same simulator run (each
starts with `sim/reset`). A second run with a wall and a door runs `tests/ros_door_check.py`
(Nav2 through the door, round it while it is closed, and through it again once it opens).
A third run with `layouts/room_auto_door.yaml` runs `tests/ros_auto_door_check.py` (Nav2
through an automatic door that is never commanded: it closes behind the robot and opens as
the robot comes back to it). A fourth run with `layouts/room_movers.yaml` runs `tests/ros_nav_check.py --no-collisions`
(the Nav2 goals among people and a cart on the move, without touching them).
It ends with `tests/ros_hazard_check.py` in a fifth run with a
drop, and logs the collisions seen during the Nav2 checks to `build/smoke_collisions.log`.

## With the simulator

With the simulator running, from the ROS 2 container
(`docker compose -f docker/compose.yaml run --rm -v $PWD/tests:/tests shell`):

```bash
python3 /tests/ros_teleop_check.py   # drive the robot and print odometry
python3 /tests/ros_sensor_check.py   # topic rates, LiDAR ranges against the room walls, IMU
python3 /tests/ros_camera_check.py /tmp/images
python3 /tests/ros_nav_check.py      # Nav2 goals (needs slam.launch.py nav:=true)
python3 /tests/ros_nav_check.py --world office   # route in an Isaac scene (sim started with the same --world)
python3 /tests/ros_dock_check.py     # charger, battery and RETURN_HOME (same)
python3 /tests/ros_shelf_check.py    # shelf docking, towing and placing (same)
python3 /tests/ros_command_check.py  # MOVE_TO_POSE, MOVE_TO_LOCATION, SPEAK, goal_pose, cancel_all (same)
python3 /tests/ros_detection_check.py   # object_detection/result while turning in place (same)
python3 /tests/ros_hazard_check.py   # steps and collisions (no Nav2; sim started with --drop 1.0 0.0 0.4 0.8)
python3 /tests/ros_degradation_check.py   # degraded sensors (sim started with --config config/degraded.yaml --person 0.0 1.0 0.0)
python3 /tests/ros_door_check.py     # Nav2 and a door (sim started with --layout layouts/room_door.yaml)
python3 /tests/ros_auto_door_check.py   # Nav2 and an automatic door (sim started with --layout layouts/room_auto_door.yaml)
python3 /tests/ros_nav_check.py --no-collisions   # Nav2 goals among moving people (sim started with --layout layouts/room_movers.yaml)
```

The kachaka-api check runs on the host with the official client (same setup):

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_check.py
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_aio_check.py  # asyncio client and its streams
```

`tests/api_map_check.py` checks `switch_map` and `set_robot_pose` on a saved
map; its docstring gives the steps (save the SLAM map, then restart the ROS 2
side with `navigation.launch.py`).

`scripts/record_demo.sh` records the GIFs in `docs/images/` (`scripts/record_demo.sh dock` for the charger, `shelf` for the shelf, `front_stop` for the front obstacle stop, `notebooks sensors|control|map` for the demo notebooks in JupyterLab, with Chromium on the host); `scripts/record_torch_demo.sh docs/images/torch.gif` records the torch lights; `scripts/record_dashboard_demo.py docs/images/dashboard.gif` records the web dashboard (with the simulator, Nav2 and the dashboard running, and Chromium on the host), and with `--scenario reset` it records `docs/images/sim_reset.gif`. `scripts/detection_demo.py --out docs/images/object_detection.gif` records object detection (kachaka-api only).
