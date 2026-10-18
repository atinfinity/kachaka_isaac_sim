# kachaka_isaac_sim

[日本語](README.ja.md)

A simulator for the [Kachaka](https://kachaka.life/) home robot on
NVIDIA Isaac Sim 6.1, with the same ROS 2 interface as the official
[kachaka-api](https://github.com/pf-robotics/kachaka-api) bridge. Code written
for a real Kachaka (topics under `/kachaka`) runs against the simulator unchanged.

![Nav2 driving the simulated Kachaka while slam_toolbox builds the map](docs/demo.gif)

*Left: Isaac Sim. Right: RViz with slam_toolbox and Nav2 (2× speed).*

## Features

- Robot model converted from the official URDF and meshes.
- Differential drive with the real speed limits (0.3 m/s, 1.57 rad/s), torque-limited wheels and casters.
- Sensors: 2D LiDAR, front/back RGB cameras, ToF depth camera, IMU.
- Odometry (fused and wheel), TF, `joint_states`, `robot_description`, version stub.
- Charging dock and battery: the battery drains while driving and charges on the dock.
- Same topic names, message types and QoS as the real bridge.
- Worlds: a walled room with a Kachaka Shelf, or Isaac Sim scenes (office, hospital, warehouse).
- Docker image with slam_toolbox, Nav2, RViz and teleop, set up for Kachaka.
  Nav2 never drives backwards, since Kachaka has no rear range sensor.
- `kachaka_command/execute` with every command type of the real bridge: MOVE_TO_POSE,
  MOVE_TO_LOCATION, RETURN_HOME, SPEAK (text only) and the Kachaka Shelf commands
  (DOCK_SHELF, UNDOCK_SHELF, MOVE_SHELF, RETURN_SHELF), built on Nav2. Also
  `goal_pose` and auto homing.
- The [kachaka-api](https://github.com/pf-robotics/kachaka-api) gRPC API on port 26400,
  so the official Python client (`KachakaApiClient`) works without a robot.

### Commands

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 13, move_to_pose_command_x: 1.0, move_to_pose_command_y: 0.5,
      move_to_pose_command_yaw: 1.57}, cancel_all: true}"   # MOVE_TO_POSE_COMMAND
ros2 topic pub --once /kachaka/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
```

| Command | What it does |
|---|---|
| MOVE_TO_POSE | Nav2 goal in the map frame |
| MOVE_TO_LOCATION | Nav2 goal to a location from `layout/locations/list`; the charger (`home`) runs RETURN_HOME |
| SPEAK | no audio: the text goes to the log and to `sim/speech` |

As on the real robot, `cancel_all: true` stops the running command; without
it a command waits until the earlier ones finish. `tts_on_success` is sent to
`sim/speech` after a successful command. `goal_pose` sends MOVE_TO_POSE
without `cancel_all`, like the real bridge.

### Returning to the charger

![RETURN_HOME: Nav2 drives to the charger and the robot backs onto it](docs/dock.gif)

*RETURN_HOME (2× speed). Nav2 drives to a point in front of the charger, then the robot backs onto it, as the real one does.*

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 8}}"   # RETURN_HOME_COMMAND
```

This needs the ROS 2 stack with Nav2 (`NAV=true ... up slam`, or `up navigation`),
which runs `command_server`. The robot leaves the charger with any Nav2 goal.
Battery rates and the charger's size are estimates (`config/robot.yaml`).

### Carrying a shelf

![The robot docks under the shelf, tows it to another location, turning on the way, puts it down and drives out forward](docs/shelf.gif)

*DOCK_SHELF, then MOVE_SHELF to another location and UNDOCK_SHELF (3× speed).*

The room has one Kachaka Shelf (`S01`, home location `S01_home`) and one other
location (`L01`). The robot drives in under a shelf forward, raises its pin and
tows the shelf on its casters (≤ 0.5 m/s, `drive.max_linear_vel_towing`; Nav2's
own limit of 0.3 m/s is lower). Nav2's footprint grows to the shelf's while towing.
Undocking lowers the pin and drives forward out of the shelf; it fails if
something blocks the way.

```bash
ros2 action send_goal /kachaka/kachaka_command/execute kachaka_interfaces/action/ExecKachakaCommand \
    "{kachaka_command: {command_type: 1, move_shelf_command_target_shelf_id: S01,
      move_shelf_command_destination_location_id: L01, move_shelf_command_undock_on_destination: true}}"   # MOVE_SHELF_COMMAND
```

| Command | What it does |
|---|---|
| DOCK_SHELF | docks with the shelf in front of the robot (within 1.5 m) |
| UNDOCK_SHELF | puts the towed shelf down and drives out forward |
| MOVE_SHELF | docks with the shelf if needed, tows it to a location, optionally undocks |
| RETURN_SHELF | MOVE_SHELF to the shelf's home location, then undocks |
| RETURN_HOME | returns a towed shelf home first, then docks with the charger |

Shelves are listed on `layout/shelves/list` and their homes on
`layout/locations/list`. Place shelves with `--shelf X Y YAW`. The shelf lands
where SLAM puts the location, a few centimetres off the world.
`robot_description` is the robot alone, as on the real Kachaka; RViz draws the
shelves from `/shelf_markers` (MarkerArray, published by `shelf_markers`).

## Requirements

- Ubuntu 24.04, x86_64
- NVIDIA RTX GPU and driver supported by Isaac Sim 6.1 (tested on an RTX 2080 Ti, driver 595)
- [uv](https://docs.astral.sh/uv/), Docker with Compose, git, and about 40 GB of disk

## Quick start

```bash
# 1. Install Isaac Sim 6.1 into .venv (large download)
uv sync

# 2. Fetch the official Kachaka description and build the robot USD
uv run python scripts/fetch_assets.py
uv run python scripts/build_usd.py

# 3. Start the simulator (GUI; add --headless for no window)
scripts/run_sim.sh
```

`scripts/run_sim.sh` sets `OMNI_KIT_ACCEPT_EULA=YES`. By running it you accept
the [NVIDIA Isaac Sim license](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html).
The first start compiles shaders and takes a few minutes.

In another terminal, run the ROS 2 tools in Docker:

```bash
docker compose -f docker/compose.yaml build
xhost +local:            # allow RViz windows from containers

# Drive with the keyboard
docker compose -f docker/compose.yaml run --rm teleop

# Build a map with slam_toolbox (add NAV=true to also send Nav2 goals from RViz)
NAV=true docker compose -f docker/compose.yaml up slam

# Save the map, then navigate on it
docker compose -f docker/compose.yaml run --rm shell \
    ros2 run nav2_map_server map_saver_cli -f /maps/map
docker compose -f docker/compose.yaml up navigation
```

The containers use host networking, so they find the simulator on the same
machine through DDS discovery.

## Simulator options

```bash
scripts/run_sim.sh --world office        # Isaac Sim scenes: office, hospital, warehouse, simple_room
scripts/run_sim.sh --world /path/to.usd  # any USD scene
scripts/run_sim.sh --x 1.0 --y 0.5 --yaw 1.57
scripts/run_sim.sh --no-cameras          # faster, when you only need LiDAR and odometry
scripts/run_sim.sh --render-hz 20        # render/camera loop rate (see Performance)
scripts/run_sim.sh --charger X Y YAW     # place the charger (default: by the west wall of the room)
scripts/run_sim.sh --no-charger
scripts/run_sim.sh --shelf X Y YAW       # place a shelf (repeatable; default: one in the room)
scripts/run_sim.sh --no-shelves
scripts/run_sim.sh --api-port 26400      # kachaka-api gRPC port (default 26400; 0 turns it off)
scripts/run_sim.sh --help
```

Robot parameters (speed limits, sensor resolution and rates, noise) are in
[`config/robot.yaml`](config/robot.yaml). Values not published by Preferred
Robotics are marked *(est.)*.

Isaac Sim scenes are streamed from NVIDIA's asset server. The first load of
the office scene takes several minutes.

## ROS 2 interface

Namespace `/kachaka`, as on the real robot.

| Topic | Type | Rate |
|---|---|---|
| `manual_control/cmd_vel` | `geometry_msgs/Twist` (subscribed) | |
| `goal_pose` | `geometry_msgs/PoseStamped` (subscribed): MOVE_TO_POSE | |
| `odometry/odometry` | `nav_msgs/Odometry` | 20 Hz |
| `wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | 20 Hz |
| `lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `imu/imu` | `sensor_msgs/Imu` | 100 Hz |
| `front_camera/image_raw`, `back_camera/image_raw` | `sensor_msgs/Image` (rgb8, 640×480) + `camera_info` | 10 Hz |
| `tof_camera/image_raw` | `sensor_msgs/Image` (16UC1, mm) + `camera_info` | 10 Hz |
| `joint_states`, `robot_description` | | |
| `robot_info/battery_state` | `sensor_msgs/BatteryState` (charging or discharging) | 1 Hz |
| `robot_info/version` | stub | |
| `sim/ground_truth` | `nav_msgs/Odometry` (simulator only) | 20 Hz |
| `sim/charger_pose` | `geometry_msgs/PoseStamped` (simulator only, latched) | |
| `sim/charger_detection` | `geometry_msgs/PoseStamped`, charger relative to the robot within 1.5 m (simulator only) | 20 Hz |
| `sim/layout` | `std_msgs/String`, JSON with the shelves and extra locations (simulator only, latched) | |
| `sim/shelf_poses` | `geometry_msgs/PoseArray`, shelves in the map frame (simulator only) | 2 Hz |
| `sim/shelf_detection` | `geometry_msgs/PoseArray`, shelves relative to the robot (simulator only) | 20 Hz |
| `sim/shelf_docked` | `std_msgs/String`, id of the towed shelf or empty (simulator only, latched) | |
| `sim/shelf_dock` | `std_srvs/SetBool` service: raise or lower the pin (simulator only) | |
| `sim/speech` | `std_msgs/String`, text of SPEAK and `tts_on_success` (simulator only) | |
| `sim/command/start`, `sim/command/cancel` | `std_msgs/String`, JSON: commands from the kachaka-api server to `command_server` (simulator only) | |
| `sim/command/state` | `std_msgs/String`, JSON: running command and last result (simulator only, latched) | |

`kachaka_sim_bringup`'s `command_server` provides what the real robot does
itself: the `kachaka_command/execute` action, `goal_pose`,
`layout/locations/list` and `layout/shelves/list`, and the
`auto_homing/set_enabled` service. Do not run it with a real Kachaka
(`command_server:=false`).

`/clock` and `/tf` are published at the top level; use `use_sim_time:=true` on
the ROS 2 side. See [docs/design.md](docs/design.md) for the details and the
decisions behind them.

### Using the ROS 2 package with a real Kachaka

`ros2_ws/src/kachaka_sim_bringup` only depends on the `/kachaka` topics, so it
should also work with a real robot (`use_sim_time:=false`, `command_server:=false`). This has not been
tested.

## kachaka-api (gRPC)

The simulator serves the [kachaka-api](https://github.com/pf-robotics/kachaka-api)
gRPC API on port 26400, like the real robot. Install the official client on
the host (no ROS needed) and connect to the simulator:

```bash
pip install kachaka-api
```

```python
from kachaka_api import KachakaApiClient

client = KachakaApiClient("127.0.0.1:26400")
print(client.get_robot_pose())
client.update_resolver()
client.speak("hello")
client.move_to_location("L01")
client.move_shelf("S01", "L01")
client.return_shelf("S01")
client.return_home()
```

Commands and the locations need the ROS 2 side (`slam.launch.py nav:=true`),
since `command_server` runs them. The robot pose is in slam_toolbox's map
frame and `get_png_map` returns its `/map`.

| Area | Supported |
|---|---|
| Robot info | serial number, version, `is_ready`, errors and error codes, pose, battery |
| Sensors | odometry, wheel odometry, IMU, LiDAR, front/back/ToF camera info, images and compressed images, static TF |
| Map | `get_png_map`, `get_map_list`, `get_current_map_id` (one map, `sim`) |
| Commands | MOVE_TO_POSE, MOVE_TO_LOCATION, RETURN_HOME, SPEAK, DOCK_SHELF, UNDOCK_SHELF, MOVE_SHELF, RETURN_SHELF; `cancel_command`, `get_command_state`, `get_last_command_result`, `proceed` |
| Layout | `get_locations`, `get_default_location_id`, `get_shelves`, `get_moving_shelf_id` |
| Control | `set_manual_control_enabled`, `set_robot_velocity`, `set_robot_stop`, auto homing, speaker volume |
| No-ops | torches, `activate_laser_scan`; `get_object_detection` is always empty |

The other RPCs return `UNIMPLEMENTED` and the other commands fail
([#12](https://github.com/atinfinity/kachaka_isaac_sim/issues/12)).

## Performance

Physics runs at 120 Hz. Rendering, which the cameras need, runs at
`--render-hz` (default 10 Hz). Measured real-time factor, headless, RTX 2080 Ti,
640×480 cameras:

| `--render-hz` | 10 | 20 | 30 |
|---|---|---|---|
| RTF | 1.45 | 0.96 | 0.71 |

The simulator is paced to real time (`--realtime 1.0`), so RTF above 1 means it
keeps up. Use `--no-cameras` when you do not need images.

### Camera images on the ROS 2 side

Fast DDS's default socket buffers drop large images. Subscribers need bigger
buffers:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304
```

`docker/compose.yaml` sets this already.

## Tests

With the simulator running, from the ROS 2 container
(`docker compose -f docker/compose.yaml run --rm -v $PWD/tests:/tests shell`):

```bash
python3 /tests/ros_teleop_check.py   # drive the robot and print odometry
python3 /tests/ros_sensor_check.py   # topic rates, LiDAR ranges against the room walls, IMU
python3 /tests/ros_camera_check.py /tmp/images
python3 /tests/ros_nav_check.py      # Nav2 goals (needs slam.launch.py nav:=true)
python3 /tests/ros_dock_check.py     # charger, battery and RETURN_HOME (same)
python3 /tests/ros_shelf_check.py    # shelf docking, towing and placing (same)
python3 /tests/ros_command_check.py  # MOVE_TO_POSE, MOVE_TO_LOCATION, SPEAK, goal_pose, cancel_all (same)
```

The kachaka-api check runs on the host with the official client (same setup):

```bash
uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_check.py
```

`scripts/record_demo.sh` records the GIFs above (`scripts/record_demo.sh dock` for the charger, `shelf` for the shelf).

## License

Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
This is an unofficial project, not affiliated with Preferred Robotics, Inc.,
the maker of Kachaka.

The robot model comes from [pf-robotics/kachaka-api](https://github.com/pf-robotics/kachaka-api)
(Apache-2.0), the shelf model from [GAI-313/kachaka_shelf_description](https://github.com/GAI-313/kachaka_shelf_description)
(MIT). Both are downloaded by `scripts/fetch_assets.py` and not redistributed here.
