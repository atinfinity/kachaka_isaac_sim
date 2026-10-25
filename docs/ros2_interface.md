# ROS 2 interface

[日本語](ja/ros2_interface.md) · [README](../README.md)

Namespace `/kachaka`, as on the real robot.

| Topic | Type | Rate |
|---|---|---|
| `manual_control/cmd_vel` | `geometry_msgs/Twist` (subscribed), ≤ 0.3 m/s | |
| `goal_pose` | `geometry_msgs/PoseStamped` (subscribed): MOVE_TO_POSE | |
| `odometry/odometry` | `nav_msgs/Odometry` | 20 Hz |
| `wheel_odometry/wheel_odometry` | `nav_msgs/Odometry` | 20 Hz |
| `lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| `imu/imu` | `sensor_msgs/Imu` | 100 Hz |
| `front_camera/image_raw`, `back_camera/image_raw` | `sensor_msgs/Image` (rgb8, 640×480) + `camera_info` | 10 Hz |
| `tof_camera/image_raw` | `sensor_msgs/Image` (16UC1, mm) + `camera_info` | 10 Hz |
| `object_detection/result` | `kachaka_interfaces/ObjectDetectionListStamped`, see [Object detection](robot.md#object-detection) (`object_detection_relay`) | 10 Hz |
| `joint_states`, `robot_description` | | |
| `robot_info/battery_state` | `sensor_msgs/BatteryState` (level, charging or discharging, voltage, current; see [Battery](robot.md#battery)) | 1 Hz |
| `robot_info/version` | stub | |
| `sim/ground_truth` | `nav_msgs/Odometry` (simulator only) | 20 Hz |
| `sim/charger_pose` | `geometry_msgs/PoseStamped` (simulator only, latched) | |
| `sim/charger_detection` | `geometry_msgs/PoseStamped`, charger relative to the robot within 1.5 m (simulator only) | 20 Hz |
| `sim/layout` | `std_msgs/String`, JSON with the shelves and extra locations (simulator only, latched) | |
| `sim/shelf_poses` | `geometry_msgs/PoseArray`, shelves in the map frame (simulator only) | 2 Hz |
| `sim/shelf_detection` | `geometry_msgs/PoseArray`, shelves relative to the robot (simulator only) | 20 Hz |
| `sim/shelf_docked` | `std_msgs/String`, id of the towed shelf or empty (simulator only, latched) | |
| `sim/shelf_dock` | `std_srvs/SetBool` service: raise or lower the pin (simulator only) | |
| `sim/autonomous/cmd_vel` | `geometry_msgs/Twist` (subscribed), autonomous motion from `nav_bridge` and `command_server`, ≤ 0.8 m/s (simulator only, see [Speed limits](robot.md#speed-limits)) | |
| `sim/speech` | `std_msgs/String`, text of SPEAK and `tts_on_success` (simulator only) | |
| `sim/front_stop/state` | `std_msgs/String`, JSON `{"enabled", "muted", "blocked"}` of the front obstacle stop (simulator only, latched, on change) | |
| `sim/front_stop/set_enabled`, `sim/front_stop/mute` | `std_srvs/SetBool` services: switch the front obstacle stop (the app setting), or mute it while a command runs (`command_server`) (simulator only) | |
| `sim/cliff` | `std_msgs/String`, JSON `{"enabled", "paused", "over"}` of the step sensors (simulator only, latched, on change) | |
| `sim/collision` | `std_msgs/String`, JSON `{"stamp", "count", "with", "pose"}` for each collision of the body (simulator only) | |
| `sim/errors` | `std_msgs/String`, JSON list of the error codes `GetError` lists (simulator only, latched, on change) | |
| `sim/power_button` | `std_srvs/Trigger` service: resume from a pause after a step or a collision (simulator only) | |
| `sim/object_detection` | `std_msgs/String`, JSON detections of each front camera image (simulator only) | 10 Hz |
| `sim/torch` | `std_msgs/String`, JSON `{"front", "back"}`: torch levels 0-255 (simulator only, latched) | |
| `sim/reset` | `std_srvs/SetBool` service: back to the start state, `data: true` also clears the SLAM map (simulator only, see [Resetting the simulator](robot.md#resetting-the-simulator)) | |
| `sim/reset/event` | `std_msgs/String`, JSON `{"id", "clear_map"}` after each reset (simulator only) | |
| `sim/movers` | `std_msgs/String`, JSON `{"stamp", "frame_id", "movers": [...], "closest"}`: the moving people and boxes' ground truth (simulator only, 20 Hz, see [Moving people and boxes](worlds.md#moving-people-and-boxes)) | |
| `sim/doors` | `std_msgs/String`, JSON `{"doors": [...]}`: each door's state (simulator only, latched, on change and every 0.1 s while a panel moves, see [Doors](worlds.md#doors)) | |
| `sim/doors/<id>/set_open` | `std_srvs/SetBool` service: open (`true`) or close a door (simulator only) | |
| `sim/command/start`, `sim/command/cancel` | `std_msgs/String`, JSON: commands from the kachaka-api server to `command_server` (simulator only) | |
| `sim/command/state` | `std_msgs/String`, JSON: running command and last result (simulator only, latched) | |

`kachaka_sim_bringup`'s `command_server` provides what the real robot does
itself: the `kachaka_command/execute` action, `goal_pose`,
`layout/locations/list` and `layout/shelves/list`, and the
`auto_homing/set_enabled` service. Do not run it with a real Kachaka
(`command_server:=false`).

`/clock` and `/tf` are published at the top level; use `use_sim_time:=true` on
the ROS 2 side. See [docs/design.md](design.md) for the details and the
decisions behind them.

## Using the ROS 2 package with a real Kachaka

`ros2_ws/src/kachaka_sim_bringup` only depends on the `/kachaka` topics, so it
should also work with a real robot (`use_sim_time:=false`, `command_server:=false`). This has not been
tested. `nav_bridge` then sends Nav2's commands to `manual_control/cmd_vel`,
which the real bridge caps at 0.3 m/s: lower MPPI's `vx_max` and the velocity
smoother's `max_velocity` in `nav2_params.yaml` (0.8 m/s for the simulator) to match.
