# Kachaka Isaac Sim — design decisions

Agreed design for the simulator (2026-10-18). Numbers marked *(est.)* are not
published by Preferred Robotics; they come from the community Gazebo model
(CyberAgentAILab/kachaka_ros2_dev_kit) and live in `config/robot.yaml`.

## Goal
A drop-in replacement for a real Kachaka when developing and testing ROS 2
applications. Synthetic data generation is a secondary use.

## Scope
- **MVP**: differential drive, odometry/TF, LiDAR, front/back RGB cameras,
  ToF depth, IMU, and stubs (battery, version, `set_enabled` services,
  `robot_description`, `joint_states`). Kachaka Shelf placed as a static obstacle.
- **Phase 2**: shelf docking (shelf as a caster articulation, joint created
  between robot and shelf when the docking pin rises), `goal_pose`, the other
  `kachaka_command/execute` commands. Charger, battery and RETURN_HOME are done.
- One robot; namespace and frame prefix are parameters.

## Runtime
- Isaac Sim 6.1 in a project uv venv (`isaacsim[all,extscache]==6.1.0.0`).
- `scripts/run_sim.py` starts the sim, GUI or `--headless`.
- Isaac Sim publishes with its bundled ROS 2 Jazzy libraries. Nav2, slam_toolbox,
  RViz and teleop run in a `ros:jazzy` container (host network, Fast DDS,
  `/clock` + `use_sim_time`).

## Robot model
- Official URDF from `pf-robotics/kachaka-api` (Apache-2.0), pinned commit,
  fetched by `scripts/fetch_assets.py`, converted to USD by `scripts/build_usd.py`.
  Generated USD is not committed.
- Added after import: two low-friction caster spheres, sensors, physics params.
  Drive wheels use sphere colliders (smooth rolling on any floor mesh).
- Wheels are articulation joints driven by velocity; torque limited to a
  realistic value (Kachaka Pro motor max 2 Nm).
- Velocity limits: 0.3 m/s, 1.57 rad/s (real bridge full scale), configurable
  up to 0.8 m/s; 0.5 m/s when towing a shelf. Accel 2 m/s², 3 rad/s² *(est.)*.

## ROS 2 interface (namespace `kachaka`)
Same names, types and QoS as `kachaka_grpc_ros2_bridge`; optional remap to
generic names (`/cmd_vel`, `/scan`, `/odom`).

| Topic | Type | Notes |
|---|---|---|
| `manual_control/cmd_vel` | Twist (sub) | clamped to limits |
| `lidar/scan` | LaserScan | PhysX raycasts, 240°, 400 pts, 10 Hz, 0.1–10 m *(est.)* |
| `front_camera/image_raw`, `camera_info` | Image, CameraInfo | 640×480, 10 Hz (lowered from 1280×720 for real-time) |
| `back_camera/...` | same | |
| `tof_camera/image_raw`, `camera_info` | depth Image | 160×120, 0.01–5 m *(est.)*, 16UC1 mm (real format unknown) |
| `imu/imu` | Imu | |
| `odometry/odometry` | Odometry | ground truth + configurable noise; drives TF odom→base_footprint |
| `wheel_odometry/wheel_odometry` | Odometry | integrated from wheel joints (slip shows) |
| `sim/ground_truth` | Odometry | sim-only |
| `robot_info/battery_state` | BatteryState | SensorDataQoS, 1 Hz, voltage −1, CHARGING or DISCHARGING, like the real bridge |
| `robot_info/version` | stub | |
| `manual_control/set_enabled` | SetBool | stub, always succeeds |
| `sim/charger_pose`, `sim/charger_detection` | PoseStamped | sim-only, see Charger below |

`kachaka_sim_bringup`'s `command_server` adds the parts the real robot runs
itself: `kachaka_command/execute` (RETURN_HOME), `layout/locations/list`
(charger, id `home`) and `auto_homing/set_enabled`.
| `robot_description`, `joint_states` | | |

TF follows the real robot: URDF frames plus `laser_link`, `camera_front`,
`camera_back`, `tof_camera` (placed at the URDF sensor poses until real values
are known).

## World
Isaac Sim bundled indoor assets plus Kachaka Shelf models from
`GAI-313/kachaka_shelf_description` (MIT) as obstacles.

## Acceptance criteria (MVP)
1. Teleop drives the robot at the commanded speed.
2. RViz shows all sensors and TF with real-robot frame names.
3. slam_toolbox builds a map and Nav2 reaches a goal.
4. A single headless smoke-test script passes.
5. GUI run at real-time factor ≈ 1.0 on an RTX 2080 Ti.

## Implementation notes (deviations found while building)
- **LiDAR via PhysX raycasts, not RTX LiDAR.** The sensor sits inside the body
  mesh and RTX rays cannot ignore the robot's own geometry. Raycasts skip the
  robot's colliders and only see collision shapes, which suits a 2D scan.
- **Physics 120 Hz, render 10 Hz.** Control, odometry, LiDAR and IMU run in
  physics callbacks; rendering (cameras) runs at `--render-hz` (default 10,
  a multiple of the camera rate). `/persistent/simulation/minFrameRate` is set
  to 1 so each update runs all its physics substeps. Measured RTF on an
  RTX 2080 Ti, headless, 640×480 cameras: 1.45 at 10 Hz, 0.96 at 20 Hz,
  0.71 at 30 Hz.
- **QoS.** Odometry, wheel odometry, scan, IMU and cameras use SensorDataQoS
  like the real bridge. Nav2 expects reliable `/odom`, so `kachaka_sim_bringup`
  relays it (`nav_bridge`), together with `/cmd_vel` → `manual_control/cmd_vel`.
- **No reversing in Nav2.** The LiDAR covers 240° to the front and nothing
  senses range behind the robot, so MPPI `vx_min` and the velocity smoother's
  minimum x are 0, and the behavior trees drop the `BackUp` recovery.
  `tests/ros_nav_check.py` fails on any backward `cmd_vel`.
- **DDS buffers.** Receiving 640×480 RGB at 10 Hz needs larger socket buffers
  on the client: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304`.
- **ToF camera** points 45° upward, as in the official URDF.
- **Camera near clip** is short (2.5 cm RGB, 1 cm ToF, set in `config/robot.yaml`)
  so close objects stay visible; the lenses sit just behind the body shell.
- The default room has 2.4 m walls so the upward ToF sees them.
- **Callbacks drained each frame.** The simulator spins its ROS node once per
  render frame (10 Hz) but runs every ready callback, not just one. With one,
  `cmd_vel` queued up and the robot kept moving ~0.5 s after Nav2 stopped it.

## Charger, battery and RETURN_HOME
- **Dock model.** An L-shaped dark grey dock after the photos in Kachaka's
  support pages: a back panel (collider) with a green LED, a thin floor plate
  and two contacts (visual only). Sizes are estimates (`config/robot.yaml`).
- **Charging** is decided from ground truth: the robot charges when its
  `base_footprint` is within 4 cm and 0.15 rad of the docked pose. The battery
  is linear: 0.5/h drain, 1.0/h charge *(est.)*, updated each second.
- **Backward docking**, the one exception to "never reverse": the real robot
  backs onto its charger and drives forward to leave. Nav2's docking server
  (`opennav_docking`, `dock_backwards: true`) drives to a staging pose 0.9 m in
  front of the dock, then reverses slowly (≤ 0.1 m/s). Leaving is an ordinary
  forward Nav2 goal.
- **Detection.** The SLAM map drifts a few cm from the world, too much for the
  contacts. The simulator publishes the dock pose relative to the robot within
  1.5 m (`sim/charger_detection`); `command_server` relays it to the docking
  server's `detected_dock_pose` (reliable QoS, which the server requires). How
  the real robot finds its dock is not published.
- **Sensors on the charger.** The real robot stops its LiDAR and ToF while
  charging. `charger.sensors_off` does the same but is off by default:
  slam_toolbox stamps `map→odom` with the last scan, so without scans Nav2's
  TF lookups fail and it cannot drive off the charger. The real robot
  localizes itself and does not have this problem.
- **Auto homing** in `command_server`: after `auto_homing_idle_timeout` (300 s)
  without velocity commands, or below 15 % battery. Retries wait 60 s.
- Result error codes of `kachaka_command/execute` are the simulator's own
  (negative); the real ones are not published.
