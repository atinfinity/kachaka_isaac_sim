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
  between robot and shelf when the docking pin rises), charger, `goal_pose`,
  `kachaka_command/execute` action.
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
| `lidar/scan` | LaserScan | RTX LiDAR, 240°, 400 pts, 10 Hz, 0.1–10 m *(est.)* |
| `front_camera/image_raw`, `camera_info` | Image, CameraInfo | 1280×720 *(est.)*, may be lowered for real-time |
| `back_camera/...` | same | |
| `tof_camera/image_raw`, `camera_info` | depth Image | 160×120, 0.01–5 m *(est.)* |
| `imu/imu` | Imu | |
| `odometry/odometry` | Odometry | ground truth + configurable noise; drives TF odom→base_footprint |
| `wheel_odometry/wheel_odometry` | Odometry | integrated from wheel joints (slip shows) |
| `sim/ground_truth` | Odometry | sim-only |
| `robot_info/battery_state`, `robot_info/version` | stubs | |
| `manual_control/set_enabled`, `auto_homing/set_enabled` | SetBool | stubs, always succeed |
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
