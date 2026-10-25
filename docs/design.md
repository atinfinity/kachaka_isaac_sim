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
- **Phase 2** (done): `goal_pose` and the `kachaka_command/execute` commands,
  charger, battery, RETURN_HOME and shelf docking/carrying.
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
- Added after import: three low-friction caster spheres (two front on the floor,
  one rear 6 mm above it so the rigid body can tilt over 4–5 mm door sills),
  sensors, physics params.
  Drive wheels use sphere colliders (smooth rolling on any floor mesh).
- Wheels are articulation joints driven by velocity; torque limited to a
  realistic value (Kachaka Pro motor max 2 Nm).
- Velocity limits: manual cmd_vel 0.3 m/s, 1.57 rad/s (real bridge full
  scale); autonomous motion (`sim/autonomous/cmd_vel`) 0.8 m/s, the spec top
  speed *(est.)*; both at most 0.5 m/s while towing a shelf, the spec top speed
  with a shelf *(est.)*. Accel 2 m/s², 3 rad/s² *(est.)*. See Speed limits below.

## ROS 2 interface (namespace `kachaka`)
Same names, types and QoS as `kachaka_grpc_ros2_bridge`; optional remap to
generic names (`/cmd_vel`, `/scan`, `/odom`).

| Topic | Type | Notes |
|---|---|---|
| `manual_control/cmd_vel` | Twist (sub) | clamped to the manual limits |
| `sim/autonomous/cmd_vel` | Twist (sub) | sim-only, clamped to the autonomous limits, see Speed limits below |
| `lidar/scan` | LaserScan | PhysX raycasts, 240°, 400 pts, 10 Hz, 0.1–10 m *(est.)* |
| `front_camera/image_raw`, `camera_info` | Image, CameraInfo | 640×480, 10 Hz (lowered from 1280×720 for real-time) |
| `back_camera/...` | same | |
| `tof_camera/image_raw`, `camera_info` | depth Image | 160×120, 0.01–5 m *(est.)*, 16UC1 mm (real format unknown) |
| `imu/imu` | Imu | |
| `odometry/odometry` | Odometry | ground truth + configurable noise; drives TF odom→base_footprint |
| `wheel_odometry/wheel_odometry` | Odometry | integrated from wheel joints (slip shows) |
| `sim/ground_truth` | Odometry | sim-only |
| `robot_info/battery_state` | BatteryState | SensorDataQoS, 1 Hz, CHARGING or DISCHARGING; voltage, current, charge, capacity, temperature (the real bridge: voltage −1 only, `battery.bridge_fields_only`) |
| `robot_info/version` | stub | |
| `manual_control/set_enabled` | SetBool | stub, always succeeds |
| `sim/charger_pose`, `sim/charger_detection` | PoseStamped | sim-only, see Charger below |
| `sim/layout`, `sim/shelf_poses`, `sim/shelf_detection`, `sim/shelf_docked`, `sim/shelf_dock` (srv) | | sim-only, see Kachaka Shelf below |
| `sim/front_stop/state`, `sim/front_stop/set_enabled` (srv), `sim/front_stop/mute` (srv) | | sim-only, see Front obstacle stop below |
| `sim/reset` (srv), `sim/reset/event` | SetBool, String | sim-only, see Reset below |

`kachaka_sim_bringup`'s `command_server` adds the parts the real robot runs
itself: `kachaka_command/execute` (all command types, see Commands below), `goal_pose`,
`layout/locations/list` (charger id `home`, shelf homes `<shelf>_home`),
`layout/shelves/list` and `auto_homing/set_enabled`.
| `robot_description`, `joint_states` | | |

### Speed limits

The real robot has two speed regimes: its bridge scales manual cmd_vel to
0.3 m/s and 1.57 rad/s, while its own autonomous motion reaches the spec top
speed, 0.8 m/s alone and 0.5 m/s with a shelf. In the simulator, Nav2's
commands reach the robot through `nav_bridge`, so on `manual_control/cmd_vel`
they could not be told apart from manual ones.

- **Two inputs.** `ros_node` also subscribes the simulator-only
  `sim/autonomous/cmd_vel` and hands the sim loop `(v, w, autonomous)`.
  `VelocityCommand.set_target(..., autonomous=...)` clamps the linear speed
  with `DriveLimits.linear_cap(autonomous, towing)`: `max_linear_vel` (0.3)
  for manual commands, `max_linear_vel_auto` (0.8 *(est.)*) for autonomous
  ones, and at most `max_linear_vel_towing` (0.5 *(est.)*) for both while the
  limiter's `towing` flag is set (`set_shelf_docked` in `robot.py` and
  `fake_sim.py`). The angular cap and the accelerations stay the same.
- **Who uses which.** `nav_bridge` forwards Nav2's `/cmd_vel` to
  `sim/autonomous/cmd_vel` when `use_sim_time` is true (`bridge.launch.py`
  sets its `cmd_vel_topic`), and to `manual_control/cmd_vel` for a real robot.
  `command_server` (simulator only) drives MOVE_FORWARD, ROTATE_IN_PLACE,
  undocking and the like on `sim/autonomous/cmd_vel`. The gRPC
  `SetRobotVelocity`, teleop and anything else on `manual_control/cmd_vel`
  stay at 0.3 m/s.
- **Nav2 while towing.** MPPI's `vx_max` and the smoother's maximum are 0.8
  m/s. On `sim/shelf_docked`, and every second since the controller server
  subscribes volatile, `command_server` publishes `nav2_msgs/SpeedLimit` on
  `speed_limit`: 0.5 m/s absolute while towing, 0 (no limit) otherwise. MPPI
  scales `vx_max` and `wz_max` by the same ratio, so Nav2 also turns slower
  with a shelf (0.98 rad/s); the docking servers have their own slow speeds.
- **Wheel joint limit.** The official URDF limits the drive wheels to 10
  rad/s, 0.45 m/s at the 0.045 m radius, which would cap the physics below
  0.5 and 0.8 m/s. `_configure_drives` in `robot.py` raises the wheels' max
  joint velocity to 1.2 x (the top linear cap + full-rate turning) / radius.
- **Unknowns.** Whether the real robot lowers its manual cap while towing is
  not published (the towing cap is above 0.3 m/s anyway), and all autonomous
  values are estimates until measured (#4).

### Front obstacle stop

The real robot stops by itself in front of an obstacle (an app setting turns
it off). Every velocity source ends on `manual_control/cmd_vel` or
`sim/autonomous/cmd_vel`, so the
simulator applies the stop where it limits the velocity (`robot.py`,
`safety.py`):

- At each LiDAR scan, `front_clearance` takes the exact raycast distances (no
  noise, no `min_range` cut-off, since on the real robot the ToF camera
  covers the LiDAR's blind range) and finds the nearest point in a lane ahead
  of the body (`front_x` 0.23 m, `half_width` 0.15 m).
- Each physics step, forward motion brakes (`VelocityCommand.update(...,
  forward_blocked=True)`, at `max_linear_accel`) while the clearance is at
  most `margin + v * scan period + v^2 / (2 * max_linear_accel)`. It is
  released once the clearance grows by 3 cm more, so the robot does not
  creep up in starts and stops. The command is kept, so the robot drives on
  when the way clears; backing up and turning are never blocked.
- The lane (+-0.15 m) is inside a shelf's legs (inner faces at +-0.172 m),
  so docking under a shelf works without an exemption; the towed shelf is
  ignored by the LiDAR already. On the charger with `sensors_off`, the
  clearance is treated as free.
- `command_server` mutes it (`sim/front_stop/mute`) for
  MOVE_BY_VELOCITY_WITH_MUTED_SENSORS and MOVE_FORWARD with `mute_sensors`,
  and fails a drive with −9 after the stop holds it for 1 s.

### Torch lights

`torches.py` puts a USD SphereLight (radius 1 cm) with a cone (`ShapingAPI`)
at `camera_front` and `camera_back`, turned like the USD cameras so it shines
along the optical axis. This placement is an assumption: the real torches'
position is not documented and the URDF has no torch frame. Product photos
show a small unlabelled round window a few cm beside the front camera, which
may be the torch; a few cm makes no visible difference here. The lights cast no
shadows, otherwise the body shell in front of the lenses would block them.
`SetFrontTorchIntensity` / `SetBackTorchIntensity` arrive on gRPC threads, so
`KachakaSimNode.set_torch` changes the intensity in the simulation loop
(`run_in_sim`) and publishes the levels on `sim/torch`. Level 255 maps to
`torch.max_intensity`, linearly; brightness, cone and colour are estimates.

### Reset

Tests used to restart the simulator to get back to the start state, which
takes about a minute. `sim/reset` (SetBool; `data` = also clear the SLAM map)
does it in place. The service callback runs in the simulation loop between
steps, so it changes the stage directly:

- `SimRobot.reset` lets go of a towed shelf, sends every shelf home
  (`Shelves.reset`), teleports the articulation to the spawn pose with zero
  root and joint velocities, zeroes the velocity limiter and ignores
  `cmd_vel` for 1 s (Nav2's controller may still be sending), resets the
  wheel odometry, the battery and the front stop.
- The fused odometry (the one SLAM uses) keeps its drift: the teleport is
  added to it as an exact move, with no new drift. SLAM's map -> odom
  already corrects that drift, and the robot then stands still, so SLAM has
  nothing to correct a change with. Zeroing the drift instead left the map
  pose off by a few cm for good, enough to fail docking (measured). With
  `data: true` the fused odometry restarts at the spawn pose with no drift,
  matching slam_toolbox's new map.
- The node releases the emergency stop, turns the torches off and publishes
  `sim/reset/event` ({"id", "clear_map"}); the gRPC server hears it through
  `tap("reset")`.
- `command_server` on the event: bumps `request_id` (stops the running and
  queued commands) and `reset_epoch` (their `run()` then publishes no
  result), publishes `sim/command/state` with `running`/`last` cleared and
  `"reset": id`, clears the costmaps, and with clear_map calls
  `slam_toolbox/reset`.
- The gRPC server clears history, last result, errors, sounds, volume and
  auto homing. It clears history again when `sim/command/state` first
  carries the new reset id, which drops a result that command_server sent
  before it heard of the reset. Under AMCL (`navigation.launch.py`) it
  publishes `/initialpose` at the last map -> odom, where the robot now is.

No gRPC call: kachaka-api has none for this, and a test can call the ROS
service from the ROS side.

TF follows the real robot: URDF frames plus `laser_link`, `camera_front`,
`camera_back`, `tof_camera` (placed at the URDF sensor poses until real values
are known).

## World
Isaac Sim bundled indoor assets plus Kachaka Shelf models from
`GAI-313/kachaka_shelf_description` (MIT) that the robot can dock with and carry.

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
  relays it (`nav_bridge`), together with `/cmd_vel` → `sim/autonomous/cmd_vel`
  (`manual_control/cmd_vel` with `use_sim_time:=false`, for a real robot).
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
  `base_footprint` is within 4 cm and 0.15 rad of the docked pose.
- **Battery** (`power.py`), updated each second from the mean power since the
  last update. Off the charger: idle power + losses proportional to the wheel
  rim speed (×`towing_factor` with a shelf) + the wheels' mechanical power
  over the motor efficiency. The torque is the velocity drive's own,
  `damping × (target − speed)` capped at `wheel_max_torque`, computed rather
  than read back from PhysX. `fake_sim.py` has no torques, only the first two
  parts. On the charger: constant current up to `cv_level` (0.8), then a rate
  proportional to what is left, with a floor so it reaches 1.0. Voltage from a
  level-to-voltage table; current = power / voltage. All values *(est.)*.
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

## Commands
- **Moving.** MOVE_TO_POSE is a Nav2 `navigate_to_pose` goal in the map
  frame. MOVE_TO_LOCATION uses the location's pose; for the charger it runs
  RETURN_HOME, since driving forward onto the charger would hit it. A towed
  shelf comes along with its footprint.
- **SPEAK** has no audio. The text and `tts_on_success` go to the log and the
  simulator-only `sim/speech` topic, so tests can check them.
- **Queue.** As on the real robot (kachaka-api docs), `cancel_all` stops the
  running command and drops queued ones; without it the command waits for the
  earlier ones. The ROS action leaves `cancel_all` false by default, and
  `goal_pose` sends MOVE_TO_POSE that way, as the real bridge does.
- **Canceling** a Nav2 or docking goal waits until the goal has ended. Nav2
  servers take a new goal as a preemption, and a late cancel request then
  stopped the next command's goal.
- Error −10: navigation failed.

## Kachaka Shelf
- **Model.** The 3-layer shelf mesh from `GAI-313/kachaka_shelf_description`
  as a rigid body (6 kg *(est.)*) on four low-friction caster spheres. Its pose
  is the `base_footprint` pose of a robot docked under it.
- **Docking.** `sim/shelf_dock` raises the pin when the robot is within 5 cm
  and 0.2 rad of that pose (either direction along the shelf's x axis) and
  joins robot and shelf with a fixed joint; lowering the pin removes it. While
  towing, the speed is capped at `drive.max_linear_vel_towing` (0.5 m/s, see
  Speed limits below) and the LiDAR ignores the towed shelf.
- **LiDAR sees the legs only.** The scan plane (z 0.105 m) lies just under the
  bottom plate, so its collider starts at 0.107 m. Otherwise the shelf showed
  up in the map as a closed box and Nav2 could not plan out from under it.
- **Forward docking with a second docking server.** Jazzy's `opennav_docking`
  takes the docking direction per server, so `shelf_docking.launch.py` runs
  `shelf_docking_server` (`dock_backwards: false`, SimpleNonChargingDock with
  external detection) next to the charger's. Staging is 0.9 m in front of the
  docked pose. The server aims 0.25 m past the dock pose and checks collisions
  up to that point, so `dock_collision_threshold` is 0.85 m: far enough that
  the shelf's own legs do not count.
- **Side.** A shelf can be entered from either end. `command_server` picks the
  end facing the robot, switches to the other one if the staging pose is in
  an occupied or unknown costmap cell, and retries the other end when staging fails.
- **Detection.** As with the charger, the simulator publishes shelf poses
  relative to the robot (`sim/shelf_detection`) and `command_server` relays the
  chosen one to `shelf_detected_dock_pose`. To put a shelf down at a location,
  it relays the location's pose (from SLAM) instead, so placement is as good
  as SLAM: 3–10 cm in the test room.
- **Footprint.** On `sim/shelf_docked`, `command_server` publishes the shelf's
  footprint to both costmaps' `footprint` topics (and back on undock) and
  clears the costmaps.
- **Shelf legs in the map.** slam_toolbox mapped the legs while the shelf
  stood still, and they stay in `/map` after the robot docks under it. With
  the towing footprint the inflation around them covered the robot's own cell,
  so NavFn found no path until SLAM cleared them (up to a minute).
  `footprint_map_filter` relays `/map` to `/nav_map` with the cells under the
  robot's current footprint (plus 5 cm) set free, and the global costmap's
  static layer reads `/nav_map`. A robot is never inside an obstacle, so this
  holds without a shelf too.
- **Undocking** lowers the pin and drives 0.50 m forward at 0.15 m/s on
  `sim/autonomous/cmd_vel`. It refuses (error −9) when the LiDAR sees
  something in that path, since the robot does not reverse. At 0.40 m the
  robot's front corners (0.28 m from its centre) hit the shelf's legs when
  it then turned around, and it stayed stuck there (#39).
- **Parked shelves in the costmaps.** The LiDAR sees only the four legs, and
  the robot fits between them, so Nav2 planned straight through a parked
  shelf and the robot ran into its legs. `command_server` publishes the
  outlines of the shelves not on the robot on `/parked_shelves`
  (PointCloud2, map frame, 2 Hz), a marking-only source for both costmaps.
  The shelf being docked to stays in: the docking controller skips its
  collision checks near it. A towed shelf's old outline is cleared with the
  costmaps when it is docked, or by the LiDAR's rays later.
- **Loop closures under a shelf.** slam_toolbox's `loop_search_space_dimension`
  is 3 m, so a loop closure moves the pose by at most 1.5 m. At the default
  8 m, a scan taken while turning on the spot with a shelf just docked
  matched the wrong end of the 5 x 4 m room and threw the map pose 3.7 m
  off (#39).
- **Squaring up.** The docking controller stops on position alone and put
  shelves down up to ~0.13 rad off the location's heading. MOVE_SHELF and
  RETURN_SHELF turn on the spot onto the location's axis (either direction)
  before lowering the pin when the robot is more than 0.02 rad off.
- **RViz.** `robot_description` stays the official URDF without the shelf, as
  on the real robot. `shelf_markers` draws the shelves as a MarkerArray on
  `/shelf_markers`: standing shelves in the map frame from
  `layout/shelves/list`, the towed one in `base_footprint` from
  `sim/shelf_detection`, so it moves with the robot.
- **Not modelled.** The real robot finds shelves with its cameras and shows
  each shelf's speed mode; here detection is handed over by the simulator and
  speed mode is always the default.

## kachaka-api gRPC server
- **In the simulator process.** `kachaka_isaac_sim/api_server.py` serves the
  `KachakaApi` service from the pip package's generated stubs (kachaka-api
  3.17.5.0, grpcio 1.66.1) on `--api-port` (26400). The ROS node hands each
  message it publishes to the server (`KachakaSimNode.tap`), which keeps the
  latest one and converts it to the proto type on request, so sensor RPCs
  need no ROS round trip.
- **Cursors.** Each stream keeps its latest value with a cursor (monotonic
  ns). A request whose cursor equals the latest one waits for the next
  value, as on the real robot; the client's `start_command` relies on this.
- **Commands go through `command_server`.** Isaac Sim's bundled rclpy cannot
  load the `kachaka_interfaces` types, so the server cannot call
  `kachaka_command/execute`. It publishes JSON on `sim/command/start` (with a
  fresh command id) and `sim/command/cancel` instead. `command_server` sends
  the command to its own action under that id as the goal UUID, so queueing,
  `cancel_all` and `tts_on_success` behave as for ROS clients. It publishes the
  running command and the last result of every command (ROS, gRPC, auto
  homing) on the latched `sim/command/state`, which backs `GetCommandState`
  and `GetLastCommandResult`. Without `command_server`, `StartCommand` fails
  with −6.
- **Pose and map.** `GetRobotPose` composes map→odom (slam_toolbox or AMCL,
  from `/tf`) with odometry; `GetPngMap` converts `/map` to the robot's PNG
  format (occupied 174, free 253, unknown 234, top row = max y).
- **Saved maps** (`kachaka_isaac_sim/maps.py`) are Nav2 yaml + pgm pairs in
  `maps/`, so `map_saver_cli` output and the maps the API writes are the same
  files. Under SLAM the only current map is `sim`, the live `/map`. The server
  tells the modes apart by map_server's parameter service: with it
  (`navigation.launch.py`), `GetCurrentMapId` is the launch map until
  `SwitchMap`. `SwitchMap` publishes `sim/map/load`; `command_server` calls
  map_server's `load_map`, gives AMCL the pose in the request (`"pose"`:
  `initial_pose` or the map's charger pose) on `/initialpose` until AMCL
  reports it (position and heading) and stays quiet for 1.5 s, clears the
  costmaps and answers on `sim/map/loaded`. AMCL re-initialises when the new
  map reaches it, so an `/initialpose` published after the answer alone could
  be overtaken (seen: ~20° off after an elevator ride). `SetRobotPose`
  publishes `/initialpose` directly.
- **Locations** are the charger (`home`, the default), each shelf's home
  (`<id>_home`) and the layout's extra locations.
- **Manual control.** `SetRobotVelocity` takes values normalized to the speed
  limits and publishes on `manual_control/cmd_vel`; it fails with −12 while
  manual control is disabled, so the client enables it first, as on the robot.
- **Odometry-driven commands.** MOVE_FORWARD, ROTATE_IN_PLACE,
  MOVE_BY_VELOCITY_WITH_MUTED_SENSORS, LOCK and DEPART_FROM_CHARGER run in
  `command_server` on `cmd_vel` and odometry, without Nav2. MOVE_FORWARD
  checks the LiDAR 0.15 m ahead as it goes and stops with −9. The simulated
  robot follows a velocity command about 0.35 s late, so ROTATE_IN_PLACE
  slows down toward the end and stops when the turn left equals what the
  robot will still coast at its current rate; it lands within about 0.03 rad
  and returns once the robot stands still.
- **Shortcuts and history.** Shortcuts come from `config/robot.yaml`;
  `StartShortcutCommand` starts their commands one after another. The history
  is the last 100 results from `sim/command/state`, in memory.
- **Emergency stop.** `SetEmergencyStop` cancels every command (`{"all": true}`
  on `sim/command/cancel` also drops the queue), zeroes the wheel command in
  the sim node and refuses `StartCommand` (−14) until `RestartRobot`.
- **Shelf reset** runs in the simulation loop: `KachakaSimNode.run_in_sim`
  queues the call, which runs after the next `spin_once`, and the gRPC thread
  waits on its future.
- **Sounds** have no audio: `PlaySound` publishes the name on `sim/sound`.
- **Checked in CI without Isaac Sim.** `scripts/fake_sim.py` runs
  `KachakaSimNode` and this server with a kinematic robot in a 2D copy of the
  default room (`kachaka_isaac_sim/flat_world.py`: walls, boxes, people and
  shelf legs as boxes for the LiDAR and for collisions), and mirrors
  `SimRobot`'s limiter, front stop, odometry, IMU, battery, charger, shelf
  towing and reset. The robot follows the limited command; only the yaw rate
  lags (first order, 0.3 s, `--turn-lag`) so that `ROTATE_IN_PLACE` ends where
  it does in Isaac Sim (the command server stops early by `ROTATE_LAG`). It
  stops at obstacles instead of bumping into them; when it already overlaps
  one (a shelf reset onto it), it moves freely until clear, as if physics had
  pushed them apart. `--trace` logs the pose and the command once a second.
  `tests/api_fake_check.py` brings it up with SLAM, Nav2 and `command_server`
  and runs `tests/api_check.py --no-cameras`. Cameras, object detection, the
  hazard sensors, physics effects, Isaac scenes and saved-map switching are
  checked with the simulator only.

## Web dashboard

- **A separate process that only talks kachaka-api.** `scripts/dashboard.py`
  wraps a `KachakaApiClient` in a stdlib `ThreadingHTTPServer`
  (`kachaka_isaac_sim/dashboard.py`), so it needs no ROS, does not load the
  simulator, and works against a real robot. It is off unless started.
- **Polling, no websockets.** The page (`dashboard.html`, plain JS and a
  canvas) polls `/api/state` every 0.5 s, `/api/history` every 3 s and the map
  and layout every 10 s, since the SLAM map grows. The server caches the
  layout and the map between those refreshes.
- **Idle.** With no command the robot reports PENDING and an empty command;
  the dashboard shows that as "idle".
- **Commands** are sent with `wait_for_completion=False`, so a request returns
  at once and the page follows the command through the state. Only the
  commands the page has buttons for are accepted (`parse_command`); anything
  else is a 400.
- **No path.** kachaka-api has no RPC for the planned path, so the page draws
  the trail of poses it has seen.

## Object detection

The real robot runs a detector network on the front camera and reports
`ObjectDetection {label, roi, score, distance_median}` (kachaka-api) and
`/kachaka/object_detection/result` (ROS 2 bridge). The simulator has no
network; it reads the answer from the renderer.

- **Ground truth from Replicator.** The front camera's render product has two
  more annotators: `instance_id_segmentation` (which prim each pixel shows)
  and `distance_to_camera`. Each target (a person, a shelf, the charger) is a
  prim subtree; `detection.detect` maps instance ids to targets and works on
  the pixels of each. Only visible pixels count, so occlusion needs no extra
  work, and the boxes and distances match the image they are stamped with.
- **Noise and limits are guesses.** Range, minimum size, the score curve, box
  noise and the miss rate are in `config/robot.yaml` `object_detection`, all
  marked est.; the real values are not published.
- **Features are empty.** `GetObjectDetectionFeatures` returns the stamp and
  no features: there is no network whose embeddings to fake.
- **ROS 2 through a relay.** Isaac Sim's rclpy cannot load
  `kachaka_interfaces`, so the simulator publishes JSON on
  `sim/object_detection` and `object_detection_relay` (in `bridge.launch.py`)
  republishes it as `ObjectDetectionListStamped` with the real bridge's label
  numbers (PERSON 0, SHELF 1, CHARGER 2, DOOR 3; UNSPECIFIED is dropped, as
  the bridge does).
- **People.** Isaac Sim's characters (`Isaac/People/Characters`,
  `people.characters`, taken in turn; `world.add_person`) with one invisible
  box collider, so the PhysX LiDAR, Nav2 and the front stop see them as
  obstacles. Two stand in the room by default (`people.room`); they do not
  move. The characters' rest pose is a T-pose, and the bundled animations
  (`Isaac/People/Animations`) are for another rig, played through the
  animation graph's retargeting; so `world.add_actor` binds a one-frame
  SkelAnimation of the rest pose with the upper arms turned down. The
  characters face −y, so they are turned 90° to face along the pose's yaw.
  Without the asset server, people are capsule mannequins.

## Steps and collisions

Sources: the real error codes and texts are in kachaka-api
`docs/api/ERROR_HANDLING.md` (21051 "Kachaka is paused", 21052 "Step
detected", resumed with the power button). That the robot has two step
sensors at the front comes from secondary sources; their positions, height
and range are not published, so `cliff` in `config/robot.yaml` is assumed.
No bumper is published.

- **Step sensors** (`physics_sensors.CliffSensors`, `hazards.Cliff`): two
  downward PhysX raycasts from the base link every physics step, ignoring the
  robot and a towed shelf. A sensor sees a drop when no floor is within
  `height + threshold`. A floor-to-drop transition pauses the robot: the
  velocity limiter is zeroed at once (the wheels' torque limit does the
  braking), `{"all": true}` goes to `sim/command/cancel` (command_server and
  the API server drop their commands), `take_cmd` returns zero and the API
  refuses commands with 21051. After the power button, a sensor that still
  sees the drop only blocks forward motion (the same limiter input as the
  front obstacle stop), so the robot can back away without pausing again.
  At 0.3 m/s the casters stop about 1.5 cm before the edge.
- **Errors**: `KachakaSimNode.error_codes()` derives the list from the state
  (emergency stop, cliff pause, collision stop) and `publish_errors()` hands
  it to GetError and `sim/errors`, so SetEmergencyStop, RestartRobot, the
  power button and sim/reset cannot leave stale codes behind.
- **Collisions** (`physics_sensors.ContactSensor`, `hazards.CollisionCounter`):
  `PhysxContactReportAPI` on the base link (applied before play) and
  `subscribe_contact_report_events`. The wheels are separate links and are
  not reported; the casters are colliders of the base link and are filtered
  by collider path. The floor and a towed shelf never count. Events are
  queued in the callback and handled in `post_step`. A contact with an
  object counts when it starts 0.5 s after the last event with that object,
  so one push against a wall is one collision.
- **Drops** (`world.add_floor_with_drops`, `hazards.floor_tiles`): the floor
  is a set of static boxes around the holes, the ground plane lies
  `drop.depth` below. The boxes are triangle-mesh colliders: cooked convex
  hulls of 40 m × 20 m × 2 cm boxes came out a few millimetres short at the
  edges, and the step sensors saw a drop at the seam between two tiles.

## Doors

Issue #26. A door is only the moving panel; the doorway's walls are plain
static boxes (`--wall`, `world.add_wall`).

- **Panel** (`world.add_door`, `doors.Doors`): a kinematic rigid body with a
  box collider. `Doors.step` runs before each physics step and sets the
  panel's USD translate and orient ops; PhysX follows them as kinematic
  targets. `RigidPrim.set_world_poses` (tensor API) moved the collider but
  not the rendered panel, so it is not used. A kinematic panel is not
  stopped by the robot: a closing door pushes it (no safety edge, assumed).
- **Geometry** (`doors.panel_pose`): a sliding panel is 2 × 5 cm wider than
  the doorway, runs just off the wall's −y face and slides +x until it
  clears the doorway. A hinged panel is 1 cm narrower than the doorway and
  turns about its −x end by up to 90° into the −y side.
- **State** (`doors.DoorState`): `opening` goes from 0 (closed) to 1 (open)
  at `sliding_speed` / `hinged_speed` (`door` in `config/robot.yaml`). A
  `--door-cycle` follows the simulation time since the start or the last
  `sim/reset`, starting in the start state; a `set_open` call stops the cycle
  until the next reset. `sim/doors` is published when a state changes
  (open, closed, opening, closing), not on every step.
- **Nav2**: nothing door-specific. The panel is in the LiDAR scan, so the
  obstacle layer marks a closed door and clears it again once it is open
  (the scan raytraces through the doorway); the BT replans at 1 Hz. The
  static layer is another matter while mapping: slam_toolbox learns a door
  it sees closed, and with `occupancy_threshold` 0.1 a cell needs about
  nine scans through it for each one that hit it before it is free again
  (and it adds scans only while the robot moves). So
  `tests/ros_door_check.py` freezes the SLAM map
  (`slam_toolbox/pause_new_measurements`) after driving through the open
  door, as when navigating on a saved map with the door open. A door that
  was closed when the map was saved stays a wall for the planner.
- **Automatic doors** (#63, `doors.AutoSpec`, `DoorSpec.senses`): the
  sensor area is a rectangle in the doorway's frame, `|along| <= width / 2 +
  margin` and `|across| <= range`, on both sides at once. `Doors` gets the
  robot's ground-truth pose and the movers' poses as callables and checks
  them before each step; `DoorState.step` keeps the door's target open while
  the last sensing is less than `hold` s ago, so a door that is closing turns
  round as soon as something comes in. Only sliding doors (the usual kind),
  and not with a cycle. A `set_open` command overrides the sensor until
  `sim/reset`, as it stops a cycle. The sensor sees the true pose, not a
  scan: no blind spots, no reaction delay (assumed). Nav2 still sees a
  closed automatic door as an obstacle, so a goal behind the only doorway
  may not be planned until the door has opened (not tested); `tests/ros_auto_door_check.py`
  sends the robot up to the door first.
- Elevators: see [Elevator](#elevator).

## Elevator

Issue #48. `kachaka_isaac_sim/elevator.py`, `floor_maps.py`,
`scripts/make_maps.py`, `layouts/elevator.yaml`, `layouts/elevator_vertical.yaml`.

- **Floors side by side**: floor k is the room moved k × 10 m along x
  (`FLOOR_SPACING`) by default. A layout's `floors` entries are sub-layouts
  (walls, doors, boxes, people) moved to their floor; shelves, the charger
  and the spawn pose stay on the first floor. These side-by-side
  coordinates are the "logical" ones that layouts, `floor_maps`, `floor_of`
  and `ElevatorState` always use.
- **Stacked floors** (`elevator.stack: vertical`): the stage is built in
  logical coordinates, then each upper floor's groups (its room, look and
  car) and each wall, box and person on it (by `floor_of` of its place) get
  an extra translate op put first (`world.lift`, `stage_offset`): back to
  x of the first floor and `FLOOR_HEIGHT` (3 m, assumed) up per floor. Each
  upper floor has a slab (`add_slab`, exact triangle meshes, top at the
  floor) over the room and car with the car's inside left open
  (`slab_tiles`). Car walls are `FLOOR_HEIGHT` tall, so they line the shaft.
  A floor under another gets a ceiling light (`add_ceiling_light`, a rect
  light at `CEILING_LIGHT` x the dome light), as the slab keeps the dome
  light out.
  The car's floor is a kinematic box (`/World/Elevator/Platform`) that
  `Elevator` moves every pre-physics step to `ElevatorState.position`
  (smoothstep between floors while riding); it reaches 2 mm under the walls
  and slab (kinematic and static bodies do not collide), so the step
  sensors see no gap at the doorway. Friction carries the robot and any
  shelf; nothing is teleported, `odom` and the map frame need no shift. The
  robot's logical pose (`logical_pose`: floor from its height above the
  start, rounded) is what `Elevator`, `sim/elevator` and `to_map` use;
  `sim/elevator` adds `stack` and `robot_z`. Doors and movers (they set
  world poses) on upper floors are not lifted, so the layout rejects them;
  the car's yaw must be a multiple of 90° (`inside_rect`). Object detection
  does not see people on upper floors (its targets are logical poses).
  The recording camera follows the robot's height (`--record-follow`), and
  a far near clip cuts the floor above.
- **Car**: walls behind a doorway in a room wall, and a sliding door
  (`doors.Doors`, its own instance, not listed on `sim/doors`) on each
  floor. `ElevatorState` is plain Python: `go` sets the target, `step` closes
  the door, waits `ride` s per floor once it is closed, then reports the
  arrival and opens the door on the new floor. While closing, the door
  reopens as long as the robot's base_footprint is within `door_sensor` of
  the doorway (assumed door sensor).
- **The ride** (side by side) happens between physics steps (main loop, `take_ride`): the
  robot articulation and any shelf inside the car (and the towed shelf) are
  moved by the floor offset with their velocities kept. `NoisyOdometry.shift`
  moves its origin by the same amount, so `odom` does not jump; wheel
  odometry integrates the wheels and never sees it. `SimRobot.map_origin`
  moves too, so sim-only map poses stay in the current floor's map frame.
- **Maps**: SLAM's map frame is the spawn pose, so each floor's map frame is
  the spawn pose moved to that floor; a pose in the car is the same on every
  floor's map, and after a ride the next floor's map is loaded and AMCL is
  given the pose it had. `floor_map` rasterises the layout's static geometry
  (doors open, movers and people left out); the origin is snapped so that
  cell centres never sit on a wall face (otherwise thin walls came out one
  cell jagged).
- **Floor looks**: each floor's room gets a floor tint and its name in
  seven-segment strokes (`FLOOR_LOOKS`, `label_segments`), thin boxes
  without colliders, so the PhysX-raycast LiDAR and step sensors miss them.
- **ROS**: `sim/elevator/go` (String) and latched `sim/elevator` (JSON).
  kachaka-api has no elevator calls; on the real robot the building's system
  drives the ride (sources in the README).
- **Not done**: RViz markers for the car; people riding; more than one car.

## Layout files

Issue #25. `--layout FILE` (`kachaka_isaac_sim/layout.py`, examples in
`layouts/`) places the spawn pose, charger, shelves, locations, people,
boxes, walls, doors and drops on top of any world.

- **Plain Python**: the file is read and every pose is settled before Isaac
  Sim starts, so a bad file fails fast (`layout: ...`, exit 1) and the parser
  is unit-tested without the simulator (`tests/unit/test_layout.py`).
- **Defaults**: a key that is left out keeps what the world has without a
  layout; a key that is given replaces it (`shelves: []`, `charger: null`).
  The command-line options keep their meaning on top: the placement options
  (`--shelf`, `--charger`, `--person`, `--no-*`, `--x/--y/--yaw`) replace the
  layout's, the additive ones (`--wall`, `--door`, `--drop`) add to it.
- **Randomisation**: any pose number can be `[lo, hi]`. Fixed objects are
  placed first, then the randomised ones in file order, each drawn up to 200
  times until its footprint circle (half its outline's diagonal) keeps
  `clearance` from the objects already placed, the layout's walls and drops,
  and in the room from its walls and boxes. The seed comes from
  `--layout-seed`, then the file, then a random one, and is printed so a run
  can be repeated. Circles are coarse (a shelf's is wide) but simple; they
  cannot see the furniture of an Isaac Sim scene.
- **Overlap check**: so after `play` each object's box is put to a PhysX
  `overlap_box` query against the stage (excluding the object itself and the
  robot, and for the spawn pose the charger and shelves it starts on), and a
  hit prints `[kachaka] layout warning`. It warns rather than fails, since a
  person standing against a table is often what was meant.
- **Not done**: `fake_sim.py` (the tests without Isaac Sim) does not read
  layouts.
