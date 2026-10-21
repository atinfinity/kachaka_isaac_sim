"""ROS 2 side of the simulator: one rclpy node inside the Isaac Sim process.

Topic names, types and QoS follow the official kachaka-api ROS 2 bridge so a
ROS 2 app written for the real robot works unchanged. Everything is stamped
with simulation time, and /clock is published from here.
"""

from __future__ import annotations

import json
import math
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ShutdownException, SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

from .drive import Pose2D, quat_from_yaw
from .hazards import ERROR_COLLISION, ERROR_EMERGENCY_STOP, ERROR_PAUSED, ERROR_STEP_DETECTED
from .urdf_tools import read_joints

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def to_time(t: float) -> Time:
    sec = math.floor(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class KachakaSimNode:
    """Owns the rclpy node, publishers and the latest commands from ROS."""

    def __init__(self, cfg: SimpleNamespace, urdf_path: Path):
        rclpy.init()
        self.cfg = cfg
        self.node = Node("kachaka_isaac_sim")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        ns = "/" + cfg.ros.namespace.strip("/") if cfg.ros.namespace else ""
        self.ns = ns
        self.prefix = cfg.ros.frame_prefix or ""
        self.generic = bool(cfg.ros.remap_generic)
        self.manual_control_enabled = True
        self.cmd: tuple[float, float] | None = None  # newest Twist, consumed by the sim loop
        self.shelf_reset_handler = None  # shelf_id -> (ok, message); set when the world has shelves
        self.emergency_stop = False  # set by the gRPC SetEmergencyStop: velocity commands are ignored
        self.cliff = None  # hazards.Cliff, set by the sim
        self.collision_stopped = False  # collision.stop: a collision stopped the robot
        self._pending: list = []  # (function, Future) to run in the sim loop, see run_in_sim
        self._pending_lock = threading.Lock()
        # Sees every message published here: tap(name, msg...). Set by the
        # kachaka-api gRPC server (api_server.py).
        self.tap = lambda name, *msg: None
        n = self.node

        # --- inputs ---------------------------------------------------------
        n.create_subscription(Twist, f"{ns}/manual_control/cmd_vel", self._on_cmd_vel, qos_profile_sensor_data)
        # Simulator only: autonomous motion (Nav2 through nav_bridge, command_server)
        # has a higher speed cap than manual cmd_vel, as on the real robot.
        n.create_subscription(Twist, f"{ns}/sim/autonomous/cmd_vel", self._on_autonomous_cmd_vel,
                              qos_profile_sensor_data)
        if self.generic:
            n.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        n.create_service(SetBool, f"{ns}/manual_control/set_enabled", self._on_manual_control)
        # auto_homing/set_enabled is served by kachaka_sim_bringup's command
        # server, which does the returning (it needs Nav2).

        # --- outputs --------------------------------------------------------
        self.clock_pub = n.create_publisher(Clock, "/clock", 10)
        # The real bridge publishes all sensor streams with SensorDataQoS.
        sensor = qos_profile_sensor_data
        self.odom_pub = n.create_publisher(Odometry, f"{ns}/odometry/odometry", sensor)
        self.wheel_odom_pub = n.create_publisher(Odometry, f"{ns}/wheel_odometry/wheel_odometry", sensor)
        self.gt_pub = n.create_publisher(Odometry, f"{ns}/sim/ground_truth", sensor)
        self.generic_odom_pub = n.create_publisher(Odometry, "/odom", 10) if self.generic else None
        self.scan_pub = n.create_publisher(LaserScan, f"{ns}/lidar/scan", sensor)
        self.generic_scan_pub = n.create_publisher(LaserScan, "/scan", sensor) if self.generic else None
        self.imu_pub = n.create_publisher(Imu, f"{ns}/imu/imu", sensor)
        self.image_pubs: dict[str, tuple] = {}
        self.joint_pub = n.create_publisher(JointState, f"{ns}/joint_states", 10)
        self.tf_pub = n.create_publisher(TFMessage, "/tf", 100)
        self.tf_static_pub = n.create_publisher(TFMessage, "/tf_static", LATCHED)
        self.battery_pub = n.create_publisher(BatteryState, f"{ns}/robot_info/battery_state", sensor)
        self.charger_pub = n.create_publisher(PoseStamped, f"{ns}/sim/charger_pose", LATCHED)
        self.charger_detection_pub = n.create_publisher(PoseStamped, f"{ns}/sim/charger_detection", sensor)
        # Shelves (sim-only; kachaka_sim_bringup's command_server turns them
        # into the real robot's layout topics and shelf commands).
        self.layout_pub = n.create_publisher(String, f"{ns}/sim/layout", LATCHED)
        self.shelf_poses_pub = n.create_publisher(PoseArray, f"{ns}/sim/shelf_poses", sensor)
        self.shelf_detection_pub = n.create_publisher(PoseArray, f"{ns}/sim/shelf_detection", sensor)
        self.shelf_docked_pub = n.create_publisher(String, f"{ns}/sim/shelf_docked", LATCHED)
        self.shelf_dock_handler = None  # set by the sim: bool -> (success, message)
        n.create_service(SetBool, f"{ns}/sim/shelf_dock", self._on_shelf_dock)
        # Object detection (sim-only JSON; kachaka_sim_bringup's
        # object_detection_relay publishes it as the real bridge's
        # /kachaka/object_detection/result, whose type rclpy here cannot load).
        self.object_detection_pub = n.create_publisher(String, f"{ns}/sim/object_detection", sensor)
        self.object_detection_enabled = False  # set by the sim when the front camera detects objects
        # Front obstacle stop (simulator only). set_enabled stands in for the
        # smartphone app's setting; command_server mutes the stop while a
        # command with muted sensors runs.
        self.front_stop = None  # safety.FrontStop, set by the sim
        self._front_stop_state = None
        self.front_stop_pub = n.create_publisher(String, f"{ns}/sim/front_stop/state", LATCHED)
        n.create_service(SetBool, f"{ns}/sim/front_stop/set_enabled", self._on_front_stop_enabled)
        n.create_service(SetBool, f"{ns}/sim/front_stop/mute", self._on_front_stop_mute)
        # Hazards (simulator only): step sensors, collisions and the errors
        # they raise (GetError). The power button resumes from a pause.
        self.cancel_pub = n.create_publisher(String, f"{ns}/sim/command/cancel", 10)
        self.cliff_pub = n.create_publisher(String, f"{ns}/sim/cliff", LATCHED)
        self.collision_pub = n.create_publisher(String, f"{ns}/sim/collision", 10)
        self.errors_pub = n.create_publisher(String, f"{ns}/sim/errors", LATCHED)
        self._cliff_state = None
        self._errors = None
        n.create_service(Trigger, f"{ns}/sim/power_button", self._on_power_button)
        # Torch levels (simulator only; the real robot cannot read them back).
        self.torches = None  # torches.Torches, set by the sim
        self.torch_pub = n.create_publisher(String, f"{ns}/sim/torch", LATCHED)
        # Reset to the start state without restarting (simulator only). The
        # event tells kachaka_sim_bringup's command_server to drop its
        # commands; data=true also asks it to clear the SLAM map.
        self.reset_handler = None  # set by the sim: clear_odometry -> None, runs the reset in the sim loop
        self.reset_count = 0
        self.reset_pub = n.create_publisher(String, f"{ns}/sim/reset/event", 10)
        n.create_service(SetBool, f"{ns}/sim/reset", self._on_reset)
        version_pub = n.create_publisher(String, f"{ns}/robot_info/version", LATCHED)
        desc_pub = n.create_publisher(String, f"{ns}/robot_description", LATCHED)

        version_pub.publish(String(data=str(cfg.stubs.version)))
        desc_pub.publish(String(data=self._prefixed_urdf(urdf_path)))
        self.joints = read_joints(urdf_path)
        self._publish_static_tf()

    # --- helpers --------------------------------------------------------------
    def frame(self, name: str) -> str:
        return f"{self.prefix}{name}"

    def _prefixed_urdf(self, urdf_path: Path) -> str:
        text = urdf_path.read_text()
        if not self.prefix:
            return text
        for tag in ("link name", "parent link", "child link"):
            text = text.replace(f'{tag}="', f'{tag}="{self.prefix}')
        return text

    def _publish_static_tf(self) -> None:
        msg = TFMessage()
        for j in self.joints:
            if j["type"] != "fixed":
                continue
            msg.transforms.append(self._transform(0.0, j["parent"], j["child"], j["xyz"], j["quat"]))
        self.tf_static_pub.publish(msg)
        self.static_tf = msg

    def _transform(self, t: float, parent: str, child: str, xyz, quat) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = to_time(t)
        tf.header.frame_id = self.frame(parent)
        tf.child_frame_id = self.frame(child)
        tr = tf.transform.translation
        tr.x, tr.y, tr.z = (float(c) for c in xyz)
        r = tf.transform.rotation
        r.w, r.x, r.y, r.z = (float(c) for c in quat)
        return tf

    # --- callbacks ------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z, False)

    def _on_autonomous_cmd_vel(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z, True)

    def _on_manual_control(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        # The real robot needs manual control enabled before cmd_vel moves it;
        # the simulator accepts cmd_vel either way and only records the flag.
        self.manual_control_enabled = req.data
        self.tap("manual_control", req.data)
        res.success = True
        return res

    def _on_shelf_dock(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if self.shelf_dock_handler is None:
            res.success, res.message = False, "no shelves in this world"
        else:
            res.success, res.message = self.shelf_dock_handler(req.data)
        return res

    def _on_front_stop_enabled(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        return self._set_front_stop(res, "enabled", req.data)

    def _on_front_stop_mute(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        return self._set_front_stop(res, "muted", req.data)

    def _set_front_stop(self, res: SetBool.Response, attr: str, value: bool) -> SetBool.Response:
        if self.front_stop is None:
            res.success, res.message = False, "the simulator is not running yet"
        else:
            setattr(self.front_stop, attr, value)
            res.success = True
        return res

    def _on_reset(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if self.reset_handler is None:
            res.success, res.message = False, "the simulator is not running yet"
            return res
        # Service callbacks run in the sim loop between steps, so the stage
        # can be changed right here.
        self.reset_handler(bool(req.data))
        self.emergency_stop = False
        self.collision_stopped = False
        if self.cliff is not None:
            self.cliff.reset()
        self.manual_control_enabled = True
        self.cmd = None
        if self.torches is not None:
            for name in list(self.torches.levels):
                self.torches.set(name, 0)
            self.publish_torch()
        self.reset_count += 1
        info = {"id": self.reset_count, "clear_map": bool(req.data)}
        self.reset_pub.publish(String(data=json.dumps(info)))
        self.tap("reset", info)
        self.publish_errors()
        self.publish_cliff()
        res.success, res.message = True, json.dumps(info)
        return res

    def _on_power_button(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        res.success = self.power_button()
        res.message = "resumed" if res.success else "not paused"
        return res

    # --- hazards and errors ---------------------------------------------------
    @property
    def paused(self) -> bool:
        """Stopped by a step or a collision until the power button."""
        return self.collision_stopped or (self.cliff is not None and self.cliff.paused)

    def error_codes(self) -> list[int]:
        """The errors GetError lists now."""
        codes = []
        if self.emergency_stop:
            codes.append(ERROR_EMERGENCY_STOP)
        if self.cliff is not None and self.cliff.paused:
            codes += [ERROR_PAUSED, ERROR_STEP_DETECTED]
        if self.collision_stopped:
            codes.append(ERROR_COLLISION)
        return codes

    def publish_errors(self) -> None:
        """Hand the errors to GetError and publish them (latched JSON list)."""
        codes = self.error_codes()
        self.tap("errors", codes)
        if codes != self._errors:
            self._errors = codes
            self.errors_pub.publish(String(data=json.dumps(codes)))

    def pause(self, collision: bool = False) -> None:
        """A step or (with collision.stop) a collision: drop the velocity
        command, cancel all commands and raise the errors. The sim has
        already stopped the wheels."""
        if collision:
            self.collision_stopped = True
        self.cmd = None
        self.cancel_pub.publish(String(data=json.dumps({"all": True})))
        self.publish_errors()

    def power_button(self) -> bool:
        """Resume from a pause; True if the robot was paused. Runs in the sim loop."""
        was = self.paused
        self.collision_stopped = False
        if self.cliff is not None:
            self.cliff.resume()
        self.publish_errors()
        self.publish_cliff()
        return was

    def publish_cliff(self) -> None:
        """Step sensor state on change (latched JSON)."""
        c = self.cliff
        if c is None:
            return
        state = {"enabled": c.enabled, "paused": c.paused, "over": c.over}
        if state != self._cliff_state:
            self._cliff_state = state
            self.cliff_pub.publish(String(data=json.dumps(state)))

    def publish_collision(self, t: float, count: int, other: str, pose: Pose2D) -> None:
        """A counted collision (JSON event with the running count)."""
        event = {"stamp": round(t, 4), "count": count, "with": other,
                 "pose": [round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 3)]}
        self.collision_pub.publish(String(data=json.dumps(event)))
        self.tap("collision", event)

    # --- called from the sim loop --------------------------------------------
    def spin_once(self, max_callbacks: int = 200) -> None:
        """Run every ready callback. `Executor.spin_once` runs only one, which
        at the render rate lets commands queue up and arrive late."""
        for _ in range(max_callbacks):
            try:
                handler, _, _ = self.executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except (TimeoutException, ShutdownException):
                return
            handler()
            if handler.exception() is not None:
                raise handler.exception()

    def take_cmd(self) -> tuple[float, float, bool] | None:
        cmd, self.cmd = self.cmd, None
        if self.emergency_stop or self.paused:
            return 0.0, 0.0, False
        return cmd

    def run_in_sim(self, fn) -> Future:
        """Run `fn()` in the simulation loop (between steps, where touching the
        stage is safe) and return a Future of its result. For other threads."""
        future = Future()
        with self._pending_lock:
            self._pending.append((fn, future))
        return future

    def run_pending(self) -> None:
        """Called by the simulation loop."""
        with self._pending_lock:
            pending, self._pending = self._pending, []
        for fn, future in pending:
            try:
                future.set_result(fn())
            except Exception as e:  # noqa: BLE001 - handed to the caller
                future.set_exception(e)

    def set_torch(self, name: str, level: int) -> Future:
        """From any thread: set a torch ("front" or "back", 0-255) in the
        simulation loop and publish the levels. The Future holds the level."""
        def apply():
            level_set = self.torches.set(name, level)
            self.publish_torch()
            return level_set
        return self.run_in_sim(apply)

    def publish_torch(self) -> None:
        """Latched JSON {"front", "back"} of the torch levels (0-255)."""
        self.torch_pub.publish(String(data=json.dumps(self.torches.state())))

    def publish_front_stop(self, blocked: bool) -> None:
        """Latched JSON {"enabled", "muted", "blocked"}, sent when it changes."""
        fs = self.front_stop
        state = {"enabled": fs.enabled, "muted": fs.muted, "blocked": blocked}
        if state != self._front_stop_state:
            self._front_stop_state = state
            self.front_stop_pub.publish(String(data=json.dumps(state)))

    def publish_clock(self, t: float) -> None:
        self.clock_pub.publish(Clock(clock=to_time(t)))

    def _odom_msg(self, t: float, frame_id: str, pose: Pose2D, v: float, w: float, cov_xy: float) -> Odometry:
        msg = Odometry()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(frame_id)
        msg.child_frame_id = self.frame("base_footprint")
        # The bundled rclpy asserts on numpy scalars, so cast everything.
        msg.pose.pose.position.x = float(pose.x)
        msg.pose.pose.position.y = float(pose.y)
        q = msg.pose.pose.orientation
        q.w, q.x, q.y, q.z = (float(c) for c in quat_from_yaw(pose.yaw))
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)
        for i in (0, 7):
            msg.pose.covariance[i] = cov_xy
        msg.pose.covariance[35] = cov_xy
        for i in (0, 7, 35):
            msg.twist.covariance[i] = 1e-3
        return msg

    def publish_odometry(self, t: float, fused: Pose2D, wheel: Pose2D, gt: Pose2D, v: float, w: float,
                         wheel_v: float, wheel_w: float) -> None:
        odom = self._odom_msg(t, "odom", fused, v, w, 1e-3)
        self.odom_pub.publish(odom)
        if self.generic_odom_pub:
            self.generic_odom_pub.publish(odom)
        wheel_odom = self._odom_msg(t, "odom", wheel, wheel_v, wheel_w, 1e-2)
        self.wheel_odom_pub.publish(wheel_odom)
        self.tap("odometry", odom)
        self.tap("wheel_odometry", wheel_odom)
        self.gt_pub.publish(self._odom_msg(t, "world", gt, v, w, 0.0))
        tf = self._transform(t, "odom", "base_footprint", (fused.x, fused.y, 0.0), quat_from_yaw(fused.yaw))
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def publish_joints(self, t: float, names: list[str], positions: list[float], velocities: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = to_time(t)
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self.joint_pub.publish(msg)
        # Moving joints on /tf, as robot_state_publisher would do.
        by_name = dict(zip(names, positions))
        tfs = []
        for j in self.joints:
            if j["name"] not in by_name:
                continue
            q = float(by_name[j["name"]])
            ax = j["axis"]
            if j["type"] == "prismatic":
                xyz = tuple(o + a * q for o, a in zip(j["xyz"], ax))
                quat = j["quat"]
            else:
                xyz = j["xyz"]
                quat = _quat_mul(j["quat"], _axis_angle(ax, q))
            tfs.append(self._transform(t, j["parent"], j["child"], xyz, quat))
        self.tf_pub.publish(TFMessage(transforms=tfs))

    def publish_scan(self, t: float, lidar, scan_time: float) -> None:
        cfg = self.cfg.lidar
        msg = LaserScan()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(cfg.frame_id)
        msg.angle_min = float(lidar.angle_min)
        msg.angle_max = float(lidar.angle_max)
        msg.angle_increment = float(lidar.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = float(scan_time)
        msg.range_min = float(cfg.min_range)
        msg.range_max = float(cfg.max_range)
        msg.ranges = lidar.ranges
        self.scan_pub.publish(msg)
        if self.generic_scan_pub:
            self.generic_scan_pub.publish(msg)
        self.tap("scan", msg)

    def publish_imu(self, t: float, imu) -> None:
        cfg = self.cfg.imu
        msg = Imu()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(cfg.frame_id)
        o = msg.orientation
        o.w, o.x, o.y, o.z = (float(c) for c in imu.orientation)
        av, la = msg.angular_velocity, msg.linear_acceleration
        av.x, av.y, av.z = (float(c) for c in imu.angular_velocity)
        la.x, la.y, la.z = (float(c) for c in imu.linear_acceleration)
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 1e-3
            msg.angular_velocity_covariance[i] = float(cfg.gyro_noise) ** 2
            msg.linear_acceleration_covariance[i] = float(cfg.accel_noise) ** 2
        self.imu_pub.publish(msg)
        self.tap("imu", msg)

    def add_camera(self, name: str) -> None:
        n, base = self.node, f"{self.ns}/{name}"
        sensor = qos_profile_sensor_data
        self.image_pubs[name] = (
            n.create_publisher(Image, f"{base}/image_raw", sensor),
            n.create_publisher(CameraInfo, f"{base}/camera_info", sensor),
            n.create_publisher(CameraInfo, f"{base}/image_raw/camera_info", sensor),
        )

    def publish_image(self, t: float, name: str, frame_id: str, data, encoding: str, fx: float) -> None:
        img_pub, info_pub, info_pub2 = self.image_pubs[name]
        h, w = data.shape[:2]
        img = Image()
        img.header.stamp = to_time(t)
        img.header.frame_id = self.frame(frame_id)
        img.height, img.width = h, w
        img.encoding = encoding
        img.is_bigendian = 0
        img.step = int(data.strides[0])
        img.data = data.tobytes()
        img_pub.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.height, info.width = h, w
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info_pub.publish(info)
        info_pub2.publish(info)
        self.tap(name, img, info)

    def publish_battery(self, t: float, percentage: float, charging: bool) -> None:
        # Fields as the real bridge fills them: no voltage, only charging or
        # discharging.
        msg = BatteryState()
        msg.header.stamp = to_time(t)
        msg.voltage = -1.0
        msg.percentage = float(percentage)
        msg.present = True
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging
                                   else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.battery_pub.publish(msg)
        self.tap("battery", msg)

    def publish_charger_detection(self, t: float, pose: Pose2D) -> None:
        """Docked base_footprint pose relative to the robot (sim-only)."""
        msg = PoseStamped()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame("base_footprint")
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        q = msg.pose.orientation
        q.w, q.x, q.y, q.z = (float(c) for c in quat_from_yaw(pose.yaw))
        self.charger_detection_pub.publish(msg)

    def publish_charger_pose(self, pose: Pose2D) -> None:
        """Docked base_footprint pose in the map frame (sim-only, latched)."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        q = msg.pose.orientation
        q.w, q.x, q.y, q.z = (float(c) for c in quat_from_yaw(pose.yaw))
        self.charger_pub.publish(msg)
        self.tap("charger", pose)

    def publish_layout(self, shelves: list[dict], locations: list[dict]) -> None:
        """Shelves (id, name, home, size) and extra locations (id, name, pose),
        poses as [x, y, yaw] in the map frame (sim-only, latched JSON)."""
        layout = {"shelves": shelves, "locations": locations}
        self.layout_pub.publish(String(data=json.dumps(layout)))
        self.tap("layout", layout)

    def add_doors(self, doors, to_map) -> None:
        """Doors (doors.Doors): a SetBool service sim/doors/<id>/set_open per
        door (data: open) and the latched JSON state on sim/doors. `to_map`
        turns a world Pose2D into the map frame."""
        self.doors = doors
        self._door_poses = [to_map(s.pose) for s in doors.specs]
        self.doors_pub = self.node.create_publisher(String, f"{self.ns}/sim/doors", LATCHED)
        for spec in doors.specs:
            def on_set(req, res, door_id=spec.id):
                # Service callbacks run in the sim loop between steps.
                res.success, res.message = self.doors.command(door_id, bool(req.data))
                return res
            self.node.create_service(SetBool, f"{self.ns}/sim/doors/{spec.id}/set_open", on_set)

    def publish_doors(self) -> None:
        """Latched JSON {"doors": [{id, kind, pose [x, y, yaw] of the doorway
        centre in the map frame, width, state (open, closed, opening or
        closing), opening (0-1), cycling}]}, sent when a state changes."""
        doors = [{"id": sp.id, "kind": sp.kind, "pose": [round(p.x, 4), round(p.y, 4), round(p.yaw, 4)],
                  "width": sp.width, "state": st.state, "opening": round(st.opening, 3), "cycling": st.cycling}
                 for sp, st, p in zip(self.doors.specs, self.doors.states, self._door_poses)]
        msg = {"doors": doors}
        self.doors_pub.publish(String(data=json.dumps(msg)))
        self.tap("doors", msg)

    def publish_object_detection(self, t: float, frame_id: str, objects: list[dict]) -> None:
        """Detections in the front camera image (detection.Detection.to_dict:
        kachaka-api label, roi [x, y, width, height], score, distance)."""
        stamp = to_time(t)
        msg = {"stamp": [stamp.sec, stamp.nanosec], "frame_id": self.frame(frame_id), "objects": objects}
        self.object_detection_pub.publish(String(data=json.dumps(msg)))
        self.tap("object_detection", msg)

    @staticmethod
    def _pose(p: Pose2D) -> Pose:
        msg = Pose()
        msg.position.x, msg.position.y = float(p.x), float(p.y)
        q = msg.orientation
        q.w, q.x, q.y, q.z = (float(c) for c in quat_from_yaw(p.yaw))
        return msg

    def publish_shelf_poses(self, t: float, poses: list[Pose2D]) -> None:
        """Shelf poses in the map frame, in sim/layout order (sim-only)."""
        msg = PoseArray()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = "map"
        msg.poses = [self._pose(p) for p in poses]
        self.shelf_poses_pub.publish(msg)
        self.tap("shelf_poses", poses)

    def publish_shelf_detection(self, t: float, poses: list[Pose2D]) -> None:
        """Shelf poses relative to the robot, in sim/layout order (sim-only)."""
        msg = PoseArray()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame("base_footprint")
        msg.poses = [self._pose(p) for p in poses]
        self.shelf_detection_pub.publish(msg)

    def publish_shelf_docked(self, shelf_id: str) -> None:
        """Id of the shelf the robot tows, or "" (sim-only, latched)."""
        self.shelf_docked_pub.publish(String(data=shelf_id))
        self.tap("shelf_docked", shelf_id)

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def _axis_angle(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
