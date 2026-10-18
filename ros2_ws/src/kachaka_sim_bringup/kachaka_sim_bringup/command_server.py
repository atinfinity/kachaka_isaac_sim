"""The real robot's command and layout interfaces, built on Nav2, for the simulator.

- Action `/kachaka/kachaka_command/execute` (kachaka_interfaces/ExecKachakaCommand):
  - RETURN_HOME docks on the charger through Nav2's docking server: Nav2
    drives to a staging pose in front of the dock, then the robot backs onto
    it, as the real one does. A towed shelf is returned to its home first.
  - DOCK_SHELF docks with the nearest shelf in front of the robot. The robot
    drives forward under it through a second docking server
    (`shelf_docking_server`), then raises its pin (simulator `sim/shelf_dock`).
  - UNDOCK_SHELF lowers the pin and drives forward out from under the shelf.
    It fails if something stands in the way.
  - MOVE_SHELF docks with a shelf (if needed), takes it to a location and
    puts it down there precisely by "docking" at the location; it undocks if
    asked to. RETURN_SHELF moves a shelf to its home and undocks.
  - MOVE_TO_POSE sends a Nav2 NavigateToPose goal in the map frame.
    MOVE_TO_LOCATION does the same with a location's pose; for the charger
    it runs RETURN_HOME, since the robot backs onto it.
  - SPEAK has no audio: the text goes to the log and to the simulator-only
    topic `sim/speech`, as does `tts_on_success` after a successful command.
  - `cancel_all` stops the running command and drops queued ones; without
    it the command waits until the earlier ones finish, as on the real robot.
- Topic `/kachaka/goal_pose` (PoseStamped): sends MOVE_TO_POSE through the
  action without `cancel_all`, like the real bridge.
- Topic `/kachaka/layout/locations/list`: the charger, the shelves' homes and
  the world's other locations. Topic `/kachaka/layout/shelves/list`: the
  shelves with their current poses. Both come from simulator topics
  (`sim/charger_pose`, `sim/layout`, `sim/shelf_poses`).
- Relays the simulator's dock detections (the dock relative to the robot) to
  the docking servers, so docking does not depend on how well the SLAM map
  lines up with the world.
- Enlarges the Nav2 costmaps' footprint while a shelf is towed.
- Service `/kachaka/auto_homing/set_enabled`. When enabled the robot returns
  home after `auto_homing_idle_timeout` seconds without velocity commands, or
  when the battery falls below `auto_homing_low_battery`.

A real Kachaka provides all of this itself, so do not run this node with one.
"""

import json
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon, Pose2D, PoseArray, PoseStamped, Twist
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import (
    KachakaCommand,
    Location,
    LocationList,
    Shelf,
    ShelfList,
    ShelfSize,
)
from nav2_msgs.action import DockRobot, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

# Result error codes. The real robot's codes are not published, so these are
# the simulator's own.
OK = 0
ERROR_UNSUPPORTED = -1
ERROR_NO_CHARGER = -2
ERROR_DOCKING_FAILED = -3
ERROR_NOT_CHARGING = -4
ERROR_CANCELED = -5
ERROR_SERVER_UNAVAILABLE = -6
ERROR_NO_SHELF = -7
ERROR_UNKNOWN_LOCATION = -8
ERROR_BLOCKED = -9
ERROR_NAVIGATION_FAILED = -10
ERROR_OTHER_SHELF_DOCKED = -11

CHARGER_ID = "home"  # the id the real robot gives its charger

FOOTPRINT = [(0.25, 0.13), (0.25, -0.13), (-0.16, -0.13), (-0.16, 0.13)]
SHELF_FIND_RANGE = 1.5  # m, DOCK_SHELF looks this far for a shelf
UNDOCK_DISTANCE = 0.40  # m, driven forward to leave a shelf
UNDOCK_SPEED = 0.15  # m/s
UNDOCK_CLEARANCE = 0.10  # m, free space wanted beyond the undock distance
STAGING_DISTANCE = 0.9  # m, shelf_docking_server's staging_x_offset (negated)
STAGING_BUSY_COST = 50  # global costmap value (0-100) above which the other side is tried first


@dataclass
class P2:
    x: float
    y: float
    yaw: float


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _compose(a: P2, b: P2) -> P2:
    """b given relative to a, in a's parent frame."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return P2(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, _wrap(a.yaw + b.yaw))


def _relative(a: P2, b: P2) -> P2:
    """b relative to a (both in the same frame)."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    dx, dy = b.x - a.x, b.y - a.y
    return P2(c * dx + s * dy, -s * dx + c * dy, _wrap(b.yaw - a.yaw))


def _from_pose(p) -> P2:
    return P2(p.position.x, p.position.y, _yaw(p.orientation))


def _stamped(frame: str, stamp, p: P2) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y = p.x, p.y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(p.yaw / 2), math.cos(p.yaw / 2)
    return msg


def facing_away(rel: P2) -> P2:
    """The heading along a dock's axis (rel.yaw or rel.yaw + pi) that points
    away from the robot, so the robot drives forward onto rel from its side."""
    if rel.x * math.cos(rel.yaw) + rel.y * math.sin(rel.yaw) >= 0.0:
        return rel
    return _flipped(rel)


def _flipped(p: P2) -> P2:
    return P2(p.x, p.y, _wrap(p.yaw + math.pi))


def polygon(points) -> Polygon:
    return Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])


class CommandServer(Node):
    def __init__(self):
        super().__init__("kachaka_command_server")
        ns = self.declare_parameter("kachaka_namespace", "kachaka").value.strip("/")
        prefix = f"/{ns}" if ns else ""
        self.auto_homing = bool(self.declare_parameter("auto_homing_enabled", True).value)
        self.idle_timeout = float(self.declare_parameter("auto_homing_idle_timeout", 300.0).value)
        self.low_battery = float(self.declare_parameter("auto_homing_low_battery", 0.15).value)
        self.charge_timeout = float(self.declare_parameter("charge_wait_timeout", 10.0).value)
        self.dock_type = self.declare_parameter("dock_type", "simple_charging_dock").value
        self.max_staging_time = float(self.declare_parameter("max_staging_time", 120.0).value)
        self.shelf_dock_type = self.declare_parameter("shelf_dock_type", "shelf_dock").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        # Footprint while towing: the robot and the shelf (0.328 x 0.488 m) over it.
        self.towing_footprint = [(0.26, 0.25), (0.26, -0.25), (-0.17, -0.25), (-0.17, 0.25)]

        self.charger: PoseStamped | None = None
        self.battery: BatteryState | None = None
        self.shelves: list[dict] = []  # from sim/layout, map frame
        self.other_locations: list[dict] = []
        self.shelf_poses: list[P2] = []  # map frame, layout order
        self.shelf_detection: PoseArray | None = None  # relative to the robot, layout order
        self.docked_id = ""
        self.scan: LaserScan | None = None
        self.odom: P2 | None = None
        self.costmap: OccupancyGrid | None = None
        self.last_activity = self.get_clock().now()
        self.lock = threading.Lock()  # one command at a time
        self.request_id = 0  # bumped by each command that cancels the others; older ones stop
        self.auto_retry_at = 0.0  # monotonic time; wait after a failed auto homing
        self.shelf_target = None  # what the shelf docking server should see (see _relay_shelf)
        self.dock_error = 0  # DockRobot error code of the last failed docking

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        group = ReentrantCallbackGroup()
        sensor = qos_profile_sensor_data
        self.locations_pub = self.create_publisher(LocationList, f"{prefix}/layout/locations/list", LATCHED)
        self.shelves_pub = self.create_publisher(ShelfList, f"{prefix}/layout/shelves/list", LATCHED)
        self.footprint_pubs = [self.create_publisher(Polygon, f"{c}/footprint", LATCHED)
                               for c in ("local_costmap", "global_costmap")]
        self.clear_clients = [self.create_client(ClearEntireCostmap, f"{c}/clear_entirely_{c}", callback_group=group)
                              for c in ("local_costmap", "global_costmap")]
        self.cmd_pub = self.create_publisher(Twist, f"{prefix}/manual_control/cmd_vel", 10)
        self.speech_pub = self.create_publisher(String, f"{prefix}/sim/speech", 10)
        self.create_subscription(PoseStamped, f"{prefix}/sim/charger_pose", self._on_charger, LATCHED,
                                 callback_group=group)
        self.create_subscription(String, f"{prefix}/sim/layout", self._on_layout, LATCHED, callback_group=group)
        self.create_subscription(PoseArray, f"{prefix}/sim/shelf_poses", self._on_shelf_poses, sensor,
                                 callback_group=group)
        self.create_subscription(PoseArray, f"{prefix}/sim/shelf_detection",
                                 lambda m: setattr(self, "shelf_detection", m), sensor, callback_group=group)
        self.create_subscription(String, f"{prefix}/sim/shelf_docked", self._on_shelf_docked, LATCHED,
                                 callback_group=group)
        self.shelf_dock_client = self.create_client(SetBool, f"{prefix}/sim/shelf_dock", callback_group=group)
        # docking_server subscribes reliable, so the relays cannot use sensor QoS.
        detection_pub = self.create_publisher(PoseStamped, "detected_dock_pose", 1)
        self.create_subscription(PoseStamped, f"{prefix}/sim/charger_detection", detection_pub.publish,
                                 sensor, callback_group=group)
        self.shelf_detection_pub = self.create_publisher(PoseStamped, "shelf_detected_dock_pose", 1)
        self.create_subscription(BatteryState, f"{prefix}/robot_info/battery_state", self._on_battery,
                                 sensor, callback_group=group)
        self.create_subscription(LaserScan, f"{prefix}/lidar/scan", lambda m: setattr(self, "scan", m),
                                 sensor, callback_group=group)
        self.create_subscription(Odometry, f"{prefix}/odometry/odometry",
                                 lambda m: setattr(self, "odom", _from_pose(m.pose.pose)), sensor,
                                 callback_group=group)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda m: setattr(self, "costmap", m),
                                 LATCHED, callback_group=group)
        self.create_subscription(Twist, f"{prefix}/manual_control/cmd_vel", self._on_cmd_vel,
                                 sensor, callback_group=group)
        self.create_service(SetBool, f"{prefix}/auto_homing/set_enabled", self._on_set_auto_homing,
                            callback_group=group)
        self.dock_client = ActionClient(self, DockRobot, "dock_robot", callback_group=group)
        self.shelf_client = ActionClient(self, DockRobot, "shelf_dock_robot", callback_group=group)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=group)
        # The real bridge's goal_pose: MOVE_TO_POSE through the action, queued.
        self.self_client = ActionClient(self, ExecKachakaCommand, f"{prefix}/kachaka_command/execute",
                                        callback_group=group)
        self.create_subscription(PoseStamped, f"{prefix}/goal_pose", self._on_goal_pose, 5, callback_group=group)
        self.action_server = ActionServer(
            self, ExecKachakaCommand, f"{prefix}/kachaka_command/execute",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_timer(1.0, self._check_auto_homing, callback_group=group)
        self.create_timer(0.05, self._relay_shelf, callback_group=group)

    # --- inputs ---------------------------------------------------------------
    def _on_charger(self, msg: PoseStamped) -> None:
        self.charger = msg
        p = msg.pose.position
        self.get_logger().info(f"charger at ({p.x:.2f}, {p.y:.2f}) in {msg.header.frame_id}")
        self._publish_locations()

    def _on_layout(self, msg: String) -> None:
        layout = json.loads(msg.data)
        self.shelves = layout.get("shelves", [])
        self.other_locations = layout.get("locations", [])
        self.shelf_poses = [P2(*s["home"]) for s in self.shelves]
        self.get_logger().info(f"{len(self.shelves)} shelves, {len(self.other_locations)} other locations")
        self._publish_locations()
        self._publish_shelves()

    def _on_shelf_poses(self, msg: PoseArray) -> None:
        if len(msg.poses) == len(self.shelves):
            self.shelf_poses = [_from_pose(p) for p in msg.poses]
            self._publish_shelves()

    def _on_shelf_docked(self, msg: String) -> None:
        changed = msg.data != self.docked_id
        self.docked_id = msg.data
        for pub in self.footprint_pubs:
            pub.publish(polygon(self.towing_footprint if self.docked_id else FOOTPRINT))
        if changed and self.docked_id:
            # The shelf's legs were marked as obstacles inside the new footprint.
            for client in self.clear_clients:
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
        if changed:
            self.get_logger().info(f"towing {self.docked_id}" if self.docked_id else "not towing")

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        if msg.linear.x or msg.angular.z:
            self.last_activity = self.get_clock().now()

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        goal = ExecKachakaCommand.Goal()
        goal.kachaka_command.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        p = _from_pose(msg.pose)
        goal.kachaka_command.move_to_pose_command_x = p.x
        goal.kachaka_command.move_to_pose_command_y = p.y
        goal.kachaka_command.move_to_pose_command_yaw = p.yaw
        self.self_client.send_goal_async(goal)
        self.get_logger().info(f"sent goal_pose ({p.x:.3f}, {p.y:.3f}, {p.yaw:.3f})")

    def _on_set_auto_homing(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.auto_homing = req.data
        self.last_activity = self.get_clock().now()
        self.get_logger().info(f"auto homing {'enabled' if req.data else 'disabled'}")
        res.success = True
        return res

    @property
    def charging(self) -> bool:
        return self.battery is not None and \
            self.battery.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    # --- layout ---------------------------------------------------------------
    def locations(self) -> dict[str, tuple[str, int, P2]]:
        """id -> (name, type, docked robot pose in the map frame)."""
        locs = {}
        if self.charger is not None:
            locs[CHARGER_ID] = ("Charger", Location.LOCATION_TYPE_CHARGER, _from_pose(self.charger.pose))
        for s in self.shelves:
            locs[f"{s['id']}_home"] = (f"{s['name']} home", Location.LOCATION_TYPE_SHELF_HOME, P2(*s["home"]))
        for loc in self.other_locations:
            locs[loc["id"]] = (loc["name"], Location.LOCATION_TYPE_OTHER, P2(*loc["pose"]))
        return locs

    def _publish_locations(self) -> None:
        msg = LocationList(default_location_id=CHARGER_ID if self.charger is not None else "")
        for lid, (name, ltype, p) in self.locations().items():
            msg.locations.append(Location(id=lid, name=name, type=ltype, pose=Pose2D(x=p.x, y=p.y, theta=p.yaw)))
        self.locations_pub.publish(msg)

    def _publish_shelves(self) -> None:
        msg = ShelfList()
        for s, p in zip(self.shelves, self.shelf_poses):
            msg.shelves.append(Shelf(
                id=s["id"], name=s["name"], pose=Pose2D(x=p.x, y=p.y, theta=p.yaw),
                size=ShelfSize(width=float(s["width"]), depth=float(s["depth"])),
                home_location_id=f"{s['id']}_home", speed_mode=Shelf.DEFAULT_SPEED_MODE))
        self.shelves_pub.publish(msg)

    def shelf_index(self, shelf_id: str) -> int | None:
        return next((i for i, s in enumerate(self.shelves) if s["id"] == shelf_id), None)

    def detected(self, i: int) -> P2 | None:
        det = self.shelf_detection
        if det is None or i >= len(det.poses):
            return None
        return _from_pose(det.poses[i])

    def robot_in_map(self):
        """(robot pose in the map frame, stamp) from TF, or None."""
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        tr = t.transform
        return P2(tr.translation.x, tr.translation.y, _yaw(tr.rotation)), t.header.stamp

    # --- detection relay for the shelf docking server -------------------------
    def _relay_shelf(self) -> None:
        """Publish the dock pose the shelf docking server should approach:
        ("shelf", index, flip): the simulator's detection of that shelf,
        turned by pi if the robot enters it from the other side;
        ("pose", P2): a fixed pose in the map frame, to put a shelf down."""
        target = self.shelf_target
        if target is None:
            return
        if target[0] == "shelf":
            det = self.shelf_detection
            if det is None or target[1] >= len(det.poses):
                return
            p = _from_pose(det.poses[target[1]])
            if target[2]:
                p.yaw = _wrap(p.yaw + math.pi)
            self.shelf_detection_pub.publish(_stamped(det.header.frame_id, det.header.stamp, p))
        else:
            robot = self.robot_in_map()
            if robot is None:
                return
            # In the robot frame, stamped like the transform used, so the
            # docking server can look it up without extrapolating.
            self.shelf_detection_pub.publish(_stamped(self.base_frame, robot[1], _relative(robot[0], target[1])))

    # --- kachaka_command/execute ----------------------------------------------
    def _execute(self, goal_handle) -> ExecKachakaCommand.Result:
        cmd = goal_handle.request.kachaka_command
        handlers = {
            KachakaCommand.RETURN_HOME_COMMAND: self._return_home,
            KachakaCommand.DOCK_SHELF_COMMAND: self._dock_shelf_command,
            KachakaCommand.UNDOCK_SHELF_COMMAND:
                lambda stopped: self._undock_shelf(stopped, cmd.undock_shelf_command_target_shelf_id),
            KachakaCommand.MOVE_SHELF_COMMAND: lambda stopped: self._move_shelf(
                stopped, cmd.move_shelf_command_target_shelf_id, cmd.move_shelf_command_destination_location_id,
                cmd.move_shelf_command_undock_on_destination),
            KachakaCommand.RETURN_SHELF_COMMAND:
                lambda stopped: self._return_shelf(stopped, cmd.return_shelf_command_target_shelf_id),
            KachakaCommand.MOVE_TO_POSE_COMMAND: lambda stopped: self._move_to_pose(
                stopped, P2(cmd.move_to_pose_command_x, cmd.move_to_pose_command_y, cmd.move_to_pose_command_yaw)),
            KachakaCommand.MOVE_TO_LOCATION_COMMAND:
                lambda stopped: self._move_to_location(stopped, cmd.move_to_location_command_target_location_id),
            KachakaCommand.SPEAK_COMMAND: lambda stopped: self._speak(cmd.speak_command_text),
        }
        handler = handlers.get(cmd.command_type)
        if handler is None:
            msg = f"command_type {cmd.command_type} is not supported by the simulator yet"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return ExecKachakaCommand.Result(success=False, error_code=ERROR_UNSUPPORTED, message=msg)
        code, msg = self.run(handler, goal_handle, goal_handle.request.cancel_all)
        if code != OK:
            self.get_logger().warn(f"command {cmd.command_type} failed: {msg} ({code})")
        elif goal_handle.request.tts_on_success:
            self._speak(goal_handle.request.tts_on_success)
        result = ExecKachakaCommand.Result(success=code == OK, error_code=code, message=msg)
        if code == OK:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def run(self, handler, goal_handle=None, cancel_all=True) -> tuple[int, str]:
        """Run one command and block until it ends. With cancel_all it stops
        the running command and the queued ones; without, it waits its turn.
        Either way a later command with cancel_all stops it."""
        if cancel_all:
            self.request_id += 1
        my_id = self.request_id

        def stopped() -> bool:
            return my_id != self.request_id or (goal_handle is not None and goal_handle.is_cancel_requested)

        with self.lock:
            if stopped():
                return ERROR_CANCELED, "canceled before it started"
            self.last_activity = self.get_clock().now()
            try:
                return handler(stopped)
            finally:
                self.shelf_target = None
                self.last_activity = self.get_clock().now()

    def return_home(self) -> tuple[int, str]:
        return self.run(self._return_home)

    def _return_home(self, stopped) -> tuple[int, str]:
        """Dock on the charger and wait until it charges."""
        if self.charging:
            return OK, "already on the charger"
        if self.charger is None:
            return ERROR_NO_CHARGER, "no charger (is the simulator publishing sim/charger_pose?)"
        if self.docked_id:
            code, msg = self._return_shelf(stopped, self.docked_id)
            if code != OK:
                return code, msg
        # Nav2's dock pose points into the dock, opposite to the docked robot.
        c = _from_pose(self.charger.pose)
        dock = _stamped(self.charger.header.frame_id, self.get_clock().now().to_msg(),
                        P2(c.x, c.y, _wrap(c.yaw + math.pi)))
        self.get_logger().info("returning home")
        code, msg = self._dock(self.dock_client, dock, self.dock_type, stopped)
        if code != OK:
            return code, msg

        # Nav2 only knows it reached the dock pose; the simulator decides
        # whether the contacts touch.
        deadline = time.monotonic() + self.charge_timeout
        while not self.charging:
            if stopped():
                return ERROR_CANCELED, "canceled"
            if time.monotonic() > deadline:
                return ERROR_NOT_CHARGING, "docked but not charging"
            time.sleep(0.1)
        self.get_logger().info("charging")
        return OK, ""

    def _dock(self, client: ActionClient, dock: PoseStamped, dock_type: str, stopped) -> tuple[int, str]:
        """Run a DockRobot goal: Nav2 drives to the staging pose, then the
        docking server's controller onto the dock. Sets self.dock_error."""
        self.dock_error = 0
        if not client.wait_for_server(timeout_sec=5.0):
            return ERROR_SERVER_UNAVAILABLE, f"Nav2 docking server ({client._action_name}) is not running"
        goal = DockRobot.Goal(use_dock_id=False, dock_pose=dock, dock_type=dock_type,
                              max_staging_time=self.max_staging_time, navigate_to_staging_pose=True)
        handle = self._wait(client.send_goal_async(goal), stopped)
        if handle is None or not handle.accepted:
            return ERROR_CANCELED if stopped() else ERROR_DOCKING_FAILED, "docking goal not accepted"
        result_future = handle.get_result_async()
        if self._wait(result_future, stopped) is None:
            self._cancel(handle, result_future)
            return ERROR_CANCELED, "canceled"
        res = result_future.result().result
        if not res.success:
            self.dock_error = res.error_code
            return ERROR_DOCKING_FAILED, f"docking failed: error {res.error_code} {res.error_msg}".strip()
        return OK, ""

    def _cancel(self, handle, result_future) -> None:
        """Cancel a goal and wait until it has ended, so a late cancel cannot
        hit the next command's goal (Nav2 servers take a new goal as a preemption)."""
        self._wait(handle.cancel_goal_async(), lambda: False, timeout=5.0)
        self._wait(result_future, lambda: False, timeout=5.0)

    @staticmethod
    def _wait(future, stopped, timeout=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        while not future.done():
            if stopped() or (deadline is not None and time.monotonic() > deadline):
                return None
            time.sleep(0.05)
        return future.result()

    # --- moving and speaking -----------------------------------------------------
    def _move_to_pose(self, stopped, target: P2) -> tuple[int, str]:
        """A Nav2 goal in the map frame. A towed shelf comes along."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return ERROR_SERVER_UNAVAILABLE, "Nav2 (navigate_to_pose) is not running"
        goal = NavigateToPose.Goal(pose=_stamped(self.map_frame, Time().to_msg(), target))
        self.get_logger().info(f"moving to ({target.x:.2f}, {target.y:.2f}, {target.yaw:.2f})")
        handle = self._wait(self.nav_client.send_goal_async(goal), stopped)
        if handle is None or not handle.accepted:
            return ERROR_CANCELED if stopped() else ERROR_NAVIGATION_FAILED, "navigation goal not accepted"
        result_future = handle.get_result_async()
        if self._wait(result_future, stopped) is None:
            self._cancel(handle, result_future)
            return ERROR_CANCELED, "canceled"
        res = result_future.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            return ERROR_NAVIGATION_FAILED, f"navigation failed: error {res.result.error_code}"
        return OK, ""

    def _move_to_location(self, stopped, location_id: str) -> tuple[int, str]:
        loc = self.locations().get(location_id)
        if loc is None:
            return ERROR_UNKNOWN_LOCATION, f"unknown location {location_id!r}"
        if loc[1] == Location.LOCATION_TYPE_CHARGER:
            return self._return_home(stopped)
        return self._move_to_pose(stopped, loc[2])

    def _speak(self, text: str) -> tuple[int, str]:
        self.get_logger().info(f"speak: {text}")
        self.speech_pub.publish(String(data=text))
        return OK, ""

    # --- shelves ----------------------------------------------------------------
    def _set_pin(self, up: bool) -> tuple[bool, str]:
        if not self.shelf_dock_client.wait_for_service(timeout_sec=5.0):
            return False, "sim/shelf_dock is not available"
        res = self._wait(self.shelf_dock_client.call_async(SetBool.Request(data=up)), lambda: False)
        # Wait for the latched state, which drives the footprint.
        deadline = time.monotonic() + 2.0
        while res.success and bool(self.docked_id) != up and time.monotonic() < deadline:
            time.sleep(0.05)
        return res.success, res.message

    def _dock_shelf_command(self, stopped) -> tuple[int, str]:
        """DOCK_SHELF: the nearest shelf in front of the robot."""
        if self.docked_id:
            return OK, f"already docked with {self.docked_id}"
        best = None
        for i in range(len(self.shelves)):
            d = self.detected(i)
            if d is None:
                continue
            dist = math.hypot(d.x, d.y)
            if dist <= SHELF_FIND_RANGE and (d.x > 0.0 or dist < 0.1) and (best is None or dist < best[0]):
                best = (dist, i)
        if best is None:
            return ERROR_NO_SHELF, "no shelf in front of the robot"
        return self._dock_shelf(stopped, best[1])

    def _dock_shelf(self, stopped, i: int) -> tuple[int, str]:
        shelf_id = self.shelves[i]["id"]
        d = self.detected(i)
        robot = self.robot_in_map()
        if d is None or robot is None:
            return ERROR_NO_SHELF, f"shelf {shelf_id} is not detected"
        if math.hypot(d.x, d.y) > 0.03:
            self.get_logger().info(f"docking with shelf {shelf_id}")
            dock = _compose(robot[0], d)

            def target(heading: P2):
                flip = abs(_wrap(heading.yaw - dock.yaw)) > math.pi / 2
                return ("shelf", i, flip)

            code, msg = self._dock_shelf_at(dock, target, robot, stopped)
            if code != OK:
                return code, msg
        ok, msg = self._set_pin(True)
        if not ok:
            return ERROR_DOCKING_FAILED, f"could not dock with shelf {shelf_id}: {msg}"
        return OK, ""

    def _staging_cost(self, dock: P2) -> int:
        """Global costmap value (0-100, -1 unknown) at a dock's staging pose."""
        grid = self.costmap
        if grid is None:
            return 0
        info = grid.info
        x = dock.x - STAGING_DISTANCE * math.cos(dock.yaw) - info.origin.position.x
        y = dock.y - STAGING_DISTANCE * math.sin(dock.yaw) - info.origin.position.y
        col, row = int(x / info.resolution), int(y / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return -1
        return grid.data[row * info.width + col]

    def _dock_shelf_at(self, dock: P2, target, robot, stopped) -> tuple[int, str]:
        """Drive forward onto a dock pose in the map frame (under a shelf, or
        to put one down) from either end of the shelf's axis: first from the
        robot's side, unless its staging pose is near an obstacle; then from
        the other side if Nav2 cannot reach the first staging pose.
        `target(heading)` gives the shelf docking server's detection."""
        first = _compose(robot[0], facing_away(_relative(robot[0], dock)))
        second = _flipped(first)
        def cost(heading: P2) -> int:
            c = self._staging_cost(heading)
            return 101 if c < 0 else c  # unknown counts as worst

        if cost(first) >= STAGING_BUSY_COST and cost(second) < cost(first):
            first, second = second, first
        code, msg = ERROR_DOCKING_FAILED, ""
        for heading in (first, second):
            if stopped():
                return ERROR_CANCELED, "canceled"
            self.shelf_target = target(heading)
            code, msg = self._dock(self.shelf_client, _stamped(self.map_frame, robot[1], heading),
                                   self.shelf_dock_type, stopped)
            self.shelf_target = None
            if code == OK or self.dock_error != DockRobot.Result.FAILED_TO_STAGE:
                return code, msg
            self.get_logger().info("cannot reach the staging pose; trying the other side")
        return code, msg

    def _path_blocked(self) -> bool:
        """Anything in the scan in the robot's way for the undock distance."""
        scan = self.scan
        if scan is None:
            return False
        try:
            t = self.tf.lookup_transform(self.base_frame, scan.header.frame_id, Time()).transform
        except TransformException:
            return False
        laser = P2(t.translation.x, t.translation.y, _yaw(t.rotation))
        x_max = FOOTPRINT[0][0] + UNDOCK_DISTANCE + UNDOCK_CLEARANCE
        half = FOOTPRINT[0][1] + 0.02
        for k, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            a = scan.angle_min + k * scan.angle_increment
            p = _compose(laser, P2(r * math.cos(a), r * math.sin(a), 0.0))
            if FOOTPRINT[0][0] < p.x < x_max and abs(p.y) < half:
                return True
        return False

    def _undock_shelf(self, stopped, shelf_id: str = "") -> tuple[int, str]:
        """Lower the pin and drive forward out from under the shelf."""
        if not self.docked_id:
            return ERROR_NO_SHELF, "no shelf is docked"
        if shelf_id and shelf_id != self.docked_id:
            return ERROR_NO_SHELF, f"shelf {shelf_id} is not docked (towing {self.docked_id})"
        if self._path_blocked():
            return ERROR_BLOCKED, "something is in front of the robot; cannot leave the shelf"
        if self.odom is None:
            return ERROR_SERVER_UNAVAILABLE, "no odometry"
        ok, msg = self._set_pin(False)
        if not ok:
            return ERROR_DOCKING_FAILED, f"could not undock: {msg}"
        self.get_logger().info("leaving the shelf")
        start = self.odom
        deadline = time.monotonic() + 3.0 * UNDOCK_DISTANCE / UNDOCK_SPEED
        cmd = Twist()
        try:
            while math.hypot(self.odom.x - start.x, self.odom.y - start.y) < UNDOCK_DISTANCE:
                if stopped():
                    return ERROR_CANCELED, "canceled"
                if time.monotonic() > deadline:
                    return ERROR_BLOCKED, "could not drive out from under the shelf"
                cmd.linear.x = UNDOCK_SPEED
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
        finally:
            self.cmd_pub.publish(Twist())
        return OK, ""

    def _move_shelf(self, stopped, shelf_id: str, location_id: str, undock: bool) -> tuple[int, str]:
        i = self.shelf_index(shelf_id)
        if i is None:
            return ERROR_NO_SHELF, f"unknown shelf {shelf_id!r}"
        loc = self.locations().get(location_id)
        if loc is None:
            return ERROR_UNKNOWN_LOCATION, f"unknown location {location_id!r}"
        if self.docked_id and self.docked_id != shelf_id:
            return ERROR_OTHER_SHELF_DOCKED, f"towing another shelf ({self.docked_id})"
        if not self.docked_id:
            code, msg = self._dock_shelf(stopped, i)
            if code != OK:
                return code, msg
        if stopped():
            return ERROR_CANCELED, "canceled"
        # Put the shelf down by docking at the location: Nav2 drives to its
        # staging pose, then the docking controller lines the robot up on it.
        robot = self.robot_in_map()
        if robot is None:
            return ERROR_SERVER_UNAVAILABLE, "no map -> robot transform"
        self.get_logger().info(f"moving shelf {shelf_id} to {location_id}")
        code, msg = self._dock_shelf_at(loc[2], lambda heading: ("pose", heading), robot, stopped)
        if code != OK:
            return code, msg
        if undock:
            return self._undock_shelf(stopped, shelf_id)
        return OK, ""

    def _return_shelf(self, stopped, shelf_id: str) -> tuple[int, str]:
        shelf_id = shelf_id or self.docked_id
        if not shelf_id:
            return ERROR_NO_SHELF, "no shelf given and none is docked"
        return self._move_shelf(stopped, shelf_id, f"{shelf_id}_home", True)

    # --- auto homing ------------------------------------------------------------
    def _check_auto_homing(self) -> None:
        if not self.auto_homing or self.charger is None or self.charging or self.lock.locked() \
                or time.monotonic() < self.auto_retry_at:
            return
        idle = (self.get_clock().now() - self.last_activity).nanoseconds * 1e-9
        low = self.battery is not None and self.battery.percentage < self.low_battery
        if idle < self.idle_timeout and not low:
            return
        why = "battery low" if low else f"idle for {idle:.0f} s"
        self.get_logger().info(f"auto homing: {why}")
        code, msg = self.return_home()
        if code != OK:
            self.get_logger().warn(f"auto homing failed: {msg}")
            self.auto_retry_at = time.monotonic() + 60.0


def main():
    rclpy.init()
    node = CommandServer()
    executor = MultiThreadedExecutor(num_threads=8)  # queued commands each hold a thread
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
