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
  - MOVE_FORWARD and ROTATE_IN_PLACE drive on odometry. MOVE_FORWARD stops
    short of anything the LiDAR sees ahead unless `mute_sensors` is set;
    backward it does not look (the robot has no rear range sensor).
    MOVE_BY_VELOCITY_WITH_MUTED_SENSORS drives at a speed for a time.
  - LOCK keeps the robot still for a time. LOCALIZE succeeds at once:
    localization (SLAM or AMCL) runs all the time.
  - DEPART_FROM_CHARGER drives forward off the charger.
  - DOCK_ANY_SHELF_WITH_REGISTRATION docks with the shelf standing at a
    location. Every shelf in the simulator is registered already, and the
    robot enters it from whichever side Nav2 reaches (`dock_forward` is not
    used).
  - `cancel_all` stops the running command and drops queued ones; without
    it the command waits until the earlier ones finish, as on the real robot.
- Topic `/kachaka/goal_pose` (PoseStamped): sends MOVE_TO_POSE through the
  action without `cancel_all`, like the real bridge.
- The simulator's kachaka-api gRPC server (kachaka_isaac_sim/api_server.py)
  starts commands on `sim/command/start` and cancels the running one on
  `sim/command/cancel` (JSON in std_msgs/String; `{"all": true}` also drops
  the queued ones). Fields of commands that KachakaCommand.msg lacks (LOCK,
  MOVE_FORWARD, ...) come in the JSON only. `sim/command/state`
  (latched JSON) reports the running command and the last result of every
  command, from either side, like the real robot's GetCommandState and
  GetLastCommandResult.
- Topic `/kachaka/layout/locations/list`: the charger, the shelves' homes and
  the world's other locations. Topic `/kachaka/layout/shelves/list`: the
  shelves with their current poses. Both come from simulator topics
  (`sim/charger_pose`, `sim/layout`, `sim/shelf_poses`).
- Relays the simulator's dock detections (the dock relative to the robot) to
  the docking servers, so docking does not depend on how well the SLAM map
  lines up with the world.
- Enlarges the Nav2 costmaps' footprint while a shelf is towed.
- `sim/map/load` (JSON {"id", "map_id"}) loads `<maps_dir>/<map_id>.yaml`
  into Nav2's map_server and answers on `sim/map/loaded` ({"id", "success",
  "message"}), for the gRPC SwitchMap. Only navigation.launch.py runs a
  map_server.
- `sim/reset/event` (JSON {"id", "clear_map"}, from the simulator's
  `sim/reset` service): drops the running and queued commands without a
  result, clears the costmaps, goes back to the default settings and, with
  clear_map, clears slam_toolbox's map. `sim/command/state` then carries
  "reset": id.
- Service `/kachaka/auto_homing/set_enabled`. When enabled the robot returns
  home after `auto_homing_idle_timeout` seconds without velocity commands, or
  when the battery falls below `auto_homing_low_battery`.

A real Kachaka provides all of this itself, so do not run this node with one.
"""

import contextlib
import json
import math
import threading
import time
import uuid
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
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
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
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID

try:
    from slam_toolbox.srv import Reset as SlamReset
except ImportError:  # slam_toolbox not installed: sim/reset cannot clear the map
    SlamReset = None

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
ERROR_NOT_ON_CHARGER = -13  # -12 is the gRPC server's (manual control disabled)

CHARGER_ID = "home"  # the id the real robot gives its charger

# kachaka-api commands that KachakaCommand.msg lacks (Command oneof field numbers).
LOCK_COMMAND = 15
MOVE_FORWARD_COMMAND = 16
ROTATE_IN_PLACE_COMMAND = 17
DOCK_ANY_SHELF_WITH_REGISTRATION_COMMAND = 18
LOCALIZE_COMMAND = 19
MOVE_BY_VELOCITY_WITH_MUTED_SENSORS_COMMAND = 22
DEPART_FROM_CHARGER_COMMAND = 23

FOOTPRINT = [(0.25, 0.13), (0.25, -0.13), (-0.16, -0.13), (-0.16, 0.13)]
SHELF_FIND_RANGE = 1.5  # m, DOCK_SHELF looks this far for a shelf
UNDOCK_DISTANCE = 0.50  # m, driven forward to leave a shelf: far enough to turn on the spot after
UNDOCK_SPEED = 0.15  # m/s
UNDOCK_CLEARANCE = 0.10  # m, free space wanted beyond the undock distance
STAGING_DISTANCE = 0.9  # m, shelf_docking_server's staging_x_offset (negated)
STAGING_BUSY_COST = 50  # global costmap value (0-100) above which the other side is tried first
# Autonomous top speed (config/robot.yaml drive.max_linear_vel_auto) and the
# cap while towing a shelf (drive.max_linear_vel_towing); both est. until measured.
MAX_SPEED = 0.8  # m/s
MAX_TOWING_SPEED = 0.5  # m/s
MOVE_SPEED = 0.2  # m/s, MOVE_FORWARD when no speed is given
MIN_SPEED = 0.05  # m/s, slowest speed while closing in on the distance
STOP_CLEARANCE = 0.05  # m, MOVE_FORWARD stops this far from what the LiDAR sees
LOOK_AHEAD = 0.15  # m, MOVE_FORWARD watches this far ahead (more than it travels between checks)
ROTATE_SPEED = 1.0  # rad/s
MIN_ROTATE_SPEED = 0.15  # rad/s
ROTATE_TOLERANCE = 0.01  # rad
ROTATE_DECEL = 0.8  # rad/s^2, planned deceleration: slow near the end
ROTATE_LAG = 0.35  # s, the robot still turns this long at its rate after a stop
FRONT_STOP_WAIT = 1.0  # s, MOVE_FORWARD gives up after the front obstacle stop holds the robot this long
DEPART_DISTANCE = 0.30  # m, driven forward off the charger
SHELF_AT_LOCATION = 0.5  # m, a shelf this close to a location stands at it
SQUARE_TOLERANCE = 0.02  # rad, a shelf put down further off its location's axis is turned onto it
SQUARE_ATTEMPTS = 3  # turns at most: the towed shelf lags a turn and settles after it
SQUARE_SETTLE = 0.3  # s, wait after a turn before measuring the shelf again
OUTLINE_STEP = 0.025  # m, spacing of the parked shelves' outline points (half a costmap cell)


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


def shelf_outline(p: P2, depth: float, width: float, step: float = OUTLINE_STEP) -> list[tuple[float, float]]:
    """Points along the edges of a shelf standing at `p`, `depth` along its
    x axis and `width` along its y axis."""
    hx, hy = depth / 2, width / 2
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    points = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            points.append((p.x + c * x - s * y, p.y + s * x + c * y))
    return points


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
        self.maps_dir = self.declare_parameter("maps_dir", "/maps").value
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
        self.odom_turn_rate = 0.0  # rad/s, from odometry
        self.front_blocked = False  # the simulator's front obstacle stop holds the robot
        self.costmap: OccupancyGrid | None = None
        self.last_activity = self.get_clock().now()
        self.lock = threading.Lock()  # one command at a time
        self.request_id = 0  # bumped by each command that cancels the others; older ones stop
        self.run_seq = 0  # numbers the commands as they start running
        self.cancel_seq = 0  # sim/command/cancel stops the running command with this number
        self.state_lock = threading.Lock()
        self.state = {"running": None, "last": None}  # published on sim/command/state
        self.auto_retry_at = 0.0  # monotonic time; wait after a failed auto homing
        self.shelf_target = None  # what the shelf docking server should see (see _relay_shelf)
        self.dock_error = 0  # DockRobot error code of the last failed docking
        self.api_fields: dict[str, dict] = {}  # gRPC command id -> its fields, all of them
        self.reset_epoch = 0  # bumped by sim/reset; commands from before it report nothing

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
        # The simulator caps autonomous motion apart from manual cmd_vel.
        self.cmd_pub = self.create_publisher(Twist, f"{prefix}/sim/autonomous/cmd_vel", 10)
        # Nav2's controller_server slows down while a shelf is docked. It
        # subscribes volatile, so the limit is sent again every second.
        self.speed_limit_pub = self.create_publisher(SpeedLimit, "speed_limit", 10)
        self.speech_pub = self.create_publisher(String, f"{prefix}/sim/speech", 10)
        # Shelves standing in the room, as an obstacle source for the costmaps:
        # the LiDAR sees only their legs, and Nav2 would plan between them.
        self.parked_pub = self.create_publisher(PointCloud2, "parked_shelves", 1)
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
        self.mute_client = self.create_client(SetBool, f"{prefix}/sim/front_stop/mute", callback_group=group)
        self.create_subscription(String, f"{prefix}/sim/front_stop/state", self._on_front_stop, LATCHED,
                                 callback_group=group)
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
                                 self._on_odom, sensor, callback_group=group)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", lambda m: setattr(self, "costmap", m),
                                 LATCHED, callback_group=group)
        for topic in ("manual_control/cmd_vel", "sim/autonomous/cmd_vel"):
            self.create_subscription(Twist, f"{prefix}/{topic}", self._on_cmd_vel, sensor, callback_group=group)
        self.create_service(SetBool, f"{prefix}/auto_homing/set_enabled", self._on_set_auto_homing,
                            callback_group=group)
        self.dock_client = ActionClient(self, DockRobot, "dock_robot", callback_group=group)
        self.shelf_client = ActionClient(self, DockRobot, "shelf_dock_robot", callback_group=group)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=group)
        # The real bridge's goal_pose: MOVE_TO_POSE through the action, queued.
        self.self_client = ActionClient(self, ExecKachakaCommand, f"{prefix}/kachaka_command/execute",
                                        callback_group=group)
        self.create_subscription(PoseStamped, f"{prefix}/goal_pose", self._on_goal_pose, 5, callback_group=group)
        self.state_pub = self.create_publisher(String, f"{prefix}/sim/command/state", LATCHED)
        self.create_subscription(String, f"{prefix}/sim/command/start", self._on_api_start, 10,
                                 callback_group=group)
        self.create_subscription(String, f"{prefix}/sim/command/cancel", self._on_api_cancel, 10,
                                 callback_group=group)
        self.load_map_client = self.create_client(LoadMap, "map_server/load_map", callback_group=group)
        self.map_loaded_pub = self.create_publisher(String, f"{prefix}/sim/map/loaded", 10)
        self.create_subscription(String, f"{prefix}/sim/map/load", self._on_map_load, 10, callback_group=group)
        self.slam_reset_client = (self.create_client(SlamReset, "slam_toolbox/reset", callback_group=group)
                                  if SlamReset else None)
        self.create_subscription(String, f"{prefix}/sim/reset/event", self._on_sim_reset, 10, callback_group=group)
        self._set_state()
        self.action_server = ActionServer(
            self, ExecKachakaCommand, f"{prefix}/kachaka_command/execute",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_timer(1.0, self._check_auto_homing, callback_group=group)
        self.create_timer(1.0, self._publish_speed_limit, callback_group=group)
        self.create_timer(0.05, self._relay_shelf, callback_group=group)
        self.create_timer(0.5, self._publish_parked_shelves, callback_group=group)

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
            self._publish_speed_limit()

    def _publish_parked_shelves(self) -> None:
        """Outlines of the shelves not on the robot, in the map frame. The
        docking controller skips its collision checks near the shelf it
        drives under, so that one stays in too."""
        points = []
        for s, p in zip(self.shelves, self.shelf_poses):
            if s["id"] != self.docked_id:
                points += [(x, y, 0.1) for x, y in shelf_outline(p, float(s["depth"]), float(s["width"]))]
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        self.parked_pub.publish(create_cloud_xyz32(header, points))

    def _max_speed(self) -> float:
        return MAX_TOWING_SPEED if self.docked_id else MAX_SPEED

    def _publish_speed_limit(self) -> None:
        """Absolute limit while towing; 0 (none: Nav2's own vx_max) otherwise."""
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = MAX_TOWING_SPEED if self.docked_id else 0.0
        self.speed_limit_pub.publish(msg)

    def _on_front_stop(self, msg: String) -> None:
        self.front_blocked = bool(json.loads(msg.data).get("blocked"))

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

    def _on_api_start(self, msg: String) -> None:
        """A command from the gRPC server, sent through the action under the
        gRPC command id so sim/command/state reports it under that id."""
        req = json.loads(msg.data)
        goal = ExecKachakaCommand.Goal(cancel_all=bool(req.get("cancel_all")),
                                       tts_on_success=str(req.get("tts_on_success", "")))
        cmd = goal.kachaka_command
        command_type = int(req.get("command_type", 0))
        cmd.command_type = command_type if 0 <= command_type < 256 else KachakaCommand.COMMAND_TYPE_OTHER
        fields = req.get("fields", {})
        for k, v in fields.items():
            if k != "command_type" and hasattr(cmd, k):
                setattr(cmd, k, type(getattr(cmd, k))(v))
        self.api_fields[req["id"]] = fields
        self.self_client.send_goal_async(goal, goal_uuid=UUID(uuid=list(bytes.fromhex(req["id"]))))

    def _on_api_cancel(self, msg: String) -> None:
        if msg.data and json.loads(msg.data).get("all"):
            self.request_id += 1  # stops the running command and the queued ones
            self.get_logger().info("cancel of all commands requested")
        elif self.state["running"] is not None:
            self.cancel_seq = self.run_seq
            self.get_logger().info("cancel requested")

    def _on_sim_reset(self, msg: String) -> None:
        """The simulator went back to its start state: forget everything."""
        event = json.loads(msg.data)
        self.reset_epoch += 1
        self.request_id += 1  # stops the running command and the queued ones
        self.api_fields.clear()
        self.auto_homing = bool(self.get_parameter("auto_homing_enabled").value)
        self.last_activity = self.get_clock().now()
        self.auto_retry_at = 0.0
        self.shelf_target = None
        self._set_state(running=None, last=None, reset=event.get("id"))
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        cleared = ""
        if event.get("clear_map"):
            client = self.slam_reset_client
            if client is None or not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning("sim/reset: slam_toolbox/reset is not available; the map stays")
            else:
                client.call_async(SlamReset.Request(pause_new_measurements=False))
                cleared = ", SLAM map cleared"
        self.get_logger().info(f"sim/reset {event.get('id')}: commands dropped{cleared}")

    def _on_map_load(self, msg: String) -> None:
        req = json.loads(msg.data)

        def reply(success: bool, message: str = "") -> None:
            self.map_loaded_pub.publish(String(data=json.dumps(
                {"id": req.get("id"), "success": success, "message": message})))

        if not self.load_map_client.wait_for_service(timeout_sec=2.0):
            reply(False, "map_server/load_map is not available (only navigation.launch.py runs a map_server)")
            return
        url = f"{self.maps_dir.rstrip('/')}/{req['map_id']}.yaml"
        res = self._wait(self.load_map_client.call_async(LoadMap.Request(map_url=url)), lambda: False, timeout=10.0)
        if res is None:
            reply(False, f"map_server did not answer loading {url}")
        elif res.result != LoadMap.Response.RESULT_SUCCESS:
            reply(False, f"map_server could not load {url} (result {res.result})")
        else:
            self.get_logger().info(f"loaded map {url}")
            for client in self.clear_clients:
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
            reply(True)

    def _set_state(self, **changes) -> None:
        with self.state_lock:
            self.state.update(changes)
            self.state_pub.publish(String(data=json.dumps(self.state)))

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
        info = {"id": bytes(goal_handle.goal_id.uuid).hex(), "command_type": cmd.command_type,
                "fields": {k: getattr(cmd, k) for k in cmd.get_fields_and_field_types()
                           if k != "command_type" and getattr(cmd, k)}}
        info["fields"].update(self.api_fields.pop(info["id"], {}))
        f = info["fields"]
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
            # Commands KachakaCommand.msg has no constants or fields for; the
            # numbers are kachaka-api's Command oneof field numbers.
            LOCK_COMMAND: lambda stopped: self._lock(stopped, float(f.get("lock_command_duration_sec", 0.0))),
            MOVE_FORWARD_COMMAND: lambda stopped: self._move_forward(
                stopped, float(f.get("move_forward_command_distance_meter", 0.0)),
                float(f.get("move_forward_command_speed", 0.0)), bool(f.get("move_forward_command_mute_sensors"))),
            ROTATE_IN_PLACE_COMMAND:
                lambda stopped: self._rotate(stopped, float(f.get("rotate_in_place_command_angle_radian", 0.0))),
            DOCK_ANY_SHELF_WITH_REGISTRATION_COMMAND: lambda stopped: self._dock_shelf_at_location(
                stopped, str(f.get("dock_any_shelf_with_registration_command_target_location_id", ""))),
            LOCALIZE_COMMAND: lambda stopped: (OK, "the simulator localizes all the time"),
            MOVE_BY_VELOCITY_WITH_MUTED_SENSORS_COMMAND: lambda stopped: self._move_by_velocity(
                stopped, float(f.get("move_by_velocity_with_muted_sensors_command_signed_velocity", 0.0)),
                float(f.get("move_by_velocity_with_muted_sensors_command_move_duration_sec", 0.0))),
            DEPART_FROM_CHARGER_COMMAND: self._depart_from_charger,
        }
        handler = handlers.get(cmd.command_type)
        if handler is None:
            msg = f"command_type {cmd.command_type} is not supported by the simulator yet"
            self.get_logger().warn(msg)
            self._set_state(last={**info, "success": False, "error_code": ERROR_UNSUPPORTED})
            goal_handle.abort()
            return ExecKachakaCommand.Result(success=False, error_code=ERROR_UNSUPPORTED, message=msg)
        code, msg = self.run(handler, goal_handle, goal_handle.request.cancel_all, info,
                             goal_handle.request.tts_on_success)
        if code != OK:
            self.get_logger().warn(f"command {cmd.command_type} failed: {msg} ({code})")
        result = ExecKachakaCommand.Result(success=code == OK, error_code=code, message=msg)
        if code == OK:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def run(self, handler, goal_handle=None, cancel_all=True, info=None, tts="") -> tuple[int, str]:
        """Run one command and block until it ends. With cancel_all it stops
        the running command and the queued ones; without, it waits its turn.
        Either way a later command with cancel_all stops it. `info` (id,
        command_type, fields) is what sim/command/state reports; `tts` is
        spoken after success."""
        if cancel_all:
            self.request_id += 1
        my_id = self.request_id
        my_epoch = self.reset_epoch
        my_seq = -1

        def stopped() -> bool:
            return my_id != self.request_id or self.cancel_seq == my_seq or \
                (goal_handle is not None and goal_handle.is_cancel_requested)

        code, msg = ERROR_CANCELED, "canceled before it started"
        with self.lock:
            try:
                if not stopped():
                    self.run_seq += 1
                    my_seq = self.run_seq
                    self._set_state(running=info)
                    self.last_activity = self.get_clock().now()
                    code, msg = handler(stopped)
                    if code == OK and tts:
                        self._speak(tts)
            finally:
                self.shelf_target = None
                self.last_activity = self.get_clock().now()
                if my_epoch == self.reset_epoch:  # a sim/reset already cleared the state
                    last = {**info, "success": code == OK, "error_code": code} if info else self.state["last"]
                    self._set_state(running=None, last=last)
        return code, msg

    def return_home(self) -> tuple[int, str]:
        """RETURN_HOME outside the action (auto homing)."""
        info = {"id": uuid.uuid4().hex, "command_type": KachakaCommand.RETURN_HOME_COMMAND, "fields": {}}
        return self.run(self._return_home, info=info)

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

    def _path_blocked(self, distance: float = UNDOCK_DISTANCE, clearance: float = UNDOCK_CLEARANCE) -> bool:
        """Anything in the scan in the robot's way for `distance` plus `clearance`."""
        scan = self.scan
        if scan is None:
            return False
        try:
            t = self.tf.lookup_transform(self.base_frame, scan.header.frame_id, Time()).transform
        except TransformException:
            return False
        laser = P2(t.translation.x, t.translation.y, _yaw(t.rotation))
        x_max = FOOTPRINT[0][0] + distance + clearance
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
        code, msg = self._drive(stopped, UNDOCK_DISTANCE, UNDOCK_SPEED)
        if code == ERROR_BLOCKED:
            msg = "could not drive out from under the shelf"
        return code, msg

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
        if code == OK:
            code, msg = self._square_up(stopped, loc[2], i)
        if code != OK:
            return code, msg
        if undock:
            return self._undock_shelf(stopped, shelf_id)
        return OK, ""

    def _square_up(self, stopped, dock: P2, shelf: int) -> tuple[int, str]:
        """Turn on the spot until the towed shelf lies on the dock's axis: the
        docking controller stops on position alone, up to ~0.1 rad off, and
        the shelf sits on the robot at the yaw it was picked up with, so line
        up the shelf (from its detection), not the robot."""
        for attempt in range(SQUARE_ATTEMPTS):
            if attempt:
                time.sleep(SQUARE_SETTLE)
            robot = self.robot_in_map()
            if robot is None:
                return OK, ""
            yaw = robot[0].yaw
            det = self.detected(shelf)
            if det is not None:
                yaw += det.yaw  # the shelf's heading, give or take pi
            err = _wrap(2.0 * (yaw - dock.yaw)) / 2.0  # off the axis, either direction
            if abs(err) < SQUARE_TOLERANCE:
                return OK, ""
            code, msg = self._rotate(stopped, -err)
            if code != OK:
                if code == ERROR_BLOCKED:
                    msg = "could not turn the shelf onto the location's axis"
                return code, msg
        return OK, ""

    def _return_shelf(self, stopped, shelf_id: str) -> tuple[int, str]:
        shelf_id = shelf_id or self.docked_id
        if not shelf_id:
            return ERROR_NO_SHELF, "no shelf given and none is docked"
        return self._move_shelf(stopped, shelf_id, f"{shelf_id}_home", True)

    # --- driving on odometry ------------------------------------------------------
    def _now(self) -> float:
        """Simulation time [s]."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _drive(self, stopped, distance: float, speed: float, look_ahead: bool = False) -> tuple[int, str]:
        """Drive straight `distance` m (negative: backward) at up to `speed`.
        With `look_ahead`, stop short of anything the LiDAR sees ahead."""
        if self.odom is None:
            return ERROR_SERVER_UNAVAILABLE, "no odometry"
        speed = min(abs(speed), self._max_speed())
        start = self.odom
        deadline = self._now() + 3.0 * abs(distance) / speed + 2.0
        blocked_since = None
        cmd = Twist()
        try:
            while True:
                left = abs(distance) - math.hypot(self.odom.x - start.x, self.odom.y - start.y)
                if left <= 0.005:
                    return OK, ""
                if stopped():
                    return ERROR_CANCELED, "canceled"
                if self._now() > deadline:
                    return ERROR_BLOCKED, "could not drive the whole distance"
                if look_ahead and distance > 0 and self._path_blocked(min(left, LOOK_AHEAD), STOP_CLEARANCE):
                    return ERROR_BLOCKED, "something is in front of the robot"
                if distance > 0 and self.front_blocked:
                    blocked_since = blocked_since or self._now()
                    if self._now() - blocked_since > FRONT_STOP_WAIT:
                        return ERROR_BLOCKED, "the front obstacle stop halted the robot"
                else:
                    blocked_since = None
                cmd.linear.x = math.copysign(min(speed, max(MIN_SPEED, 1.5 * left)), distance)
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
        finally:
            self.cmd_pub.publish(Twist())

    @contextlib.contextmanager
    def _muted_sensors(self):
        """Turn the simulator's front obstacle stop off for the block."""
        def mute(on: bool) -> bool:
            if not self.mute_client.wait_for_service(timeout_sec=1.0):
                return False
            res = self._wait(self.mute_client.call_async(SetBool.Request(data=on)), lambda: False, timeout=5.0)
            return bool(res and res.success)

        muted = mute(True)
        if not muted:
            self.get_logger().warn("sim/front_stop/mute failed; the front obstacle stop stays on")
        try:
            yield
        finally:
            if muted:
                mute(False)

    def _move_forward(self, stopped, distance: float, speed: float, mute_sensors: bool) -> tuple[int, str]:
        self.get_logger().info(f"moving forward {distance:.2f} m{' (sensors muted)' if mute_sensors else ''}")
        if not mute_sensors:
            return self._drive(stopped, distance, speed or MOVE_SPEED, look_ahead=True)
        with self._muted_sensors():
            return self._drive(stopped, distance, speed or MOVE_SPEED)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = _from_pose(msg.pose.pose)
        self.odom_turn_rate = msg.twist.twist.angular.z

    def _rotate(self, stopped, angle: float) -> tuple[int, str]:
        """Turn on the spot by `angle` rad (positive: counter-clockwise); more
        than a full turn is fine."""
        if self.odom is None:
            return ERROR_SERVER_UNAVAILABLE, "no odometry"
        self.get_logger().info(f"rotating {angle:.2f} rad")
        turned, last = 0.0, self.odom.yaw
        deadline = self._now() + abs(angle) / MIN_ROTATE_SPEED + 2.0
        cmd = Twist()
        try:
            while True:
                yaw = self.odom.yaw
                turned += _wrap(yaw - last)
                last = yaw
                left = angle - turned
                # The robot answers a command late: stop when the turn it will
                # still make at its current rate covers what is left.
                coast = self.odom_turn_rate * ROTATE_LAG
                if abs(left) <= ROTATE_TOLERANCE or (coast * left > 0 and abs(left) <= abs(coast)):
                    self.cmd_pub.publish(Twist())
                    self._wait_still()
                    return OK, ""
                if stopped():
                    return ERROR_CANCELED, "canceled"
                if self._now() > deadline:
                    return ERROR_BLOCKED, "could not turn the whole angle"
                speed = math.sqrt(2.0 * ROTATE_DECEL * max(0.0, abs(left) - abs(coast)))
                cmd.angular.z = math.copysign(min(ROTATE_SPEED, max(MIN_ROTATE_SPEED, speed)), left)
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
        finally:
            self.cmd_pub.publish(Twist())

    def _wait_still(self, timeout: float = 1.0) -> None:
        """Until the robot stops turning after a zero command."""
        end = time.monotonic() + timeout
        while abs(self.odom_turn_rate) > 0.02 and time.monotonic() < end:
            time.sleep(0.05)

    def _move_by_velocity(self, stopped, velocity: float, duration: float) -> tuple[int, str]:
        cmd = Twist()
        cmd.linear.x = max(-self._max_speed(), min(self._max_speed(), velocity))
        with self._muted_sensors():
            end = self._now() + duration
            try:
                while self._now() < end:
                    if stopped():
                        return ERROR_CANCELED, "canceled"
                    self.cmd_pub.publish(cmd)
                    time.sleep(0.05)
            finally:
                self.cmd_pub.publish(Twist())
        return OK, ""

    def _lock(self, stopped, duration: float) -> tuple[int, str]:
        """Stay still for `duration` s."""
        self.cmd_pub.publish(Twist())
        end = self._now() + duration
        while self._now() < end:
            if stopped():
                return ERROR_CANCELED, "canceled"
            time.sleep(0.05)
        return OK, ""

    def _depart_from_charger(self, stopped) -> tuple[int, str]:
        """The robot backs onto the charger, so it leaves forward."""
        if not self.charging:
            return ERROR_NOT_ON_CHARGER, "not on the charger"
        self.get_logger().info("departing from the charger")
        return self._drive(stopped, DEPART_DISTANCE, UNDOCK_SPEED)

    def _dock_shelf_at_location(self, stopped, location_id: str) -> tuple[int, str]:
        """DOCK_ANY_SHELF_WITH_REGISTRATION: dock with the shelf at a location."""
        loc = self.locations().get(location_id)
        if loc is None:
            return ERROR_UNKNOWN_LOCATION, f"unknown location {location_id!r}"
        target = loc[2]
        near = [(math.hypot(p.x - target.x, p.y - target.y), i) for i, p in enumerate(self.shelf_poses)]
        near = [n for n in near if n[0] <= SHELF_AT_LOCATION]
        if not near:
            return ERROR_NO_SHELF, f"no shelf at {location_id}"
        i = min(near)[1]
        if self.docked_id:
            if self.docked_id == self.shelves[i]["id"]:
                return OK, f"already docked with {self.docked_id}"
            return ERROR_OTHER_SHELF_DOCKED, f"towing another shelf ({self.docked_id})"
        return self._dock_shelf(stopped, i)

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
