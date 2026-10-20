"""kachaka-api gRPC server inside the simulator.

Serves the real robot's gRPC API (`KachakaApi`, port 26400) so programs written
with the official Python client, `kachaka_api.KachakaApiClient`, run against
the simulator. Robot state and sensors come straight from the simulator
(KachakaSimNode hands every message it publishes to `tap`). Commands go to
kachaka_sim_bringup's command_server, which runs them through Nav2 like the
ROS 2 action `kachaka_command/execute`; the two talk over simulator-only
topics with JSON in std_msgs/String, because the custom kachaka_interfaces
types cannot be imported into Isaac Sim's bundled rclpy:

- `sim/command/start`: {"id", "command_type", "fields", "cancel_all", "tts_on_success"}
- `sim/command/cancel`: cancel the running command; {"all": true} also the queued ones
- `sim/command/state` (latched, from command_server): {"running": {...} | null,
  "last": {..., "success", "error_code"} | null}
- `sim/map/load` {"id", "map_id"} and the answer `sim/map/loaded` {"id",
  "success", "message"}: SwitchMap, through Nav2's map_server

Maps live in the repository's `maps/` folder (see maps.py). The simulator has
no speaker: PlaySound and StopSound only publish on `sim/sound`.

Getters follow the real long-polling protocol: a request carrying the cursor
of the newest data waits until newer data arrives; any other cursor returns
the newest data at once. RPCs the simulator lacks return UNIMPLEMENTED.
"""

from __future__ import annotations

import io
import json
import math
import threading
import time
import uuid
from collections import deque
from concurrent import futures
from pathlib import Path

import grpc
from google.protobuf import json_format
from kachaka_api.generated import kachaka_api_pb2 as pb2
from kachaka_api.generated import kachaka_api_pb2_grpc as pb2_grpc
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .config import ROOT
from .drive import Pose2D
from .hazards import ERROR_COLLISION, ERROR_EMERGENCY_STOP, ERROR_PAUSED, ERROR_STEP_DETECTED
from .maps import MapError, MapFile, MapStore

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
MAX_LINEAR = 0.3  # SetRobotVelocity full scale, as in kachaka_api.base
MAX_ANGULAR = 1.57
CHARGER_ID = "home"  # as in command_server
# PNG map gray levels the real robot uses (kachaka_grpc_ros2_bridge's mapping_component).
PNG_OCCUPIED, PNG_FREE, PNG_UNKNOWN = 174, 253, 234
# Simulator error codes (command_server's), served by GetRobotErrorCodeJson.
ERROR_CODES = {
    -1: ("Unsupported command", "The simulator does not support this command."),
    -2: ("No charger", "There is no charger in this world."),
    -3: ("Docking failed", "Docking with the charger or the shelf failed."),
    -4: ("Not charging", "The robot docked but is not charging."),
    -5: ("Canceled", "The command was canceled."),
    -6: ("Server unavailable", "Nav2 or the command server is not running."),
    -7: ("No shelf", "No shelf found, or the robot does not tow one."),
    -8: ("Unknown location", "The location or shelf id is unknown."),
    -9: ("Blocked", "Something stands in the way."),
    -10: ("Navigation failed", "Nav2 could not reach the goal."),
    -11: ("Other shelf docked", "The robot tows another shelf."),
    -12: ("Manual control disabled", "SetRobotVelocity needs manual control enabled."),
    -13: ("Not on charger", "The robot is not on the charger."),
    -14: ("Emergency stop", "The emergency stop is on; RestartRobot releases it."),
    -15: ("Map error", "The map does not exist or cannot be read."),
    -16: ("Needs a saved map", ("Switching maps and setting the robot pose need navigation.launch.py "
                                "(map_server and AMCL); slam.launch.py maps as it goes.")),
    -17: ("Kachaka Pro only", "The standard Kachaka does not support this."),
    ERROR_COLLISION: ("Collision", ("The body hit something and the robot stopped (collision.stop); "
                                    "RestartRobot or sim/power_button resumes.")),
    # Real robot codes (kachaka-api docs/api/ERROR_HANDLING.md).
    ERROR_PAUSED: ("Kachaka is paused", "Press the power button to resume (RestartRobot or sim/power_button)."),
    ERROR_STEP_DETECTED: ("Step detected", ("Kachaka has paused because a step was detected. Move it to a flat "
                                            "surface and press the power button.")),
}
ERROR_TYPES = {ERROR_PAUSED: "warn"}
ERROR_UNKNOWN_SHELF = -8
ERROR_SERVER_UNAVAILABLE = -6
ERROR_OTHER_SHELF_DOCKED = -11
ERROR_MANUAL_CONTROL_DISABLED = -12
ERROR_MAP = -15
ERROR_NEEDS_SAVED_MAP = -16
ERROR_PRO_ONLY = -17
SLAM_MAP_ID = "sim"  # the live SLAM map
HISTORY_SIZE = 100
CHUNK = 1024 * 1024  # ExportMap stream chunk
DEFAULT_RESOLUTION = 0.05  # m/pixel, ImportImageAsMap without one
BUILTIN_SOUNDS = [("chime", "Chime"), ("alarm", "Alarm"), ("melody", "Melody")]


class Store:
    """Newest value per stream with a cursor, and long-polling reads."""

    def __init__(self):
        self.cond = threading.Condition()
        self.data: dict[str, tuple[int, object]] = {}
        self.last_cursor = 0

    def set(self, name: str, value) -> None:
        with self.cond:
            self.last_cursor = max(time.monotonic_ns(), self.last_cursor + 1)
            self.data[name] = (self.last_cursor, value)
            self.cond.notify_all()

    def peek(self, name: str):
        entry = self.data.get(name)
        return entry[1] if entry else None

    def get(self, name: str, request, context) -> tuple[int, object]:
        cursor = request.metadata.cursor if request is not None else 0
        with self.cond:
            while True:
                entry = self.data.get(name)
                if entry is not None and entry[0] != cursor:
                    return entry
                if not context.is_active():
                    context.abort(grpc.StatusCode.CANCELLED, "client went away")
                self.cond.wait(0.5)


# --- ROS message -> protobuf --------------------------------------------------
def _header(h) -> pb2.RosHeader:
    return pb2.RosHeader(stamp_nsec=h.stamp.sec * 1_000_000_000 + h.stamp.nanosec, frame_id=h.frame_id)


def _vec(v) -> pb2.Vector3:
    return pb2.Vector3(x=v.x, y=v.y, z=v.z)


def _quat(q) -> pb2.Quaternion:
    return pb2.Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


def _imu(m) -> pb2.RosImu:
    return pb2.RosImu(header=_header(m.header), orientation=_quat(m.orientation),
                      orientation_covariance=list(m.orientation_covariance),
                      angular_velocity=_vec(m.angular_velocity),
                      angular_velocity_covariance=list(m.angular_velocity_covariance),
                      linear_acceleration=_vec(m.linear_acceleration),
                      linear_acceleration_covariance=list(m.linear_acceleration_covariance))


def _odometry(m) -> pb2.RosOdometry:
    p, t = m.pose, m.twist
    return pb2.RosOdometry(
        header=_header(m.header), child_frame_id=m.child_frame_id,
        pose=pb2.Pose3dWithCovariance(pose=pb2.Pose3d(position=_vec(p.pose.position),
                                                      orientation=_quat(p.pose.orientation)),
                                      covariance=list(p.covariance)),
        twist=pb2.TwistWithCovariance(twist=pb2.Twist(linear=_vec(t.twist.linear), angular=_vec(t.twist.angular)),
                                      covariance=list(t.covariance)))


def _scan(m) -> pb2.RosLaserScan:
    return pb2.RosLaserScan(header=_header(m.header), angle_min=m.angle_min, angle_max=m.angle_max,
                            angle_increment=m.angle_increment, time_increment=m.time_increment,
                            scan_time=m.scan_time, range_min=m.range_min, range_max=m.range_max,
                            ranges=list(m.ranges), intensities=list(m.intensities))


def _camera_info(m) -> pb2.RosCameraInfo:
    return pb2.RosCameraInfo(header=_header(m.header), height=m.height, width=m.width,
                             distortion_model=m.distortion_model, D=list(m.d), K=list(m.k), R=list(m.r),
                             P=list(m.p), binning_x=m.binning_x, binning_y=m.binning_y,
                             roi=pb2.RegionOfInterest(x_offset=m.roi.x_offset, y_offset=m.roi.y_offset,
                                                      height=m.roi.height, width=m.roi.width,
                                                      do_rectify=m.roi.do_rectify))


def _image(m) -> pb2.RosImage:
    return pb2.RosImage(header=_header(m.header), height=m.height, width=m.width, encoding=m.encoding,
                        is_bigendian=bool(m.is_bigendian), step=m.step, data=bytes(m.data))


def _compressed(m) -> pb2.RosCompressedImage:
    """JPEG for colour images, PNG for depth (lossless)."""
    import cv2
    import numpy as np

    if m.encoding == "rgb8":
        rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        fmt = "rgb8; jpeg compressed bgr8"
    else:
        dtype = np.uint16 if m.encoding == "16UC1" else np.float32
        img = np.frombuffer(bytes(m.data), dtype).reshape(m.height, m.width)
        if dtype is np.float32:
            img = (np.nan_to_num(img) * 1000.0).round().clip(0, 65535).astype(np.uint16)
        ok, buf = cv2.imencode(".png", img)
        fmt = "16UC1; png compressed"
    return pb2.RosCompressedImage(header=_header(m.header), format=fmt, data=buf.tobytes() if ok else b"")


def _png_map(m: OccupancyGrid) -> pb2.Map:
    import numpy as np
    from PIL import Image

    info = m.info
    grid = np.asarray(m.data, np.int8).reshape(info.height, info.width)
    img = np.full(grid.shape, PNG_UNKNOWN, np.uint8)
    img[grid == 0] = PNG_FREE
    img[grid >= 65] = PNG_OCCUPIED
    out = io.BytesIO()
    Image.fromarray(img[::-1]).save(out, format="PNG")  # PNG rows run top (max y) down
    q = info.origin.orientation
    origin = pb2.Pose(x=info.origin.position.x, y=info.origin.position.y,
                      theta=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return pb2.Map(data=out.getvalue(), name="sim", resolution=info.resolution, width=info.width,
                   height=info.height, origin=origin)


def _transforms(m: TFMessage) -> list[pb2.RosTransformStamped]:
    return [pb2.RosTransformStamped(header=_header(t.header), child_frame_id=t.child_frame_id,
                                    translation=_vec(t.transform.translation), rotation=_quat(t.transform.rotation))
            for t in m.transforms]


# --- commands: protobuf <-> command_server JSON -------------------------------
def command_to_json(cmd: pb2.Command) -> tuple[int, dict]:
    """(command_type, fields). The type is the oneof field number, which is
    also KachakaCommand's constant; fields are named like KachakaCommand's
    (`<oneof name>_<field>`)."""
    which = cmd.WhichOneof("command")
    if which is None:
        return 0, {}
    sub = getattr(cmd, which)
    return pb2.Command.DESCRIPTOR.fields_by_name[which].number, \
        {f"{which}_{f.name}": getattr(sub, f.name) for f in sub.DESCRIPTOR.fields}


def command_from_json(command_type: int, fields: dict) -> pb2.Command:
    cmd = pb2.Command()
    fd = pb2.Command.DESCRIPTOR.fields_by_number.get(command_type)
    if fd is None:
        return cmd
    sub = getattr(cmd, fd.name)
    sub.SetInParent()
    for f in sub.DESCRIPTOR.fields:
        key = f"{fd.name}_{f.name}"
        if key in fields:
            setattr(sub, f.name, fields[key])
    return cmd


def _ns_to_dict(value):
    """config.py's SimpleNamespace back to plain dicts and lists."""
    if hasattr(value, "__dict__"):
        return {k: _ns_to_dict(v) for k, v in vars(value).items()}
    if isinstance(value, list):
        return [_ns_to_dict(v) for v in value]
    return value


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class KachakaApiServer(pb2_grpc.KachakaApiServicer):
    """The servicer. Create it before the simulator publishes anything, so
    the latched layout and charger messages reach `tap`."""

    def __init__(self, sim_node, port: int, serial: str = "SIMULATOR", maps_dir: Path | str = ROOT / "maps"):
        self.sim = sim_node
        self.maps = MapStore(maps_dir, reserved=(SLAM_MAP_ID,))
        self.history: deque = deque(maxlen=HISTORY_SIZE)
        self.started: dict[str, int] = {}  # command id -> start time [ns since the epoch]
        self.command_reset = None  # the last sim/reset command_server has handled
        self.sounds = dict(BUILTIN_SOUNDS)  # id -> name
        self.sound_data: dict[str, bytes] = {}
        self.map_replies: dict[str, dict] = {}
        self.map_cond = threading.Condition()
        api_cfg = getattr(sim_node.cfg, "api", None)
        shortcuts = getattr(api_cfg, "shortcuts", None) if api_cfg else None
        self.shortcuts = [_ns_to_dict(s) for s in shortcuts or []]  # config/robot.yaml api.shortcuts
        self.store = Store()
        self.serial = serial
        self.map_to_odom = Pose2D(0.0, 0.0, 0.0)  # from SLAM's /tf; identity until it runs
        self.layout = {"shelves": [], "locations": []}
        self.charger: Pose2D | None = None
        self.shelf_poses: list[Pose2D] = []
        self.charging = False
        self.auto_homing = True  # command_server's default
        self.volume = 5
        n, ns = sim_node.node, sim_node.ns
        self.odom_frame, self.map_frame = sim_node.frame("odom"), "map"
        self.cmd_pub = n.create_publisher(String, f"{ns}/sim/command/start", 10)
        self.cancel_pub = n.create_publisher(String, f"{ns}/sim/command/cancel", 10)
        from geometry_msgs.msg import Twist

        self._twist = Twist
        self.vel_pub = n.create_publisher(Twist, f"{ns}/manual_control/cmd_vel", 10)
        self.auto_homing_client = n.create_client(SetBool, f"{ns}/auto_homing/set_enabled")
        from geometry_msgs.msg import PoseWithCovarianceStamped

        self._pose_cov = PoseWithCovarianceStamped
        self.initial_pose_pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.map_load_pub = n.create_publisher(String, f"{ns}/sim/map/load", 10)
        self.sound_pub = n.create_publisher(String, f"{ns}/sim/sound", 10)
        self.map_params_client = n.create_client(GetParameters, "/map_server/get_parameters")
        n.create_subscription(String, f"{ns}/sim/map/loaded", self._on_map_loaded, 10)
        n.create_subscription(String, f"{ns}/sim/command/state", self._on_command_state, LATCHED)
        n.create_subscription(OccupancyGrid, "/map", lambda m: self.store.set("map", m), LATCHED)
        n.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        s = self.store
        s.set("command_state", None)
        s.set("last_result", None)
        s.set("manual_control", sim_node.manual_control_enabled)
        s.set("auto_homing", self.auto_homing)
        s.set("moving_shelf", "")
        s.set("volume", self.volume)
        s.set("static", None)  # serial number, version, map id: never change
        s.set("errors", [])
        s.set("static_tf", sim_node.static_tf)
        s.set("history", None)
        s.set("map_id", SLAM_MAP_ID)
        self.launch_map_id = ""  # map_server's yaml_filename last seen
        s.set("map_list", None)
        s.set("sounds", None)
        s.set("shortcuts", None)
        sim_node.tap = self.tap

        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=64))
        pb2_grpc.add_KachakaApiServicer_to_server(self, self.server)
        if self.server.add_insecure_port(f"0.0.0.0:{port}") == 0:
            raise RuntimeError(f"cannot listen on port {port}")
        self.server.start()

    def stop(self) -> None:
        self.server.stop(grace=0.5)

    # --- inputs from the simulator and ROS ----------------------------------
    def tap(self, name: str, *msg) -> None:
        """Called by KachakaSimNode with each message it publishes."""
        s = self.store
        if name == "battery":
            m = msg[0]
            self.charging = m.power_supply_status == m.POWER_SUPPLY_STATUS_CHARGING
            s.set("battery", m)
        elif name == "odometry":
            s.set("odometry", msg[0])
            s.set("pose", msg[0])
        elif name == "layout":
            self.layout = msg[0]
            self.shelf_poses = [Pose2D(*sh["home"]) for sh in self.layout["shelves"]]
            self._publish_layout()
        elif name == "charger":
            self.charger = msg[0]
            self._publish_layout()
        elif name == "shelf_poses":
            if len(msg[0]) == len(self.layout["shelves"]):
                self.shelf_poses = list(msg[0])
                s.set("shelves", None)
        elif name == "shelf_docked":
            s.set("moving_shelf", msg[0])
        elif name == "reset":
            self._reset()
        elif name == "object_detection":
            s.set("object_detection", msg[0])
        elif name.endswith("_camera"):
            img, info = msg
            s.set(f"{name}/image", img)
            s.set(f"{name}/info", info)
        else:  # imu, wheel_odometry, scan, manual_control
            s.set(name, msg[0])

    def _reset(self) -> None:
        """sim/reset: forget commands, history and settings made through the
        API. Runs in the sim loop right after the robot went back to the start."""
        self._clear_commands()
        s = self.store
        s.set("errors", [])
        s.set("manual_control", True)
        self.volume = 5
        s.set("volume", self.volume)
        self.sounds = dict(BUILTIN_SOUNDS)
        self.sound_data = {}
        s.set("sounds", None)
        self.auto_homing = True  # command_server goes back to its parameter too
        s.set("auto_homing", self.auto_homing)
        if self._saved_map_mode():
            # The robot is back at the odometry origin; tell AMCL so it does
            # not have to find it.
            m = self.map_to_odom
            self._publish_initial_pose(m.x, m.y, m.yaw)

    def _clear_commands(self) -> None:
        self.history.clear()
        self.started.clear()
        self.store.set("command_state", None)
        self.store.set("last_result", None)
        self.store.set("history", None)

    def _publish_layout(self) -> None:
        self.store.set("locations", None)
        self.store.set("shelves", None)

    def _on_tf(self, msg: TFMessage) -> None:
        self.store.set("tf", msg)
        for t in msg.transforms:
            if t.header.frame_id == self.map_frame and t.child_frame_id == self.odom_frame:
                tr = t.transform
                self.map_to_odom = Pose2D(tr.translation.x, tr.translation.y, _yaw(tr.rotation))

    def _on_command_state(self, msg: String) -> None:
        state = json.loads(msg.data)
        if state.get("reset") != self.command_reset:
            # command_server has dropped its commands for a sim/reset; what it
            # sent before that belongs to the old run.
            self.command_reset = state.get("reset")
            self._clear_commands()
        running, last = state.get("running"), state.get("last")
        prev_running = self.store.peek("command_state")
        if (running or {}).get("id") != (prev_running or {}).get("id"):
            self.store.set("command_state", running)
            if running:
                self.started.setdefault(running["id"], time.time_ns())
        prev_last = self.store.peek("last_result")
        if last and last.get("id") != (prev_last or {}).get("id"):
            self.store.set("last_result", last)
            self.history.appendleft(pb2.History(
                id=last["id"], command=self._command(last), success=last["success"], error_code=last["error_code"],
                command_executed_time=self.started.pop(last["id"], time.time_ns()) // 1_000_000_000))
            self.store.set("history", None)

    def _on_map_loaded(self, msg: String) -> None:
        reply = json.loads(msg.data)
        with self.map_cond:
            self.map_replies[reply.get("id")] = reply
            self.map_cond.notify_all()

    # --- helpers --------------------------------------------------------------
    @staticmethod
    def _meta(cursor: int) -> pb2.Metadata:
        return pb2.Metadata(cursor=cursor)

    def _robot_pose(self, odom) -> pb2.Pose:
        p = odom.pose.pose
        x, y, yaw = p.position.x, p.position.y, _yaw(p.orientation)
        m = self.map_to_odom
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        theta = math.atan2(math.sin(m.yaw + yaw), math.cos(m.yaw + yaw))
        return pb2.Pose(x=m.x + c * x - s * y, y=m.y + s * x + c * y, theta=theta)

    def _locations(self) -> list[pb2.Location]:
        locs = []
        if self.charger is not None:
            c = self.charger
            locs.append(pb2.Location(id=CHARGER_ID, name="Charger", type=pb2.LOCATION_TYPE_CHARGER,
                                     pose=pb2.Pose(x=c.x, y=c.y, theta=c.yaw)))
        for sh in self.layout["shelves"]:
            x, y, yaw = sh["home"]
            locs.append(pb2.Location(id=f"{sh['id']}_home", name=f"{sh['name']} home",
                                     type=pb2.LOCATION_TYPE_SHELF_HOME, pose=pb2.Pose(x=x, y=y, theta=yaw)))
        for loc in self.layout["locations"]:
            x, y, yaw = loc["pose"]
            locs.append(pb2.Location(id=loc["id"], name=loc["name"], type=pb2.LOCATION_TYPE_UNSPECIFIED,
                                     pose=pb2.Pose(x=x, y=y, theta=yaw)))
        return locs

    def _shelves(self) -> list[pb2.Shelf]:
        out = []
        for sh, p in zip(self.layout["shelves"], self.shelf_poses):
            out.append(pb2.Shelf(id=sh["id"], name=sh["name"], pose=pb2.Pose(x=p.x, y=p.y, theta=p.yaw),
                                 size=pb2.ShelfSize(width=sh["width"], depth=sh["depth"]),
                                 appearance=pb2.SHELF_APPEARANCE_DEFAULT_SHELF,
                                 home_location_id=f"{sh['id']}_home",
                                 speed_mode=pb2.SHELF_SPEED_MODE_NORMAL))
        return out

    @staticmethod
    def _command(entry) -> pb2.Command:
        return command_from_json(entry["command_type"], entry.get("fields", {}))

    def _start(self, command: pb2.Command, cancel_all: bool, tts_on_success: str = "") -> tuple[pb2.Result, str]:
        if self.sim.emergency_stop:
            return pb2.Result(success=False, error_code=ERROR_EMERGENCY_STOP), ""
        if self.sim.paused:
            return pb2.Result(success=False, error_code=ERROR_PAUSED), ""
        if self.cmd_pub.get_subscription_count() == 0:
            return pb2.Result(success=False, error_code=ERROR_SERVER_UNAVAILABLE), ""
        command_type, fields = command_to_json(command)
        cmd_id = uuid.uuid4().hex
        self.cmd_pub.publish(String(data=json.dumps({
            "id": cmd_id, "command_type": command_type, "fields": fields,
            "cancel_all": cancel_all, "tts_on_success": tts_on_success})))
        return pb2.Result(success=True), cmd_id

    def _wait_future(self, future, timeout: float):
        """The simulation loop spins the node, so a service future completes
        while this gRPC thread waits."""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _saved_map_mode(self) -> bool:
        """navigation.launch.py (map_server + AMCL) rather than SLAM."""
        return self.map_params_client.service_is_ready()

    def _refresh_map_id(self) -> None:
        """The map map_server was launched with, if it runs. load_map (SwitchMap)
        does not change that parameter, so it counts only when it changes:
        a new navigation.launch.py run."""
        if not self._saved_map_mode():
            return
        res = self._wait_future(self.map_params_client.call_async(GetParameters.Request(names=["yaml_filename"])),
                                2.0)
        if res and res.values and res.values[0].string_value:
            map_id = Path(res.values[0].string_value).stem
            if map_id != self.launch_map_id:
                self.launch_map_id = map_id
                self.store.set("map_id", map_id)

    def _map_name(self, map_id: str) -> str:
        try:
            return str(self.maps.meta(map_id).get("name") or map_id)
        except MapError:
            return map_id

    def _live_map(self) -> MapFile | None:
        m = self.store.peek("map")
        if m is None:
            return None
        i = m.info
        c = self.charger
        return MapFile.from_occupancy_grid(m.data, i.width, i.height, i.resolution,
                                           (i.origin.position.x, i.origin.position.y, _yaw(i.origin.orientation)),
                                           name="slam", charger_pose=(c.x, c.y, c.yaw) if c else None)

    def _publish_initial_pose(self, x: float, y: float, yaw: float) -> bool:
        if self.initial_pose_pub.get_subscription_count() == 0:  # no AMCL
            return False
        msg = self._pose_cov()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.sim.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = 0.05 ** 2
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)
        return True

    # --- robot info -----------------------------------------------------------
    def GetRobotSerialNumber(self, request, context):
        cursor, _ = self.store.get("static", request, context)
        return pb2.GetRobotSerialNumberResponse(metadata=self._meta(cursor), serial_number=self.serial)

    def GetRobotVersion(self, request, context):
        cursor, _ = self.store.get("static", request, context)
        return pb2.GetRobotVersionResponse(metadata=self._meta(cursor), version=str(self.sim.cfg.stubs.version))

    def IsReady(self, request, context):
        return pb2.IsReadyResponse(ready=self.store.peek("odometry") is not None)

    def GetError(self, request, context):
        cursor, errors = self.store.get("errors", request, context)
        return pb2.GetErrorResponse(metadata=self._meta(cursor), error_codes=errors)

    def GetRobotErrorCodeJson(self, request, context):
        items = [{"code": code, "error_type": ERROR_TYPES.get(code, "error"), "title": title, "description": desc, "title_en": title,
                  "description_en": desc, "ref_url": ""} for code, (title, desc) in ERROR_CODES.items()]
        return pb2.GetRobotErrorCodeJsonResponse(json=json.dumps(items), result=pb2.Result(success=True))

    def GetRobotPose(self, request, context):
        cursor, odom = self.store.get("pose", request, context)
        return pb2.GetRobotPoseResponse(metadata=self._meta(cursor), pose=self._robot_pose(odom))

    def GetBatteryInfo(self, request, context):
        cursor, m = self.store.get("battery", request, context)
        status = pb2.POWER_SUPPLY_STATUS_CHARGING if m.power_supply_status == m.POWER_SUPPLY_STATUS_CHARGING \
            else pb2.POWER_SUPPLY_STATUS_DISCHARGING
        return pb2.GetBatteryInfoResponse(metadata=self._meta(cursor), remaining_percentage=m.percentage * 100.0,
                                          power_supply_status=status)

    def GetPngMap(self, request, context):
        cursor, m = self.store.get("map", request, context)
        return pb2.GetPngMapResponse(metadata=self._meta(cursor), map=_png_map(m))

    def GetCurrentMapId(self, request, context):
        self._refresh_map_id()
        cursor, map_id = self.store.get("map_id", request, context)
        return pb2.GetCurrentMapIdResponse(metadata=self._meta(cursor), id=map_id)

    def GetMapList(self, request, context):
        cursor, _ = self.store.get("map_list", request, context)
        entries = [] if self._saved_map_mode() else [pb2.MapListEntry(id=SLAM_MAP_ID, name="SLAM (live)")]
        entries += [pb2.MapListEntry(id=i, name=self._map_name(i)) for i in self.maps.ids()]
        return pb2.GetMapListResponse(metadata=self._meta(cursor), map_list_entries=entries)

    def LoadMapPreview(self, request, context):
        if request.map_id == SLAM_MAP_ID:
            m = self.store.peek("map")
            if m is None:
                return pb2.LoadMapPreviewResponse(result=pb2.Result(success=False, error_code=ERROR_MAP))
            return pb2.LoadMapPreviewResponse(result=pb2.Result(success=True), map=_png_map(m))
        try:
            m = self.maps.load(request.map_id)
        except MapError:
            return pb2.LoadMapPreviewResponse(result=pb2.Result(success=False, error_code=ERROR_MAP))
        h, w = m.grid.shape
        return pb2.LoadMapPreviewResponse(result=pb2.Result(success=True), map=pb2.Map(
            data=m.png(), name=m.name or request.map_id, resolution=m.resolution, width=w, height=h,
            origin=pb2.Pose(x=m.origin[0], y=m.origin[1], theta=m.origin[2])))

    def ExportMap(self, request, context):
        """A zip of the Nav2 map pair (the real robot's format is its own)."""
        try:
            m = self._live_map() if request.map_id == SLAM_MAP_ID else self.maps.load(request.map_id)
        except MapError:
            m = None
        if m is None:
            yield pb2.ExportMapResponse(end_of_stream=pb2.ExportMapResponse.EndOfStream(
                result=pb2.Result(success=False, error_code=ERROR_MAP)))
            return
        data = m.zip(request.map_id)
        for i in range(0, len(data), CHUNK):
            yield pb2.ExportMapResponse(middle_of_stream=pb2.ExportMapResponse.MiddleOfStream(data=data[i:i + CHUNK]))
        yield pb2.ExportMapResponse(end_of_stream=pb2.ExportMapResponse.EndOfStream(result=pb2.Result(success=True)))

    def ImportMap(self, request_iterator, context):
        data = b"".join(r.data for r in request_iterator)
        try:
            map_id = self.maps.save(MapFile.from_zip(data))
        except (MapError, OSError):
            return pb2.ImportMapResponse(result=pb2.Result(success=False, error_code=ERROR_MAP))
        self.store.set("map_list", None)
        return pb2.ImportMapResponse(result=pb2.Result(success=True), map_id=map_id)

    def ImportImageAsMap(self, request_iterator, context):
        data, charger, resolution = b"", None, 0.0
        for r in request_iterator:
            data += r.data
            if r.HasField("charger_pose"):
                charger = (r.charger_pose.x, r.charger_pose.y, r.charger_pose.theta)
            resolution = r.resolution or resolution
        try:
            m = MapFile.from_image(data, resolution or DEFAULT_RESOLUTION, name="imported", charger_pose=charger)
            map_id = self.maps.save(m)
        except (OSError, ValueError, MapError):  # PIL raises OSError subclasses for bad images
            return pb2.ImportImageAsMapResponse(result=pb2.Result(success=False, error_code=ERROR_MAP))
        self.store.set("map_list", None)
        return pb2.ImportImageAsMapResponse(result=pb2.Result(success=True), map_id=map_id)

    def SwitchMap(self, request, context):
        def fail(code):
            return pb2.SwitchMapResponse(result=pb2.Result(success=False, error_code=code))

        if request.inherit_docking_state_and_docked_shelf:
            return fail(ERROR_PRO_ONLY)
        saved = self._saved_map_mode()
        if request.map_id == SLAM_MAP_ID:
            return pb2.SwitchMapResponse(result=pb2.Result(success=True)) if not saved else fail(ERROR_NEEDS_SAVED_MAP)
        try:
            meta = self.maps.meta(request.map_id)
        except MapError:
            return fail(ERROR_MAP)
        if not saved:
            return fail(ERROR_NEEDS_SAVED_MAP)
        if self.map_load_pub.get_subscription_count() == 0:
            return fail(ERROR_SERVER_UNAVAILABLE)
        req_id = uuid.uuid4().hex
        self.map_load_pub.publish(String(data=json.dumps({"id": req_id, "map_id": request.map_id})))
        with self.map_cond:
            if not self.map_cond.wait_for(lambda: req_id in self.map_replies, timeout=20.0):
                return fail(ERROR_SERVER_UNAVAILABLE)
            reply = self.map_replies.pop(req_id)
        if not reply.get("success"):
            return fail(ERROR_MAP)
        if request.HasField("initial_pose"):
            p = request.initial_pose
            pose = (p.x, p.y, p.theta)
        else:
            pose = tuple(meta.get("charger_pose") or (0.0, 0.0, 0.0))
        self._publish_initial_pose(*pose)
        self.store.set("map_id", request.map_id)
        return pb2.SwitchMapResponse(result=pb2.Result(success=True))

    def SetRobotPose(self, request, context):
        p = request.pose
        if not self._publish_initial_pose(p.x, p.y, p.theta):
            return pb2.SetRobotPoseResponse(result=pb2.Result(success=False, error_code=ERROR_NEEDS_SAVED_MAP))
        return pb2.SetRobotPoseResponse(result=pb2.Result(success=True))

    def GetStaticTransform(self, request, context):
        cursor, m = self.store.get("static_tf", request, context)
        return pb2.GetStaticTransformResponse(metadata=self._meta(cursor), transforms=_transforms(m))

    def GetDynamicTransform(self, request, context):
        """Every /tf message, until the client goes away."""
        s, cursor = self.store, 0
        while context.is_active():
            with s.cond:
                entry = s.data.get("tf")
                if entry is None or entry[0] == cursor:
                    s.cond.wait(0.5)
                    continue
            cursor, m = entry
            yield pb2.GetDynamicTransformResponse(transforms=_transforms(m))

    # --- sensors --------------------------------------------------------------
    def GetRosImu(self, request, context):
        cursor, m = self.store.get("imu", request, context)
        return pb2.GetRosImuResponse(metadata=self._meta(cursor), imu=_imu(m))

    def GetRosOdometry(self, request, context):
        cursor, m = self.store.get("odometry", request, context)
        return pb2.GetRosOdometryResponse(metadata=self._meta(cursor), odometry=_odometry(m))

    def GetRosWheelOdometry(self, request, context):
        cursor, m = self.store.get("wheel_odometry", request, context)
        return pb2.GetRosWheelOdometryResponse(metadata=self._meta(cursor), odometry=_odometry(m))

    def GetRosLaserScan(self, request, context):
        cursor, m = self.store.get("scan", request, context)
        return pb2.GetRosLaserScanResponse(metadata=self._meta(cursor), scan=_scan(m))

    def _camera(self, name, kind, request, context):
        cursor, m = self.store.get(f"{name}/{'info' if kind == 'info' else 'image'}", request, context)
        if kind == "info":
            return cursor, _camera_info(m)
        return cursor, _image(m) if kind == "raw" else _compressed(m)

    def GetFrontCameraRosCameraInfo(self, request, context):
        c, m = self._camera("front_camera", "info", request, context)
        return pb2.GetFrontCameraRosCameraInfoResponse(metadata=self._meta(c), camera_info=m)

    def GetFrontCameraRosImage(self, request, context):
        c, m = self._camera("front_camera", "raw", request, context)
        return pb2.GetFrontCameraRosImageResponse(metadata=self._meta(c), image=m)

    def GetFrontCameraRosCompressedImage(self, request, context):
        c, m = self._camera("front_camera", "compressed", request, context)
        return pb2.GetFrontCameraRosCompressedImageResponse(metadata=self._meta(c), image=m)

    def GetBackCameraRosCameraInfo(self, request, context):
        c, m = self._camera("back_camera", "info", request, context)
        return pb2.GetBackCameraRosCameraInfoResponse(metadata=self._meta(c), camera_info=m)

    def GetBackCameraRosImage(self, request, context):
        c, m = self._camera("back_camera", "raw", request, context)
        return pb2.GetBackCameraRosImageResponse(metadata=self._meta(c), image=m)

    def GetBackCameraRosCompressedImage(self, request, context):
        c, m = self._camera("back_camera", "compressed", request, context)
        return pb2.GetBackCameraRosCompressedImageResponse(metadata=self._meta(c), image=m)

    def GetTofCameraRosCameraInfo(self, request, context):
        c, m = self._camera("tof_camera", "info", request, context)
        return pb2.GetTofCameraRosCameraInfoResponse(metadata=self._meta(c), camera_info=m)

    def GetTofCameraRosImage(self, request, context):
        if self.charging:  # the ToF camera is off on the charger
            c, _ = self.store.get("battery", request, context)
            return pb2.GetTofCameraRosImageResponse(metadata=self._meta(c), is_available=False)
        c, m = self._camera("tof_camera", "raw", request, context)
        return pb2.GetTofCameraRosImageResponse(metadata=self._meta(c), image=m, is_available=True)

    def GetTofCameraRosCompressedImage(self, request, context):
        if self.charging:
            c, _ = self.store.get("battery", request, context)
            return pb2.GetTofCameraRosCompressedImageResponse(metadata=self._meta(c), is_available=False)
        c, m = self._camera("tof_camera", "compressed", request, context)
        return pb2.GetTofCameraRosCompressedImageResponse(metadata=self._meta(c), image=m, is_available=True)

    def _object_detection(self, request, context) -> tuple[int, pb2.RosHeader, list]:
        """The newest front camera detections (detection.py). Without them
        (--no-cameras, or object_detection disabled) the newest odometry
        paces an always-empty result."""
        if not self.sim.object_detection_enabled:
            cursor, m = self.store.get("odometry", request, context)
            return cursor, pb2.RosHeader(stamp_nsec=_header(m.header).stamp_nsec,
                                         frame_id=self.sim.frame("camera_front")), []
        cursor, d = self.store.get("object_detection", request, context)
        sec, nsec = d["stamp"]
        return cursor, pb2.RosHeader(stamp_nsec=sec * 1_000_000_000 + nsec, frame_id=d["frame_id"]), d["objects"]

    def GetObjectDetection(self, request, context):
        cursor, header, objects = self._object_detection(request, context)
        return pb2.GetObjectDetectionResponse(metadata=self._meta(cursor), header=header, objects=[
            pb2.ObjectDetection(label=o["label"], score=o["score"], distance_median=o["distance"],
                                roi=pb2.RegionOfInterest(x_offset=o["roi"][0], y_offset=o["roi"][1],
                                                         width=o["roi"][2], height=o["roi"][3]))
            for o in objects])

    def GetObjectDetectionFeatures(self, request, context):
        # The real robot returns its detector's feature maps; the simulator
        # has no detector network, so the list stays empty. Paced like
        # GetObjectDetection.
        cursor, header, _ = self._object_detection(request, context)
        return pb2.GetObjectDetectionFeaturesResponse(metadata=self._meta(cursor), header=header)

    def ActivateLaserScan(self, request, context):
        return pb2.ActivateLaserScanResponse(result=pb2.Result(success=True))

    # --- commands -------------------------------------------------------------
    def StartCommand(self, request, context):
        result, cmd_id = self._start(request.command, request.cancel_all, request.tts_on_success)
        return pb2.StartCommandResponse(result=result, command_id=cmd_id)

    def CancelCommand(self, request, context):
        running = self.store.peek("command_state")
        self.cancel_pub.publish(String(data=""))
        cmd = self._command(running) if running else pb2.Command()
        return pb2.CancelCommandResponse(result=pb2.Result(success=True), command=cmd)

    def GetCommandState(self, request, context):
        cursor, running = self.store.get("command_state", request, context)
        if running is None:
            return pb2.GetCommandStateResponse(metadata=self._meta(cursor), state=pb2.COMMAND_STATE_PENDING)
        return pb2.GetCommandStateResponse(metadata=self._meta(cursor), state=pb2.COMMAND_STATE_RUNNING,
                                           command=self._command(running), command_id=running["id"])

    def GetLastCommandResult(self, request, context):
        cursor, last = self.store.get("last_result", request, context)
        if last is None:
            return pb2.GetLastCommandResultResponse(metadata=self._meta(cursor))
        return pb2.GetLastCommandResultResponse(
            metadata=self._meta(cursor), command=self._command(last), command_id=last["id"],
            result=pb2.Result(success=last["success"], error_code=last["error_code"]))

    def Proceed(self, request, context):
        return pb2.ProceedResponse(result=pb2.Result(success=True))

    def GetHistoryList(self, request, context):
        cursor, _ = self.store.get("history", request, context)
        return pb2.GetHistoryListResponse(metadata=self._meta(cursor), histories=list(self.history))

    def GetShortcuts(self, request, context):
        cursor, _ = self.store.get("shortcuts", request, context)
        return pb2.GetShortcutsResponse(metadata=self._meta(cursor), shortcuts=[
            pb2.Shortcut(id=s["id"], name=s["name"]) for s in self.shortcuts])

    def StartShortcutCommand(self, request, context):
        """Runs the shortcut's commands one after another, stopping at the
        first that fails."""
        shortcut = next((s for s in self.shortcuts if s["id"] == request.target_shortcut_id), None)
        if shortcut is None:
            return pb2.StartShortcutCommandResponse(result=pb2.Result(success=False, error_code=ERROR_UNKNOWN_SHELF))
        try:
            commands = [json_format.ParseDict(c, pb2.Command()) for c in shortcut["commands"]]
        except json_format.ParseError:
            return pb2.StartShortcutCommandResponse(result=pb2.Result(success=False, error_code=-1))
        if not commands:
            return pb2.StartShortcutCommandResponse(result=pb2.Result(success=True))
        result, first = self._start(commands[0], request.cancel_all)
        if result.success and len(commands) > 1:
            threading.Thread(target=self._run_rest, args=(first, commands[1:]), daemon=True).start()
        return pb2.StartShortcutCommandResponse(result=result)

    def _run_rest(self, cmd_id: str, commands: list) -> None:
        s = self.store
        for command in commands:
            with s.cond:  # wait for the previous command's result
                s.cond.wait_for(lambda prev=cmd_id: (s.peek("last_result") or {}).get("id") == prev)
            if not s.peek("last_result")["success"]:
                return
            result, cmd_id = self._start(command, cancel_all=False)
            if not result.success:
                return

    # --- layout ---------------------------------------------------------------
    def GetLocations(self, request, context):
        cursor, _ = self.store.get("locations", request, context)
        return pb2.GetLocationsResponse(metadata=self._meta(cursor), locations=self._locations(),
                                        default_location_id=CHARGER_ID if self.charger is not None else "")

    def GetLocationsDigest(self, request, context):
        cursor, _ = self.store.get("locations", request, context)
        return pb2.GetLocationsDigestResponse(metadata=self._meta(cursor), locations=[
            pb2.LocationDigest(id=loc.id, name=loc.name, type=loc.type) for loc in self._locations()])

    def GetShelves(self, request, context):
        cursor, _ = self.store.get("shelves", request, context)
        return pb2.GetShelvesResponse(metadata=self._meta(cursor), shelves=self._shelves())

    def GetShelvesDigest(self, request, context):
        cursor, _ = self.store.get("shelves", request, context)
        return pb2.GetShelvesDigestResponse(metadata=self._meta(cursor), shelves=[
            pb2.ShelfDigest(id=s.id, name=s.name) for s in self._shelves()])

    def GetMovingShelfId(self, request, context):
        cursor, shelf_id = self.store.get("moving_shelf", request, context)
        return pb2.GetMovingShelfIdResponse(metadata=self._meta(cursor), shelf_id=shelf_id)

    # --- manual control and settings ----------------------------------------
    def SetManualControlEnabled(self, request, context):
        self.sim.manual_control_enabled = request.enable
        self.store.set("manual_control", request.enable)
        if not request.enable:
            self.vel_pub.publish(self._twist())
        return pb2.SetManualControlEnabledResponse(result=pb2.Result(success=True))

    def GetManualControlEnabled(self, request, context):
        cursor, enabled = self.store.get("manual_control", request, context)
        return pb2.GetManualControlEnabledResponse(metadata=self._meta(cursor), enabled=enabled)

    def SetRobotVelocity(self, request, context):
        # Like the real robot: only with manual control enabled. The velocity
        # goes out on manual_control/cmd_vel, so the simulator and the
        # auto-homing idle timer see it as they see ROS commands.
        if not self.sim.manual_control_enabled:
            return pb2.SetRobotVelocityResponse(
                result=pb2.Result(success=False, error_code=ERROR_MANUAL_CONTROL_DISABLED))
        t = self._twist()
        t.linear.x = max(-1.0, min(1.0, request.linear)) * MAX_LINEAR
        t.angular.z = max(-1.0, min(1.0, request.angular)) * MAX_ANGULAR
        self.vel_pub.publish(t)
        return pb2.SetRobotVelocityResponse(result=pb2.Result(success=True))

    def SetAutoHomingEnabled(self, request, context):
        if not self.auto_homing_client.wait_for_service(timeout_sec=2.0):
            return pb2.SetAutoHomingEnabledResponse(
                result=pb2.Result(success=False, error_code=ERROR_SERVER_UNAVAILABLE))
        future = self.auto_homing_client.call_async(SetBool.Request(data=request.enable))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:  # the sim loop spins the node
            time.sleep(0.02)
        ok = future.done() and future.result().success
        if ok:
            self.auto_homing = request.enable
            self.store.set("auto_homing", request.enable)
        return pb2.SetAutoHomingEnabledResponse(
            result=pb2.Result(success=ok, error_code=0 if ok else ERROR_SERVER_UNAVAILABLE))

    def GetAutoHomingEnabled(self, request, context):
        cursor, enabled = self.store.get("auto_homing", request, context)
        return pb2.GetAutoHomingEnabledResponse(metadata=self._meta(cursor), enabled=enabled)

    def SetFrontTorchIntensity(self, request, context):
        self.sim.set_torch("front", request.intensity)
        return pb2.SetFrontTorchIntensityResponse(result=pb2.Result(success=True))

    def SetBackTorchIntensity(self, request, context):
        self.sim.set_torch("back", request.intensity)
        return pb2.SetBackTorchIntensityResponse(result=pb2.Result(success=True))

    def GetSpeakerVolume(self, request, context):
        cursor, volume = self.store.get("volume", request, context)
        return pb2.GetSpeakerVolumeResponse(metadata=self._meta(cursor), volume=volume)

    def SetSpeakerVolume(self, request, context):
        self.volume = max(0, min(10, request.volume))
        self.store.set("volume", self.volume)
        return pb2.SetSpeakerVolumeResponse(result=pb2.Result(success=True))

    # --- sounds (no audio) -----------------------------------------------------
    def GetSoundList(self, request, context):
        cursor, _ = self.store.get("sounds", request, context)
        return pb2.GetSoundListResponse(metadata=self._meta(cursor), sounds=[
            pb2.Sound(id=i, name=n) for i, n in self.sounds.items()])

    def AddSound(self, request, context):
        sound_id = uuid.uuid4().hex
        self.sounds[sound_id] = request.name
        self.sound_data[sound_id] = request.data
        self.store.set("sounds", None)
        return pb2.AddSoundResponse(result=pb2.Result(success=True), sound_id=sound_id)

    def DeleteSound(self, request, context):
        if request.sound_id not in self.sound_data:  # built-in ones stay
            return pb2.DeleteSoundResponse(result=pb2.Result(success=False, error_code=-1))
        del self.sounds[request.sound_id], self.sound_data[request.sound_id]
        self.store.set("sounds", None)
        return pb2.DeleteSoundResponse(result=pb2.Result(success=True))

    def PlaySound(self, request, context):
        name = self.sounds.get(request.sound_id)
        if name is None:
            return pb2.PlaySoundResponse(result=pb2.Result(success=False, error_code=-1))
        self.sound_pub.publish(String(data=json.dumps({"play": request.sound_id, "name": name,
                                                       "loop": request.loop})))
        print(f"[kachaka] sound: {name}{' (loop)' if request.loop else ''}", flush=True)
        return pb2.PlaySoundResponse(result=pb2.Result(success=True))

    def StopSound(self, request, context):
        self.sound_pub.publish(String(data=json.dumps({"stop": True})))
        return pb2.StopSoundResponse(result=pb2.Result(success=True))

    # --- shelves, emergency stop, restart ----------------------------------------
    def ResetShelfPose(self, request, context):
        handler = self.sim.shelf_reset_handler
        if handler is None:
            return pb2.ResetShelfPoseResponse(result=pb2.Result(success=False, error_code=ERROR_UNKNOWN_SHELF))
        try:
            ok, msg = self.sim.run_in_sim(lambda: handler(request.shelf_id)).result(timeout=10.0)
        except Exception:  # noqa: BLE001 - timeout or a simulator error
            return pb2.ResetShelfPoseResponse(result=pb2.Result(success=False, error_code=ERROR_SERVER_UNAVAILABLE))
        code = 0 if ok else ERROR_OTHER_SHELF_DOCKED if msg.startswith("the robot tows") else ERROR_UNKNOWN_SHELF
        return pb2.ResetShelfPoseResponse(result=pb2.Result(success=ok, error_code=code))

    def SetEmergencyStop(self, request, context):
        """Stops the robot and refuses commands until RestartRobot."""
        self.sim.emergency_stop = True
        self.cancel_pub.publish(String(data=json.dumps({"all": True})))
        self.vel_pub.publish(self._twist())
        self.sim.publish_errors()
        return pb2.SetEmergencyStopResponse(result=pb2.Result(success=True))

    def RestartRobot(self, request, context):
        """Does not restart the simulator: stops all commands, releases the
        emergency stop and resumes from a pause (like the power button)."""
        self.cancel_pub.publish(String(data=json.dumps({"all": True})))
        self.vel_pub.publish(self._twist())
        self.sim.emergency_stop = False
        try:
            self.sim.run_in_sim(self.sim.power_button).result(timeout=5.0)
        except Exception:  # noqa: BLE001 - timeout or a simulator error
            self.sim.publish_errors()
        return pb2.RestartRobotResponse(result=pb2.Result(success=True))
