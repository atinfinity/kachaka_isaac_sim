"""The real robot's command and layout interfaces, built on Nav2, for the simulator.

- Action `/kachaka/kachaka_command/execute` (kachaka_interfaces/ExecKachakaCommand).
  RETURN_HOME docks on the charger through Nav2's docking server: Nav2 drives
  to a staging pose in front of the dock, then the robot backs onto it, as the
  real one does. Other commands are not supported yet and abort.
- Topic `/kachaka/layout/locations/list` with the charger, from the simulator's
  `/kachaka/sim/charger_pose`.
- Relays the simulator's `/kachaka/sim/charger_detection` (the dock relative
  to the robot) to the docking server's `detected_dock_pose`, so docking does
  not depend on how well the SLAM map lines up with the world.
- Service `/kachaka/auto_homing/set_enabled`. When enabled the robot returns
  home after `auto_homing_idle_timeout` seconds without velocity commands, or
  when the battery falls below `auto_homing_low_battery`.

A real Kachaka provides all of this itself, so do not run this node with one.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand, Location, LocationList
from nav2_msgs.action import DockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

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

CHARGER_ID = "home"  # the id the real robot gives its charger


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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

        self.charger: PoseStamped | None = None
        self.battery: BatteryState | None = None
        self.last_activity = self.get_clock().now()
        self.lock = threading.Lock()  # one return-home at a time
        self.request_id = 0  # bumped by each new request; older ones stop
        self.auto_retry_at = 0.0  # monotonic time; wait after a failed auto homing

        group = ReentrantCallbackGroup()
        self.locations_pub = self.create_publisher(LocationList, f"{prefix}/layout/locations/list", LATCHED)
        self.create_subscription(PoseStamped, f"{prefix}/sim/charger_pose", self._on_charger, LATCHED,
                                 callback_group=group)
        # docking_server subscribes reliable, so the relay cannot use sensor QoS.
        detection_pub = self.create_publisher(PoseStamped, "detected_dock_pose", 1)
        self.create_subscription(PoseStamped, f"{prefix}/sim/charger_detection", detection_pub.publish,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(BatteryState, f"{prefix}/robot_info/battery_state", self._on_battery,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(Twist, f"{prefix}/manual_control/cmd_vel", self._on_cmd_vel,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_service(SetBool, f"{prefix}/auto_homing/set_enabled", self._on_set_auto_homing,
                            callback_group=group)
        self.dock_client = ActionClient(self, DockRobot, "dock_robot", callback_group=group)
        self.action_server = ActionServer(
            self, ExecKachakaCommand, f"{prefix}/kachaka_command/execute",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_timer(1.0, self._check_auto_homing, callback_group=group)

    # --- inputs ---------------------------------------------------------------
    def _on_charger(self, msg: PoseStamped) -> None:
        self.charger = msg
        p = msg.pose.position
        loc = Location(id=CHARGER_ID, name="Charger", type=Location.LOCATION_TYPE_CHARGER,
                       pose=Pose2D(x=p.x, y=p.y, theta=_yaw(msg.pose.orientation)))
        self.locations_pub.publish(LocationList(locations=[loc], default_location_id=CHARGER_ID))
        self.get_logger().info(f"charger at ({p.x:.2f}, {p.y:.2f}) in {msg.header.frame_id}")

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        if msg.linear.x or msg.angular.z:
            self.last_activity = self.get_clock().now()

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

    # --- kachaka_command/execute ----------------------------------------------
    def _execute(self, goal_handle) -> ExecKachakaCommand.Result:
        cmd = goal_handle.request.kachaka_command
        if cmd.command_type != KachakaCommand.RETURN_HOME_COMMAND:
            msg = f"command_type {cmd.command_type} is not supported by the simulator yet"
            self.get_logger().warn(msg)
            goal_handle.abort()
            return ExecKachakaCommand.Result(success=False, error_code=ERROR_UNSUPPORTED, message=msg)
        code, msg = self.return_home(goal_handle)
        result = ExecKachakaCommand.Result(success=code == OK, error_code=code, message=msg)
        if code == OK:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def return_home(self, goal_handle=None) -> tuple[int, str]:
        """Dock on the charger and wait until it charges. Blocks."""
        self.request_id += 1
        my_id = self.request_id
        with self.lock:
            self.last_activity = self.get_clock().now()
            try:
                return self._return_home(goal_handle, my_id)
            finally:
                self.last_activity = self.get_clock().now()

    def _return_home(self, goal_handle, my_id: int) -> tuple[int, str]:
        def stopped() -> bool:
            return my_id != self.request_id or (goal_handle is not None and goal_handle.is_cancel_requested)

        if self.charging:
            return OK, "already on the charger"
        if self.charger is None:
            return ERROR_NO_CHARGER, "no charger (is the simulator publishing sim/charger_pose?)"
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            return ERROR_SERVER_UNAVAILABLE, "Nav2 docking server is not running"

        # Nav2's dock pose points into the dock, opposite to the docked robot.
        dock = PoseStamped()
        dock.header.frame_id = self.charger.header.frame_id
        dock.header.stamp = self.get_clock().now().to_msg()
        dock.pose.position = self.charger.pose.position
        yaw = _yaw(self.charger.pose.orientation) + math.pi
        dock.pose.orientation.z, dock.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal = DockRobot.Goal(use_dock_id=False, dock_pose=dock, dock_type=self.dock_type,
                              max_staging_time=300.0, navigate_to_staging_pose=True)
        self.get_logger().info("returning home")
        handle = self._wait(self.dock_client.send_goal_async(goal), stopped)
        if handle is None or not handle.accepted:
            return ERROR_CANCELED if stopped() else ERROR_DOCKING_FAILED, "docking goal not accepted"
        result_future = handle.get_result_async()
        if self._wait(result_future, stopped) is None:
            handle.cancel_goal_async()
            return ERROR_CANCELED, "canceled"
        res = result_future.result().result
        if not res.success:
            return ERROR_DOCKING_FAILED, f"docking failed: error {res.error_code} {res.error_msg}".strip()

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

    @staticmethod
    def _wait(future, stopped):
        while not future.done():
            if stopped():
                return None
            time.sleep(0.05)
        return future.result()

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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
