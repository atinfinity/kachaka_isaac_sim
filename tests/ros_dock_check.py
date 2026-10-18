"""Check the charger, battery and RETURN_HOME in the default room.

Run against the simulator with SLAM + Nav2 up (slam.launch.py nav:=true,
which also starts kachaka_sim_bringup's command_server). Steps:

1. The charger is on /kachaka/layout/locations/list and the battery discharges.
2. An unsupported command aborts.
3. RETURN_HOME docks the robot: it reports success, the battery charges, the
   robot sits at the charger pose.
4. A Nav2 goal drives the robot off the charger and the battery discharges.
5. A RETURN_HOME canceled midway reports canceled.

    python3 tests/ros_dock_check.py
    python3 tests/ros_dock_check.py --demo   # only turn, dock and leave (for the GIF)
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand, Location, LocationList
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CHARGING = BatteryState.POWER_SUPPLY_STATUS_CHARGING
DISCHARGING = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING


class DockCheck(Node):
    def __init__(self):
        super().__init__("dock_check")
        self.command = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.locations = None
        self.battery = None
        self.odom = None
        self.create_subscription(LocationList, "/kachaka/layout/locations/list",
                                 lambda m: setattr(self, "locations", m), LATCHED)
        self.create_subscription(BatteryState, "/kachaka/robot_info/battery_state",
                                 lambda m: setattr(self, "battery", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/kachaka/odometry/odometry",
                                 lambda m: setattr(self, "odom", m.pose.pose), qos_profile_sensor_data)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def run_action(self, client, goal, timeout, cancel_after=None):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        if cancel_after is not None:
            self.spin_for(cancel_after)
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=10.0)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if not result.done():
            handle.cancel_goal_async()
            return None
        return result.result()

    def command_goal(self, command_type):
        goal = ExecKachakaCommand.Goal()
        goal.kachaka_command.command_type = command_type
        return goal

    def nav_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return goal


def main():
    demo = "--demo" in sys.argv[1:]
    rclpy.init()
    n = DockCheck()
    failures = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    for client, name in ((n.command, "kachaka_command/execute"), (n.nav, "navigate_to_pose")):
        if not client.wait_for_server(timeout_sec=60.0):
            print(f"{name} action server not available", flush=True)
            return 1

    # 1. Charger location and battery.
    n.wait_for(lambda: n.locations is not None and n.battery is not None and n.odom is not None, 15.0)
    chargers = [loc for loc in (n.locations.locations if n.locations else []) if loc.type == Location.LOCATION_TYPE_CHARGER]
    check("charger location", len(chargers) == 1,
          f"({chargers[0].pose.x:.2f}, {chargers[0].pose.y:.2f}, {chargers[0].pose.theta:.2f})" if chargers else "")
    b = n.battery
    check("battery discharging", b is not None and b.power_supply_status == DISCHARGING and b.voltage == -1.0,
          f"({b.percentage:.3f})" if b else "")
    if not chargers:
        return 1
    home = chargers[0].pose

    # 2. Unsupported commands abort.
    if not demo:
        res = n.run_action(n.command, n.command_goal(KachakaCommand.SPEAK_COMMAND), 10.0)
        check("unsupported command aborts", res is not None and not res.result.success,
              res.result.message if res else "")

    # The charger is behind the robot at the start, outside the SLAM map.
    # Turn around so Nav2 can plan to it.
    res = n.run_action(n.nav, n.nav_goal(0.0, 0.0, math.pi), 60.0)
    check("turn around", res is not None and res.status == 4)

    # 3. Return home.
    t0 = time.monotonic()
    res = n.run_action(n.command, n.command_goal(KachakaCommand.RETURN_HOME_COMMAND), 180.0)
    ok = res is not None and res.result.success
    check("RETURN_HOME succeeds", ok, f"in {time.monotonic() - t0:.0f} s" + (f" {res.result.message}" if res else ""))
    n.wait_for(lambda: n.battery.power_supply_status == CHARGING, 3.0)
    check("charging", n.battery.power_supply_status == CHARGING)
    p = n.odom
    yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    err = math.hypot(p.position.x - home.x, p.position.y - home.y)
    dyaw = abs(math.atan2(math.sin(yaw - home.theta), math.cos(yaw - home.theta)))
    check("docked pose", err < 0.05 and dyaw < 0.15, f"(error {err:.3f} m, {dyaw:.3f} rad)")
    level = n.battery.percentage
    n.spin_for(3.0)
    check("battery rising", n.battery.percentage > level, f"({level:.4f} -> {n.battery.percentage:.4f})")

    # A second RETURN_HOME on the charger succeeds at once.
    if not demo:
        res = n.run_action(n.command, n.command_goal(KachakaCommand.RETURN_HOME_COMMAND), 10.0)
        check("RETURN_HOME when docked", res is not None and res.result.success)

    # 4. Drive off the charger with a Nav2 goal.
    res = n.run_action(n.nav, n.nav_goal(0.0, 0.0, 0.0), 120.0)
    check("leave the charger", res is not None and res.status == 4)
    n.wait_for(lambda: n.battery.power_supply_status == DISCHARGING, 3.0)
    check("discharging again", n.battery.power_supply_status == DISCHARGING)

    # 5. Cancel a RETURN_HOME.
    if not demo:
        res = n.run_action(n.command, n.command_goal(KachakaCommand.RETURN_HOME_COMMAND), 30.0,
                           cancel_after=3.0)
        check("RETURN_HOME cancel", res is not None and res.status == 5 and not res.result.success,
              f"(status {res.status if res else None})")

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    n.destroy_node()
    rclpy.try_shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
