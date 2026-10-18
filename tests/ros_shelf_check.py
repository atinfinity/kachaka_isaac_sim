"""Check the Kachaka Shelf commands in the default room.

Run against the simulator with SLAM + Nav2 up (slam.launch.py nav:=true,
which also starts command_server and the shelf docking server). Steps:

1. The shelf is on /kachaka/layout/shelves/list and its home, the room's
   other location and the charger are on /kachaka/layout/locations/list.
2. DOCK_SHELF with no shelf in front of the robot fails; UNDOCK_SHELF with
   no shelf docked fails.
3. Nav2 drives in front of the shelf; DOCK_SHELF docks with it.
4. MOVE_SHELF tows it to the other location (turning on the way) and puts
   it down there without undocking; UNDOCK_SHELF drives out forward.
5. RETURN_SHELF takes it home again.

    python3 tests/ros_shelf_check.py
    python3 tests/ros_shelf_check.py --demo   # dock, tow, turn, undock (for the GIF)
"""

import math
import sys
import time

import rclpy
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand, Location, LocationList, ShelfList
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SHELF = "S01"
DESTINATION = "L01"


def _yaw(q) -> float:
    return 2 * math.atan2(q.z, q.w)


def _axis_err(a: float, b: float) -> float:
    """Angle between two headings along the same axis (either direction)."""
    d = abs(math.atan2(math.sin(a - b), math.cos(a - b)))
    return min(d, math.pi - d)


class ShelfCheck(Node):
    def __init__(self):
        super().__init__("shelf_check")
        self.command = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.locations = None
        self.shelves = None
        self.docked = None
        self.odom = None
        self.create_subscription(LocationList, "/kachaka/layout/locations/list",
                                 lambda m: setattr(self, "locations", m), LATCHED)
        self.create_subscription(ShelfList, "/kachaka/layout/shelves/list",
                                 lambda m: setattr(self, "shelves", m), LATCHED)
        self.create_subscription(String, "/kachaka/sim/shelf_docked",
                                 lambda m: setattr(self, "docked", m.data), LATCHED)
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

    def run_action(self, client, goal, timeout):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if not result.done():
            handle.cancel_goal_async()
            return None
        return result.result()

    def command_goal(self, command_type, **fields):
        goal = ExecKachakaCommand.Goal()
        goal.kachaka_command.command_type = command_type
        for k, v in fields.items():
            setattr(goal.kachaka_command, k, v)
        return goal

    def nav_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return goal

    def shelf(self, shelf_id):
        return next((s for s in (self.shelves.shelves if self.shelves else []) if s.id == shelf_id), None)

    def location(self, loc_id):
        return next((loc for loc in (self.locations.locations if self.locations else []) if loc.id == loc_id), None)


def main():
    demo = "--demo" in sys.argv[1:]
    rclpy.init()
    n = ShelfCheck()
    failures = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    def command(name, command_type, timeout=240.0, expect=True, **fields):
        t0 = time.monotonic()
        res = n.run_action(n.command, n.command_goal(command_type, **fields), timeout)
        ok = res is not None and res.result.success == expect
        detail = f"in {time.monotonic() - t0:.0f} s"
        if res is not None:
            detail += f" (code {res.result.error_code}) {res.result.message}"
        check(name, ok, detail)
        n.spin_for(1.0)
        return ok

    def shelf_at(name, target, tol=0.12):
        """The shelf stands at a location: the docked robot pose there. The
        robot puts it down where SLAM says the location is, a few centimetres
        off the simulator's ground truth."""
        n.spin_for(1.5)  # shelves/list follows the simulator at 2 Hz
        s = n.shelf(SHELF)
        if s is None or target is None:
            check(name, False, "(no shelf or location)")
            return
        err = math.hypot(s.pose.x - target.x, s.pose.y - target.y)
        dyaw = _axis_err(s.pose.theta, target.theta)
        check(name, err < tol and dyaw < 0.1, f"(error {err:.3f} m, {dyaw:.3f} rad)")

    for client, name in ((n.command, "kachaka_command/execute"), (n.nav, "navigate_to_pose")):
        if not client.wait_for_server(timeout_sec=60.0):
            print(f"{name} action server not available", flush=True)
            return 1

    # 1. Layout.
    n.wait_for(lambda: n.locations is not None and n.shelf(SHELF) is not None and n.docked is not None
               and n.location(DESTINATION) is not None, 20.0)
    s = n.shelf(SHELF)
    home = n.location(f"{SHELF}_home")
    dest = n.location(DESTINATION)
    check("shelf listed", s is not None and s.home_location_id == f"{SHELF}_home" and s.size.width > 0.4,
          f"({s.pose.x:.2f}, {s.pose.y:.2f}, {s.pose.theta:.2f})" if s else "")
    check("shelf home location", home is not None and home.type == Location.LOCATION_TYPE_SHELF_HOME)
    check("other location", dest is not None and dest.type == Location.LOCATION_TYPE_OTHER)
    check("charger location", n.location("home") is not None)
    if s is None or home is None or dest is None:
        return 1
    shelf_at("shelf at home", home.pose)

    # 2. Commands that must fail.
    if not demo:
        command("DOCK_SHELF with no shelf in front fails", KachakaCommand.DOCK_SHELF_COMMAND, 30.0, expect=False)
        command("UNDOCK_SHELF with no shelf fails", KachakaCommand.UNDOCK_SHELF_COMMAND, 30.0, expect=False)

    # 3. Dock: drive in front of the shelf, facing it, then DOCK_SHELF.
    yaw = home.pose.theta
    res = n.run_action(n.nav, n.nav_goal(home.pose.x - 0.9 * math.cos(yaw), home.pose.y - 0.9 * math.sin(yaw), yaw),
                       120.0)
    check("drive in front of the shelf", res is not None and res.status == 4)
    command("DOCK_SHELF", KachakaCommand.DOCK_SHELF_COMMAND)
    check("towing", n.docked == SHELF, f"({n.docked!r})")

    # 4. Tow it to the other location (with a turn on the way), put it down, leave.
    command(f"MOVE_SHELF to {DESTINATION}", KachakaCommand.MOVE_SHELF_COMMAND,
            move_shelf_command_target_shelf_id=SHELF, move_shelf_command_destination_location_id=DESTINATION,
            move_shelf_command_undock_on_destination=False)
    check("still towing", n.docked == SHELF)
    shelf_at(f"shelf at {DESTINATION}", dest.pose)
    p0 = n.odom
    command("UNDOCK_SHELF", KachakaCommand.UNDOCK_SHELF_COMMAND)
    check("not towing", n.docked == "")
    p1 = n.odom
    ahead = (p1.position.x - p0.position.x) * math.cos(_yaw(p0.orientation)) + \
        (p1.position.y - p0.position.y) * math.sin(_yaw(p0.orientation))
    check("left forward", ahead > 0.3, f"({ahead:.2f} m)")
    shelf_at(f"shelf stays at {DESTINATION}", dest.pose)

    # 5. Return it home.
    if not demo:
        command("RETURN_SHELF", KachakaCommand.RETURN_SHELF_COMMAND, return_shelf_command_target_shelf_id=SHELF)
        check("not towing after RETURN_SHELF", n.docked == "")
        shelf_at("shelf back home", home.pose)

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    n.destroy_node()
    rclpy.try_shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
