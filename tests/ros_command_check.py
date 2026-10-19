"""Check the moving and speaking commands of kachaka_command/execute and goal_pose.

Run against the simulator with SLAM + Nav2 up (slam.launch.py nav:=true,
which also starts command_server). Steps:

1. SPEAK reports success and its text appears on /kachaka/sim/speech. An
   unsupported command fails.
2. MOVE_TO_POSE drives to a pose; `tts_on_success` is spoken afterwards.
3. MOVE_TO_LOCATION drives to the room's other location; an unknown location fails.
4. A PoseStamped on /kachaka/goal_pose drives the robot there.
5. Without `cancel_all` a command waits for the running one; with it, the
   running one is canceled.
6. MOVE_TO_LOCATION to the charger docks and charges.

    python3 tests/ros_command_check.py
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand, LocationList
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from sim_reset import add_reset_args, reset_sim
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
# Nav2's goal checker: 0.25 m, 0.25 rad. SLAM adds a little.
XY_TOL = 0.3
YAW_TOL = 0.35
ERROR_UNSUPPORTED = -1
ERROR_CANCELED = -5
ERROR_UNKNOWN_LOCATION = -8


def _yaw(q) -> float:
    return 2 * math.atan2(q.z, q.w)


def _angle(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class CommandCheck(Node):
    def __init__(self):
        super().__init__("command_check")
        self.command = ActionClient(self, ExecKachakaCommand, "/kachaka/kachaka_command/execute")
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/kachaka/goal_pose", 5)
        self.speech = []  # (monotonic time, text)
        self.locations = None
        self.battery = None
        self.create_subscription(String, "/kachaka/sim/speech",
                                 lambda m: self.speech.append((time.monotonic(), m.data)), 10)
        self.create_subscription(LocationList, "/kachaka/layout/locations/list",
                                 lambda m: setattr(self, "locations", m), LATCHED)
        self.create_subscription(BatteryState, "/kachaka/robot_info/battery_state",
                                 lambda m: setattr(self, "battery", m), qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

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

    def robot(self):
        """(x, y, yaw) in the map frame, or None."""
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def goal(self, command_type, cancel_all=False, tts_on_success="", **fields):
        goal = ExecKachakaCommand.Goal(cancel_all=cancel_all, tts_on_success=tts_on_success)
        goal.kachaka_command.command_type = command_type
        for k, v in fields.items():
            setattr(goal.kachaka_command, k, v)
        return goal

    def send(self, goal):
        """Send a goal; returns the result future, or None if rejected."""
        future = self.command.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        return handle.get_result_async()

    def result(self, future, timeout):
        if future is None:
            return None
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result().result if future.done() else None


def main():
    ap = argparse.ArgumentParser()
    add_reset_args(ap)
    args = ap.parse_args()
    rclpy.init()
    n = CommandCheck()
    if not args.no_reset:
        reset_sim(n, args.clear_map)
    failures = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    def command(name, goal, timeout=120.0, expect=True, code=None):
        t0 = time.monotonic()
        res = n.result(n.send(goal), timeout)
        ok = res is not None and res.success == expect and (code is None or res.error_code == code)
        detail = f"in {time.monotonic() - t0:.0f} s"
        if res is not None:
            detail += f" (code {res.error_code}) {res.message}"
        check(name, ok, detail)
        return res

    def at(name, x, y, yaw):
        n.spin_for(0.5)
        r = n.robot()
        if r is None:
            check(name, False, "(no map -> base_footprint)")
            return
        err, dyaw = math.hypot(r[0] - x, r[1] - y), _angle(r[2], yaw)
        check(name, err < XY_TOL and dyaw < YAW_TOL, f"(error {err:.2f} m, {dyaw:.2f} rad)")

    def heard(text, since):
        return any(t >= since and s == text for t, s in n.speech)

    def move_goal(x, y, yaw, **kw):
        return n.goal(KachakaCommand.MOVE_TO_POSE_COMMAND, move_to_pose_command_x=x, move_to_pose_command_y=y,
                      move_to_pose_command_yaw=yaw, **kw)

    if not n.command.wait_for_server(timeout_sec=60.0):
        print("kachaka_command/execute action server not available", flush=True)
        return 1
    n.wait_for(lambda: n.locations is not None and n.robot() is not None, 30.0)
    locs = {loc.id: loc.pose for loc in (n.locations.locations if n.locations else [])}

    # 1. SPEAK and an unsupported command.
    t = time.monotonic()
    command("SPEAK", n.goal(KachakaCommand.SPEAK_COMMAND, speak_command_text="hello"), 10.0)
    check("speech published", n.wait_for(lambda: heard("hello", t), 3.0))
    command("unsupported command fails", n.goal(KachakaCommand.COMMAND_TYPE_OTHER), 10.0, expect=False,
            code=ERROR_UNSUPPORTED)

    # 2. MOVE_TO_POSE with tts_on_success.
    t = time.monotonic()
    command("MOVE_TO_POSE", move_goal(1.6, -0.3, math.pi, tts_on_success="arrived"))
    at("at the pose", 1.6, -0.3, math.pi)
    check("tts_on_success spoken", n.wait_for(lambda: heard("arrived", t), 3.0))

    # 3. MOVE_TO_LOCATION.
    command("MOVE_TO_LOCATION unknown fails",
            n.goal(KachakaCommand.MOVE_TO_LOCATION_COMMAND, move_to_location_command_target_location_id="nowhere"),
            10.0, expect=False, code=ERROR_UNKNOWN_LOCATION)
    if "L01" in locs:
        command("MOVE_TO_LOCATION L01",
                n.goal(KachakaCommand.MOVE_TO_LOCATION_COMMAND, move_to_location_command_target_location_id="L01"))
        p = locs["L01"]
        at("at L01", p.x, p.y, p.theta)
    else:
        check("location L01 listed", False)

    # 4. goal_pose.
    gp = PoseStamped()
    gp.header.frame_id = "map"
    gp.pose.position.x, gp.pose.position.y = 0.2, -1.5
    gp.pose.orientation.w = 1.0
    n.goal_pose_pub.publish(gp)
    ok = n.wait_for(lambda: (r := n.robot()) is not None and math.hypot(r[0] - 0.2, r[1] + 1.5) < XY_TOL, 120.0)
    check("goal_pose reached", ok)
    n.spin_for(3.0)  # let the goal_pose command finish

    # 5. Queueing and cancel_all.
    t = time.monotonic()
    move = n.send(move_goal(0.0, 0.0, 0.0))
    n.spin_for(1.0)
    speak = n.send(n.goal(KachakaCommand.SPEAK_COMMAND, speak_command_text="queued"))
    n.spin_for(2.0)
    check("queued command waits", not heard("queued", t) and not move.done())
    res = n.result(move, 120.0)
    t_move = time.monotonic()
    check("running command finishes first", res is not None and res.success,
          f"(code {res.error_code})" if res else "")
    res = n.result(speak, 10.0)
    check("queued command runs after it", res is not None and res.success
          and n.wait_for(lambda: heard("queued", t), 3.0) and min(s for s, x in n.speech if x == "queued") >= t_move - 0.5)

    first = n.send(move_goal(-1.8, 1.3, math.pi / 2))
    n.spin_for(3.0)
    second = n.send(move_goal(0.2, -1.5, 0.0, cancel_all=True))
    res = n.result(first, 30.0)
    check("cancel_all cancels the running command", res is not None and not res.success
          and res.error_code == ERROR_CANCELED, f"(code {res.error_code})" if res else "")
    res = n.result(second, 120.0)
    check("cancel_all command runs", res is not None and res.success, f"(code {res.error_code})" if res else "")
    at("at the second pose", 0.2, -1.5, 0.0)

    # 6. MOVE_TO_LOCATION to the charger.
    command("MOVE_TO_LOCATION home", n.goal(KachakaCommand.MOVE_TO_LOCATION_COMMAND,
                                            move_to_location_command_target_location_id="home"), 180.0)
    check("charging", n.wait_for(lambda: n.battery is not None and n.battery.power_supply_status
                                 == BatteryState.POWER_SUPPLY_STATUS_CHARGING, 5.0))

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    n.destroy_node()
    rclpy.try_shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
