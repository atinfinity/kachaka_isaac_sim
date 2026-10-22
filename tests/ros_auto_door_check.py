"""Nav2 through an automatic door that is never commanded (#63).

Run against the simulator started with layouts/room_auto_door.yaml (a wall
at x = 0.6 whose only way through is an automatic sliding door at y = -0.6;
scripts/smoke_test.sh --nav does this) and SLAM + Nav2 up
(slam.launch.py nav:=true). The door senses the robot within door.auto.range
of the doorway (config/robot.yaml) and closes door.auto.hold s after it
leaves.

1. The robot starts in the sensor area, so the door is open. It drives east
   through the doorway. Once it is past the wall the SLAM map is frozen
   (slam_toolbox/pause_new_measurements) with the doorway open, as on a
   saved map: from here on only the costmap's obstacle layer sees the door.
2. Out of the sensor area east of the wall, the door closes behind it.
3. The robot is sent up to the closed door: the door opens by itself as
   the robot comes near.
4. The robot is sent west through the doorway and on out of the sensor
   area: the door closes behind it again.

No door command is sent. Each leg must reach its goal, the crossings must
be in the doorway, and the robot must touch nothing (sim/collision). Poses
are in the world frame, which is the map frame here (spawn at the origin).

    python3 tests/ros_auto_door_check.py
"""

import argparse
import json
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sim_reset import add_reset_args, reset_sim
from slam_toolbox.srv import Pause
from std_msgs.msg import String

# The geometry of layouts/room_auto_door.yaml.
WALL_X = 0.6
DOORWAY = (-1.0, -0.2)  # y range of the doorway
EAST = (1.8, -0.6, 0.0)  # 1.2 m from the doorway: out of the sensor area
FRONT = (1.1, -0.6, math.pi)  # 0.5 m in front of the door, facing it
WEST = (-0.8, -0.6, math.pi)  # 1.4 m from the doorway on the other side
TOLERANCE = 0.35  # m
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AutoDoorCheck(Node):
    def __init__(self):
        super().__init__("auto_door_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.slam_pause = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.gt = None
        self.crossings = []  # y where the robot crossed x = WALL_X
        self.collisions = 0
        self.door = None  # the door's entry in sim/doors
        self.states = []  # door states seen, in order
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.create_subscription(String, "/kachaka/sim/collision", self.on_collision, 10)
        self.create_subscription(String, "/kachaka/sim/doors", self.on_doors, LATCHED)

    def on_gt(self, msg):
        p = msg.pose.pose.position
        if self.gt is not None and (self.gt[0] - WALL_X) * (p.x - WALL_X) < 0:
            self.crossings.append(round(p.y, 2))
        self.gt = (p.x, p.y)

    def on_collision(self, msg):
        self.collisions += 1
        print(f"  collision: {msg.data}", flush=True)

    def on_doors(self, msg):
        self.door = json.loads(msg.data)["doors"][0]
        state = self.door["state"]
        if not self.states or self.states[-1] != state:
            self.states.append(state)
            print(f"  door: {state}", flush=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def toggle_slam(self):
        """Pause or resume slam_toolbox's new measurements (a toggle)."""
        if not self.slam_pause.wait_for_service(timeout_sec=5.0):
            return False
        future = self.slam_pause.call_async(Pause.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None

    def wait_door(self, state, timeout=10.0):
        end = self.now() + timeout
        while (self.door is None or self.door["state"] != state) and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.door is not None and self.door["state"] == state

    def go(self, goal, during=None, timeout=240.0):
        """Navigate to `goal`; `during()` runs while it drives."""
        x, y, yaw = goal
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        result = handle.get_result_async()
        start = self.now()
        while not result.done() and self.now() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if during:
                during()
        if not result.done():
            handle.cancel_goal_async()
            return False, "timeout"
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        err = math.hypot(self.gt[0] - x, self.gt[1] - y)
        return ok and err < TOLERANCE, f"status {result.result().status}, error {err:.2f} m, {self.now() - start:.0f} s"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_reset_args(ap)
    args = ap.parse_args()
    rclpy.init()
    node = AutoDoorCheck()
    if not args.no_reset:
        reset_sim(node, args.clear_map)
    if not node.client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose not found (slam.launch.py nav:=true?)")
        return 1
    results = []

    def check(name, ok, why=""):
        print(f"{name}: {'OK' if ok else 'FAIL'}{f' ({why})' if why else ''}", flush=True)
        results.append((name, ok))

    def leg(name, goal, expect=None, during=None):
        node.crossings = []
        ok, why = node.go(goal, during)
        check(name, ok, why)
        if expect:
            lo, hi = expect
            crossed = bool(node.crossings) and all(lo <= y <= hi for y in node.crossings)
            check(f"{name} crossing", crossed, f"y {node.crossings}, expected {lo} to {hi}")

    node.wait_door("open")
    auto = node.door is not None and node.door.get("auto") is True
    check("automatic door, open for the robot at the spawn", auto and node.door["state"] == "open",
          f"{node.door}")
    if node.door is None:
        return 1

    paused = []

    def freeze_past_the_wall():
        # Still in the sensor area here, so the doorway is open in the map.
        if not paused and node.gt[0] > WALL_X + 0.4:
            paused.append(node.toggle_slam())

    leg("east through the open door", EAST, DOORWAY, freeze_past_the_wall)
    check("SLAM map frozen", bool(paused) and paused[0])
    try:
        check("door closes behind the robot", node.wait_door("closed"), node.door["state"])
        node.states = [node.door["state"]]
        leg("up to the closed door", FRONT)
        opened = node.wait_door("open", timeout=5.0)
        check("door opens by itself as the robot comes near", opened, " -> ".join(node.states))
        leg("west through the door", WEST, DOORWAY)
        check("door closes behind the robot again", node.wait_door("closed"), node.door["state"])
    finally:
        if paused and paused[0]:
            node.toggle_slam()  # resume mapping
    check("no collisions", node.collisions == 0, str(node.collisions))
    failed = [n for n, ok in results if not ok]
    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
