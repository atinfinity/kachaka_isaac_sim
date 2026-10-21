"""Nav2 around a door that closes and opens (#26).

Run against the simulator started with a partition wall and a door (see
DOOR_ARGS; scripts/smoke_test.sh --nav does this) and SLAM + Nav2 up
(slam.launch.py nav:=true). A wall at x = 0.6 splits the room. It has the
door at y = -0.6 and an open gap at its north end (y 1.2-2.0).

1. Door open: the robot drives east through the doorway. Then the SLAM map
   is frozen (slam_toolbox/pause_new_measurements) with the door open, as
   when navigating on a saved map: from here on only the costmap's obstacle
   layer sees the door. (Left running, slam_toolbox would learn the closed
   door, and it takes many more scans of the open door to unlearn it.)
2. The door closes just after the robot is sent back west: Nav2 replans and
   goes round through the north gap.
3. The door opens while the robot waits west of it, then the robot is sent
   east: Nav2 goes through the doorway.

Each leg must reach its goal, cross the wall where expected and touch
nothing (sim/collision). Poses are in the world frame, which is the map
frame here (spawn at the origin).

    python3 tests/ros_door_check.py
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
from std_srvs.srv import SetBool

# scripts/run_sim.py arguments for this check.
DOOR_ARGS = ("--no-shelves --no-people --wall 0.6 -2.0 0.6 -1.0 --wall 0.6 -0.2 0.6 1.2 "
             "--door 0.6 -0.6 1.5708")
WALL_X = 0.6
DOORWAY = (-1.0, -0.2)  # y range of the doorway
GAP = (1.2, 2.0)  # y range of the open gap at the north end
EAST = (1.8, -0.6, 0.0)
WEST = (-0.4, -0.6, 0.0)  # facing the door
TOLERANCE = 0.35  # m
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DoorCheck(Node):
    def __init__(self):
        super().__init__("door_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.door = self.create_client(SetBool, "/kachaka/sim/doors/D01/set_open")
        self.slam_pause = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.gt = None
        self.crossings = []  # y where the robot crossed x = WALL_X
        self.collisions = 0
        self.door_state = None
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
        self.door_state = json.loads(msg.data)["doors"][0]["state"]

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_door(self, open_):
        future = self.door.call_async(SetBool.Request(data=open_))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        print(f"  door: {res.message if res else 'no answer'}", flush=True)
        return res is not None and res.success

    def toggle_slam(self):
        """Pause or resume slam_toolbox's new measurements (a toggle)."""
        if not self.slam_pause.wait_for_service(timeout_sec=5.0):
            return False
        future = self.slam_pause.call_async(Pause.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None

    def wait_door(self, state, timeout=10.0):
        end = self.now() + timeout
        while self.door_state != state and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.door_state == state

    def go(self, goal, during=None, timeout=240.0):
        """Navigate to `goal`; `during(elapsed)` runs while it drives."""
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
                during(self.now() - start)
        if not result.done():
            handle.cancel_goal_async()
            return False, "timeout"
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        err = math.hypot(self.gt[0] - x, self.gt[1] - y)
        return ok and err < TOLERANCE, f"status {result.result().status}, error {err:.2f} m, {self.now() - start:.0f} s"


def once(fn):
    done = []

    def run(elapsed):
        if not done and fn(elapsed):
            done.append(True)
    return run


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_reset_args(ap)
    args = ap.parse_args()
    rclpy.init()
    node = DoorCheck()
    if not args.no_reset:
        reset_sim(node, args.clear_map)
    if not node.client.wait_for_server(timeout_sec=60.0) or not node.door.wait_for_service(timeout_sec=10.0):
        print("navigate_to_pose or sim/doors/D01/set_open not found (simulator started with DOOR_ARGS?)")
        return 1
    results = []

    def leg(name, goal, expect, during=None):
        node.crossings = []
        ok, why = node.go(goal, during)
        lo, hi = expect
        crossed = bool(node.crossings) and all(lo <= y <= hi for y in node.crossings)
        print(f"{name}: {'OK' if ok else 'FAIL'} ({why})", flush=True)
        print(f"{name} crossing: {'OK' if crossed else 'FAIL'} (y {node.crossings}, expected {lo} to {hi})",
              flush=True)
        results.extend([(name, ok), (f"{name} crossing", crossed)])

    door_open = node.wait_door("open")
    print(f"door open at the start: {'OK' if door_open else 'FAIL'} ({node.door_state})", flush=True)
    results.append(("door open at the start", door_open))
    leg("through the open door", EAST, DOORWAY)
    paused = node.toggle_slam()
    print(f"SLAM map frozen: {'OK' if paused else 'FAIL'}", flush=True)
    results.append(("SLAM map frozen", paused))
    try:
        leg("round the closing door", WEST, GAP, once(lambda t: t > 1.0 and node.set_door(False)))
        closed = node.wait_door("closed")
        results.append(("door closed", closed))
        opened = node.set_door(True) and node.wait_door("open")
        results.append(("door opened", opened))
        leg("through the door once it opens", EAST, DOORWAY)
    finally:
        if paused:
            node.toggle_slam()  # resume mapping
    results.append(("no collisions", node.collisions == 0))
    print(f"collisions: {'OK' if node.collisions == 0 else 'FAIL'} ({node.collisions})", flush=True)
    failed = [n for n, ok in results if not ok]
    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
