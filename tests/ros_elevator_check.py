"""Nav2 from one floor to another by the elevator (#48).

Run against the simulator started with layouts/elevator.yaml and Nav2 on
the first floor's saved map (scripts/smoke_test.sh --nav does this):

    python3 scripts/make_maps.py layouts/elevator.yaml
    scripts/run_sim.sh --layout layouts/elevator.yaml
    ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/elevator_1F.yaml

1. On 1F, Nav2 drives to the landing and into the car (door open).
2. sim/elevator/go 2F: the door closes, the car rides, the door opens on 2F.
   The robot must now be on 2F (ground truth 10 m further along x).
3. The 2F map is loaded (sim/map/load, command_server -> map_server) and
   AMCL is given the same map pose: each floor's map frame is the spawn
   pose moved to that floor, so the robot's map pose does not change.
4. Nav2 drives out of the car to a goal on 2F, past 2F's own wall and desk.

Each leg must reach its goal and touch nothing (sim/collision). Goals are
in the map frame (the spawn pose, the room's middle).

    python3 tests/ros_elevator_check.py
"""

import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sim_reset import add_reset_args, reset_sim
from std_msgs.msg import String

FLOOR_SPACING = 10.0  # m along x between floors in the stage (kachaka_isaac_sim/elevator.py)
LANDING = (1.7, 0.0, 0.0)  # in front of the car door on the east wall
IN_CAR = (3.25, 0.0, math.pi)  # the car's middle, facing the door
GOAL_2F = (-1.5, 0.2, math.pi)  # west of 2F's partition wall, below its desk
TOLERANCE = 0.35  # m
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ElevatorCheck(Node):
    def __init__(self):
        super().__init__("elevator_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.go_pub = self.create_publisher(String, "/kachaka/sim/elevator/go", 10)
        self.map_load_pub = self.create_publisher(String, "/kachaka/sim/map/load", 10)
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.gt = None  # world frame
        self.amcl = None  # map frame
        self.elevator = None
        self.loaded = []
        self.collisions = 0
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(String, "/kachaka/sim/collision", self.on_collision, 10)
        self.create_subscription(String, "/kachaka/sim/elevator", self.on_elevator, LATCHED)
        self.create_subscription(String, "/kachaka/sim/map/loaded", lambda m: self.loaded.append(json.loads(m.data)),
                                 10)

    def on_gt(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.gt = (p.x, p.y, 2 * math.atan2(q.z, q.w))

    def on_amcl(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.amcl = (p.x, p.y, 2 * math.atan2(q.z, q.w))

    def on_collision(self, msg):
        self.collisions += 1
        print(f"  collision: {msg.data}", flush=True)

    def on_elevator(self, msg):
        self.elevator = json.loads(msg.data)

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while not cond() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def set_pose(self, pose, timeout=20.0):
        """Give AMCL the map pose; repeat until it answers near it."""
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.amcl = None
            self.initial_pose.publish(msg)
            if self.spin_until(lambda: self.amcl is not None, 2.0) and math.hypot(self.amcl[0] - x,
                                                                                    self.amcl[1] - y) < 0.1:
                return True
        return False

    def go(self, goal, floor, timeout=180.0, accept_within=10.0):
        """Navigate to `goal` (map frame) on `floor` (0: the first). The goal
        is sent again until accepted for `accept_within` seconds: on a saved
        map Nav2 only becomes active once AMCL has a pose."""
        x, y, yaw = goal
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        end = time.monotonic() + accept_within
        while True:
            future = self.client.send_goal_async(msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            handle = future.result()
            if handle is not None and handle.accepted:
                break
            if time.monotonic() > end:
                return False, "goal rejected"
            self.spin_until(lambda: False, 2.0)
        result = handle.get_result_async()
        start = time.monotonic()
        if not self.spin_until(result.done, timeout):
            handle.cancel_goal_async()
            return False, "timeout"
        ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        err = math.hypot(self.gt[0] - floor * FLOOR_SPACING - x, self.gt[1] - y)
        return ok and err < TOLERANCE, \
            f"status {result.result().status}, error {err:.2f} m, {time.monotonic() - start:.0f} s"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_reset_args(ap)
    ap.add_argument("--maps", default="elevator", help="map id prefix: <prefix>_1F, <prefix>_2F (default: elevator)")
    args = ap.parse_args()
    rclpy.init()
    node = ElevatorCheck()
    if not args.no_reset:
        reset_sim(node, args.clear_map)
    if not node.client.wait_for_server(timeout_sec=60.0) \
            or not node.spin_until(lambda: node.elevator is not None and node.gt is not None, 10.0):
        print("navigate_to_pose or sim/elevator not found (simulator started with layouts/elevator.yaml?)")
        return 1
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)
        return ok

    print(f"elevator: {node.elevator}", flush=True)
    check("car on 1F with the door open", node.elevator["floor"] == "1F" and node.elevator["state"] == "open"
          and node.elevator["door"] == "open", f"{node.elevator['floor']}, {node.elevator['state']}")
    check("AMCL at the spawn pose on 1F", node.set_pose((0.0, 0.0, 0.0)))
    if check("to the landing", *node.go(LANDING, 0, accept_within=120.0)) and check("into the car", *node.go(IN_CAR, 0)):
        node.spin_until(lambda: node.elevator["robot_in_car"], 3.0)
        check("robot in the car", node.elevator["robot_in_car"])
        before = node.gt
        map_pose = node.amcl
        node.go_pub.publish(String(data="2F"))
        t0 = time.monotonic()
        arrived = node.spin_until(lambda: node.elevator["floor"] == "2F" and node.elevator["state"] == "open", 40.0)
        e = node.elevator
        check("ride to 2F", arrived and e["robot_floor"] == "2F" and e["robot_in_car"],
              f"{e['floor']}, {e['state']}, robot on {e['robot_floor']}, {time.monotonic() - t0:.0f} s")
        node.spin_until(lambda: False, 1.0)
        dx, dy = node.gt[0] - before[0], node.gt[1] - before[1]
        check("carried 10 m along x", abs(dx - FLOOR_SPACING) < 0.05 and abs(dy) < 0.05, f"({dx:.3f}, {dy:.3f})")
        node.loaded.clear()
        node.map_load_pub.publish(String(data=json.dumps({"id": 1, "map_id": f"{args.maps}_2F"})))
        node.spin_until(lambda: node.loaded, 20.0)
        res = node.loaded[0] if node.loaded else {"success": False, "message": "no answer on sim/map/loaded"}
        if check(f"load {args.maps}_2F", res["success"], res.get("message", "")) \
                and check("AMCL at the same map pose on 2F", node.set_pose(map_pose)):
            node.spin_until(lambda: False, 2.0)
            check("out of the car to the 2F goal", *node.go(GOAL_2F, 1))
            amcl, gt = node.amcl, node.gt
            err = math.hypot(amcl[0] - (gt[0] - FLOOR_SPACING), amcl[1] - gt[1])
            check("AMCL agrees with ground truth on 2F", err < 0.2, f"{err:.2f} m")
    check("no collisions", node.collisions == 0, f"{node.collisions}")
    node.destroy_node()
    rclpy.shutdown()
    failed = [n for n, ok in results if not ok]
    print(f"{len(results) - len(failed)}/{len(results)} passed" + (f"; failed: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
