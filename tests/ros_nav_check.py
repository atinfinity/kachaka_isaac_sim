"""Send Nav2 goals and check the robot reaches them.

Run against the simulator with SLAM + Nav2 up (slam.launch.py nav:=true),
started with the same `--world` and its default spawn pose. Goals are in the
world frame; the SLAM map frame starts at the spawn pose, so they are moved
into it. Success is judged on /kachaka/sim/ground_truth (world frame). The
check also fails if Nav2 ever commands a backward speed: Kachaka has no rear
range sensor.

    python3 tests/ros_nav_check.py                  # default room
    python3 tests/ros_nav_check.py --world office   # simulator started with --world office
"""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

# Per world: spawn pose (as in kachaka_isaac_sim/scenes.py) and goals
# (x, y, yaw), world frame. In the room the first goal turns the robot around:
# the area behind it at the start is outside the LiDAR field of view, so it is
# not on the SLAM map yet.
WORLDS = {
    "room": ((0.0, 0.0, 0.0), [(1.6, -0.3, math.pi), (-1.8, 1.3, math.pi / 2), (0.2, -1.5, 0.0), (0.0, 0.0, 0.0)]),
    "office": ((0.0, 0.0, 1.5708), [(0.0, 6.0, math.pi / 2), (-3.0, 10.0, math.pi / 2), (-3.0, 17.0, math.pi / 2),
                                    (-3.0, 22.0, math.pi / 2), (-3.0, 30.0, math.pi / 2), (0.0, 0.0, -math.pi / 2)]),
    "hospital": ((0.0, 0.0, 0.0), [(8.0, 3.0, math.pi / 2), (6.0, 10.5, math.pi), (-2.0, 10.5, math.pi),
                                   (-8.0, 10.5, math.pi), (0.0, 0.0, 0.0)]),
    # Warehouse: stay in one aisle. Racks show only their legs at the LiDAR
    # height, so a route across a rack row tries to squeeze between legs.
    "warehouse": ((-13.0, 11.0, 1.5708), [(-13.0, 18.0, math.pi / 2), (-13.0, 24.0, -math.pi / 2),
                                          (-13.0, 11.0, math.pi / 2)]),
    "simple_room": ((-2.5, 0.0, 0.0), [(-2.5, 2.5, 0.0), (2.5, 2.5, -math.pi / 2), (2.5, -2.5, math.pi),
                                       (-2.5, -2.5, math.pi / 2), (-2.5, 0.0, 0.0)]),
}
TOLERANCE = 0.35  # m, Nav2 goal checker 0.25 plus SLAM drift


def to_map(spawn, x, y, yaw):
    """World pose -> SLAM map frame (origin at the spawn pose)."""
    sx, sy, syaw = spawn
    dx, dy = x - sx, y - sy
    c, s = math.cos(syaw), math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy, yaw - syaw


class NavCheck(Node):
    def __init__(self):
        super().__init__("nav_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.gt = None
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.min_vx = 0.0
        self.create_subscription(Twist, "/kachaka/manual_control/cmd_vel", self.on_cmd, 10)
        self.map_info = None
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map_info", m.info), latched)

    def map_bounds(self):
        i = self.map_info
        if i is None:
            return None
        x0, y0 = i.origin.position.x, i.origin.position.y
        return x0, y0, x0 + i.width * i.resolution, y0 + i.height * i.resolution

    def wait_map_covers(self, x, y, timeout=30.0):
        """SLAM grows the map as the robot sees more; Nav2 rejects goals outside it."""
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            b = self.map_bounds()
            if b and b[0] < x < b[2] and b[1] < y < b[3]:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"  map bounds {self.map_bounds()} do not cover ({x}, {y})", flush=True)
        return False

    def on_cmd(self, msg):
        self.min_vx = min(self.min_vx, msg.linear.x)

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)

    def go(self, x, y, yaw, timeout=300.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if not result.done():
            handle.cancel_goal_async()
            return False, "timeout"
        return result.result().status == GoalStatus.STATUS_SUCCEEDED, f"status {result.result().status}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="room", choices=sorted(WORLDS))
    args = ap.parse_args()
    spawn, goals = WORLDS[args.world]
    rclpy.init()
    node = NavCheck()
    if not node.client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose action server not found")
        return 1
    ok_all = True
    for x, y, yaw in goals:
        mx, my, myaw = to_map(spawn, x, y, yaw)
        node.wait_map_covers(mx, my)
        start = node.get_clock().now()
        ok, why = node.go(mx, my, myaw)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        dt = (node.get_clock().now() - start).nanoseconds * 1e-9
        err = math.hypot(node.gt[0] - x, node.gt[1] - y) if node.gt else float("nan")
        ok = ok and err < TOLERANCE
        ok_all &= ok
        print(f"goal ({x:+.1f}, {y:+.1f}): {'OK' if ok else 'FAIL'} ({why}, error {err:.2f} m, {dt:.0f} s)", flush=True)
    backward = node.min_vx < -0.01
    ok_all &= not backward
    print(f"backward motion: {'FAIL' if backward else 'OK'} (min cmd_vel x {node.min_vx:+.2f} m/s)", flush=True)
    print("ALL PASSED" if ok_all else "FAILED", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
