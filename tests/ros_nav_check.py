"""Send Nav2 goals in the default room and check the robot reaches them.

Run against the simulator with SLAM + Nav2 up (slam.launch.py nav:=true).
Success is judged on /kachaka/sim/ground_truth. The SLAM map frame starts at
the spawn pose, which is the world origin, so goals and ground truth line up.

    python3 tests/ros_nav_check.py
"""

import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

# The first goal turns the robot around: the area behind it at the start is
# outside the LiDAR field of view, so it is not on the SLAM map yet.
GOALS = [(1.6, -0.3, math.pi), (-1.8, 1.3, math.pi / 2), (0.2, -1.5, 0.0), (0.0, 0.0, 0.0)]
TOLERANCE = 0.35  # m, Nav2 goal checker 0.25 plus SLAM drift


class NavCheck(Node):
    def __init__(self):
        super().__init__("nav_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.gt = None
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.map_info = None
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map_info", m.info), latched)

    def map_bounds(self):
        i = self.map_info
        if i is None:
            return None
        x0, y0 = i.origin.position.x, i.origin.position.y
        return x0, y0, x0 + i.width * i.resolution, y0 + i.height * i.resolution

    def wait_map_covers(self, x, y, timeout=15.0):
        """SLAM grows the map as the robot sees more; Nav2 rejects goals outside it."""
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            b = self.map_bounds()
            if b and b[0] < x < b[2] and b[1] < y < b[3]:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"  map bounds {self.map_bounds()} do not cover ({x}, {y})", flush=True)
        return False

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)

    def go(self, x, y, yaw, timeout=120.0):
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
    rclpy.init()
    node = NavCheck()
    if not node.client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose action server not found")
        return 1
    ok_all = True
    for x, y, yaw in GOALS:
        node.wait_map_covers(x, y)
        start = node.get_clock().now()
        ok, why = node.go(x, y, yaw)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        dt = (node.get_clock().now() - start).nanoseconds * 1e-9
        err = math.hypot(node.gt[0] - x, node.gt[1] - y) if node.gt else float("nan")
        ok = ok and err < TOLERANCE
        ok_all &= ok
        print(f"goal ({x:+.1f}, {y:+.1f}): {'OK' if ok else 'FAIL'} ({why}, error {err:.2f} m, {dt:.0f} s)", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
