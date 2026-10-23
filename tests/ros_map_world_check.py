"""A world built from an occupancy map (#24): the walls stand where the map
says, and Nav2 on that map drives in it.

Run against the simulator started on a map world at the map origin, with Nav2
on the same map (scripts/smoke_test.sh --nav does this with the elevator
layout's first-floor map):

    scripts/run_sim.sh --world map:maps/elevator_1F.yaml
    ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/elevator_1F.yaml
    python3 tests/ros_map_world_check.py --map /maps/elevator_1F.yaml

1. The world frame is the map frame: AMCL's pose (TF map -> base_footprint)
   matches ground truth at the start.
2. The LiDAR sees the map's walls: most scan points, placed by ground truth,
   fall on an occupied cell of /map (or next to one).
3. The charger stands at the map's `charger_pose` (sim/charger_pose).
4. Nav2 drives to --goal and touches nothing (sim/collision).
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TOLERANCE = 0.35  # m, goal
LOCALIZED = 0.10  # m, AMCL vs ground truth at the start
ON_WALL = 0.90  # share of scan points on (or next to) an occupied cell


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__("map_world_check")
        self.gt = self.scan = self.map = self.charger = None
        self.collisions = []
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/kachaka/lidar/scan", lambda m: setattr(self, "scan", m),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), LATCHED)
        self.create_subscription(PoseStamped, "/kachaka/sim/charger_pose", lambda m: setattr(self, "charger", m),
                                 LATCHED)
        self.create_subscription(String, "/kachaka/sim/collision",
                                 lambda m: self.collisions.append(json.loads(m.data)), 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def lookup(self, parent, child):
        try:
            t = self.tf.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def on_wall_share(self):
        """Share of finite scan points, placed by ground truth, within one
        cell of an occupied /map cell."""
        s, m = self.scan, self.map
        laser = self.lookup("base_footprint", s.header.frame_id)
        if laser is None:
            return None
        gx, gy, gyaw = self.gt
        lx = gx + math.cos(gyaw) * laser[0] - math.sin(gyaw) * laser[1]
        ly = gy + math.sin(gyaw) * laser[0] + math.cos(gyaw) * laser[1]
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + gyaw + laser[2]
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        px, py = lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])
        i = m.info
        grid = np.asarray(m.data, np.int16).reshape(i.height, i.width) >= 65
        # Grow the walls by one cell: a hit lands on the wall's face, a cell edge.
        grown = grid.copy()
        grown[1:] |= grid[:-1]
        grown[:-1] |= grid[1:]
        grown[:, 1:] |= grown[:, :-1].copy()
        grown[:, :-1] |= grown[:, 1:].copy()
        col = np.floor((px - i.origin.position.x) / i.resolution).astype(int)
        row = np.floor((py - i.origin.position.y) / i.resolution).astype(int)
        inside = (col >= 0) & (col < i.width) & (row >= 0) & (row < i.height)
        hits = np.zeros(len(px), bool)
        hits[inside] = grown[row[inside], col[inside]]
        return float(hits.mean()) if len(px) else None, len(px)

    def go(self, x, y, yaw, timeout=180.0):
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return False, "no navigate_to_pose server"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.nav.send_goal_async(goal)
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
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", required=True, help="the map yaml the world was built from (for its charger_pose)")
    ap.add_argument("--goal", type=float, nargs=3, default=(-1.5, 1.0, math.pi / 2), metavar=("X", "Y", "YAW"),
                    help="Nav2 goal in the map frame (default: the elevator room's north-west corner)")
    args = ap.parse_args()
    with open(args.map) as f:
        meta = yaml.safe_load(f) or {}
    rclpy.init()
    node = Check()
    failed = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'}{f' ({detail})' if detail else ''}", flush=True)
        if not ok:
            failed.append(name)

    ready = node.wait(lambda: node.gt and node.scan and node.map
                      and node.lookup("map", "base_footprint") is not None, 60.0)
    if not ready:
        check("ready", False, "no ground truth, scan, /map or TF map -> base_footprint within 60 s")
        return 1
    est = node.lookup("map", "base_footprint")
    err = math.hypot(est[0] - node.gt[0], est[1] - node.gt[1])
    check("localized", err < LOCALIZED, f"AMCL ({est[0]:.2f}, {est[1]:.2f}), ground truth "
          f"({node.gt[0]:.2f}, {node.gt[1]:.2f}), {err:.2f} m apart")
    share = node.on_wall_share()
    if share is None or share[0] is None:
        check("scan on the map's walls", False, "no scan points or no laser TF")
    else:
        check("scan on the map's walls", share[0] >= ON_WALL, f"{share[0]:.0%} of {share[1]} points")
    want = meta.get("charger_pose")
    if want:
        node.wait(lambda: node.charger is not None, 10.0)
        c = node.charger
        if c is None:
            check("charger at charger_pose", False, "no sim/charger_pose")
        else:
            p = c.pose.position
            d = math.hypot(p.x - want[0], p.y - want[1])
            check("charger at charger_pose", d < 0.02, f"({p.x:.2f}, {p.y:.2f}) for {want[:2]}")
    node.collisions.clear()
    t0 = time.monotonic()
    ok, why = node.go(*args.goal)
    node.wait(lambda: False, 1.0)
    gx, gy = args.goal[:2]
    d = math.hypot(node.gt[0] - gx, node.gt[1] - gy)
    check(f"goal ({gx:+.1f}, {gy:+.1f})", ok and d < TOLERANCE,
          f"{why}, error {d:.2f} m, {time.monotonic() - t0:.0f} s")
    hit = sorted({c.get("with", "?") for c in node.collisions})
    check("collisions", not hit, f"{len(node.collisions)}{': ' + ', '.join(hit) if hit else ''}")
    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
