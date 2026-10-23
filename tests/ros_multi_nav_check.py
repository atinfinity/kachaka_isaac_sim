"""Two robots navigate at once on one saved map (#17): their paths cross at
right angles, and they pass each other without touching.

    python3 scripts/make_maps.py --spawn -1.7 0.2 0                # maps/room.yaml
    scripts/run_sim.sh --no-cameras --no-shelves --robot -1.7 0.2 0 --robot 0.3 -1.8 1.5708
    ros2 launch kachaka_sim_bringup fleet.launch.py map:=/maps/room.yaml robots:=2 rviz:=false
    python3 tests/ros_multi_nav_check.py

1. Each robot's AMCL gets its start (from ground truth, in the shared map
   frame: robot 0's spawn) until TF map -> {prefix}base_footprint exists.
2. Each robot's {namespace}/other_robots (fleet_outlines) has the other
   robot's footprint around where the other robot is.
3. Both get a goal at the same moment; the straight lines between start and
   goal cross at right angles, 2 m from both starts, so they reach the
   crossing together. Both must arrive within the timeout.
4. The robots' centres stay more than MIN_GAP apart, and neither reports a
   collision on sim/collision.

Nose to nose, the two Nav2 stacks can block each other (they don't
negotiate), which is a fleet manager's job, so the paths don't meet head on.
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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_list
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# (Kachaka namespace, Nav2 namespace, TF frame prefix)
ROBOTS = (("/kachaka", "", ""), ("/kachaka_1", "/kachaka_1", "kachaka_1/"))
GOALS = ((2.0, 0.2, 0.0), (0.3, 1.6, math.pi / 2))  # world frame, x y yaw, per robot
OUTLINE_TOLERANCE = 0.15  # m between the outline's centre and the robot's (its footprint is off-centre by 0.045)
MIN_GAP = 0.45  # m between the centres; the body is 0.387 x 0.24 m (official URDF)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relative(a, b):
    """Pose b in the frame of pose a (x, y, yaw tuples)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return c * dx + s * dy, -s * dx + c * dy, math.remainder(b[2] - a[2], math.tau)


class Check(Node):
    def __init__(self):
        super().__init__("multi_nav_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gt = [None for _ in ROBOTS]
        self.collisions = []
        self.min_gap = math.inf
        self.closest = ()
        self.initial = []
        self.nav = []
        self.others = [None for _ in ROBOTS]
        for i, (ns, nav_ns, _) in enumerate(ROBOTS):
            self.create_subscription(Odometry, f"{ns}/sim/ground_truth", lambda m, i=i: self.on_gt(i, m),
                                     qos_profile_sensor_data)
            self.create_subscription(String, f"{ns}/sim/collision",
                                     lambda m, i=i: self.on_collision(i, json.loads(m.data)), 10)
            self.initial.append(self.create_publisher(PoseWithCovarianceStamped, f"{nav_ns}/initialpose", 10))
            self.nav.append(ActionClient(self, NavigateToPose, f"{nav_ns}/navigate_to_pose"))
            self.create_subscription(PointCloud2, f"{nav_ns}/other_robots",
                                     lambda m, i=i: self.others.__setitem__(i, m), 1)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def on_collision(self, i, c):
        if not self.collisions:
            poses = "; ".join(f"{ns} ({g[0]:+.2f}, {g[1]:+.2f}, {math.degrees(g[2]):+.0f} deg)"
                              for (ns, _, _), g in zip(ROBOTS, self.gt) if g)
            print(f"first contact: {ROBOTS[i][0]} with {c.get('with', '?')} at {poses}", flush=True)
        self.collisions.append((i, c))

    def on_gt(self, i, m):
        p = m.pose.pose
        self.gt[i] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if all(self.gt):
            (x0, y0, _), (x1, y1, _) = self.gt
            gap = math.hypot(x1 - x0, y1 - y0)
            if gap < self.min_gap:
                self.min_gap, self.closest = gap, tuple(self.gt)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def localized(self, i) -> bool:
        try:
            self.tf.lookup_transform("map", f"{ROBOTS[i][2]}base_footprint", Time())
        except TransformException:
            return False
        return True

    def set_poses(self, origin, timeout=90.0) -> list[bool]:
        """Each robot's ground truth in the map frame (robot 0's spawn
        `origin`) as its initial pose, until AMCL publishes its TF."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and not all(self.localized(i) for i in range(len(ROBOTS))):
            for i, pub in enumerate(self.initial):
                if self.localized(i):
                    continue
                x, y, yaw = relative(origin, self.gt[i])
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = 0.01
                pub.publish(msg)
            self.wait(lambda: False, 2.0)
        return [self.localized(i) for i in range(len(ROBOTS))]

    def send(self, i, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        x, y, yaw = pose
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.nav[i].send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        return handle if handle is not None and handle.accepted else None


def outline_near(msg, pose) -> tuple[bool, str]:
    """Whether the cloud's points (map frame) centre on `pose` (x, y, yaw in
    the map frame)."""
    if msg is None:
        return False, "nothing received"
    points = read_points_list(msg, field_names=("x", "y"))
    if not points:
        return False, "empty"
    cx = sum(p.x for p in points) / len(points)
    cy = sum(p.y for p in points) / len(points)
    d = math.hypot(cx - pose[0], cy - pose[1])
    return d < OUTLINE_TOLERANCE, f"{len(points)} points, centre {d:.2f} m from the robot"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--timeout", type=float, default=300.0, help="s (wall clock) for both robots to arrive")
    ap.add_argument("--stagger", type=float, default=0.0, help="s (wall clock) before robot 1 starts")
    args = ap.parse_args()
    rclpy.init()
    n = Check()
    failed = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'}{f' ({detail})' if detail else ''}", flush=True)
        if not ok:
            failed.append(name)

    if not n.wait(lambda: all(n.gt), 60.0):
        check("ready", False, "no ground truth from " + ", ".join(ns for (ns, _, _), g in zip(ROBOTS, n.gt) if not g))
        return 1
    origin = n.gt[0]
    localized = n.set_poses(origin)
    for (ns, _, prefix), ok in zip(ROBOTS, localized):
        check(f"{ns}: AMCL publishes map -> {prefix}base_footprint", ok)
    if not all(localized):
        return 1
    n.wait(lambda: False, 3.0)  # fresh outlines after the initial poses
    for i, (_, nav_ns, _) in enumerate(ROBOTS):
        other = 1 - i
        ok, detail = outline_near(n.others[i], relative(origin, n.gt[other]))
        check(f"{nav_ns}/other_robots: {ROBOTS[other][0]}'s footprint", ok, detail)
    for i, (ns, nav_ns, _) in enumerate(ROBOTS):
        if not n.nav[i].wait_for_server(timeout_sec=60.0):
            check(f"{ns}: {nav_ns}/navigate_to_pose", False, "no server")
            return 1
    n.wait(lambda: False, 2.0)
    n.min_gap = math.inf
    start = time.monotonic()
    handles, results = [], []
    for i, goal in enumerate(GOALS):
        if i and args.stagger:
            n.wait(lambda: False, args.stagger)
        h = n.send(i, relative(origin, goal))
        handles.append(h)
        results.append(h.get_result_async() if h else None)
        if h is None:
            check(f"{ROBOTS[i][0]}: goal accepted", False)
    n.wait(lambda: all(r is None or r.done() for r in results), args.timeout)
    took = time.monotonic() - start
    for i, (ns, _, _) in enumerate(ROBOTS):
        r = results[i]
        if r is None:
            continue
        if not r.done():
            handles[i].cancel_goal_async()
            check(f"{ns}: reaches its goal", False, f"not within {args.timeout:.0f} s")
            continue
        ok = r.result().status == GoalStatus.STATUS_SUCCEEDED
        gx, gy, _ = GOALS[i]
        x, y, _ = n.gt[i]
        check(f"{ns}: reaches its goal", ok,
              f"status {r.result().status}, {math.hypot(x - gx, y - gy):.2f} m from it, both done in {took:.0f} s")
    at = "; ".join(f"({x:+.2f}, {y:+.2f}, {math.degrees(t):+.0f} deg)" for x, y, t in n.closest)
    check("the robots keep apart", n.min_gap > MIN_GAP, f"closest {n.min_gap:.2f} m between centres, at {at}")
    hit = sorted({f"{ROBOTS[i][0]} with {c.get('with', '?')}" for i, c in n.collisions})
    check("collisions", not hit, ", ".join(hit))
    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
