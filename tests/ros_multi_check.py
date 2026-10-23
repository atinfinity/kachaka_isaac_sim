"""Two robots in one simulator (#17): each has its own topics, TF frames and
odometry, drives on its own commands, and sees the other.

Run against two robots facing each other in the default room:

    scripts/run_sim.sh --no-cameras --no-shelves --robot -1.0 0.0 0.0 --robot 1.0 0.0 3.1416
    python3 tests/ros_multi_check.py

1. One /clock publisher (robot 0's node).
2. Each robot publishes ground truth, scan and odometry in its namespace, and
   TF {prefix}odom -> {prefix}base_footprint; its odometry starts at zero.
3. Each robot's LiDAR sees the other: the rays towards it end on its LiDAR
   housing (fleet.LIDAR_HEIGHT_BODY), which faces the viewer 0.12 to 0.21 m in
   front of its centre.
4. Turning robot 1 moves robot 1 only.
5. No collisions (sim/collision of either robot).
"""

import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

ROBOTS = (("/kachaka", ""), ("/kachaka_1", "kachaka_1/"))  # namespace, TF frame prefix
RADIUS = (0.10, 0.25)  # m, how far short of the other's centre a ray may end (its housing, robots face each other)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of(m):
    p = m.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


class Check(Node):
    def __init__(self):
        super().__init__("multi_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.st = [{} for _ in ROBOTS]
        self.collisions = []
        for i, (ns, _) in enumerate(ROBOTS):
            for key, typ, topic in (("gt", Odometry, "sim/ground_truth"), ("odom", Odometry, "odometry/odometry"),
                                    ("scan", LaserScan, "lidar/scan")):
                self.create_subscription(typ, f"{ns}/{topic}", lambda m, i=i, k=key: self.st[i].__setitem__(k, m),
                                         qos_profile_sensor_data)
            self.create_subscription(String, f"{ns}/sim/collision",
                                     lambda m, i=i: self.collisions.append((i, json.loads(m.data))), 10)
        self.cmd = self.create_publisher(Twist, f"{ROBOTS[1][0]}/manual_control/cmd_vel", qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def lookup(self, parent, child):
        try:
            t = self.tf.lookup_transform(parent, child, Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def gt(self, i):
        return pose_of(self.st[i]["gt"])

    def toward(self, i, j):
        """Robot i's shortest range within 3 degrees of robot j's centre, and
        the distance from robot i's laser to that centre."""
        s = self.st[i]["scan"]
        (xi, yi, ai), (xj, yj, _) = self.gt(i), self.gt(j)
        lx, ly, la = self.lookup(f"{ROBOTS[i][1]}base_footprint", s.header.frame_id)
        xi, yi = xi + math.cos(ai) * lx - math.sin(ai) * ly, yi + math.sin(ai) * lx + math.cos(ai) * ly
        r = np.asarray(s.ranges, float)
        angles = s.angle_min + np.arange(len(r)) * s.angle_increment
        bearing = math.atan2(yj - yi, xj - xi) - ai - la
        near = np.abs(np.angle(np.exp(1j * (angles - bearing)))) < math.radians(3)
        hits = r[near & np.isfinite(r)]
        return (float(hits.min()) if len(hits) else math.inf), math.hypot(xj - xi, yj - yi)

    def turn(self, seconds, w):
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            m = Twist()
            m.angular.z = w
            self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())


def main():
    rclpy.init()
    n = Check()
    failed = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'}{f' ({detail})' if detail else ''}", flush=True)
        if not ok:
            failed.append(name)

    ready = n.wait(lambda: all(len(s) == 3 for s in n.st), 60.0)
    if not ready:
        missing = [f"{ns}/{k}" for (ns, _), s in zip(ROBOTS, n.st) for k in ("gt", "odom", "scan") if k not in s]
        check("ready", False, f"missing {', '.join(missing)}")
        return 1
    n.wait(lambda: False, 1.0)
    clocks = n.count_publishers("/clock")
    check("one /clock publisher", clocks == 1, f"{clocks}")
    for i, (ns, prefix) in enumerate(ROBOTS):
        ok = n.wait(lambda p=prefix: n.lookup(f"{p}odom", f"{p}base_footprint") is not None, 10.0)
        check(f"{ns}: TF {prefix}odom -> {prefix}base_footprint", ok)
        x, y, a = pose_of(n.st[i]["odom"])
        check(f"{ns}: odometry starts at zero", math.hypot(x, y) < 0.02 and abs(a) < 0.02,
              f"({x:.3f}, {y:.3f}, {a:.3f})")
    for i, j in ((0, 1), (1, 0)):
        if n.lookup(f"{ROBOTS[i][1]}base_footprint", n.st[i]["scan"].header.frame_id) is None:
            check(f"{ROBOTS[i][0]} sees {ROBOTS[j][0]}", False, "no laser TF")
            continue
        r, d = n.toward(i, j)
        check(f"{ROBOTS[i][0]} sees {ROBOTS[j][0]}", d - RADIUS[1] <= r <= d - RADIUS[0],
              f"ray {r:.2f} m, {d:.2f} m from the laser to its centre")
    before = [n.gt(0), n.gt(1)]
    n.turn(2.0, 0.5)
    n.wait(lambda: False, 1.0)
    after = [n.gt(0), n.gt(1)]
    turned = math.remainder(after[1][2] - before[1][2], math.tau)
    still = math.hypot(after[0][0] - before[0][0], after[0][1] - before[0][1]) + abs(after[0][2] - before[0][2])
    check("robot 1 turns on its own commands", turned > 0.5, f"{turned:+.2f} rad")
    check("robot 0 stays put", still < 0.01, f"moved {still:.3f}")
    hit = sorted({f"{ROBOTS[i][0]} with {c.get('with', '?')}" for i, c in n.collisions})
    check("collisions", not hit, ", ".join(hit))
    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
