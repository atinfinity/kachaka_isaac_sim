"""Check sensor topics of the simulator (robot at the origin of the 'room'
world). Runs in a ROS 2 Jazzy environment, not in the Isaac Sim venv."""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

TOPICS = {
    "scan": ("/kachaka/lidar/scan", LaserScan),
    "imu": ("/kachaka/imu/imu", Imu),
    "odom": ("/kachaka/odometry/odometry", Odometry),
}
rclpy.init()
n = Node("sensor_check")
last, counts, stamps = {}, {k: 0 for k in TOPICS}, {k: [] for k in TOPICS}


def cb(k, m):
    last[k] = m
    counts[k] += 1
    stamps[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)


for k, (topic, typ) in TOPICS.items():
    n.create_subscription(typ, topic, lambda m, k=k: cb(k, m), qos_profile_sensor_data)
end = time.time() + 30
while len(last) < len(TOPICS) and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(last) < len(TOPICS):
    print("missing topics:", set(TOPICS) - set(last))
    sys.exit(1)
for k in TOPICS:
    stamps[k].clear()
end = time.time() + 5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
for k, s in stamps.items():
    if len(s) > 1:
        print(f"{k:5s} rate {(len(s) - 1) / (s[-1] - s[0]):6.1f} Hz (sim time)")

s = last["scan"]
print(f"scan frame={s.header.frame_id} n={len(s.ranges)} angle=[{math.degrees(s.angle_min):.1f}, "
      f"{math.degrees(s.angle_max):.1f}] deg range=[{s.range_min}, {s.range_max}]")


def beam(base_deg):
    # laser_frame is base_link rotated +90 deg about z
    a = math.radians(base_deg - 90.0)
    i = round((a - s.angle_min) / s.angle_increment)
    return s.ranges[i] if 0 <= i < len(s.ranges) else float("nan")


for deg, expect in ((0, 2.5 - 0.156), (90, 2.0), (-90, 2.0)):
    print(f"beam {deg:+4d} deg: {beam(deg):.3f} m (room wall {expect:.3f} m)")
finite = [r for r in s.ranges if math.isfinite(r)]
print(f"finite {len(finite)}/{len(s.ranges)}, min {min(finite):.3f} m")
i = last["imu"]
print(f"imu frame={i.header.frame_id} acc=({i.linear_acceleration.x:+.2f}, {i.linear_acceleration.y:+.2f}, "
      f"{i.linear_acceleration.z:+.2f}) gyro_z={i.angular_velocity.z:+.4f}")
