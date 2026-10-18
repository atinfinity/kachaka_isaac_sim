"""Drive the simulated robot over ROS 2 and print odometry. Runs in a ROS 2
Jazzy environment (e.g. the docker container), not in the Isaac Sim venv."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node("teleop_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
st = {}
for k, t in [("odom", "/kachaka/odometry/odometry"), ("wheel", "/kachaka/wheel_odometry/wheel_odometry"),
             ("gt", "/kachaka/sim/ground_truth")]:
    n.create_subscription(Odometry, t, lambda m, k=k: st.__setitem__(k, m), 10)
pub = n.create_publisher(Twist, "/kachaka/manual_control/cmd_vel", qos_profile_sensor_data)
while len(st) < 3:
    rclpy.spin_once(n, timeout_sec=0.1)


def now():
    return n.get_clock().now().nanoseconds * 1e-9


def drive(sec, v=0.0, w=0.0):
    end = now() + sec
    while now() < end:
        m = Twist()
        m.linear.x, m.angular.z = v, w
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)


def show(tag):
    for k in ("gt", "odom", "wheel"):
        m = st[k]
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        print(f"{tag:12s} {k:5s} x={p.x:+.3f} y={p.y:+.3f} yaw={yaw:+.3f} "
              f"v={m.twist.twist.linear.x:+.3f} w={m.twist.twist.angular.z:+.3f}")


drive(0.5)
show("start")
t0, s0 = time.time(), now()
drive(3.0, 0.2, 0.0)
show("fwd 0.2 3s")
drive(3.0, 0.0, 1.0)
show("turn 1.0 3s")
drive(1.0, 1.0, 0.0)
show("cmd v=1.0")
drive(1.5)
show("stopped")
print(f"RTF {(now() - s0) / (time.time() - t0):.2f}")
