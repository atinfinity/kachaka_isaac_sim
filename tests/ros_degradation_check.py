"""Sensor degradation (#21): the sensors of a simulator started with
config/degraded.yaml show what it asks for.

    scripts/run_sim.sh --config config/degraded.yaml --person 0.0 1.0 0.0
    python3 tests/ros_degradation_check.py

The robot stands at the origin of the default room, a person 1 m to its left.

1. LiDAR dropout: about `dropout` of the rays return nothing in the closed room.
2. LiDAR sees through the person (`invisible: ["/World/People/*"]`): the rays
   to the left reach past them.
3. Camera noise: two front frames of a still scene differ.
4. IMU: the gyro reads a turn while the robot stands still, and the IMU
   orientation turns away from ground truth at that rate.
5. Odometry scale errors: driving forward and turning, the odometry reads
   (1 + distance_scale_error) times the distance and (1 + turn_scale_error)
   times the turn of ground truth.

Runs in a ROS 2 Jazzy environment (the docker container).
"""

import argparse
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
from sensor_msgs.msg import Image, Imu, LaserScan

NS = "/kachaka"


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__("degradation_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.last = {}
        self.scans, self.imus, self.images = [], [], []
        sub = self.create_subscription
        sub(LaserScan, f"{NS}/lidar/scan", self.on_scan, qos_profile_sensor_data)
        sub(Imu, f"{NS}/imu/imu", self.on_imu, qos_profile_sensor_data)
        sub(Image, f"{NS}/front_camera/image_raw", self.on_image, qos_profile_sensor_data)
        for k, t in (("odom", f"{NS}/odometry/odometry"), ("gt", f"{NS}/sim/ground_truth")):
            sub(Odometry, t, lambda m, k=k: self.last.__setitem__(k, m), qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", qos_profile_sensor_data)

    def on_scan(self, m):
        self.last["scan"] = m
        self.scans.append(m)

    def on_imu(self, m):
        self.last["imu"] = m
        self.imus.append(m)

    def on_image(self, m):
        self.last["image"] = m
        self.images.append(m)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, sec, v=0.0, w=0.0):
        end = self.now() + sec
        while self.now() < end:
            m = Twist()
            m.linear.x, m.angular.z = v, w
            self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, k):
        p = self.last[k].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)


def unwrap_turn(yaws) -> float:
    return float(np.sum(np.angle(np.exp(1j * np.diff(yaws)))))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--dropout", type=float, default=0.2)
    ap.add_argument("--distance-scale-error", type=float, default=0.1)
    ap.add_argument("--turn-scale-error", type=float, default=-0.05)
    ap.add_argument("--person", type=float, nargs=2, default=(0.0, 1.0), metavar=("X", "Y"))
    args = ap.parse_args()
    rclpy.init()
    n = Check()
    failed = []

    def check(name, ok, detail):
        print(f"{name}: {'OK' if ok else 'FAIL'} ({detail})", flush=True)
        if not ok:
            failed.append(name)

    end = time.monotonic() + 60
    while len(n.last) < 5 and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.last) < 5:
        print(f"missing: {sorted({'scan', 'imu', 'image', 'odom', 'gt'} - set(n.last))}")
        return 1

    # Standing still.
    n.drive(1.0)
    n.scans.clear(), n.imus.clear(), n.images.clear()
    g0 = n.pose("gt")
    n.drive(4.0)
    g1 = n.pose("gt")

    s = n.scans[-1]
    r = np.array([m.ranges for m in n.scans], float)
    lost = float(np.mean(~np.isfinite(r)))
    check("LiDAR dropout", abs(lost - args.dropout) < 0.08, f"{lost:.0%} of {r.size} rays lost, asked "
          f"{args.dropout:.0%}")

    # Rays towards the person. The laser frame is turned on the base (URDF),
    # and the scan is centred on the robot's forward direction.
    px, py = args.person[0] - g1[0], args.person[1] - g1[1]
    angles = s.angle_min + np.arange(r.shape[1]) * s.angle_increment
    forward = (angles[0] + angles[-1]) / 2
    bearing = math.atan2(py, px) - g1[2] + forward
    near = np.abs(np.angle(np.exp(1j * (angles - bearing)))) < math.radians(4)
    to_person = r[:, near]
    reach = float(np.nanmedian(np.where(np.isfinite(to_person), to_person, np.nan)))
    dist = math.hypot(px, py)
    check("LiDAR sees through the person", reach > dist + 0.3,
          f"rays towards them reach {reach:.2f} m, they stand {dist:.2f} m away")

    if len(n.images) >= 2:
        a, b = (np.frombuffer(m.data, np.uint8).astype(float) for m in n.images[-2:])
        diff = float(np.std(a - b))
        check("camera noise", diff > 4.0, f"two still frames differ by {diff:.1f} levels (stddev)")
    else:
        check("camera noise", False, f"{len(n.images)} frames")

    gz = float(np.mean([m.angular_velocity.z for m in n.imus]))
    iy = [yaw_of(m.orientation) for m in n.imus]
    t = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in n.imus]
    rate = (unwrap_turn(iy) - (g1[2] - g0[2])) / max(t[-1] - t[0], 1e-6)
    check("IMU gyro bias", abs(gz) > 1e-3 and abs(rate - gz) < 0.2 * abs(gz) + 5e-4,
          f"still robot: gyro z {gz:+.4f} rad/s, orientation drifts {rate:+.4f} rad/s from ground truth")

    # Driving.
    o0, g0 = n.pose("odom"), n.pose("gt")
    n.drive(3.0, 0.2, 0.0)
    n.drive(1.0)
    o1, g1 = n.pose("odom"), n.pose("gt")
    ratio = math.hypot(o1[0] - o0[0], o1[1] - o0[1]) / max(math.hypot(g1[0] - g0[0], g1[1] - g0[1]), 1e-6)
    want = 1 + args.distance_scale_error
    check("odometry distance", abs(ratio - want) < 0.03, f"reads {ratio:.3f} of the distance, asked {want:.3f}")

    yo, yg = [o1[2]], [g1[2]]
    end = n.now() + 3.0
    while n.now() < end:
        n.drive(0.1, 0.0, 0.8)
        yo.append(n.pose("odom")[2])
        yg.append(n.pose("gt")[2])
    n.drive(1.0)
    yo.append(n.pose("odom")[2])
    yg.append(n.pose("gt")[2])
    ratio = unwrap_turn(yo) / unwrap_turn(yg)
    want = 1 + args.turn_scale_error
    check("odometry turn", abs(ratio - want) < 0.03, f"reads {ratio:.3f} of the turn, asked {want:.3f}")

    print("ALL PASSED" if not failed else f"FAILED: {', '.join(failed)}", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
