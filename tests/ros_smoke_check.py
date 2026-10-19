"""Pass/fail checks of the simulator's ROS 2 interface (room world, robot at
the origin). Run by scripts/smoke_test.sh inside the ROS 2 container.

Checks topic presence and rates, LiDAR ranges against the room walls, IMU
gravity, camera images, TF, robot_description, driving with speed limits,
and the front obstacle stop at the east wall (on, muted, off).
"""

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

NS = "/kachaka"
RATES = {  # topic: (type, expected Hz in sim time)
    f"{NS}/lidar/scan": (LaserScan, 10.0),
    f"{NS}/imu/imu": (Imu, 100.0),
    f"{NS}/odometry/odometry": (Odometry, 20.0),
    f"{NS}/wheel_odometry/wheel_odometry": (Odometry, 20.0),
    f"{NS}/front_camera/image_raw": (Image, 10.0),
    f"{NS}/back_camera/image_raw": (Image, 10.0),
    f"{NS}/tof_camera/image_raw": (Image, 10.0),
}

failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)
    if not ok:
        failures.append(name)


EAST_WALL = 2.5  # m, inner face of the room's east wall
FRONT_X = 0.23  # m, config/robot.yaml front_stop.front_x


class Smoke(Node):
    def __init__(self):
        super().__init__("smoke_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.last, self.stamps = {}, {t: [] for t in RATES}
        for topic, (typ, _) in RATES.items():
            self.create_subscription(typ, topic, lambda m, t=topic: self.on_msg(t, m), qos_profile_sensor_data)
        self.gt = None
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", lambda m: setattr(self, "gt", m),
                                 qos_profile_sensor_data)
        self.tf_pairs = set()
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.description = None
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{NS}/robot_description", lambda m: setattr(self, "description", m.data),
                                 latched)
        self.cmd = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10)
        self.front_stop = {}
        self.create_subscription(String, f"{NS}/sim/front_stop/state",
                                 lambda m: setattr(self, "front_stop", json.loads(m.data)), latched)

    def call(self, service, value):
        client = self.create_client(SetBool, service)
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return False
            future = client.call_async(SetBool.Request(data=value))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            return bool(future.result() and future.result().success)
        finally:
            self.destroy_client(client)

    def x(self):
        return self.gt.pose.pose.position.x

    def on_msg(self, topic, msg):
        self.last[topic] = msg
        self.stamps[topic].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_tf(self, msg):
        for t in msg.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, sec, v=0.0, w=0.0):
        end = self.sim_now() + sec
        while self.sim_now() < end:
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.cmd.publish(m)
            self.spin_for(0.05)


def main():
    rclpy.init()
    n = Smoke()

    # Wait for every topic (cameras may need shader warm-up).
    end = time.time() + 180
    while (len(n.last) < len(RATES) or n.gt is None) and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    missing = sorted(set(RATES) - set(n.last))
    check("all topics published", not missing, f"missing {missing}" if missing else "")

    for s in n.stamps.values():
        s.clear()
    n.spin_for(5.0)
    for topic, (_, hz) in RATES.items():
        s = n.stamps[topic]
        rate = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 and s[-1] > s[0] else 0.0
        # Images can drop over DDS; require most of them.
        lo = 0.7 if "camera" in topic else 0.9
        check(f"rate {topic}", lo * hz <= rate <= 1.1 * hz, f"{rate:.1f} Hz (expected {hz:.0f})")

    scan = n.last.get(f"{NS}/lidar/scan")
    if scan:
        def beam(base_deg):  # laser_frame is base_link rotated +90 deg about z
            i = round((math.radians(base_deg - 90.0) - scan.angle_min) / scan.angle_increment)
            return scan.ranges[i]
        for deg, wall in ((0, 2.5 - 0.156), (90, 2.0), (-90, 2.0)):
            r = beam(deg)
            check(f"lidar beam {deg:+d} deg", abs(r - wall) < 0.15, f"{r:.3f} m (wall {wall:.3f} m)")

    imu = n.last.get(f"{NS}/imu/imu")
    if imu:
        a = imu.linear_acceleration
        g = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
        check("imu gravity at rest", abs(g - 9.81) < 0.3, f"|a| = {g:.2f} m/s^2")

    for cam, enc in (("front_camera", "rgb8"), ("back_camera", "rgb8"), ("tof_camera", "16UC1")):
        img = n.last.get(f"{NS}/{cam}/image_raw")
        if img:
            data = bytes(img.data)
            ok = img.encoding == enc and len(data) == img.step * img.height and any(data[::97])
            check(f"{cam} image", ok, f"{img.width}x{img.height} {img.encoding}")

    for pair in (("odom", "base_footprint"),):
        check(f"tf {pair[0]} -> {pair[1]}", pair in n.tf_pairs)
    check("robot_description", bool(n.description) and "<robot" in n.description)

    # Drive: 0.2 m/s for 3 s, then a 1.0 m/s command must be clamped to 0.3 m/s.
    n.drive(0.5)
    p0 = n.gt.pose.pose.position
    n.drive(3.0, 0.2)
    p1 = n.gt.pose.pose.position
    dist = math.hypot(p1.x - p0.x, p1.y - p0.y)
    check("drive 0.2 m/s for 3 s", 0.45 < dist < 0.7, f"moved {dist:.2f} m")
    n.drive(1.0, 1.0)
    v = n.gt.twist.twist.linear.x
    check("speed limit 0.3 m/s", 0.25 < v < 0.32, f"v = {v:.3f} m/s for a 1.0 m/s command")
    n.drive(1.0)
    v = n.gt.twist.twist.linear.x
    check("stops", abs(v) < 0.02, f"v = {v:.3f} m/s")

    # Front obstacle stop: full speed at the east wall, then muted and off.
    fs = f"{NS}/sim/front_stop"
    check("front stop on by default", n.front_stop.get("enabled") is True, json.dumps(n.front_stop))
    n.drive(8.0, 0.3)
    gap = EAST_WALL - n.x() - FRONT_X
    v = n.gt.twist.twist.linear.x
    check("front stop halts at the wall", 0.0 < gap < 0.08 and abs(v) < 0.02, f"gap {gap:.3f} m, v = {v:.3f} m/s")
    check("front stop state blocked", n.front_stop.get("blocked") is True, json.dumps(n.front_stop))
    n.drive(0.5)
    for label, service, on, off in (("muted", "mute", True, False), ("off", "set_enabled", False, True)):
        x0 = n.x()
        ok = n.call(f"{fs}/{service}", on)
        n.drive(0.6, 0.05)
        moved = n.x() - x0
        check(f"front stop {label}: drives on", ok and moved > 0.01, f"moved {moved:.3f} m")
        n.drive(1.5, -0.2)  # back off the wall
        n.call(f"{fs}/{service}", off)
        n.drive(3.0, 0.2)  # and up to it again
    x0 = n.x()
    n.drive(1.0, 0.1)
    moved = n.x() - x0
    check("front stop back on: holds", abs(moved) < 0.01 and n.front_stop.get("blocked") is True,
          f"moved {moved:.3f} m, {json.dumps(n.front_stop)}")
    n.drive(2.0, -0.2)

    print(f"\n{'ALL PASSED' if not failures else f'{len(failures)} FAILED: {failures}'}", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
