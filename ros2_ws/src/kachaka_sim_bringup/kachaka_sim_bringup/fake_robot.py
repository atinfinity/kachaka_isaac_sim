"""A stand-in for the simulator: a point robot in an empty square room.

Publishes what slam.launch.py and navigation.launch.py need from the robot, so
Nav2 and slam_toolbox can be started and driven without Isaac Sim (CI):
/clock, TF odom -> base_footprint -> base_link -> laser_frame,
/kachaka/odometry/odometry and /kachaka/lidar/scan. It integrates
/kachaka/manual_control/cmd_vel and /kachaka/sim/autonomous/cmd_vel (nav_bridge
with use_sim_time) perfectly, so the odometry is exact.

    ros2 run kachaka_sim_bringup fake_robot
"""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

HALF_ROOM = 2.0  # m, walls at x, y = +-HALF_ROOM around the start pose
PILLAR = (1.0, 1.0, 0.15)  # x, y, half size: breaks the room's symmetry for SLAM
LASER_XYZ = (0.1, 0.0, 0.12)
SAMPLES, FOV, MAX_RANGE = 400, math.radians(240.0), 10.0
RATE = 20.0  # Hz
CMD_TIMEOUT = 0.5  # s


def _ray_box(px: float, py: float, dx: float, dy: float, lo: tuple, hi: tuple, inside: bool) -> float:
    """Distance along (dx, dy) to an axis-aligned box: its walls from inside, its faces from outside."""
    t_near, t_far = -math.inf, math.inf
    for p, d, a, b in ((px, dx, lo[0], hi[0]), (py, dy, lo[1], hi[1])):
        if abs(d) < 1e-12:
            if not a <= p <= b:
                return math.inf
            continue
        t1, t2 = (a - p) / d, (b - p) / d
        t_near, t_far = max(t_near, min(t1, t2)), min(t_far, max(t1, t2))
    if inside:
        return t_far
    return t_near if t_near <= t_far and t_near > 0 else math.inf


def scan_ranges(x: float, y: float, yaw: float) -> list[float]:
    """Ranges from the laser at robot pose (x, y, yaw), angles from -FOV/2 to +FOV/2."""
    lx = x + math.cos(yaw) * LASER_XYZ[0]
    ly = y + math.sin(yaw) * LASER_XYZ[0]
    room = ((-HALF_ROOM, -HALF_ROOM), (HALF_ROOM, HALF_ROOM))
    px, py, h = PILLAR
    pillar = ((px - h, py - h), (px + h, py + h))
    out = []
    for i in range(SAMPLES):
        a = yaw - FOV / 2 + FOV * i / (SAMPLES - 1)
        dx, dy = math.cos(a), math.sin(a)
        r = min(_ray_box(lx, ly, dx, dy, *room, inside=True), _ray_box(lx, ly, dx, dy, *pillar, inside=False))
        out.append(float(r) if r < MAX_RANGE else math.inf)
    return out


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.x = self.y = self.yaw = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = -math.inf
        self.start = time.monotonic()
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/kachaka/odometry/odometry", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/kachaka/lidar/scan", qos_profile_sensor_data)
        for topic in ("manual_control/cmd_vel", "sim/autonomous/cmd_vel"):
            self.create_subscription(Twist, f"/kachaka/{topic}", self._on_cmd, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            self._tf(0.0, "base_footprint", "base_link", 0.0, 0.0, 0.0),
            self._tf(0.0, "base_link", "laser_frame", *LASER_XYZ[:2], 0.0, z=LASER_XYZ[2]),
        ])
        self.last = 0.0
        self.tick = 0
        self.create_timer(1.0 / RATE, self._step)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = time.monotonic() - self.start

    @staticmethod
    def _stamp(t: float) -> Time:
        sec = math.floor(t)
        return Time(sec=sec, nanosec=int((t - sec) * 1e9))

    def _tf(self, t: float, parent: str, child: str, x: float, y: float, yaw: float, z: float = 0.0):
        msg = TransformStamped()
        msg.header.stamp = self._stamp(t)
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = x, y, z
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    def _step(self) -> None:
        # Sim time starts at 1 s: a zero stamp means "latest" to TF.
        t = time.monotonic() - self.start + 1.0
        dt, self.last = (t - self.last if self.last else 0.0), t
        v, w = self.cmd if t - 1.0 - self.cmd_time < CMD_TIMEOUT else (0.0, 0.0)
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        stamp = self._stamp(t)
        self.clock_pub.publish(Clock(clock=stamp))

        tf = self._tf(t, "odom", "base_footprint", self.x, self.y, self.yaw)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odom_pub.publish(odom)

        self.tick += 1
        if self.tick % 2 == 0:  # 10 Hz
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = "laser_frame"
            scan.angle_min, scan.angle_max = -FOV / 2, FOV / 2
            scan.angle_increment = FOV / (SAMPLES - 1)
            scan.scan_time = 0.1
            scan.range_min, scan.range_max = 0.1, MAX_RANGE
            scan.ranges = scan_ranges(self.x, self.y, self.yaw)
            self.scan_pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
