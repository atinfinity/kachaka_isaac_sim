"""ROS 2 side of the simulator: one rclpy node inside the Isaac Sim process.

Topic names, types and QoS follow the official kachaka-api ROS 2 bridge so a
ROS 2 app written for the real robot works unchanged. Everything is stamped
with simulation time, and /clock is published from here.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .drive import Pose2D, quat_from_yaw
from .urdf_tools import read_joints

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def to_time(t: float) -> Time:
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class KachakaSimNode:
    """Owns the rclpy node, publishers and the latest commands from ROS."""

    def __init__(self, cfg: SimpleNamespace, urdf_path: Path):
        rclpy.init()
        self.cfg = cfg
        self.node = Node("kachaka_isaac_sim")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        ns = "/" + cfg.ros.namespace.strip("/") if cfg.ros.namespace else ""
        self.ns = ns
        self.prefix = cfg.ros.frame_prefix or ""
        self.generic = bool(cfg.ros.remap_generic)
        self.manual_control_enabled = True
        self.auto_homing_enabled = True
        self.cmd: tuple[float, float] | None = None  # newest Twist, consumed by the sim loop
        n = self.node

        # --- inputs ---------------------------------------------------------
        n.create_subscription(Twist, f"{ns}/manual_control/cmd_vel", self._on_cmd_vel, qos_profile_sensor_data)
        if self.generic:
            n.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        n.create_service(SetBool, f"{ns}/manual_control/set_enabled", self._on_manual_control)
        n.create_service(SetBool, f"{ns}/auto_homing/set_enabled", self._on_auto_homing)

        # --- outputs --------------------------------------------------------
        self.clock_pub = n.create_publisher(Clock, "/clock", 10)
        # The real bridge publishes all sensor streams with SensorDataQoS.
        sensor = qos_profile_sensor_data
        self.odom_pub = n.create_publisher(Odometry, f"{ns}/odometry/odometry", sensor)
        self.wheel_odom_pub = n.create_publisher(Odometry, f"{ns}/wheel_odometry/wheel_odometry", sensor)
        self.gt_pub = n.create_publisher(Odometry, f"{ns}/sim/ground_truth", sensor)
        self.generic_odom_pub = n.create_publisher(Odometry, "/odom", 10) if self.generic else None
        self.scan_pub = n.create_publisher(LaserScan, f"{ns}/lidar/scan", sensor)
        self.generic_scan_pub = n.create_publisher(LaserScan, "/scan", sensor) if self.generic else None
        self.imu_pub = n.create_publisher(Imu, f"{ns}/imu/imu", sensor)
        self.joint_pub = n.create_publisher(JointState, f"{ns}/joint_states", 10)
        self.tf_pub = n.create_publisher(TFMessage, "/tf", 100)
        self.tf_static_pub = n.create_publisher(TFMessage, "/tf_static", LATCHED)
        self.battery_pub = n.create_publisher(BatteryState, f"{ns}/robot_info/battery_state", 10)
        version_pub = n.create_publisher(String, f"{ns}/robot_info/version", LATCHED)
        desc_pub = n.create_publisher(String, f"{ns}/robot_description", LATCHED)

        version_pub.publish(String(data=str(cfg.stubs.version)))
        desc_pub.publish(String(data=self._prefixed_urdf(urdf_path)))
        self.joints = read_joints(urdf_path)
        self._publish_static_tf()

    # --- helpers --------------------------------------------------------------
    def frame(self, name: str) -> str:
        return f"{self.prefix}{name}"

    def _prefixed_urdf(self, urdf_path: Path) -> str:
        text = urdf_path.read_text()
        if not self.prefix:
            return text
        for tag in ("link name", "parent link", "child link"):
            text = text.replace(f'{tag}="', f'{tag}="{self.prefix}')
        return text

    def _publish_static_tf(self) -> None:
        msg = TFMessage()
        for j in self.joints:
            if j["type"] != "fixed":
                continue
            msg.transforms.append(self._transform(0.0, j["parent"], j["child"], j["xyz"], j["quat"]))
        self.tf_static_pub.publish(msg)

    def _transform(self, t: float, parent: str, child: str, xyz, quat) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = to_time(t)
        tf.header.frame_id = self.frame(parent)
        tf.child_frame_id = self.frame(child)
        tr = tf.transform.translation
        tr.x, tr.y, tr.z = (float(c) for c in xyz)
        r = tf.transform.rotation
        r.w, r.x, r.y, r.z = (float(c) for c in quat)
        return tf

    # --- callbacks ------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z)

    def _on_manual_control(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        # The real robot needs manual control enabled before cmd_vel moves it;
        # the simulator accepts cmd_vel either way and only records the flag.
        self.manual_control_enabled = req.data
        res.success = True
        return res

    def _on_auto_homing(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.auto_homing_enabled = req.data
        res.success = True
        return res

    # --- called from the sim loop --------------------------------------------
    def spin_once(self) -> None:
        self.executor.spin_once(timeout_sec=0.0)

    def take_cmd(self) -> tuple[float, float] | None:
        cmd, self.cmd = self.cmd, None
        return cmd

    def publish_clock(self, t: float) -> None:
        self.clock_pub.publish(Clock(clock=to_time(t)))

    def _odom_msg(self, t: float, frame_id: str, pose: Pose2D, v: float, w: float, cov_xy: float) -> Odometry:
        msg = Odometry()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(frame_id)
        msg.child_frame_id = self.frame("base_footprint")
        # The bundled rclpy asserts on numpy scalars, so cast everything.
        msg.pose.pose.position.x = float(pose.x)
        msg.pose.pose.position.y = float(pose.y)
        q = msg.pose.pose.orientation
        q.w, q.x, q.y, q.z = (float(c) for c in quat_from_yaw(pose.yaw))
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)
        for i in (0, 7):
            msg.pose.covariance[i] = cov_xy
        msg.pose.covariance[35] = cov_xy
        for i in (0, 7, 35):
            msg.twist.covariance[i] = 1e-3
        return msg

    def publish_odometry(self, t: float, fused: Pose2D, wheel: Pose2D, gt: Pose2D, v: float, w: float,
                         wheel_v: float, wheel_w: float) -> None:
        odom = self._odom_msg(t, "odom", fused, v, w, 1e-3)
        self.odom_pub.publish(odom)
        if self.generic_odom_pub:
            self.generic_odom_pub.publish(odom)
        self.wheel_odom_pub.publish(self._odom_msg(t, "odom", wheel, wheel_v, wheel_w, 1e-2))
        self.gt_pub.publish(self._odom_msg(t, "world", gt, v, w, 0.0))
        tf = self._transform(t, "odom", "base_footprint", (fused.x, fused.y, 0.0), quat_from_yaw(fused.yaw))
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def publish_joints(self, t: float, names: list[str], positions: list[float], velocities: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = to_time(t)
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in velocities]
        self.joint_pub.publish(msg)
        # Moving joints on /tf, as robot_state_publisher would do.
        by_name = dict(zip(names, positions))
        tfs = []
        for j in self.joints:
            if j["name"] not in by_name:
                continue
            q = float(by_name[j["name"]])
            ax = j["axis"]
            if j["type"] == "prismatic":
                xyz = tuple(o + a * q for o, a in zip(j["xyz"], ax))
                quat = j["quat"]
            else:
                xyz = j["xyz"]
                quat = _quat_mul(j["quat"], _axis_angle(ax, q))
            tfs.append(self._transform(t, j["parent"], j["child"], xyz, quat))
        self.tf_pub.publish(TFMessage(transforms=tfs))

    def publish_scan(self, t: float, lidar, scan_time: float) -> None:
        cfg = self.cfg.lidar
        msg = LaserScan()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(cfg.frame_id)
        msg.angle_min = float(lidar.angle_min)
        msg.angle_max = float(lidar.angle_max)
        msg.angle_increment = float(lidar.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = float(scan_time)
        msg.range_min = float(cfg.min_range)
        msg.range_max = float(cfg.max_range)
        msg.ranges = lidar.ranges
        self.scan_pub.publish(msg)
        if self.generic_scan_pub:
            self.generic_scan_pub.publish(msg)

    def publish_imu(self, t: float, imu) -> None:
        cfg = self.cfg.imu
        msg = Imu()
        msg.header.stamp = to_time(t)
        msg.header.frame_id = self.frame(cfg.frame_id)
        o = msg.orientation
        o.w, o.x, o.y, o.z = (float(c) for c in imu.orientation)
        av, la = msg.angular_velocity, msg.linear_acceleration
        av.x, av.y, av.z = (float(c) for c in imu.angular_velocity)
        la.x, la.y, la.z = (float(c) for c in imu.linear_acceleration)
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 1e-3
            msg.angular_velocity_covariance[i] = float(cfg.gyro_noise) ** 2
            msg.linear_acceleration_covariance[i] = float(cfg.accel_noise) ** 2
        self.imu_pub.publish(msg)

    def publish_battery(self, t: float, percentage: float) -> None:
        msg = BatteryState()
        msg.header.stamp = to_time(t)
        msg.voltage = float(self.cfg.stubs.battery_voltage)
        msg.percentage = float(percentage)
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(msg)

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def _axis_angle(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
