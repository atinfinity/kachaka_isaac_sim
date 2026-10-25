"""A GPU-free stand-in for the simulator, for kachaka-api regression checks in
CI. It runs the simulator's own ROS node (ros_node.py) and kachaka-api gRPC
server (api_server.py), and replaces Isaac Sim with a kinematic robot in the
2D default room (flat_world.py): no physics, cameras or object detection.

Run it in the ROS image (docker/Dockerfile) with SLAM + Nav2 and the
command server, as for the simulator:

    python3 scripts/fake_sim.py &
    ros2 launch kachaka_sim_bringup slam.launch.py nav:=true rviz:=false

What it mirrors from SimRobot (robot.py): the velocity limiter and front
obstacle stop, wheel and fused odometry, a LiDAR scan of the room's walls,
boxes, people and shelf legs, the IMU, battery and charger, Kachaka Shelf
docking and towing, and sim/reset. The robot stops at whatever it would
run into (a rectangle the size of the body), without bumping or sliding.
Needs build/kachaka.urdf (scripts/build_usd.py --urdf-only).
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.config import DEFAULT_CONFIG, load_config
from kachaka_isaac_sim.drive import (
    NoisyOdometry,
    Pose2D,
    VelocityCommand,
    WheelOdometry,
    drive_limits,
    relative_pose,
    wheel_speeds,
    wrap_angle,
)
from kachaka_isaac_sim.flat_world import (
    Box,
    FlatShelves,
    any_inside,
    cast_rays,
    outline,
    room_boxes,
    transform,
)
from kachaka_isaac_sim.physics_sensors import matrix_to_quat, quat_to_matrix
from kachaka_isaac_sim.power import Charger, EnergyMeter, battery_from_config, drive_power_from_config
from kachaka_isaac_sim.safety import FrontStop, FrontStopZone, front_clearance
from kachaka_isaac_sim.scenes import SHELF_SIZE, SPAWN
from kachaka_isaac_sim.shelves import ShelfSpec
from kachaka_isaac_sim.urdf_tools import frame_pose, read_joints

ROBOT_URDF = ROOT / "build/kachaka.urdf"
LEFT_WHEEL = "base_l_drive_wheel_joint"
RIGHT_WHEEL = "base_r_drive_wheel_joint"
DOCKING_JOINT = "docking_joint"
PIN_UP = 0.012  # m, docking_joint upper limit (robot.py)
# Body outline in base_footprint: 387 x 240 mm with the front at
# front_stop.front_x (est., from the body mesh).
BODY_LENGTH, BODY_HALF_WIDTH = 0.387, 0.12
# s, time constant of the yaw rate response, tuned so ROTATE_IN_PLACE ends
# where it does in Isaac Sim (command_server.py ROTATE_LAG).
TURN_LAG = 0.3


class FlatLidar:
    """Lidar (physics_sensors.py) over flat_world boxes: same angles, noise,
    range cut-off and attributes, so ros_node.publish_scan and
    front_clearance take it as they are."""

    def __init__(self, cfg, mount_xyz, mount_quat, rng: random.Random):
        self.cfg = cfg
        self.rng = rng
        self.offset = np.array(mount_xyz, dtype=float)
        self.mount_rot = quat_to_matrix(*mount_quat)
        n = int(cfg.samples)
        fov = math.radians(cfg.fov_deg)
        fwd = self.mount_rot.T @ np.array([1.0, 0.0, 0.0])
        center = math.atan2(fwd[1], fwd[0])
        self.angle_min = center - fov / 2
        self.angle_increment = fov / (n - 1)
        self.angles = self.angle_min + self.angle_increment * np.arange(n)
        self.local_dirs = np.stack([np.cos(self.angles), np.sin(self.angles), np.zeros(n)], axis=1)
        base_dirs = self.local_dirs @ self.mount_rot.T
        self.base_angles = np.arctan2(base_dirs[:, 1], base_dirs[:, 0])
        self.ranges: list[float] = [math.inf] * n
        self.raw = np.full(n, math.inf)

    @property
    def angle_max(self) -> float:
        return float(self.angles[-1])

    def scan(self, pose: Pose2D, boxes) -> None:
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        ox = pose.x + c * self.offset[0] - s * self.offset[1]
        oy = pose.y + s * self.offset[0] + c * self.offset[1]
        cfg = self.cfg
        raw = cast_rays(ox, oy, pose.yaw + self.base_angles, boxes)
        raw[raw > cfg.max_range] = math.inf
        self.raw = raw
        noisy = raw + np.array([self.rng.gauss(0.0, cfg.noise_stddev) for _ in raw])
        noisy[(noisy < cfg.min_range) | (noisy > cfg.max_range) | ~np.isfinite(raw)] = math.inf
        self.ranges = [float(r) for r in noisy]


class FlatImu:
    """The attributes ros_node.publish_imu reads: a robot on flat ground
    feels gravity, its forward and centripetal acceleration and its yaw rate."""

    def __init__(self, cfg, mount_quat, rng: random.Random):
        self.cfg = cfg
        self.rng = rng
        self.mount_rot = quat_to_matrix(*mount_quat)
        self.orientation = (1.0, 0.0, 0.0, 0.0)
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.array([0.0, 0.0, 9.81])

    def update(self, yaw: float, v: float, w: float, accel: float) -> None:
        c, s = math.cos(yaw), math.sin(yaw)
        base_rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        to_imu = self.mount_rot.T
        g, a = self.cfg.gyro_noise, self.cfg.accel_noise
        self.angular_velocity = to_imu @ np.array([0.0, 0.0, w]) + np.array([self.rng.gauss(0, g) for _ in range(3)])
        self.linear_acceleration = (to_imu @ np.array([accel, v * w, 9.81])
                                    + np.array([self.rng.gauss(0, a) for _ in range(3)]))
        self.orientation = matrix_to_quat(base_rot @ self.mount_rot)


class FakeRobot:
    """SimRobot without Isaac Sim: the robot follows the limited velocity
    command exactly unless the body would end up inside an obstacle."""

    def __init__(self, cfg, node, spawn: Pose2D, people, charger: Charger | None, shelves: FlatShelves | None,
                 seed: int | None):
        self.cfg = cfg
        self.node = node
        self.t = 0.0
        self.spawn = spawn
        self.gt = Pose2D(spawn.x, spawn.y, spawn.yaw)
        self.static_boxes = room_boxes(people, float(cfg.people.height))
        self.body = outline(cfg.front_stop.front_x - BODY_LENGTH, cfg.front_stop.front_x, BODY_HALF_WIDTH)
        rng = random.Random(seed)
        d = cfg.drive
        self.limiter = VelocityCommand(drive_limits(d))
        self.v = self.w = 0.0
        self.wheel_angles = [0.0, 0.0]
        self.wheel_rates = [0.0, 0.0]
        self.wheel_odom = WheelOdometry(d.wheel_radius, d.wheel_separation)
        self.fused_odom = NoisyOdometry(cfg.odometry.fused_noise_xy, cfg.odometry.fused_noise_yaw, seed)
        self.odom_period = 1.0 / cfg.odometry.rate
        self.battery = battery_from_config(cfg.battery)
        self.drive_power = drive_power_from_config(cfg.battery)
        self.energy = EnergyMeter()
        self.charger = charger
        self.on_charger = charger is not None and charger.docked(self.gt)
        self.sensors_off_on_charger = bool(cfg.charger.sensors_off)
        self.shelves = shelves
        self.hold_until = -math.inf
        self.next_blocked_log = 0.0
        self.turn_lag = TURN_LAG
        self.trace = False  # log the pose and the command once a second
        self.next_trace = 0.0
        joints = read_joints(ROBOT_URDF)
        self.lidar = FlatLidar(cfg.lidar, *frame_pose(joints, cfg.lidar.frame_id), rng)
        self.lidar_period = 1.0 / cfg.lidar.rate
        fs = cfg.front_stop
        self.front_stop = FrontStop(
            FrontStopZone(fs.front_x, fs.half_width, fs.margin, self.lidar_period, d.max_linear_accel),
            bool(fs.enabled))
        self.front_blocked = False
        self.imu = FlatImu(cfg.imu, frame_pose(joints, cfg.imu.frame_id)[1], rng)
        self.imu_period = 1.0 / cfg.imu.rate
        self.next = {"odom": 0.0, "lidar": 0.0, "imu": 0.0, "battery": 0.0, "shelves": 0.0}
        node.front_stop = self.front_stop
        node.publish_front_stop(False)
        node.reset_handler = self.reset
        if shelves is not None:
            node.shelf_dock_handler = self.set_shelf_docked
            node.shelf_reset_handler = shelves.reset
            node.publish_shelf_docked("")

    @property
    def towing(self) -> bool:
        return self.shelves is not None and self.shelves.docked is not None

    @property
    def sensors_paused(self) -> bool:
        return self.on_charger and self.sensors_off_on_charger

    def obstacles(self):
        return self.static_boxes + (self.shelves.legs() if self.shelves is not None else [])

    def set_shelf_docked(self, dock: bool) -> tuple[bool, str]:
        sh = self.shelves
        ok, msg = sh.dock(self.gt) if dock else sh.undock()
        if ok:
            self.limiter.towing = self.towing
            self.node.publish_shelf_docked(sh.specs[sh.docked].id if self.towing else "")
        return ok, msg

    def reset(self, clear_odometry: bool = False, hold: float = 1.0) -> None:
        if self.towing:
            self.set_shelf_docked(False)
        if self.shelves is not None:
            self.shelves.reset()
        o = self.spawn
        self.gt = Pose2D(o.x, o.y, o.yaw)
        self.v = self.w = 0.0
        self.wheel_rates = [0.0, 0.0]
        lim = self.limiter
        lim.target = lim.current = (0.0, 0.0)
        lim.last_cmd_time = None
        self.hold_until = self.t + hold
        self.wheel_odom.reset()
        if clear_odometry:
            self.fused_odom.reset(o)
        else:
            self.fused_odom.teleported()
        self.on_charger = self.charger is not None and self.charger.docked(self.gt)
        self.battery.reset()
        self.energy = EnergyMeter()
        self.front_stop.enabled = bool(self.cfg.front_stop.enabled)
        self.front_stop.muted = False
        self.front_stop.blocked = False
        self.node.publish_front_stop(False)

    def _overlaps(self, pose: Pose2D, boxes: list[Box]) -> bool:
        """Whether the body outline crosses a box or a box centre lies inside the body."""
        if any_inside(transform(self.body, pose), boxes):
            return True
        x_mid = self.cfg.front_stop.front_x - BODY_LENGTH / 2
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        body = Box(pose.x + c * x_mid, pose.y + s * x_mid, pose.yaw, BODY_LENGTH, 2 * BODY_HALF_WIDTH)
        return any_inside(np.array([(b.x, b.y) for b in boxes]), [body])

    def _hits(self, old: Pose2D, new: Pose2D) -> bool:
        """Whether moving from `old` to `new` runs into an obstacle. Once the
        robot overlaps one (a shelf reset onto it, say), it may move freely
        until it is clear: Isaac Sim's physics would push the two apart."""
        boxes = self.obstacles()
        return self._overlaps(new, boxes) and not self._overlaps(old, boxes)

    def _due(self, key: str, period: float) -> bool:
        if self.t + 1e-9 >= self.next[key]:
            self.next[key] += period
            return True
        return False

    def step(self, dt: float) -> None:
        node, cfg = self.node, self.cfg
        cmd = node.take_cmd()
        if cmd is not None and self.t >= self.hold_until:
            self.limiter.set_target(*cmd[:2], self.t, autonomous=cmd[2])
        if node.paused:  # a step or a collision (not simulated here): stop at once
            self.limiter.target = self.limiter.current = (0.0, 0.0)
        self.front_blocked = self.front_stop.update(self.limiter.current[0])
        v, w = self.limiter.update(self.t, dt, self.front_blocked)
        # Isaac Sim's robot keeps turning a while after its yaw rate command
        # drops (wheel drive and casters); a first-order lag stands in for it.
        if self.turn_lag > 0:
            w = self.w + (w - self.w) * min(1.0, dt / self.turn_lag)
        # Move along the arc, unless the body would end up in an obstacle.
        g = self.gt
        mid = g.yaw + w * dt / 2
        new = Pose2D(g.x + v * dt * math.cos(mid), g.y + v * dt * math.sin(mid), wrap_angle(g.yaw + w * dt))
        if (v or w) and self._hits(g, new):
            if self.t >= self.next_blocked_log:  # at most once a second
                self.next_blocked_log = self.t + 1.0
                print(f"[fake_sim] t={self.t:.1f}: blocked at ({g.x:.2f}, {g.y:.2f}, {g.yaw:.2f}) "
                      f"driving v={v:.2f} w={w:.2f}", flush=True)
            v = w = 0.0
            self.limiter.current = (0.0, 0.0)
        else:
            self.gt = new
        accel = (v - self.v) / dt if dt > 0 else 0.0
        self.v, self.w = v, w
        d = cfg.drive
        self.wheel_rates = list(wheel_speeds(v, w, d.wheel_radius, d.wheel_separation))
        self.wheel_angles = [wrap_angle(a + r * dt) for a, r in zip(self.wheel_angles, self.wheel_rates)]
        if self.shelves is not None:
            self.shelves.follow(self.gt)
        self.t += dt
        t = self.t
        node.publish_clock(t)
        self.wheel_odom.update(self.wheel_angles[0], self.wheel_angles[1], dt)
        # No wheel torques in the flat model: idle and speed losses only.
        self.energy.add(self.drive_power(self.wheel_rates, (0.0, 0.0), d.wheel_radius, self.towing), dt)

        if self._due("imu", self.imu_period):
            self.imu.update(self.gt.yaw, v, w, accel)
            node.publish_imu(t, self.imu)

        if self._due("odom", self.odom_period):
            gt = self.gt
            self.on_charger = self.charger is not None and self.charger.docked(gt)
            fused = self.fused_odom.update(gt)
            wo = self.wheel_odom
            node.publish_odometry(t, fused, wo.pose, gt, v, w, wo.v, wo.w)
            if self.charger is not None:
                rel = relative_pose(gt, self.charger.pose)
                if math.hypot(rel.x, rel.y) <= cfg.charger.detection_range:
                    node.publish_charger_detection(t, rel)
            names = [LEFT_WHEEL, RIGHT_WHEEL, DOCKING_JOINT]
            node.publish_joints(t, names, [*self.wheel_angles, PIN_UP if self.towing else 0.0],
                                [*self.wheel_rates, 0.0])
            if self.shelves is not None:
                poses = self.shelves.poses
                node.publish_shelf_detection(t, [relative_pose(gt, p) for p in poses])
                if self._due("shelves", 0.5):
                    node.publish_shelf_poses(t, [relative_pose(self.spawn, p) for p in poses])

        if self._due("lidar", self.lidar_period):
            if self.sensors_paused:
                self.front_stop.clearance = math.inf
            else:
                lidar = self.lidar
                lidar.scan(self.gt, self.obstacles())
                node.publish_scan(t, lidar, self.lidar_period)
                fs = cfg.front_stop
                self.front_stop.clearance = front_clearance(
                    lidar.raw, lidar.local_dirs, lidar.offset, lidar.mount_rot, fs.front_x, fs.half_width)
        node.publish_front_stop(self.front_blocked)
        if self.trace and t >= self.next_trace and (self.limiter.target != (0.0, 0.0) or v or w):
            self.next_trace = t + 1.0
            tv, tw = self.limiter.target
            print(f"[fake_sim] t={t:.1f}: at ({g.x:.2f}, {g.y:.2f}, {g.yaw:.2f}) cmd ({tv:+.2f}, {tw:+.2f}) "
                  f"moving ({v:+.2f}, {w:+.2f}) front clearance {self.front_stop.clearance:.2f}"
                  f"{' BLOCKED' if self.front_blocked else ''}{' towing' if self.towing else ''}", flush=True)

        if self._due("battery", 1.0):
            power = self.energy.take()
            self.battery.update(1.0, self.on_charger, self.drive_power.idle if power is None else power)
            node.publish_battery(t, self.battery, self.on_charger, bool(cfg.battery.bridge_fields_only))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot parameters (YAML)")
    p.add_argument("--rate", type=float, default=100.0, help="steps per sim second")
    p.add_argument("--realtime", type=float, default=1.0, help="target real-time factor; 0 runs as fast as possible")
    p.add_argument("--max-sim-time", type=float, default=0.0, help="exit after this many sim seconds (0: never)")
    p.add_argument("--seed", type=int, default=None, help="seed for sensor and odometry noise")
    p.add_argument("--api-port", type=int, default=26400, help="port of the kachaka-api gRPC server (0: no server)")
    p.add_argument("--no-charger", action="store_true", help="no charging dock")
    p.add_argument("--shelf", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW"),
                   help="place a Kachaka Shelf where a robot docked under it is at X, Y, YAW (world frame; "
                        "repeatable), as run_sim.py does. Default: the shelves in config 'shelf.room'")
    p.add_argument("--no-shelves", action="store_true", help="no Kachaka Shelves")
    p.add_argument("--no-people", action="store_true", help="no people")
    p.add_argument("--turn-lag", type=float, default=TURN_LAG, help="yaw rate response time constant [s]")
    p.add_argument("--trace", action="store_true", help="log the pose and the command once a second while moving")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    if not ROBOT_URDF.exists():
        print("build/kachaka.urdf is missing: run scripts/fetch_assets.py and "
              "scripts/build_usd.py --urdf-only first", file=sys.stderr)
        return 1

    from kachaka_isaac_sim.ros_node import KachakaSimNode
    from kachaka_isaac_sim.torches import Torches

    node = KachakaSimNode(cfg, ROBOT_URDF)
    node.torches = Torches(cfg.torch)
    node.publish_torch()
    api = None
    if args.api_port:
        from kachaka_isaac_sim.api_server import KachakaApiServer

        api = KachakaApiServer(node, args.api_port)

    # The same world as `run_sim.py --world room`, with poses for the ROS
    # side relative to the spawn pose (where SLAM starts its map).
    spawn = Pose2D(*SPAWN.get("room", (0.0, 0.0, 0.0)))
    charger = None
    if not args.no_charger:
        dock = Pose2D(*cfg.charger.room_pose)
        charger = Charger(dock, cfg.charger.tolerance_xy, cfg.charger.tolerance_yaw)
        node.publish_charger_pose(relative_pose(spawn, dock))
    shelves = None
    if not args.no_shelves:
        entries = ([(f"S{i + 1:02d}", f"Shelf {i + 1}", pose) for i, pose in enumerate(args.shelf)] if args.shelf
                   else [(e.id, e.name, e.pose) for e in cfg.shelf.room])
        specs = [ShelfSpec(i, name, f"/World/Shelves/{i}", Pose2D(*pose)) for i, name, pose in entries]
        shelves = FlatShelves(specs, cfg.shelf.dock_tolerance_xy, cfg.shelf.dock_tolerance_yaw)

        def in_map(p):
            m = relative_pose(spawn, p)
            return [round(m.x, 4), round(m.y, 4), round(m.yaw, 4)]

        node.publish_layout(
            [{"id": s.id, "name": s.name, "home": in_map(s.home), "width": SHELF_SIZE[1], "depth": SHELF_SIZE[0]}
             for s in specs],
            [] if args.shelf else
            [{"id": e.id, "name": e.name, "pose": in_map(Pose2D(*e.pose))} for e in cfg.shelf.room_locations])
    people = [] if args.no_people else [tuple(p) for p in cfg.people.room]
    robot = FakeRobot(cfg, node, spawn, people, charger, shelves, args.seed)
    robot.trace = args.trace
    robot.turn_lag = args.turn_lag
    node.publish_errors()  # GetError: none yet

    dt = 1.0 / args.rate
    wall_start = time.monotonic()
    where = ", charger" if charger else ""
    if api:
        where += f", kachaka-api on port {args.api_port}"
    print(f"[fake_sim] running: ROS namespace '{node.ns}', flat room{where}", flush=True)
    try:
        while True:
            node.spin_once()
            node.run_pending()
            robot.step(dt)
            if args.max_sim_time and robot.t >= args.max_sim_time:
                break
            if args.realtime > 0:
                ahead = robot.t / args.realtime - (time.monotonic() - wall_start)
                if ahead > 0:
                    time.sleep(ahead)
    except KeyboardInterrupt:
        pass
    wall = time.monotonic() - wall_start
    print(f"[fake_sim] stopped: {robot.t:.1f} s sim in {wall:.1f} s wall", flush=True)
    if api:
        api.stop()
    node.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
