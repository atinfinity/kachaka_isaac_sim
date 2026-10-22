"""Per-physics-step robot logic: drive control, odometry, LiDAR, IMU and ROS publishing.

`pre_step` and `post_step` run inside PhysX step callbacks, so control and
odometry run at the physics rate no matter how often the app renders.
Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

from isaacsim.core.experimental.prims import Articulation

from .drive import (
    NoisyOdometry,
    Pose2D,
    VelocityCommand,
    WheelOdometry,
    drive_limits,
    quat_from_yaw,
    relative_pose,
    wheel_speeds,
    yaw_from_quat,
)
from .hazards import Cliff, CollisionCounter
from .physics_sensors import CliffSensors, ContactSensor, Imu, Lidar, quat_to_matrix
from .power import Battery, Charger
from .ros_node import KachakaSimNode
from .safety import FrontStop, FrontStopZone, front_clearance
from .urdf_tools import frame_pose, read_com, read_joints

LEFT_WHEEL = "base_l_drive_wheel_joint"
RIGHT_WHEEL = "base_r_drive_wheel_joint"
DOCKING_JOINT = "docking_joint"


class SimRobot:
    def __init__(self, articulation_path: str, robot_path: str, urdf_path: Path, cfg: SimpleNamespace,
                 node: KachakaSimNode, start_time: float, seed: int | None = None, charger: Charger | None = None,
                 shelves=None, map_origin: Pose2D | None = None, contacts: ContactSensor | None = None):
        self.cfg = cfg
        self.node = node
        self.t = start_time
        self.art = Articulation(articulation_path)
        self.dof_names = self.art.dof_names
        self.i_left = self.dof_names.index(LEFT_WHEEL)
        self.i_right = self.dof_names.index(RIGHT_WHEEL)
        self.i_pin = self.dof_names.index(DOCKING_JOINT) if DOCKING_JOINT in self.dof_names else None
        self._configure_drives()

        d = cfg.drive
        self.limiter = VelocityCommand(drive_limits(d))
        self.wheel_odom = WheelOdometry(d.wheel_radius, d.wheel_separation)
        self.fused_odom = NoisyOdometry(cfg.odometry.fused_noise_xy, cfg.odometry.fused_noise_yaw, seed)
        self.odom_period = 1.0 / cfg.odometry.rate
        self.next_odom = start_time
        self.next_battery = start_time
        b = cfg.battery
        self.battery = Battery(float(b.initial_percentage), float(b.drain_per_hour), float(b.charge_per_hour))
        self.charger = charger
        self.on_charger = False
        # The real robot turns its LiDAR and ToF off on the charger.
        self.sensors_off_on_charger = bool(cfg.charger.sensors_off)
        self.shelves = shelves  # shelves.Shelves or None
        self.map_origin = map_origin or Pose2D(0.0, 0.0, 0.0)  # world pose of the map frame
        self.spawn = Pose2D(self.map_origin.x, self.map_origin.y, self.map_origin.yaw)
        self.gt = Pose2D(0.0, 0.0, 0.0)
        self.start_z = float(self.art.get_world_poses()[0].numpy()[0][2])
        self.hold_until = -math.inf  # cmd_vel is ignored until then (after a reset)
        self.next_shelves = start_time
        if shelves is not None:
            node.shelf_dock_handler = self.set_shelf_docked
            node.publish_shelf_docked("")

        joints = read_joints(urdf_path)
        self.lidar = Lidar(cfg.lidar, *frame_pose(joints, cfg.lidar.frame_id), robot_path, seed)
        self.lidar_period = 1.0 / cfg.lidar.rate
        self.next_lidar = start_time
        fs = cfg.front_stop
        self.front_stop = FrontStop(
            FrontStopZone(fs.front_x, fs.half_width, fs.margin, self.lidar_period, d.max_linear_accel),
            bool(fs.enabled))
        self.front_blocked = False
        node.front_stop = self.front_stop
        node.publish_front_stop(False)
        # Step sensors and collisions (hazards.py).
        c = cfg.cliff
        self.cliff_sensors = CliffSensors(c, robot_path, self.start_z)
        self.cliff = Cliff(len(c.sensors), bool(c.enabled))
        node.cliff = self.cliff
        node.publish_cliff()
        self.contacts = contacts
        col = cfg.collision
        self.collisions = CollisionCounter(tuple(col.ignore), float(col.debounce))
        self.collision_stop = bool(col.stop)
        node.publish_errors()
        self.imu = Imu(cfg.imu, *frame_pose(joints, cfg.imu.frame_id), read_com(urdf_path), seed)
        self.imu_period = 1.0 / cfg.imu.rate
        self.next_imu = start_time

    def _configure_drives(self) -> None:
        """Wheel velocity drives limited to the real motor torque.

        The official URDF limits the wheels to 10 rad/s (0.45 m/s), below the
        autonomous top speed: raise the limit to what the fastest cap needs
        while turning at full rate, with some margin."""
        d = self.cfg.drive
        max_eff = self.art.get_dof_max_efforts().numpy()
        max_vel = self.art.get_dof_max_velocities().numpy()
        stiff, damp = (g.numpy() for g in self.art.get_dof_gains())
        top = max(d.max_linear_vel, getattr(d, "max_linear_vel_auto", None) or 0.0)
        wheel_top = 1.2 * (top + d.max_angular_vel * d.wheel_separation / 2) / d.wheel_radius
        for i in (self.i_left, self.i_right):
            max_eff[0, i] = d.wheel_max_torque
            max_vel[0, i] = max(max_vel[0, i], wheel_top)
            stiff[0, i] = 0.0
            damp[0, i] = d.wheel_damping
        self.art.set_dof_max_efforts(max_eff)
        self.art.set_dof_max_velocities(max_vel)
        self.art.set_dof_gains(stiffnesses=stiff, dampings=damp)

    # --- physics callbacks ----------------------------------------------------
    def pre_step(self, dt: float) -> None:
        cmd = self.node.take_cmd()
        if cmd is not None and self.t >= self.hold_until:
            self.limiter.set_target(*cmd[:2], self.t, autonomous=cmd[2])
        if self.node.paused:  # a step or a collision: stop at once
            self.limiter.target = self.limiter.current = (0.0, 0.0)
        self.front_blocked = self.front_stop.update(self.limiter.current[0])
        v, w = self.limiter.update(self.t, dt, self.front_blocked or self.cliff.blocked)
        d = self.cfg.drive
        wl, wr = wheel_speeds(v, w, d.wheel_radius, d.wheel_separation)
        targets = [[0.0] * len(self.dof_names)]
        targets[0][self.i_left], targets[0][self.i_right] = wl, wr
        self.art.set_dof_velocity_targets(targets)
        if self.i_pin is not None:
            pin = [[0.0] * len(self.dof_names)]
            pin[0][self.i_pin] = self.PIN_UP if self.towing else 0.0
            self.art.set_dof_position_targets(pin)

    PIN_UP = 0.012  # m, docking_joint upper limit

    @property
    def towing(self) -> bool:
        return self.shelves is not None and self.shelves.docked is not None

    def set_shelf_docked(self, dock: bool) -> tuple[bool, str]:
        """Raise the pin into the shelf above the robot, or lower it. Called
        from ROS (sim/shelf_dock) between physics steps."""
        sh = self.shelves
        ok, msg = sh.dock(self.gt) if dock else sh.undock()
        if ok:
            self.limiter.towing = self.towing
            self.lidar.ignore = {sh.docked_path} if self.towing else set()
            self.cliff_sensors.ignore = self.lidar.ignore
            self.node.publish_shelf_docked(sh.specs[sh.docked].id if self.towing else "")
        return ok, msg

    def reset(self, clear_odometry: bool = False, hold: float = 1.0) -> None:
        """Back to the start: robot at the spawn pose and at rest, shelves
        home, wheel odometry at zero, battery at its initial level. Called
        from ROS (sim/reset) between physics steps.

        The fused odometry keeps its drift unless `clear_odometry` (sim/reset
        that also clears the SLAM map): SLAM's map -> odom has corrected that
        drift, and a robot standing still gives it nothing to correct a change
        with.

        cmd_vel is ignored for `hold` seconds, so a controller that has not
        heard of the reset yet cannot drive the robot off again."""
        if self.towing:
            self.set_shelf_docked(False)
        if self.shelves is not None:
            self.shelves.reset()
        o = self.map_origin = Pose2D(self.spawn.x, self.spawn.y, self.spawn.yaw)
        self.art.set_world_poses([[o.x, o.y, self.start_z]], [list(quat_from_yaw(o.yaw))])
        self.art.set_velocities([[0.0] * 3], [[0.0] * 3])
        self.art.set_dof_velocities([[0.0] * len(self.dof_names)])
        lim = self.limiter
        lim.target = lim.current = (0.0, 0.0)
        lim.last_cmd_time = None
        self.hold_until = self.t + hold
        self.wheel_odom.reset()
        if clear_odometry:
            self.fused_odom.reset(o)  # SLAM starts its new map here, at the spawn pose
        else:
            self.fused_odom.teleported()
        self.gt = Pose2D(o.x, o.y, o.yaw)
        self.on_charger = self.charger is not None and self.charger.docked(self.gt)
        self.battery.reset()
        self.cliff.reset()
        self.collisions.reset()
        if self.contacts is not None:
            self.contacts.take()
        self.front_stop.enabled = bool(self.cfg.front_stop.enabled)
        self.front_stop.muted = False
        self.front_stop.blocked = False
        self.node.publish_front_stop(False)

    def shift(self, dx: float, dy: float) -> None:
        """Carry the robot (and the shelf it tows) by (dx, dy), as an elevator
        ride does: speed and odometry go on as if it had not moved, and the
        map frame moves with it (each floor's map has the spawn pose moved to
        that floor as its frame). Called between physics steps."""
        pos, quat = (a.numpy()[0].astype(float) for a in self.art.get_world_poses())
        lin, ang = (a.numpy()[0].astype(float) for a in self.art.get_velocities())
        self.art.set_world_poses([[pos[0] + dx, pos[1] + dy, pos[2]]], [list(quat)])
        self.art.set_velocities([list(lin)], [list(ang)])
        if self.towing:
            self.shelves.shift([self.shelves.docked], dx, dy)
        self.fused_odom.shift(dx, dy)
        self.gt = Pose2D(self.gt.x + dx, self.gt.y + dy, self.gt.yaw)
        self.map_origin = Pose2D(self.map_origin.x + dx, self.map_origin.y + dy, self.map_origin.yaw)

    def post_step(self, dt: float) -> None:
        self.t += dt
        t = self.t
        node = self.node
        node.publish_clock(t)
        q = self.art.get_dof_positions().numpy()[0]
        self.wheel_odom.update(float(q[self.i_left]), float(q[self.i_right]), dt)

        # Base state every step: the IMU differentiates velocity.
        pos, quat = (a.numpy()[0].astype(float) for a in self.art.get_world_poses())
        lin, ang = (a.numpy()[0].astype(float) for a in self.art.get_velocities())  # at the COM, world frame
        rot = quat_to_matrix(*quat)  # quat is (w, x, y, z)
        self.imu.update(rot, lin, ang, dt)
        self._hazards(t, pos, rot)
        if t + 1e-9 >= self.next_imu:
            self.next_imu += self.imu_period
            node.publish_imu(t, self.imu)

        if t + 1e-9 >= self.next_odom:
            self.next_odom += self.odom_period
            qd = self.art.get_dof_velocities().numpy()[0]
            yaw = yaw_from_quat(*quat)
            gt = Pose2D(float(pos[0]), float(pos[1]), yaw)
            self.gt = gt
            self.on_charger = self.charger is not None and self.charger.docked(gt)
            # The COM offset only adds a sideways term, so the forward
            # projection of the COM velocity is exact.
            v = float(lin[0] * math.cos(yaw) + lin[1] * math.sin(yaw))
            w = float(ang[2])
            fused = self.fused_odom.update(gt)
            wo = self.wheel_odom
            node.publish_odometry(t, fused, wo.pose, gt, v, w, wo.v, wo.w)
            if self.charger is not None:
                rel = relative_pose(gt, self.charger.pose)
                if math.hypot(rel.x, rel.y) <= self.cfg.charger.detection_range:
                    node.publish_charger_detection(t, rel)
            node.publish_joints(t, self.dof_names, q, qd)
            if self.shelves is not None:
                # The real robot finds shelves with its cameras; the
                # simulator hands their poses over instead.
                poses = self.shelves.update()
                node.publish_shelf_detection(t, [relative_pose(gt, p) for p in poses])
                if t + 1e-9 >= self.next_shelves:
                    self.next_shelves += 0.5
                    node.publish_shelf_poses(t, [relative_pose(self.map_origin, p) for p in poses])

        if t + 1e-9 >= self.next_lidar:
            self.next_lidar += self.lidar_period
            if self.sensors_paused:
                self.front_stop.clearance = float("inf")
            else:
                self.lidar.scan(pos, rot)
                node.publish_scan(t, self.lidar, self.lidar_period)
                fs, lidar = self.cfg.front_stop, self.lidar
                self.front_stop.clearance = front_clearance(
                    lidar.raw, lidar.local_dirs, lidar.offset, lidar.mount_rot, fs.front_x, fs.half_width)
        node.publish_front_stop(self.front_blocked)

        if t + 1e-9 >= self.next_battery:
            self.next_battery += 1.0
            level = self.battery.update(1.0, self.on_charger)
            node.publish_battery(t, level, self.on_charger)

    def _hazards(self, t: float, pos, rot) -> None:
        """Step sensors every step; contact events since the last step."""
        node = self.node
        if self.cliff.update(self.cliff_sensors.read(pos, rot)):
            self.limiter.target = self.limiter.current = (0.0, 0.0)
            node.pause()
        node.publish_cliff()
        if self.contacts is None:
            return
        docked = self.shelves.docked_path if self.towing else None
        for other, found in self.contacts.take():
            if docked is not None and (other == docked or other.startswith(docked + "/")):
                continue
            if self.collisions.contact(other, t, found):
                gt = Pose2D(float(pos[0]), float(pos[1]), math.atan2(rot[1, 0], rot[0, 0]))
                node.publish_collision(t, self.collisions.count, other, gt)
                if self.collision_stop and not node.collision_stopped:
                    self.limiter.target = self.limiter.current = (0.0, 0.0)
                    node.pause(collision=True)

    @property
    def sensors_paused(self) -> bool:
        return self.on_charger and self.sensors_off_on_charger
