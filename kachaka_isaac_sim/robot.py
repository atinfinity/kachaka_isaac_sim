"""Per-physics-step robot logic: drive control, odometry and ROS publishing.

`pre_step` and `post_step` run inside PhysX step callbacks, so control and
odometry run at the physics rate no matter how often the app renders.
Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from isaacsim.core.experimental.prims import Articulation

from .drive import (
    DriveLimits,
    NoisyOdometry,
    Pose2D,
    VelocityCommand,
    WheelOdometry,
    wheel_speeds,
    yaw_from_quat,
)
from .ros_node import KachakaSimNode

LEFT_WHEEL = "base_l_drive_wheel_joint"
RIGHT_WHEEL = "base_r_drive_wheel_joint"


class SimRobot:
    def __init__(self, articulation_path: str, cfg: SimpleNamespace, node: KachakaSimNode,
                 start_time: float, seed: int | None = None):
        self.cfg = cfg
        self.node = node
        self.t = start_time
        self.art = Articulation(articulation_path)
        self.dof_names = self.art.dof_names
        self.i_left = self.dof_names.index(LEFT_WHEEL)
        self.i_right = self.dof_names.index(RIGHT_WHEEL)
        self._configure_drives()

        d = cfg.drive
        self.limiter = VelocityCommand(DriveLimits(
            d.max_linear_vel, d.max_angular_vel, d.max_linear_accel, d.max_angular_accel, d.cmd_vel_timeout))
        self.wheel_odom = WheelOdometry(d.wheel_radius, d.wheel_separation)
        self.fused_odom = NoisyOdometry(cfg.odometry.fused_noise_xy, cfg.odometry.fused_noise_yaw, seed)
        self.odom_period = 1.0 / cfg.odometry.rate
        self.next_odom = start_time
        self.next_battery = start_time
        self.battery = float(cfg.stubs.battery_percentage)

    def _configure_drives(self) -> None:
        """Wheel velocity drives limited to the real motor torque."""
        d = self.cfg.drive
        max_eff = self.art.get_dof_max_efforts().numpy()
        stiff, damp = (g.numpy() for g in self.art.get_dof_gains())
        for i in (self.i_left, self.i_right):
            max_eff[0, i] = d.wheel_max_torque
            stiff[0, i] = 0.0
            damp[0, i] = d.wheel_damping
        self.art.set_dof_max_efforts(max_eff)
        self.art.set_dof_gains(stiffnesses=stiff, dampings=damp)

    # --- physics callbacks ----------------------------------------------------
    def pre_step(self, dt: float) -> None:
        cmd = self.node.take_cmd()
        if cmd is not None:
            self.limiter.set_target(cmd[0], cmd[1], self.t)
        v, w = self.limiter.update(self.t, dt)
        d = self.cfg.drive
        wl, wr = wheel_speeds(v, w, d.wheel_radius, d.wheel_separation)
        targets = [[0.0] * len(self.dof_names)]
        targets[0][self.i_left], targets[0][self.i_right] = wl, wr
        self.art.set_dof_velocity_targets(targets)

    def post_step(self, dt: float) -> None:
        self.t += dt
        t = self.t
        node = self.node
        node.publish_clock(t)
        q = self.art.get_dof_positions().numpy()[0]
        self.wheel_odom.update(float(q[self.i_left]), float(q[self.i_right]), dt)

        if t + 1e-9 >= self.next_odom:
            self.next_odom += self.odom_period
            qd = self.art.get_dof_velocities().numpy()[0]
            pos, quat = (a.numpy()[0] for a in self.art.get_world_poses())
            # Velocities are at the centre of mass; its offset only adds a
            # sideways term, so the forward projection below is exact.
            lin, ang = (a.numpy()[0] for a in self.art.get_velocities())
            yaw = yaw_from_quat(*(float(c) for c in quat))  # (w, x, y, z)
            gt = Pose2D(float(pos[0]), float(pos[1]), yaw)
            v = float(lin[0] * math.cos(yaw) + lin[1] * math.sin(yaw))
            w = float(ang[2])
            fused = self.fused_odom.update(gt)
            wo = self.wheel_odom
            node.publish_odometry(t, fused, wo.pose, gt, v, w, wo.v, wo.w)
            node.publish_joints(t, self.dof_names, q, qd)

        if t + 1e-9 >= self.next_battery:
            self.next_battery += 1.0
            drain = self.cfg.stubs.battery_drain_per_hour / 3600.0
            self.battery = max(0.0, self.battery - drain)
            node.publish_battery(t, self.battery)
