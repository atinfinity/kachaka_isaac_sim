"""Sensors computed from physics state: LiDAR (PhysX raycasts) and IMU.

The LiDAR uses PhysX scene queries instead of the RTX LiDAR because the
LiDAR sits inside the body mesh, and RTX rays cannot ignore the robot's own
geometry. Raycasts only see collision shapes, which is what a 2D scan needs.
Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
import random
from types import SimpleNamespace

import numpy as np


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m: np.ndarray) -> tuple[float, float, float, float]:
    """Return (w, x, y, z)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s


class Lidar:
    """2D scan in the laser frame, ignoring the robot's own collision shapes."""

    def __init__(self, cfg: SimpleNamespace, mount_xyz, mount_quat, robot_path: str, seed: int | None = None):
        from omni.physx import get_physx_scene_query_interface

        self.query = get_physx_scene_query_interface()
        self.cfg = cfg
        self.robot_prefix = robot_path + "/"
        self.ignore: set[str] = set()  # rigid bodies to see through (a towed shelf)
        self.offset = np.array(mount_xyz, dtype=float)
        self.mount_rot = quat_to_matrix(*mount_quat)
        self.rng = random.Random(seed)
        n = int(cfg.samples)
        fov = math.radians(cfg.fov_deg)
        # Centre the field of view on the robot's forward direction (base +x),
        # expressed in the laser frame.
        fwd = self.mount_rot.T @ np.array([1.0, 0.0, 0.0])
        center = math.atan2(fwd[1], fwd[0])
        self.angle_min = center - fov / 2
        self.angle_increment = fov / (n - 1)
        self.angles = self.angle_min + self.angle_increment * np.arange(n)
        self.local_dirs = np.stack([np.cos(self.angles), np.sin(self.angles), np.zeros(n)], axis=1)
        self.ranges: list[float] = [math.inf] * n

    @property
    def angle_max(self) -> float:
        return float(self.angles[-1])

    def scan(self, base_pos: np.ndarray, base_rot: np.ndarray) -> list[float]:
        origin = base_pos + base_rot @ self.offset
        dirs = self.local_dirs @ (base_rot @ self.mount_rot).T
        cfg = self.cfg
        o = (float(origin[0]), float(origin[1]), float(origin[2]))
        prefix = self.robot_prefix
        ignore = self.ignore
        best = [math.inf]

        def on_hit(hit) -> bool:
            if not hit.rigid_body.startswith(prefix) and hit.rigid_body not in ignore and hit.distance < best[0]:
                best[0] = hit.distance
            return True  # hits are not sorted, so look at all of them

        ranges = self.ranges
        for i, d in enumerate(dirs):
            best[0] = math.inf
            self.query.raycast_all(o, (float(d[0]), float(d[1]), float(d[2])), cfg.max_range, on_hit)
            r = best[0]
            if r < math.inf:
                r += self.rng.gauss(0.0, cfg.noise_stddev)
                if r < cfg.min_range or r > cfg.max_range:
                    r = math.inf
            ranges[i] = float(r)
        return ranges


class Imu:
    """Angular velocity and specific force at the IMU mount, in the IMU frame.

    Acceleration is the finite difference of the mount point's velocity over
    one physics step, so it is noisy around contacts, like a real MEMS IMU.
    """

    GRAVITY = np.array([0.0, 0.0, 9.81])

    def __init__(self, cfg: SimpleNamespace, mount_xyz, mount_quat, com_local, seed: int | None = None):
        self.cfg = cfg
        self.offset = np.array(mount_xyz, dtype=float) - np.array(com_local, dtype=float)
        self.mount_rot = quat_to_matrix(*mount_quat)
        self.rng = random.Random(seed)
        self._last_v: np.ndarray | None = None
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = self.GRAVITY.copy()
        self.orientation = (1.0, 0.0, 0.0, 0.0)

    def update(self, base_rot: np.ndarray, com_lin_vel: np.ndarray, ang_vel: np.ndarray, dt: float) -> None:
        v = com_lin_vel + np.cross(ang_vel, base_rot @ self.offset)
        acc = np.zeros(3) if self._last_v is None or dt <= 0 else (v - self._last_v) / dt
        self._last_v = v
        to_imu = (base_rot @ self.mount_rot).T
        g, a = self.cfg.gyro_noise, self.cfg.accel_noise
        self.angular_velocity = to_imu @ ang_vel + np.array([self.rng.gauss(0, g) for _ in range(3)])
        self.linear_acceleration = to_imu @ (acc + self.GRAVITY) + np.array([self.rng.gauss(0, a) for _ in range(3)])
        self.orientation = matrix_to_quat(base_rot @ self.mount_rot)
