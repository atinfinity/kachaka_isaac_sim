"""Several robots in one world (run_sim.py --robot X Y YAW, repeated).

Robot 0 keeps the single-robot names, so everything written for one robot
works on it unchanged. Robot i >= 1 gets its own prim, ROS namespace, TF frame
prefix, kachaka-api port and noise seed:

    robot 0: /World/kachaka    /kachaka/...    odom, base_footprint    port P
    robot 1: /World/kachaka_1  /kachaka_1/...  kachaka_1/odom, ...     port P + 1

All robots share one map frame: the map origin is robot 0's spawn pose.

Robots see each other in their LiDAR scans through `body_ray_hits`: the
official collision box ends at 0.095 m, below the LiDAR (0.105 m), so the
PhysX rays pass over another robot. Colliders at that height would also
touch a shelf's bottom plate (0.107 m), so the parts of the body there are
intersected in 2D instead.
"""

import copy
import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from kachaka_isaac_sim.drive import Pose2D

ROBOT_PATH = "/World/kachaka"  # world.ROBOT_PATH, without importing pxr
SEED_STRIDE = 1000  # between robots' seeds; cameras use seed + n on top
# Parts of the body at the LiDAR's height, from a slice of the official
# body.stl at z = 0.105 m, in base_footprint: the housing around the laser
# (x 0.116 to 0.208, y -0.043 to 0.048) and a bump at the back (x -0.024 to
# 0.029, y -0.023 to 0.027). (centre x, centre y, half length, half width)
LIDAR_HEIGHT_BODY = ((0.162, 0.0025, 0.046, 0.0455), (0.0025, 0.002, 0.0265, 0.025))


@dataclass(frozen=True)
class RobotSpec:
    index: int
    name: str
    pose: Pose2D  # spawn, world frame
    namespace: str  # ROS namespace, no slashes
    frame_prefix: str  # TF frame prefix ("" or "kachaka_1/")
    prim_path: str
    api_port: int  # 0: no kachaka-api
    seed: int | None

    @property
    def node_name(self) -> str:
        return "kachaka_isaac_sim" if self.index == 0 else f"kachaka_isaac_sim_{self.index}"

    def config(self, cfg: SimpleNamespace) -> SimpleNamespace:
        """`cfg` with this robot's namespace and frame prefix; robot 0 gets
        `cfg` itself."""
        if self.index == 0:
            return cfg
        out = copy.deepcopy(cfg)
        out.ros.namespace = self.namespace
        out.ros.frame_prefix = self.frame_prefix
        return out


def fleet_specs(cfg: SimpleNamespace, poses: list[Pose2D], api_port: int, seed: int | None) -> list[RobotSpec]:
    """One spec per spawn pose; the first is robot 0."""
    if not poses:
        raise ValueError("no robots")
    ns0 = (cfg.ros.namespace or "").strip("/")
    specs = []
    for i, pose in enumerate(poses):
        if i == 0:
            specs.append(RobotSpec(0, ns0 or "kachaka", pose, ns0, cfg.ros.frame_prefix or "", ROBOT_PATH,
                                   api_port, seed))
            continue
        name = f"kachaka_{i}"
        if name == ns0:
            raise ValueError(f"robot {i}'s namespace {name} is robot 0's")
        specs.append(RobotSpec(i, name, pose, name, f"{name}/", f"{ROBOT_PATH}_{i}",
                               api_port + i if api_port else 0,
                               None if seed is None else seed + SEED_STRIDE * i))
    return specs


def robot_poses(robots: list[list[float]] | None, single: Pose2D) -> list[Pose2D]:
    """Spawn poses from --robot (repeated); without it, the one robot at
    `single` (--x/--y/--yaw or the world's spawn)."""
    if not robots:
        return [single]
    return [Pose2D(float(x), float(y), float(yaw)) for x, y, yaw in robots]


def body_ray_hits(origin, dirs: np.ndarray, poses: list[Pose2D], max_range: float = math.inf) -> np.ndarray:
    """Distance along each ray (2D: `origin` (x, y), `dirs` (n, 2) unit
    vectors, world frame) to the nearest LIDAR_HEIGHT_BODY part of the robots
    at `poses`; inf where a ray misses or the hit is beyond `max_range`.
    A ray starting inside a part does not see it."""
    dirs = np.asarray(dirs, float)[:, :2]
    best = np.full(len(dirs), math.inf)
    ox, oy = float(origin[0]), float(origin[1])
    for p in poses:
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        # Rays in the robot's frame.
        rx, ry = ox - p.x, oy - p.y
        lox, loy = c * rx + s * ry, -s * rx + c * ry
        ldx = c * dirs[:, 0] + s * dirs[:, 1]
        ldy = -s * dirs[:, 0] + c * dirs[:, 1]
        ldx = np.where(np.abs(ldx) < 1e-12, 1e-12, ldx)
        ldy = np.where(np.abs(ldy) < 1e-12, 1e-12, ldy)
        for cx, cy, hx, hy in LIDAR_HEIGHT_BODY:
            if abs(lox - cx) <= hx and abs(loy - cy) <= hy:
                continue
            tx1, tx2 = (cx - hx - lox) / ldx, (cx + hx - lox) / ldx
            ty1, ty2 = (cy - hy - loy) / ldy, (cy + hy - loy) / ldy
            near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = (near <= far) & (near >= 0.0) & (near <= max_range)
            best = np.where(hit & (near < best), near, best)
    return best
