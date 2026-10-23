"""Several robots in one world (run_sim.py --robot X Y YAW, repeated).

Robot 0 keeps the single-robot names, so everything written for one robot
works on it unchanged. Robot i >= 1 gets its own prim, ROS namespace, TF frame
prefix, kachaka-api port and noise seed:

    robot 0: /World/kachaka    /kachaka/...    odom, base_footprint    port P
    robot 1: /World/kachaka_1  /kachaka_1/...  kachaka_1/odom, ...     port P + 1

All robots share one map frame: the map origin is robot 0's spawn pose.
"""

import copy
from dataclasses import dataclass
from types import SimpleNamespace

from kachaka_isaac_sim.drive import Pose2D

ROBOT_PATH = "/World/kachaka"  # world.ROBOT_PATH, without importing pxr
SEED_STRIDE = 1000  # between robots' seeds; cameras use seed + n on top


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
