"""Kachaka Shelves in the running simulation: poses, docking and undocking.

`Shelves` needs Isaac Sim (create it after SimulationApp has started);
`ShelfSpec` and `dock_offset` are plain Python (scripts/fake_sim.py uses them).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace
from typing import TYPE_CHECKING

from .drive import Pose2D, quat_from_yaw, relative_pose, wrap_angle, yaw_from_quat

if TYPE_CHECKING:
    from pxr import Usd


@dataclass
class ShelfSpec:
    id: str
    name: str
    path: str
    home: Pose2D  # world frame


def dock_offset(robot: Pose2D, shelf: Pose2D, tol_xy: float, tol_yaw: float) -> Pose2D | None:
    """The shelf pose relative to the robot if the robot is under it, close
    enough to its centre and lined up with its x axis (either way), else None."""
    rel = relative_pose(robot, shelf)
    yaw_err = min(abs(wrap_angle(rel.yaw)), abs(wrap_angle(rel.yaw - math.pi)))
    if math.hypot(rel.x, rel.y) <= tol_xy and yaw_err <= tol_yaw:
        return rel
    return None


class Shelves:
    def __init__(self, stage: Usd.Stage, specs: list[ShelfSpec], cfg: SimpleNamespace):
        from isaacsim.core.experimental.prims import RigidPrim

        from .world import ShelfJoint

        self.specs = specs
        self.cfg = cfg
        self.view = RigidPrim([s.path for s in specs])
        self.joints = [ShelfJoint(stage, s.path) for s in specs]
        self.poses = [s.home for s in specs]
        self.docked: int | None = None  # index of the towed shelf

    def update(self) -> list[Pose2D]:
        pos, quat = (a.numpy() for a in self.view.get_world_poses())
        self.poses = [Pose2D(float(p[0]), float(p[1]), yaw_from_quat(*map(float, q))) for p, q in zip(pos, quat)]
        return self.poses

    @property
    def docked_path(self) -> str | None:
        return None if self.docked is None else self.specs[self.docked].path

    def dock(self, robot: Pose2D) -> tuple[bool, str]:
        if self.docked is not None:
            return True, f"already docked with {self.specs[self.docked].id}"
        self.update()
        for i, pose in enumerate(self.poses):
            rel = dock_offset(robot, pose, self.cfg.dock_tolerance_xy, self.cfg.dock_tolerance_yaw)
            if rel is None:
                continue
            # Both bodies stand on the floor, so the planar offset is enough.
            self.joints[i].attach((rel.x, rel.y, 0.0), quat_from_yaw(rel.yaw))
            self.docked = i
            return True, f"docked with {self.specs[i].id}"
        return False, "no shelf above the robot"

    def reset(self, shelf_id: str = "") -> tuple[bool, str]:
        """Put a shelf (all of them if `shelf_id` is empty) back at its home
        pose, at rest. Not while the robot tows it."""
        idx = [i for i, s in enumerate(self.specs) if not shelf_id or s.id == shelf_id]
        if not idx:
            return False, f"unknown shelf {shelf_id!r}"
        if self.docked in idx:
            return False, f"the robot tows {self.specs[self.docked].id}"
        pos = self.view.get_world_poses()[0].numpy()
        positions = [[s.home.x, s.home.y, float(pos[i][2])] for i, s in ((i, self.specs[i]) for i in idx)]
        orientations = [list(quat_from_yaw(self.specs[i].home.yaw)) for i in idx]
        self.view.set_world_poses(positions, orientations, indices=idx)
        self.view.set_velocities([[0.0] * 3] * len(idx), [[0.0] * 3] * len(idx), indices=idx)
        self.update()
        return True, ""

    def shift(self, idx: list[int], dx: float, dy: float) -> None:
        """Carry shelves by (dx, dy) without turning or stopping them (an
        elevator ride)."""
        if not idx:
            return
        pos, quat = (a.numpy() for a in self.view.get_world_poses())
        positions = [[float(pos[i][0]) + dx, float(pos[i][1]) + dy, float(pos[i][2])] for i in idx]
        self.view.set_world_poses(positions, [[float(v) for v in quat[i]] for i in idx], indices=idx)
        self.update()

    def undock(self) -> tuple[bool, str]:
        if self.docked is None:
            return True, "no shelf docked"
        i, self.docked = self.docked, None
        self.joints[i].detach()
        return True, f"undocked {self.specs[i].id}"
