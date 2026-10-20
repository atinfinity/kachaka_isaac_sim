"""A 2D stand-in for the simulated world, for scripts/fake_sim.py: what the
LiDAR plane cuts out of the default room, and Kachaka Shelves that a
kinematic robot tows. Plain Python and numpy, no Isaac Sim imports.

Everything the LiDAR sees is a box (centre, yaw, size): the walls and boxes
of the room (scenes.ROOM_*), the people's colliders and the shelves' legs.
The charger's back panel (0.09 m) is lower than the LiDAR (0.105 m) and the
shelves' bottom plates higher, so neither appears, as in the simulator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .drive import Pose2D, wrap_angle
from .scenes import PERSON_COLLIDER, ROOM_BOXES, ROOM_SIZE, ROOM_WALL, SHELF_LEG_SIZE, SHELF_LEG_XY
from .shelves import dock_offset


@dataclass
class Box:
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float


def room_boxes(people: list[tuple[float, float, float]] = (), person_height: float = 1.7) -> list[Box]:
    """The default room's walls and boxes, and people at (x, y, yaw)."""
    (sx, sy), t = ROOM_SIZE, ROOM_WALL
    boxes = [
        Box(0.0, sy / 2 + t / 2, 0.0, sx + 2 * t, t),
        Box(0.0, -sy / 2 - t / 2, 0.0, sx + 2 * t, t),
        Box(sx / 2 + t / 2, 0.0, 0.0, t, sy),
        Box(-sx / 2 - t / 2, 0.0, 0.0, t, sy),
    ]
    boxes += [Box(c[0], c[1], 0.0, s[0], s[1]) for _, c, s, _ in ROOM_BOXES]
    k = person_height / 1.7
    boxes += [Box(x, y, yaw, PERSON_COLLIDER[0] * k, PERSON_COLLIDER[1] * k) for x, y, yaw in people]
    return boxes


def shelf_legs(pose: Pose2D) -> list[Box]:
    """The four legs of a shelf at `pose` (the pose of a robot docked under it)."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    lx, ly = SHELF_LEG_XY
    return [Box(pose.x + c * px - s * py, pose.y + s * px + c * py, pose.yaw, *SHELF_LEG_SIZE)
            for px in (-lx, lx) for py in (-ly, ly)]


def cast_rays(ox: float, oy: float, angles: np.ndarray, boxes: list[Box]) -> np.ndarray:
    """Distance from (ox, oy) along each world-frame angle to the nearest box
    face, inf for none. Boxes containing the origin are ignored."""
    angles = np.asarray(angles, dtype=float)
    best = np.full(angles.shape, np.inf)
    if not boxes:
        return best
    b = np.array([(bx.x, bx.y, bx.yaw, bx.size_x / 2, bx.size_y / 2) for bx in boxes])  # (m, 5)
    c, s = np.cos(b[:, 2]), np.sin(b[:, 2])
    # Origin and directions in each box frame.
    rx, ry = ox - b[:, 0], oy - b[:, 1]
    px, py = c * rx + s * ry, -s * rx + c * ry  # (m,)
    dx = np.cos(angles)[None, :] * c[:, None] + np.sin(angles)[None, :] * s[:, None]  # (m, n)
    dy = -np.cos(angles)[None, :] * s[:, None] + np.sin(angles)[None, :] * c[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_near = np.full(dx.shape, -np.inf)
        t_far = np.full(dx.shape, np.inf)
        for p, d, h in ((px, dx, b[:, 3]), (py, dy, b[:, 4])):
            p, h = p[:, None], h[:, None]
            t1, t2 = (-h - p) / d, (h - p) / d
            parallel = np.abs(d) < 1e-12
            outside = parallel & (np.abs(p) > h)
            lo = np.where(parallel, -np.inf, np.minimum(t1, t2))
            hi = np.where(parallel, np.inf, np.maximum(t1, t2))
            hi = np.where(outside, -np.inf, hi)
            t_near, t_far = np.maximum(t_near, lo), np.minimum(t_far, hi)
    hit = (t_near <= t_far) & (t_near > 0.0)
    dist = np.where(hit, t_near, np.inf)
    return dist.min(axis=0)


def any_inside(points: np.ndarray, boxes: list[Box]) -> bool:
    """Whether any of the (n, 2) world points lies inside a box."""
    if not boxes or len(points) == 0:
        return False
    b = np.array([(bx.x, bx.y, bx.yaw, bx.size_x / 2, bx.size_y / 2) for bx in boxes])
    c, s = np.cos(b[:, 2])[:, None], np.sin(b[:, 2])[:, None]
    rx, ry = points[None, :, 0] - b[:, 0, None], points[None, :, 1] - b[:, 1, None]
    px, py = c * rx + s * ry, -s * rx + c * ry
    return bool(((np.abs(px) < b[:, 3, None]) & (np.abs(py) < b[:, 4, None])).any())


def outline(x_min: float, x_max: float, half_width: float, step: float = 0.01) -> np.ndarray:
    """Points every `step` along a rectangle in the robot frame, (n, 2)."""
    xs = np.linspace(x_min, x_max, max(2, math.ceil((x_max - x_min) / step) + 1))
    ys = np.linspace(-half_width, half_width, max(2, math.ceil(2 * half_width / step) + 1))
    return np.concatenate([
        np.stack([xs, np.full_like(xs, -half_width)], 1), np.stack([xs, np.full_like(xs, half_width)], 1),
        np.stack([np.full_like(ys, x_min), ys], 1), np.stack([np.full_like(ys, x_max), ys], 1)])


def transform(points: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Robot-frame (n, 2) points to the world frame."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.stack([pose.x + c * points[:, 0] - s * points[:, 1], pose.y + s * points[:, 0] + c * points[:, 1]], 1)


class FlatShelves:
    """Shelf poses in the world frame, with kinematic towing: a docked shelf
    keeps its pose relative to the robot. The same interface as
    shelves.Shelves for the simulator code that uses it."""

    def __init__(self, specs, tol_xy: float, tol_yaw: float):
        self.specs = specs  # shelves.ShelfSpec-like: id, name, path, home
        self.tol_xy, self.tol_yaw = tol_xy, tol_yaw
        self.poses = [Pose2D(s.home.x, s.home.y, s.home.yaw) for s in specs]
        self.docked: int | None = None
        self.offset: Pose2D | None = None  # towed shelf relative to the robot

    def follow(self, robot: Pose2D) -> None:
        if self.docked is None:
            return
        o = self.offset
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        self.poses[self.docked] = Pose2D(robot.x + c * o.x - s * o.y, robot.y + s * o.x + c * o.y,
                                         wrap_angle(robot.yaw + o.yaw))

    def legs(self) -> list[Box]:
        """Legs of the shelves the robot does not tow (the LiDAR looks
        through the towed one, see SimRobot.set_shelf_docked)."""
        return [leg for i, p in enumerate(self.poses) if i != self.docked for leg in shelf_legs(p)]

    def dock(self, robot: Pose2D) -> tuple[bool, str]:
        if self.docked is not None:
            return True, f"already docked with {self.specs[self.docked].id}"
        for i, pose in enumerate(self.poses):
            rel = dock_offset(robot, pose, self.tol_xy, self.tol_yaw)
            if rel is not None:
                self.docked, self.offset = i, rel
                return True, f"docked with {self.specs[i].id}"
        return False, "no shelf above the robot"

    def undock(self) -> tuple[bool, str]:
        if self.docked is None:
            return True, "no shelf docked"
        i, self.docked, self.offset = self.docked, None, None
        return True, f"undocked {self.specs[i].id}"

    def reset(self, shelf_id: str = "") -> tuple[bool, str]:
        idx = [i for i, s in enumerate(self.specs) if not shelf_id or s.id == shelf_id]
        if not idx:
            return False, f"unknown shelf {shelf_id!r}"
        if self.docked in idx:
            return False, f"the robot tows {self.specs[self.docked].id}"
        for i in idx:
            h = self.specs[i].home
            self.poses[i] = Pose2D(h.x, h.y, h.yaw)
        return True, ""
