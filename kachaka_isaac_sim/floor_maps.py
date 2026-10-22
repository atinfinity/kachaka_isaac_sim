"""Saved maps of a layout's floors, drawn from what the layout puts there
(scripts/make_maps.py), for navigating on each floor with map_server and
AMCL: the room's walls with the elevator's doorway, the room's boxes, the
car, and the layout's walls and standing boxes. Doors are left open; the
charger, shelves and people are left out, as they are below the LiDAR or
may move. Plain Python and numpy, no Isaac Sim imports.

Floor k's map frame is the spawn pose moved to that floor, as SLAM's map
frame is the spawn pose on the first floor: a pose in the car reads the
same on every floor's map.
"""

from __future__ import annotations

import math

import numpy as np

from .drive import Pose2D, relative_pose
from .elevator import FLOOR_SPACING, ElevatorSpec, car_walls, floor_of, shifted
from .flat_world import Box
from .maps import FREE, OCCUPIED, UNKNOWN, MapFile
from .scenes import ROOM_BOXES, ROOM_SIZE, ROOM_WALL, room_walls

MARGIN = 0.3  # m of unknown around the floor


def _segment_box(x0, y0, x1, y1, t=ROOM_WALL) -> Box:
    return Box((x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0), math.hypot(x1 - x0, y1 - y0), t)


def floor_boxes(layout, k: int, spec: ElevatorSpec) -> list[Box]:
    """What the LiDAR sees on floor k (world frame), without doors and movers."""
    dx = k * FLOOR_SPACING
    p = spec.pose
    boxes = [Box(c[0] + dx, c[1], 0.0, *size) for _, c, size in room_walls([(p.x, p.y, p.yaw, spec.door_width)])]
    boxes += [Box(c[0] + dx, c[1], 0.0, s[0], s[1]) for _, c, s, _ in ROOM_BOXES]
    boxes += [_segment_box(*a, *b) for a, b in car_walls(spec, k, ROOM_WALL)]
    boxes += [_segment_box(*w) for w in layout.walls if floor_of((w[0] + w[2]) / 2) == k]
    boxes += [Box(b.pose.x, b.pose.y, b.pose.yaw, b.size[0], b.size[1]) for b in layout.boxes
              if b.motion is None and floor_of(b.pose.x) == k]
    return boxes


def _inside(points: np.ndarray, boxes: list[Box]) -> np.ndarray:
    """Which of the (n, 2) points lie inside any box."""
    out = np.zeros(len(points), bool)
    for b in boxes:
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        rx, ry = points[:, 0] - b.x, points[:, 1] - b.y
        out |= (np.abs(c * rx + s * ry) <= b.size_x / 2) & (np.abs(-s * rx + c * ry) <= b.size_y / 2)
    return out


def floor_map(layout, k: int, spec: ElevatorSpec, spawn: Pose2D, resolution: float = 0.05,
              charger: Pose2D | None = None) -> MapFile:
    """Floor k's map. `charger` (world, on the first floor) is saved as the
    first floor's charger pose."""
    frame = shifted(spawn, k)
    (sx, sy), t = ROOM_SIZE, ROOM_WALL
    dx = k * FLOOR_SPACING
    room = Box(dx, 0.0, 0.0, sx, sy)
    car = shifted(spec.pose, k)
    w, d = spec.size
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    cy = t / 2 + d / 2  # the car's middle, in its frame
    car_box = Box(car.x - s * cy, car.y + c * cy, car.yaw, w, d)
    # Bounds in the map frame: the room and the car with their walls.
    corners = []
    for b, pad in ((room, t), (car_box, t)):
        cb, sb = math.cos(b.yaw), math.sin(b.yaw)
        for ux in (-1, 1):
            for uy in (-1, 1):
                hx, hy = ux * (b.size_x / 2 + pad), uy * (b.size_y / 2 + pad)
                q = relative_pose(frame, Pose2D(b.x + cb * hx - sb * hy, b.y + sb * hx + cb * hy, 0.0))
                corners.append((q.x, q.y))
    xs, ys = [q[0] for q in corners], [q[1] for q in corners]
    # Cell centres on multiples of the resolution, so that wall faces (and
    # the grown boxes' faces) fall between them rather than on them.
    x0, y0 = ((math.floor((v - MARGIN) / resolution) - 0.5) * resolution for v in (min(xs), min(ys)))
    width = math.ceil((max(xs) + MARGIN - x0) / resolution)
    height = math.ceil((max(ys) + MARGIN - y0) / resolution)
    # Cell centres, map frame then world.
    gx, gy = np.meshgrid(x0 + (np.arange(width) + 0.5) * resolution, y0 + (np.arange(height) + 0.5) * resolution)
    cf, sf = math.cos(frame.yaw), math.sin(frame.yaw)
    pts = np.stack([frame.x + cf * gx.ravel() - sf * gy.ravel(), frame.y + sf * gx.ravel() + cf * gy.ravel()], 1)
    grid = np.full(len(pts), UNKNOWN, np.int16)
    grid[_inside(pts, [room, car_box, Box(car.x - s * t / 2, car.y + c * t / 2, car.yaw, spec.door_width, t * 2)])] = FREE
    # Walls are thinner than some cells: grow the boxes by half a cell.
    grown = [Box(b.x, b.y, b.yaw, b.size_x + resolution, b.size_y + resolution) for b in floor_boxes(layout, k, spec)]
    grid[_inside(pts, grown)] = OCCUPIED
    dock = None
    if charger is not None and k == 0:
        q = relative_pose(frame, charger)
        dock = (q.x, q.y, q.yaw)
    return MapFile(grid.reshape(height, width), resolution, (x0, y0, 0.0), name=spec.floors[k], charger_pose=dock)
