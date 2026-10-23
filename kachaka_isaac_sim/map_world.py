"""A world built from an occupancy map (`run_sim.py --world map:<path>`).

The map is a Nav2 map pair (`<id>.yaml` and its image: resolution, origin,
negate and thresholds as map_server reads them), a Kachaka PNG export or any
other image (dark occupied, light free; `--map-resolution`, origin at the
bottom-left corner). Occupied cells become wall boxes of `map_world.wall_height`
(config/robot.yaml), merged into as few boxes as the grid allows: runs of
occupied cells along a row, then runs of equal rows stacked. Free and unknown
cells are floor.

The world frame is the map frame, so the map's poses (its `charger_pose`, a
goal clicked in RViz on the same map) are world poses, and
`navigation.launch.py map:=<the same yaml>` with the robot spawned at the map
origin localizes at once.

Plain Python (numpy, Pillow, PyYAML): no Isaac Sim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from kachaka_isaac_sim.maps import OCCUPIED, MapError, MapFile

PREFIX = "map:"


@dataclass(frozen=True)
class WallBox:
    """A box over cells [col0, col1) x [row0, row1) of the grid, in the world
    frame: centre (x, y), size (sx, sy), rotated by `yaw` (the map origin's)."""

    x: float
    y: float
    sx: float
    sy: float
    yaw: float


def is_map_world(world: str | None) -> bool:
    return bool(world) and world.startswith(PREFIX)


def load_map(path: str | Path, resolution: float | None = None) -> MapFile:
    """A Nav2 map yaml, or an image at `resolution` m per pixel."""
    path = Path(path)
    try:
        if path.suffix in (".yaml", ".yml"):
            meta = yaml.safe_load(path.read_text()) or {}
            image = Path(meta.get("image", ""))
            m = MapFile.from_files(meta, (image if image.is_absolute() else path.parent / image).read_bytes())
            if resolution is not None and not math.isclose(resolution, m.resolution):
                raise MapError(f"{path} says resolution {m.resolution}: drop --map-resolution")
            return m
        if resolution is None:
            raise MapError(f"{path} is an image: give its resolution (--map-resolution, m per pixel)")
        return MapFile.from_image(path.read_bytes(), resolution, name=path.stem)
    except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
        raise MapError(f"cannot read map {path}: {e}") from e


def merge_cells(occupied: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Cover the True cells exactly with rectangles (row0, row1, col0, col1),
    half-open: each row's runs, and a run joined to the one below it when
    both span the same columns."""
    open_runs: dict[tuple[int, int], int] = {}  # (col0, col1) -> row0, of the runs still growing
    rects = []
    rows = occupied.shape[0]
    for r in range(rows + 1):
        runs = set()
        if r < rows:
            row = np.concatenate([[False], occupied[r], [False]]).astype(np.int8)
            d = np.diff(row)
            runs = set(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist()))
        for run in [k for k in open_runs if k not in runs]:
            rects.append((open_runs.pop(run), r, *run))
        for run in runs:
            open_runs.setdefault(run, r)
    return rects


def wall_boxes(m: MapFile) -> list[WallBox]:
    """The map's occupied cells as boxes in the world (= map) frame."""
    res = m.resolution
    ox, oy, oyaw = m.origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    out = []
    for r0, r1, c0, c1 in merge_cells(m.grid == OCCUPIED):
        # Grid rows run from the origin up (MapFile), columns along x.
        lx, ly = (c0 + c1) / 2 * res, (r0 + r1) / 2 * res
        out.append(WallBox(ox + c * lx - s * ly, oy + s * lx + c * ly, (c1 - c0) * res, (r1 - r0) * res, oyaw))
    return out


def occupied_at(m: MapFile, x: float, y: float, radius: float = 0.0) -> bool:
    """Any occupied cell within `radius` of the world point (x, y)."""
    ox, oy, oyaw = m.origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    lx, ly = c * (x - ox) + s * (y - oy), -s * (x - ox) + c * (y - oy)
    res = m.resolution
    h, w = m.grid.shape
    n = math.ceil(radius / res)
    col, row = math.floor(lx / res), math.floor(ly / res)
    r0, r1, c0, c1 = max(row - n, 0), min(row + n + 1, h), max(col - n, 0), min(col + n + 1, w)
    if r0 >= r1 or c0 >= c1:
        return False
    rr, cc = np.mgrid[r0:r1, c0:c1]
    near = np.hypot((cc + 0.5) * res - lx, (rr + 0.5) * res - ly) <= radius + res * 0.71
    return bool(np.any((m.grid[r0:r1, c0:c1] == OCCUPIED) & near))


def bounds(m: MapFile) -> tuple[float, float, float, float]:
    """(x_min, y_min, x_max, y_max) of the map in the world frame."""
    ox, oy, oyaw = m.origin
    h, w = m.grid.shape
    c, s = math.cos(oyaw), math.sin(oyaw)
    xs, ys = [], []
    for lx in (0.0, w * m.resolution):
        for ly in (0.0, h * m.resolution):
            xs.append(ox + c * lx - s * ly)
            ys.append(oy + s * lx + c * ly)
    return min(xs), min(ys), max(xs), max(ys)
