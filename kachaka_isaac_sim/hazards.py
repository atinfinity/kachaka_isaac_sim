"""Collisions, cliff (step) sensing and floors with drops. Plain Python, no
Isaac Sim imports.

- Collisions: the real Kachaka's published sensors include no bumper, so
  the simulator only counts the contacts of the body with other objects
  (`CollisionCounter`) and, if configured, stops like an emergency stop.
- Cliffs: the real robot has two step sensors at the front. When one sees
  no floor it pauses (error 21052 "Step detected", with 21051 "paused")
  until the power button is pressed (`Cliff`).
- Drops: the built-in worlds are flat; `floor_tiles` builds a floor with
  holes in it from boxes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import pairwise

# Real kachaka-api error codes (docs/api/ERROR_HANDLING.md in kachaka-api).
ERROR_PAUSED = 21051
ERROR_STEP_DETECTED = 21052
# Simulator codes (api_server.ERROR_CODES, next to command_server's).
ERROR_EMERGENCY_STOP = -14
ERROR_COLLISION = -18


Rect = tuple[float, float, float, float]  # x0, y0, x1, y1


def hole_rect(x: float, y: float, size_x: float, size_y: float) -> Rect:
    """A drop centred at (x, y)."""
    return (x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2)


def floor_tiles(holes: list[Rect], extent: float = 50.0) -> list[Rect]:
    """Rectangles that cover the square [-extent, extent]^2 except the holes.

    The hole edges cut the square into a grid; every cell outside all holes is
    a tile. Cells in a row are merged, so a single hole gives a few tiles."""
    clip = [(max(x0, -extent), max(y0, -extent), min(x1, extent), min(y1, extent)) for x0, y0, x1, y1 in holes]
    clip = [r for r in clip if r[0] < r[2] and r[1] < r[3]]
    xs = sorted({-extent, extent, *(v for r in clip for v in (r[0], r[2]))})
    ys = sorted({-extent, extent, *(v for r in clip for v in (r[1], r[3]))})

    def in_hole(cx: float, cy: float) -> bool:
        return any(x0 < cx < x1 and y0 < cy < y1 for x0, y0, x1, y1 in clip)

    tiles: list[Rect] = []
    for y0, y1 in pairwise(ys):
        start = None
        for x0, x1 in pairwise(xs):
            solid = not in_hole((x0 + x1) / 2, (y0 + y1) / 2)
            if solid and start is None:
                start = x0
            elif not solid and start is not None:
                tiles.append((start, y0, x0, y1))
                start = None
        if start is not None:
            tiles.append((start, y0, xs[-1], y1))
    return tiles


class Cliff:
    """Pause on a new drop under a step sensor; block forward motion while a
    sensor still sees one.

    The robot pauses when a sensor goes from floor to drop. After a resume
    the sensors may still look over the edge: forward motion stays blocked
    until they see floor again, so backing up and turning get it away
    without pausing again at once."""

    def __init__(self, sensors: int, enabled: bool = True):
        self.enabled = enabled
        self.over = [False] * sensors  # per sensor: no floor below
        self.paused = False

    def update(self, over: list[bool]) -> bool:
        """New sensor readings; True when this pauses the robot."""
        new_drop = any(o and not before for o, before in zip(over, self.over))
        self.over = list(over)
        if self.enabled and new_drop and not self.paused:
            self.paused = True
            return True
        return False

    @property
    def blocked(self) -> bool:
        """Forward motion has to stop."""
        return self.enabled and any(self.over)

    def resume(self) -> bool:
        """The power button; True if the robot was paused."""
        was, self.paused = self.paused, False
        return was

    def reset(self) -> None:
        self.over = [False] * len(self.over)
        self.paused = False


@dataclass
class CollisionCounter:
    """Counts collisions of the body with other objects.

    A contact counts when it starts after `debounce` seconds without contact
    with that object, so one push against a wall is one collision even when
    the contact flickers."""

    ignore: tuple[str, ...] = ()  # prim path prefixes that never count (the floor)
    debounce: float = 0.5  # s
    count: int = 0
    last_seen: dict[str, float] = field(default_factory=dict)

    def ignored(self, path: str) -> bool:
        return any(path == p or path.startswith(p + "/") for p in self.ignore)

    def contact(self, other: str, t: float, found: bool) -> bool:
        """A contact event with `other` at time `t`: `found` for a new
        contact, False while it persists or as it ends. True if it counts."""
        if self.ignored(other):
            return False
        last = self.last_seen.get(other)
        self.last_seen[other] = t
        if found and (last is None or t - last >= self.debounce):
            self.count += 1
            return True
        return False

    def reset(self) -> None:
        self.count = 0
        self.last_seen.clear()
