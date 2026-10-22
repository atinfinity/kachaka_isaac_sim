"""World layout files (scripts/run_sim.py --layout): where the robot starts and
where the charger, shelves, locations, people, boxes, walls, doors and drops
are, on top of a world (the built-in room or an Isaac Sim scene).

    world: room                 # optional; --world overrides it
    seed: 7                     # optional; for the ranges below
    clearance: 0.1              # optional; m between randomised objects
    spawn: [0.0, 0.0, 0.0]
    charger: [-2.27, 0.0, 0.0]  # null: no charger
    shelves:
      - {id: S01, name: Shelf 1, pose: [1.3, -1.2, 0.0]}
      - pose: [[0.5, 1.5], [0.8, 1.4], 0.0]   # x and y drawn from a range
    locations:
      - {id: L01, name: Kitchen, pose: [0.5, 1.0, 1.5708]}
    people:
      - [2.0, 1.5, -2.5]
      - {pose: [-1.5, 1.5, 0.0], route: [[1.0, 1.5], [1.0, -1.5]], speed: 0.6, pause: 1.0}
    boxes:
      - {name: crate, pose: [0.0, -1.5, 0.0], size: [0.4, 0.3, 0.4], color: [0.5, 0.4, 0.3]}
      - {name: cart, pose: [0.0, 1.0, 0.0], size: [0.5, 0.4, 0.8], wander: [[-1, 1], [0.5, 1.5]], appear: [20, 10]}
    walls: [[0.6, -2.0, 0.6, -1.0]]
    doors:
      - {pose: [0.6, -0.6, 1.5708], kind: sliding, width: 0.8, open: true, cycle: [10, 5]}
    drops: [[1.0, 0.0, 0.4, 0.8]]   # x, y, size x, size y

Poses are [x, y, yaw] in the world frame (m, rad). Any number in a pose may
be a range [lo, hi] instead: it is drawn uniformly with the layout's seed, and
a randomised object is drawn again until it keeps `clearance` from the other
objects (and, in the room, from its walls and boxes). A key that is left out
keeps the world's default (the room has a charger, a shelf, a location and two
people); a key that is given replaces it, so `shelves: []` means no shelves.

People and boxes stand still unless they are given
- route: [[x, y], ...]: walk to each waypoint in turn, then back to the
  start pose, and again;
- wander: [[x lo, x hi], [y lo, y hi]]: walk to random points in that area
  (drawn with the layout's seed). Keep the area free of obstacles: movers
  walk straight and through anything that is not the robot;
and optionally speed (m/s; config 'movers'), pause (s at each waypoint) and
appear: [there s, away s] (gone part of the time, from a random phase).
A mover waits while the robot is in its way.

Plain Python: the simulator reads it before Isaac Sim starts.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .drive import Pose2D
from .scenes import PERSON_COLLIDER, ROOM_BOXES, ROOM_SIZE, ROOM_WALL, SHELF_SIZE

KEYS = ("world", "seed", "clearance", "spawn", "charger", "shelves", "locations", "people", "boxes", "walls",
        "doors", "drops")
DOOR_KINDS = ("sliding", "hinged")
# Footprint radii [m] for keeping randomised objects apart: half the diagonal
# of each object's outline.
ROBOT_RADIUS = 0.23  # 387 x 240 mm body
SHELF_RADIUS = math.hypot(*SHELF_SIZE) / 2
CHARGER_RADIUS = 0.2  # panel and floor plate (est.)
PERSON_RADIUS = math.hypot(*PERSON_COLLIDER) / 2
LOCATION_RADIUS = ROBOT_RADIUS  # the robot stands there
BOX_RADIUS_MIN = 0.1  # a moving box keeps at least this far out of the robot's way
TRIES = 200


class LayoutError(ValueError):
    pass


@dataclass
class Motion:
    """How a person or box moves (see movers.MoverSpec). speed None: the config's."""

    route: list[tuple[float, float]] = field(default_factory=list)
    wander: tuple[tuple[float, float], tuple[float, float]] | None = None
    speed: float | None = None
    pause: float = 0.0
    appear: tuple[float, float] | None = None


@dataclass
class Box:
    name: str
    pose: Pose2D
    size: tuple[float, float, float]
    color: tuple[float, float, float] = (0.6, 0.55, 0.5)
    motion: Motion | None = None


@dataclass
class Door:
    pose: Pose2D
    kind: str | None = None  # None: the command line's or the config's
    width: float | None = None
    open: bool = True
    cycle: tuple[float, float] | None = None
    auto: dict | None = None  # automatic: the config's door.auto with these keys replaced


@dataclass
class Layout:
    """A resolved layout: every pose is fixed. None means "the world's default"
    (for the keys that have one)."""

    path: str = ""
    world: str | None = None
    seed: int | None = None
    spawn: Pose2D | None = None
    charger: Pose2D | None = None
    no_charger: bool = False
    shelves: list[tuple[str, str, Pose2D]] | None = None  # id, name, pose
    locations: list[tuple[str, str, Pose2D]] | None = None
    people: list[Pose2D] | None = None
    people_motion: list[Motion | None] = field(default_factory=list)  # one per person
    boxes: list[Box] = field(default_factory=list)
    walls: list[tuple[float, float, float, float]] = field(default_factory=list)
    doors: list[Door] = field(default_factory=list)
    drops: list[tuple[float, float, float, float]] = field(default_factory=list)
    randomised: bool = False  # some pose was drawn from a range


RADII = {"spawn": ROBOT_RADIUS, "charger": CHARGER_RADIUS, "shelves": SHELF_RADIUS, "locations": LOCATION_RADIUS,
         "people": PERSON_RADIUS}


def read_layout(path: str | Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def parse_layout(data: dict, seed: int | None = None, world: str | None = None, path: str = "",
                 defaults: dict | None = None) -> Layout:
    """Resolve a layout. `seed` overrides the file's; without either, a
    random seed is picked (Layout.seed, to reproduce the layout). `world` is
    the world it is used with (the room's walls and boxes are kept clear of).
    `defaults` maps spawn, charger, shelves, locations and people to the
    world's default poses (lists of (x, y, yaw)); the ones the layout keeps
    are kept clear of too."""
    if not isinstance(data, dict):
        raise LayoutError(f"{path}: a layout is a mapping of {', '.join(KEYS)}")
    unknown = set(data) - set(KEYS)
    if unknown:
        raise LayoutError(f"{path}: unknown key(s) {', '.join(sorted(unknown))}; known: {', '.join(KEYS)}")
    if seed is None:
        seed = data.get("seed")
    if seed is None:
        seed = random.SystemRandom().randrange(1 << 31)
    out = Layout(path=path, world=data.get("world"), seed=int(seed))
    world = world or out.world or "room"
    placer = _Placer(random.Random(out.seed), float(data.get("clearance", 0.1)), path)
    if world == "room":
        placer.add_room()
    for key, poses in (defaults or {}).items():
        if key not in data:
            for p in poses:
                placer.circles.append((float(p[0]), float(p[1]), RADII[key]))

    # Fixed objects first, so randomised ones keep clear of all of them.
    items = []  # (key, index, entry, radius)
    if "spawn" in data:
        items.append(("spawn", 0, {"pose": data["spawn"]}, ROBOT_RADIUS))
    if "charger" in data and data["charger"] is not None:
        items.append(("charger", 0, {"pose": data["charger"]}, CHARGER_RADIUS))
    for key, radius in (("shelves", SHELF_RADIUS), ("locations", LOCATION_RADIUS), ("people", PERSON_RADIUS)):
        for i, e in enumerate(_list(data, key, path)):
            items.append((key, i, e if isinstance(e, dict) else {"pose": e}, radius))
    for i, e in enumerate(_list(data, "boxes", path)):
        if not isinstance(e, dict) or "size" not in e:
            raise LayoutError(f"{path}: boxes[{i}] needs pose and size")
        items.append(("boxes", i, e, math.hypot(*_numbers(e["size"], 3, f"boxes[{i}].size", path)[:2]) / 2))
    for i, w in enumerate(_list(data, "walls", path)):
        x0, y0, x1, y1 = _numbers(w, 4, f"walls[{i}]", path)
        out.walls.append((x0, y0, x1, y1))
        placer.add_segment(x0, y0, x1, y1, ROOM_WALL / 2)
    for i, d in enumerate(_list(data, "drops", path)):
        x, y, sx, sy = _numbers(d, 4, f"drops[{i}]", path)
        out.drops.append((x, y, sx, sy))
        placer.add_rect(x, y, sx / 2, sy / 2)

    fixed = [it for it in items if not _is_random(it[2].get("pose"))]
    random_ = [it for it in items if _is_random(it[2].get("pose"))]
    out.randomised = bool(random_)
    poses = {}
    for key, i, e, radius in fixed + random_:
        name = f"{key}[{i}]" if key not in ("spawn", "charger") else key
        if "pose" not in e:
            raise LayoutError(f"{path}: {name} needs a pose")
        poses[(key, i)] = placer.place(e["pose"], radius, name)

    if "spawn" in data:
        out.spawn = poses[("spawn", 0)]
    if "charger" in data:
        out.charger = poses.get(("charger", 0))
        out.no_charger = out.charger is None
    if "shelves" in data:
        out.shelves = [(str(e.get("id", f"S{i + 1:02d}")) if isinstance(e, dict) else f"S{i + 1:02d}",
                        str(e.get("name", f"Shelf {i + 1}")) if isinstance(e, dict) else f"Shelf {i + 1}",
                        poses[("shelves", i)]) for i, e in enumerate(_list(data, "shelves", path))]
        _unique([s[0] for s in out.shelves], "shelf", path)
    if "locations" in data:
        out.locations = [(str(e.get("id", f"L{i + 1:02d}")) if isinstance(e, dict) else f"L{i + 1:02d}",
                          str(e.get("name", f"Location {i + 1}")) if isinstance(e, dict) else f"Location {i + 1}",
                          poses[("locations", i)]) for i, e in enumerate(_list(data, "locations", path))]
        _unique([loc[0] for loc in out.locations], "location", path)
    if "people" in data:
        people = _list(data, "people", path)
        out.people = [poses[("people", i)] for i in range(len(people))]
        out.people_motion = [_motion(e, f"people[{i}]", path, PERSON_KEYS) for i, e in enumerate(people)]
    for i, e in enumerate(_list(data, "boxes", path)):
        size = _numbers(e["size"], 3, f"boxes[{i}].size", path)
        color = _numbers(e.get("color", Box.color), 3, f"boxes[{i}].color", path)
        out.boxes.append(Box(str(e.get("name", f"box{i + 1:02d}")), poses[("boxes", i)], size, color,
                             _motion(e, f"boxes[{i}]", path, BOX_KEYS)))
    for i, e in enumerate(_list(data, "doors", path)):
        if not isinstance(e, dict) or "pose" not in e:
            raise LayoutError(f"{path}: doors[{i}] needs a pose")
        unknown = set(e) - {"pose", "kind", "width", "open", "cycle", "auto"}
        if unknown:
            raise LayoutError(f"{path}: doors[{i}]: unknown key(s) {', '.join(sorted(unknown))}")
        kind = e.get("kind")
        if kind is not None and kind not in DOOR_KINDS:
            raise LayoutError(f"{path}: doors[{i}].kind must be one of {', '.join(DOOR_KINDS)}")
        pose = Pose2D(*_numbers(e["pose"], 3, f"doors[{i}].pose", path))
        cycle = tuple(_numbers(e["cycle"], 2, f"doors[{i}].cycle", path)) if e.get("cycle") else None
        width = float(e["width"]) if e.get("width") is not None else None
        auto = _door_auto(e.get("auto"), f"doors[{i}]", path)
        if auto is not None and (kind == "hinged" or cycle):
            raise LayoutError(f"{path}: doors[{i}]: an automatic door is sliding and has no cycle")
        out.doors.append(Door(pose, kind, width, bool(e.get("open", True)), cycle, auto))
    return out


MOTION_KEYS = ("route", "wander", "speed", "pause", "appear")
PERSON_KEYS = ("pose",) + MOTION_KEYS
BOX_KEYS = ("name", "pose", "size", "color") + MOTION_KEYS


def _motion(e, name: str, path: str, known: tuple[str, ...]) -> Motion | None:
    """The motion keys of a person or box entry, or None if it stands still."""
    if not isinstance(e, dict):
        return None
    unknown = set(e) - set(known)
    if unknown:
        raise LayoutError(f"{path}: {name}: unknown key(s) {', '.join(sorted(unknown))}; known: {', '.join(known)}")
    if not any(k in e for k in MOTION_KEYS):
        return None
    m = Motion()
    if e.get("route") is not None:
        route = e["route"]
        if not isinstance(route, list) or not route:
            raise LayoutError(f"{path}: {name}.route must be a list of [x, y] waypoints")
        m.route = [tuple(_numbers(p, 2, f"{name}.route[{k}]", path)) for k, p in enumerate(route)]
    if e.get("wander") is not None:
        w = e["wander"]
        if not isinstance(w, list) or len(w) != 2:
            raise LayoutError(f"{path}: {name}.wander must be [[x lo, x hi], [y lo, y hi]]")
        rx, ry = (_numbers(r, 2, f"{name}.wander", path) for r in w)
        if rx[0] > rx[1] or ry[0] > ry[1]:
            raise LayoutError(f"{path}: {name}.wander: a range is [lo, hi] with lo <= hi")
        m.wander = (rx, ry)
    if m.route and m.wander:
        raise LayoutError(f"{path}: {name}: give a route or a wander area, not both")
    if e.get("speed") is not None:
        m.speed = float(e["speed"])
        if m.speed <= 0:
            raise LayoutError(f"{path}: {name}.speed must be positive")
    m.pause = float(e.get("pause") or 0.0)
    if e.get("appear") is not None:
        a = _numbers(e["appear"], 2, f"{name}.appear", path)
        if a[0] <= 0 or a[1] < 0:
            raise LayoutError(f"{path}: {name}.appear is [there s > 0, away s >= 0]")
        m.appear = a
    return m


def _list(data: dict, key: str, path: str) -> list:
    value = data.get(key)
    if value is None:
        return []
    if not isinstance(value, list):
        raise LayoutError(f"{path}: {key} must be a list")
    return value


def _numbers(value, n: int, name: str, path: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise LayoutError(f"{path}: {name} must be a list of {n} numbers")
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError):
        raise LayoutError(f"{path}: {name} must be a list of {n} numbers") from None


def _is_random(pose) -> bool:
    return isinstance(pose, list) and any(isinstance(v, list) for v in pose)


def _unique(ids: list[str], what: str, path: str) -> None:
    dup = sorted({i for i in ids if ids.count(i) > 1})
    if dup:
        raise LayoutError(f"{path}: duplicate {what} id(s) {', '.join(dup)}")


class _Placer:
    """Draws poses and keeps randomised objects clear of what is placed."""

    def __init__(self, rng: random.Random, clearance: float, path: str):
        self.rng = rng
        self.clearance = clearance
        self.path = path
        self.circles: list[tuple[float, float, float]] = []  # x, y, radius
        self.rects: list[tuple[float, float, float, float]] = []  # centre x, y, half x, half y (axis-aligned)
        self.segments: list[tuple[float, float, float, float, float]] = []  # x0, y0, x1, y1, half thickness

    def add_room(self) -> None:
        (sx, sy), t = ROOM_SIZE, ROOM_WALL
        self.add_segment(-sx / 2, sy / 2 + t / 2, sx / 2, sy / 2 + t / 2, t / 2)
        self.add_segment(-sx / 2, -sy / 2 - t / 2, sx / 2, -sy / 2 - t / 2, t / 2)
        self.add_segment(sx / 2 + t / 2, -sy / 2, sx / 2 + t / 2, sy / 2, t / 2)
        self.add_segment(-sx / 2 - t / 2, -sy / 2, -sx / 2 - t / 2, sy / 2, t / 2)
        for _, (cx, cy, _), (bx, by, _), _ in ROOM_BOXES:
            self.add_rect(cx, cy, bx / 2, by / 2)

    def add_rect(self, x, y, hx, hy) -> None:
        self.rects.append((x, y, hx, hy))

    def add_segment(self, x0, y0, x1, y1, half) -> None:
        self.segments.append((x0, y0, x1, y1, half))

    def place(self, pose, radius: float, name: str) -> Pose2D:
        if not isinstance(pose, list) or len(pose) != 3:
            raise LayoutError(f"{self.path}: {name}.pose must be [x, y, yaw]; each may be a range [lo, hi]")
        ranges = []
        for v in pose:
            if isinstance(v, list):
                if len(v) != 2 or float(v[0]) > float(v[1]):
                    raise LayoutError(f"{self.path}: {name}.pose: a range is [lo, hi] with lo <= hi")
                ranges.append((float(v[0]), float(v[1])))
            else:
                ranges.append((float(v), float(v)))
        if not _is_random(pose):
            p = Pose2D(*(lo for lo, _ in ranges))
            self.circles.append((p.x, p.y, radius))
            return p
        for _ in range(TRIES):
            p = Pose2D(*(self.rng.uniform(lo, hi) for lo, hi in ranges))
            if self.free(p.x, p.y, radius + self.clearance):
                self.circles.append((p.x, p.y, radius))
                return p
        raise LayoutError(f"{self.path}: no free pose for {name} in its ranges after {TRIES} tries "
                          f"(clearance {self.clearance} m)")

    def free(self, x: float, y: float, r: float) -> bool:
        for cx, cy, cr in self.circles:
            if math.hypot(x - cx, y - cy) < r + cr:
                return False
        for cx, cy, hx, hy in self.rects:
            dx, dy = max(abs(x - cx) - hx, 0.0), max(abs(y - cy) - hy, 0.0)
            if math.hypot(dx, dy) < r:
                return False
        for x0, y0, x1, y1, half in self.segments:
            if _segment_distance(x, y, x0, y0, x1, y1) < r + half:
                return False
        return True


def _segment_distance(px, py, x0, y0, x1, y1) -> float:
    dx, dy = x1 - x0, y1 - y0
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / n))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


AUTO_KEYS = {"range": float, "margin": float, "hold": float, "movers": bool}


def _door_auto(value, where: str, path) -> dict | None:
    """A door's `auto`: false/absent (None), true ({}) or a mapping of
    AUTO_KEYS overriding the config's door.auto."""
    if value is None or value is False:
        return None
    if value is True:
        return {}
    if not isinstance(value, dict):
        raise LayoutError(f"{path}: {where}.auto must be true, false or a mapping")
    unknown = set(value) - set(AUTO_KEYS)
    if unknown:
        raise LayoutError(f"{path}: {where}.auto: unknown key(s) {', '.join(sorted(unknown))}")
    try:
        return {k: AUTO_KEYS[k](v) for k, v in value.items()}
    except (TypeError, ValueError):
        raise LayoutError(f"{path}: {where}.auto: bad value") from None
