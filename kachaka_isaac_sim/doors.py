"""Doors that open and close: a sliding or hinged panel in a doorway, moved
on command (sim/doors/<id>/set_open), on a fixed cycle or, for an automatic
sliding door, when the robot or a mover comes near (`AutoSpec`).

The panel is a kinematic rigid body: the LiDAR sees it, and it pushes
whatever stands in its way (no safety sensor stops it). The doorway's walls
are not part of the door; add them with scripts/run_sim.py --wall.

`DoorSpec`, `DoorState` and `panel_pose` are plain Python; `Doors` needs
Isaac Sim (create it after SimulationApp has started).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import TYPE_CHECKING

from .drive import Pose2D, quat_from_yaw

if TYPE_CHECKING:
    from pxr import Usd

KINDS = ("sliding", "hinged")
SLIDING_OVERLAP = 0.05  # m, a closed sliding panel overlaps the doorway's edges by this
HINGED_GAP = 0.01  # m, a hinged panel is this much narrower than the doorway
MOTION_PERIOD = 0.1  # s between state messages while a panel moves (for RViz)


@dataclass
class AutoSpec:
    """An automatic door's sensor: it sees a point (the robot's base_footprint,
    a mover's centre) within `range` of the doorway on either side, across a
    strip `margin` wider than the doorway at each end. The door opens while it
    sees something and closes once it has seen nothing for `hold` seconds; it
    reopens if something comes while it closes."""

    range: float = 1.0  # m, each side of the doorway
    margin: float = 0.3  # m, beyond each edge of the doorway, along the wall
    hold: float = 2.0  # s
    movers: bool = True  # moving people and boxes trigger it too


@dataclass
class DoorSpec:
    id: str
    path: str
    pose: Pose2D  # centre of the doorway, x along the wall (world frame)
    width: float  # doorway width [m]
    kind: str = "sliding"
    open: bool = True  # state at the start (and after sim/reset)
    cycle: tuple[float, float] | None = None  # (open s, closed s), repeated; None: on command only
    auto: AutoSpec | None = None  # an automatic door (sliding only)

    def __post_init__(self):
        if self.auto is not None and self.kind != "sliding":
            raise ValueError(f"door {self.id}: only sliding doors can be automatic")
        if self.auto is not None and self.cycle is not None:
            raise ValueError(f"door {self.id}: a door is automatic or cycles, not both")

    def senses(self, x: float, y: float) -> bool:
        """Whether an automatic door's sensor sees the point (world x, y)."""
        if self.auto is None:
            return False
        p = self.pose
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        dx, dy = x - p.x, y - p.y
        along, across = c * dx + s * dy, -s * dx + c * dy
        return abs(along) <= self.width / 2 + self.auto.margin and abs(across) <= self.auto.range


def panel_size(spec: DoorSpec, thickness: float) -> tuple[float, float]:
    """The panel's length along the wall and its thickness."""
    if spec.kind == "sliding":
        return spec.width + 2 * SLIDING_OVERLAP, thickness
    return spec.width - HINGED_GAP, thickness


def panel_pose(spec: DoorSpec, opening: float, thickness: float, wall: float) -> Pose2D:
    """World pose of the panel's centre, `opening` from 0 (closed) to 1 (open).

    A sliding panel runs on the -y face of the wall (`wall` thick) and slides
    towards +x until it clears the doorway. A hinged panel turns about the
    doorway's -x side by up to 90 degrees, swinging into the -y side."""
    length, t = panel_size(spec, thickness)
    if spec.kind == "sliding":
        lx, ly, lyaw = opening * (spec.width + SLIDING_OVERLAP), -(wall / 2 + t / 2 + 0.005), 0.0
    else:
        a = -opening * math.pi / 2
        lx = -length / 2 + (length / 2) * math.cos(a)  # hinge at the panel's -x end
        ly = (length / 2) * math.sin(a)
        lyaw = a
    p = spec.pose
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose2D(p.x + c * lx - s * ly, p.y + s * lx + c * ly, p.yaw + lyaw)


@dataclass
class DoorState:
    """Opening (0 closed .. 1 open) moving towards the target at `rate` per
    second. A command overrides the cycle or the sensor until the next
    sim/reset."""

    spec: DoorSpec
    rate: float  # opening per second
    opening: float = field(init=False)
    target: bool = field(init=False)
    cycling: bool = field(init=False)
    automatic: bool = field(init=False)  # the sensor drives it (no command since the reset)
    last_seen: float = field(init=False)  # when the sensor last saw something

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.opening = 1.0 if self.spec.open else 0.0
        self.target = self.spec.open
        self.cycling = self.spec.cycle is not None
        self.automatic = self.spec.auto is not None
        self.last_seen = -math.inf

    def command(self, open_: bool) -> None:
        self.target = open_
        self.cycling = self.automatic = False

    def step(self, t: float, dt: float, sensed: bool = False) -> bool:
        """Advance by `dt` at time `t` (from the start or the last reset).
        `sensed`: an automatic door's sensor sees something. True if the
        panel moved."""
        if self.automatic:
            if sensed:
                self.last_seen = t
            self.target = t - self.last_seen < self.spec.auto.hold
        elif self.cycling:
            open_s, closed_s = self.spec.cycle
            phase = t % (open_s + closed_s)
            # The cycle starts in the start state.
            self.target = (phase < open_s) if self.spec.open else (phase >= closed_s)
        goal = 1.0 if self.target else 0.0
        if self.opening == goal:
            return False
        step = self.rate * dt
        self.opening = min(goal, self.opening + step) if goal > self.opening else max(goal, self.opening - step)
        return True

    @property
    def state(self) -> str:
        if self.opening >= 1.0:
            return "open"
        if self.opening <= 0.0:
            return "closed"
        return "opening" if self.target else "closing"


def door_rate(spec: DoorSpec, cfg: SimpleNamespace) -> float:
    """Opening per second for the configured panel speed."""
    if spec.kind == "sliding":
        return float(cfg.sliding_speed) / (spec.width + SLIDING_OVERLAP)
    return float(cfg.hinged_speed) / (math.pi / 2)


class Doors:
    """The doors in the running simulation."""

    def __init__(self, stage: Usd.Stage, specs: list[DoorSpec], cfg: SimpleNamespace, wall: float,
                 robot_pose: Callable[[], Pose2D | None] = lambda: None,
                 mover_poses: Callable[[], list[Pose2D | None]] = list):
        from pxr import UsdGeom

        self.specs = specs
        self.robot_pose = robot_pose  # what automatic doors sense
        self.mover_poses = mover_poses
        self.cfg = cfg
        self.wall = wall
        self.states = [DoorState(s, door_rate(s, cfg)) for s in specs]
        # The bodies exist already (world.add_door). They are moved through
        # their USD transforms, which PhysX follows as kinematic targets: a
        # pose set through the tensor API moved the collider but not the
        # rendered panel.
        self.ops = [UsdGeom.Xformable(stage.GetPrimAtPath(s.path)).GetOrderedXformOps() for s in specs]
        self.t = 0.0
        self.changed = True  # publish the state
        self.next_motion = 0.0  # when a moving panel is published next

    def index(self, door_id: str) -> int | None:
        return next((i for i, s in enumerate(self.specs) if s.id == door_id), None)

    def command(self, door_id: str, open_: bool) -> tuple[bool, str]:
        i = self.index(door_id)
        if i is None:
            return False, f"unknown door {door_id!r}"
        self.states[i].command(open_)
        self.changed = True
        return True, f"{door_id} {'opening' if open_ else 'closing'}"

    def reset(self) -> None:
        self.t = 0.0
        for st in self.states:
            st.reset()
        self._move(list(range(len(self.states))))
        self.changed = True

    def step(self, dt: float) -> None:
        self.t += dt
        before = [st.state for st in self.states]
        sensed = self._sensed()
        moved = [i for i, st in enumerate(self.states) if st.step(self.t, dt, sensed[i])]
        if moved:
            self._move(moved)
            if self.t + 1e-9 >= self.next_motion:
                self.changed = True
        if [st.state for st in self.states] != before:
            self.changed = True
        if self.changed:
            self.next_motion = self.t + MOTION_PERIOD

    def _sensed(self) -> list[bool]:
        """Whether each automatic door's sensor sees the robot or a mover."""
        if not any(st.automatic for st in self.states):
            return [False] * len(self.states)
        robot = self.robot_pose()
        movers = [p for p in self.mover_poses() if p is not None]
        out = []
        for sp, st in zip(self.specs, self.states):
            points = ([robot] if robot is not None else []) + (movers if st.automatic and sp.auto.movers else [])
            out.append(st.automatic and any(sp.senses(p.x, p.y) for p in points))
        return out

    def panels(self) -> list[tuple[Pose2D, tuple[float, float, float]]]:
        """Each panel's world pose and size (length, thickness, height)."""
        th = float(self.cfg.thickness)
        return [(panel_pose(sp, st.opening, th, self.wall), (*panel_size(sp, th), float(self.cfg.height)))
                for sp, st in zip(self.specs, self.states)]

    def _move(self, idx: list[int]) -> None:
        from pxr import Gf

        th, z = float(self.cfg.thickness), float(self.cfg.height) / 2
        for i in idx:
            p = panel_pose(self.specs[i], self.states[i].opening, th, self.wall)
            translate, orient = self.ops[i]
            translate.Set(Gf.Vec3d(p.x, p.y, z))
            orient.Set(Gf.Quatf(*map(float, quat_from_yaw(p.yaw))))

    def take_changed(self) -> bool:
        changed, self.changed = self.changed, False
        return changed
