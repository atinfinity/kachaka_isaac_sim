"""Doors that open and close: a sliding or hinged panel in a doorway, moved
on command (sim/doors/<id>/set_open) or on a fixed cycle.

The panel is a kinematic rigid body: the LiDAR sees it, and it pushes
whatever stands in its way (no safety sensor stops it). The doorway's walls
are not part of the door; add them with scripts/run_sim.py --wall.

`DoorSpec`, `DoorState` and `panel_pose` are plain Python; `Doors` needs
Isaac Sim (create it after SimulationApp has started).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import TYPE_CHECKING

from .drive import Pose2D, quat_from_yaw

if TYPE_CHECKING:
    from pxr import Usd

KINDS = ("sliding", "hinged")
SLIDING_OVERLAP = 0.05  # m, a closed sliding panel overlaps the doorway's edges by this
HINGED_GAP = 0.01  # m, a hinged panel is this much narrower than the doorway


@dataclass
class DoorSpec:
    id: str
    path: str
    pose: Pose2D  # centre of the doorway, x along the wall (world frame)
    width: float  # doorway width [m]
    kind: str = "sliding"
    open: bool = True  # state at the start (and after sim/reset)
    cycle: tuple[float, float] | None = None  # (open s, closed s), repeated; None: on command only


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
    second. A command overrides the cycle until the next sim/reset."""

    spec: DoorSpec
    rate: float  # opening per second
    opening: float = field(init=False)
    target: bool = field(init=False)
    cycling: bool = field(init=False)

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.opening = 1.0 if self.spec.open else 0.0
        self.target = self.spec.open
        self.cycling = self.spec.cycle is not None

    def command(self, open_: bool) -> None:
        self.target = open_
        self.cycling = False

    def step(self, t: float, dt: float) -> bool:
        """Advance by `dt` at time `t` (from the start or the last reset).
        True if the panel moved."""
        if self.cycling:
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

    def __init__(self, stage: Usd.Stage, specs: list[DoorSpec], cfg: SimpleNamespace, wall: float):
        from pxr import UsdGeom

        self.specs = specs
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
        moved = [i for i, st in enumerate(self.states) if st.step(self.t, dt)]
        if moved:
            self._move(moved)
        if [st.state for st in self.states] != before:
            self.changed = True

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
