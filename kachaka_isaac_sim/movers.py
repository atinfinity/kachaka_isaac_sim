"""People and boxes that move: along a loop of waypoints, or to random
points in a region (with a seed), and optionally there only part of the
time. Set in a layout file (layout.py); everything else stands still.

A mover is a kinematic body: the LiDAR, the cameras and the ToF see it, and
it would push the robot, so it waits while the robot stands in its way, as a
person would, instead of walking into it.

`MoverSpec`, `MoverState` are plain Python; `Movers` needs Isaac Sim (create
it after SimulationApp has started).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import TYPE_CHECKING

from .drive import Pose2D, wrap_angle

if TYPE_CHECKING:
    from pxr import Usd

ROBOT_RADIUS = 0.23  # m, half the diagonal of the 387 x 240 mm body
AWAY = (1000.0, 1000.0)  # where a mover that is not there waits (off every map)


@dataclass
class MoverSpec:
    path: str  # USD prim, moved through its translate and rotateZ ops
    start: Pose2D
    radius: float  # footprint radius [m], for keeping out of the robot's way
    speed: float  # m/s
    route: list[tuple[float, float]] = field(default_factory=list)  # waypoints after start; then back to it
    wander: tuple[tuple[float, float], tuple[float, float]] | None = None  # x and y ranges for random waypoints
    pause: float = 0.0  # s at each waypoint
    appear: tuple[float, float] | None = None  # (there s, away s), repeated, from a random phase
    seed: int = 0
    z: float = 0.0  # height of the prim's origin

    @property
    def moves(self) -> bool:
        return bool(self.route) or self.wander is not None


@dataclass
class MoverState:
    """Where a mover is. It turns towards its next waypoint at `turn_rate`,
    walks there at its speed once it faces it within `TURN_AND_WALK`, and
    waits there `pause` s. It waits too while the robot is within `gap` of
    the stretch it would walk in the next `lookahead` m, and after
    `give_way` s of that turns back (or, wandering, picks another point):
    the robot may be unable to leave with the mover so close."""

    spec: MoverSpec
    turn_rate: float = 2.0  # rad/s
    lookahead: float = 0.6  # m
    gap: float = 0.15  # m between the mover's and the robot's footprints
    give_way: float = 3.0  # s
    TURN_AND_WALK = 0.6  # rad

    pose: Pose2D = field(init=False)
    t: float = field(init=False)
    present: bool = field(init=False)
    blocked: bool = field(init=False)

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        s = self.spec
        self.rng = random.Random(s.seed)
        self.pose = Pose2D(s.start.x, s.start.y, s.start.yaw)
        self.t = 0.0
        self.leg = 0  # index into route: the waypoint walked to
        self.target = self._next_target() if s.moves else None
        self.wait = 0.0
        self.blocked = False
        self.blocked_for = 0.0  # s
        self.walked = 0.0  # m, drives the walking animation
        self.phase = self.rng.uniform(0.0, sum(s.appear)) if s.appear else 0.0
        self.present = self._present()

    def _next_target(self) -> tuple[float, float]:
        s = self.spec
        if s.wander is not None:
            (x0, x1), (y0, y1) = s.wander
            return self.rng.uniform(x0, x1), self.rng.uniform(y0, y1)
        points = list(s.route) + [(s.start.x, s.start.y)]
        p = points[self.leg % len(points)]
        self.leg += 1
        return p

    def _other_target(self) -> tuple[float, float]:
        """Where to go instead of the target: back to the waypoint it came
        from (then on as before), or another random point."""
        s = self.spec
        if s.wander is not None:
            return self._next_target()
        points = list(s.route) + [(s.start.x, s.start.y)]
        self.leg -= 1  # the target comes next again
        return points[(self.leg - 1) % len(points)]

    def _present(self) -> bool:
        a = self.spec.appear
        if a is None:
            return True
        return (self.t + self.phase) % (a[0] + a[1]) < a[0]

    def in_the_way(self, robot: Pose2D | None) -> bool:
        """The robot stands ahead, within `gap` of the stretch ahead."""
        if robot is None or self.target is None:
            return False
        p = self.pose
        tx, ty = self.target
        d = math.hypot(tx - p.x, ty - p.y)
        reach = min(self.lookahead, d)
        hx, hy = math.cos(p.yaw), math.sin(p.yaw)
        if (robot.x - p.x) * hx + (robot.y - p.y) * hy < 0.0:
            return False  # behind: walking on takes it away
        ex, ey = p.x + hx * reach, p.y + hy * reach
        return _segment_distance(robot.x, robot.y, p.x, p.y, ex, ey) < self.spec.radius + ROBOT_RADIUS + self.gap

    def step(self, dt: float, robot: Pose2D | None = None) -> bool:
        """Advance by `dt`. True if the pose or presence changed."""
        self.t += dt
        was = (self.pose.x, self.pose.y, self.pose.yaw, self.present)
        present = self._present()
        # Do not appear on top of the robot: wait until it has gone.
        if present and not self.present and robot is not None and \
                math.hypot(robot.x - self.pose.x, robot.y - self.pose.y) < self.spec.radius + ROBOT_RADIUS + self.gap:
            present = False
        self.present = present
        if self.target is None:
            return was != (self.pose.x, self.pose.y, self.pose.yaw, self.present)
        if self.wait > 0.0:
            self.wait = max(0.0, self.wait - dt)
        else:
            self._walk(dt, robot if self.present else None)
        return was != (self.pose.x, self.pose.y, self.pose.yaw, self.present)

    def _walk(self, dt: float, robot: Pose2D | None) -> None:
        p = self.pose
        tx, ty = self.target
        d = math.hypot(tx - p.x, ty - p.y)
        if d < 1e-3:
            self.target = self._next_target()
            self.wait = self.spec.pause
            return
        heading = math.atan2(ty - p.y, tx - p.x)
        err = wrap_angle(heading - p.yaw)
        turn = max(-self.turn_rate * dt, min(self.turn_rate * dt, err))
        p.yaw = wrap_angle(p.yaw + turn)
        self.blocked = self.in_the_way(robot)
        self.blocked_for = self.blocked_for + dt if self.blocked else 0.0
        if self.blocked_for > self.give_way:
            self.target = self._other_target()
            self.blocked_for = 0.0
        if self.blocked or abs(err - turn) > self.TURN_AND_WALK:
            return
        step = min(d, self.spec.speed * dt)
        # Along the line to the target, not the heading: it ends on the waypoint.
        p.x += (tx - p.x) / d * step
        p.y += (ty - p.y) / d * step
        self.walked += step


# A walking cycle (two steps) and the swing at full stride, in rad: hips,
# knees (bent while the leg swings forward) and arms (against the legs).
STRIDE = 1.3  # m
HIP_SWING, KNEE_BEND, ARM_SWING = 0.4, 0.6, 0.3


GAIT_EASE = 4.0  # 1/s: from standing to full stride in a quarter second


def gait_angles(walked: float, amount: float) -> dict[str, float]:
    """Joint angles of the walking pose after `walked` m, scaled by `amount`
    (0 standing, 1 walking). Positive is forward, about the lateral axis."""
    phase = 2.0 * math.pi * walked / STRIDE
    s = math.sin(phase)
    return {
        "L_Thigh": amount * HIP_SWING * s,
        "R_Thigh": -amount * HIP_SWING * s,
        "L_Calf": -amount * KNEE_BEND * max(0.0, math.cos(phase)),
        "R_Calf": -amount * KNEE_BEND * max(0.0, -math.cos(phase)),
        "L_Upperarm": -amount * ARM_SWING * s,
        "R_Upperarm": amount * ARM_SWING * s,
    }


def _segment_distance(px, py, x0, y0, x1, y1) -> float:
    dx, dy = x1 - x0, y1 - y0
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / n))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


class Movers:
    """The moving people and boxes in the running simulation."""

    def __init__(self, stage: Usd.Stage, specs: list[MoverSpec], cfg: SimpleNamespace, robot_pose):
        from pxr import UsdGeom

        from .world import Gait, Legs

        self.specs = specs
        self.states = [MoverState(s, float(cfg.turn_rate), float(cfg.lookahead), float(cfg.gap),
                                  float(getattr(cfg, "give_way", 3.0))) for s in specs]
        self.robot_pose = robot_pose  # () -> Pose2D, the robot's ground truth
        self.ops = []
        self.gaits = []  # world.Gait of a character, or None
        self.legs = []  # world.Legs of a person, or None
        for s in specs:
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(s.path)).GetOrderedXformOps()}
            self.ops.append((ops["xformOp:translate"], ops["xformOp:rotateZ"]))
            self.gaits.append(Gait.of(stage, f"{s.path}/actor/pose"))
            self.legs.append(Legs.of(stage, s.path))
        self.amount = [0.0] * len(specs)
        self._move(range(len(specs)))

    def reset(self) -> None:
        for st in self.states:
            st.reset()
        self.amount = [0.0] * len(self.states)
        self._move(range(len(self.states)))
        self._pose_gaits(range(len(self.states)))

    def step(self, dt: float) -> None:
        robot = self.robot_pose()
        moved, posed = [], []
        for i, st in enumerate(self.states):
            walked = st.walked
            if st.step(dt, robot):
                moved.append(i)
            if self.legs[i] is None:
                continue
            # Ease into and out of the stride rather than freezing mid-step.
            goal = 1.0 if st.walked > walked else 0.0
            a = self.amount[i]
            self.amount[i] = goal if abs(goal - a) <= GAIT_EASE * dt else a + math.copysign(GAIT_EASE * dt, goal - a)
            if st.walked > walked or a != self.amount[i]:
                posed.append(i)
        if moved:
            self._move(moved)
        if posed:
            self._pose_gaits(posed)

    def _pose_gaits(self, idx) -> None:
        for i in idx:
            if self.legs[i] is None:
                continue
            angles = gait_angles(self.states[i].walked, self.amount[i])
            self.legs[i].set(angles)
            if self.gaits[i] is not None:
                self.gaits[i].set(angles)

    def poses(self) -> list[Pose2D | None]:
        """Where each mover is, or None while it is away."""
        return [Pose2D(st.pose.x, st.pose.y, st.pose.yaw) if st.present else None for st in self.states]

    def _move(self, idx) -> None:
        from pxr import Gf

        for i in idx:
            st, (translate, rotate) = self.states[i], self.ops[i]
            x, y = (st.pose.x, st.pose.y) if st.present else AWAY
            translate.Set(Gf.Vec3d(x, y, self.specs[i].z))
            rotate.Set(math.degrees(st.pose.yaw))
