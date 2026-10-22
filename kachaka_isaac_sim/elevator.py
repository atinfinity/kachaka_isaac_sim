"""An elevator between floors. The floors stand side by side in one stage,
FLOOR_SPACING apart along x, each with the same car at the same place; a
ride closes the car's door, waits, moves what is in the car to the same
place in the other floor's car and opens the door there. The robot's
odometry does not see the move, as it would not on a real ride.

Commands come from ROS (sim/elevator/go, a floor name) and the state goes
out on sim/elevator (see ros_node.KachakaSimNode.add_elevator).

How the real Kachaka rides: it asks the building's system (for example
Mitsubishi Electric Building Solutions' robot movement support service on
Ville-feuille, trial with Preferred Robotics announced 2024-09-28), which
calls the car, registers the floor and tells the robot which car to take and
when to board and leave; kachaka-api has no elevator RPCs. The car, its
door and the timing here are assumed.

`ElevatorSpec`, `ElevatorState` and the geometry helpers are plain Python;
`Elevator` needs Isaac Sim (create it after SimulationApp has started).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import TYPE_CHECKING

from .doors import DoorSpec
from .drive import Pose2D

if TYPE_CHECKING:
    from pxr import Usd

FLOOR_SPACING = 10.0  # m along x between floors in the stage
PHASES = ("open", "closing", "riding", "opening")


def floor_offset(k: int) -> tuple[float, float]:
    """Where floor k is in the stage, from the first floor."""
    return k * FLOOR_SPACING, 0.0


def floor_of(x: float) -> int:
    """The floor a world x is on."""
    return max(0, math.floor(x / FLOOR_SPACING + 0.5))


def shifted(p: Pose2D, k: int) -> Pose2D:
    dx, dy = floor_offset(k)
    return Pose2D(p.x + dx, p.y + dy, p.yaw)


@dataclass
class ElevatorSpec:
    pose: Pose2D  # middle of the doorway on the first floor, x along the wall; the car is on its +y side
    floors: list[str]  # floor names, from the first
    size: tuple[float, float] = (1.4, 1.4)  # inside: across (along the doorway), deep [m]
    door_width: float = 0.9  # m
    ride: float = 5.0  # s per floor, door closed
    start: int = 0  # floor the car is at (and after sim/reset)

    def local(self, p: Pose2D, k: int) -> tuple[float, float]:
        """`p` (world) in the car's frame on floor k."""
        o = shifted(self.pose, k)
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        dx, dy = p.x - o.x, p.y - o.y
        return c * dx + s * dy, -s * dx + c * dy

    def inside(self, p: Pose2D, k: int, wall: float) -> bool:
        """`p` is in floor k's car, past the doorway's wall."""
        x, y = self.local(p, k)
        return abs(x) <= self.size[0] / 2 and wall / 2 < y <= wall / 2 + self.size[1]

    def in_doorway(self, p: Pose2D, k: int, radius: float, wall: float) -> bool:
        """Something of `radius` at `p` would be in floor k's door's way."""
        x, y = self.local(p, k)
        return abs(x) < self.door_width / 2 + radius and abs(y) < wall / 2 + radius

    def door(self, k: int) -> DoorSpec:
        """Floor k's car door: a sliding door, open where the car starts."""
        name = self.floors[k]
        return DoorSpec(f"elevator_{name}", f"/World/Elevator/F{k + 1}/door", shifted(self.pose, k),
                        self.door_width, "sliding", open=k == self.start)


def car_walls(spec: ElevatorSpec, k: int, wall: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Floor k's car walls as (start, end) segments `wall` thick: the sides,
    the back, and the front on both sides of the doorway."""
    w, d = spec.size
    o = shifted(spec.pose, k)
    c, s = math.cos(o.yaw), math.sin(o.yaw)

    def world(x, y):
        return o.x + c * x - s * y, o.y + s * x + c * y

    hw, back, side = w / 2 + wall / 2, wall + d, spec.door_width / 2
    local = [((-hw, 0.0), (-hw, back + wall / 2)), ((hw, 0.0), (hw, back + wall / 2)),
             ((-hw, back), (hw, back)), ((-w / 2 - wall, 0.0), (-side, 0.0)), ((side, 0.0), (w / 2 + wall, 0.0))]
    return [(world(*a), world(*b)) for a, b in local]


@dataclass
class ElevatorState:
    """The car's floor and what it is doing: "open" (standing, door open),
    "closing", "riding" or "opening". `go` sends it to a floor; `step` says
    what to do with the doors and when the car arrives."""

    spec: ElevatorSpec
    floor: int = field(init=False)
    target: int = field(init=False)
    phase: str = field(init=False)
    ride_left: float = field(init=False)
    held: bool = field(init=False)  # the door is held open for something in the doorway

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.floor = self.target = self.spec.start
        self.phase = "open"
        self.ride_left = 0.0
        self.held = False

    def index(self, name: str) -> int | None:
        return self.spec.floors.index(name) if name in self.spec.floors else None

    def go(self, name: str) -> tuple[bool, str]:
        k = self.index(name)
        if k is None:
            return False, f"unknown floor {name!r}; floors: {', '.join(self.spec.floors)}"
        if self.phase == "riding":
            return False, f"riding to {self.spec.floors[self.target]}"
        self.target = k
        return True, f"going to {name}" if k != self.floor else f"at {name}"

    def step(self, dt: float, door: str, blocked: bool) -> list[tuple]:
        """Advance by `dt`. `door` is the state of the door on the car's floor
        (doors.DoorState.state), `blocked` whether something is in its way.
        Returns the actions: ("door", floor, open) and ("arrive", from, to)."""
        out = []
        if self.target != self.floor and self.phase in ("open", "opening"):
            self.phase, self.held = "closing", False
            out.append(("door", self.floor, False))
        if self.phase == "closing":
            if blocked != self.held:  # reopen for whatever is in the way, as a door sensor would
                self.held = blocked
                out.append(("door", self.floor, blocked))
            if self.target == self.floor:  # called back before it left
                self.phase, self.held = "opening", False
                out.append(("door", self.floor, True))
            elif door == "closed" and not blocked:
                self.phase = "riding"
                self.ride_left = abs(self.target - self.floor) * self.spec.ride
        elif self.phase == "riding":
            self.ride_left -= dt
            if self.ride_left <= 0.0:
                out.append(("arrive", self.floor, self.target))
                self.floor = self.target
                self.phase = "opening"
                out.append(("door", self.floor, True))
        elif self.phase == "opening" and door == "open":
            self.phase = "open"
        return out


def elevator_spec(layout_elevator, floors: list[str], cfg: SimpleNamespace) -> ElevatorSpec:
    """The ElevatorSpec for a layout's elevator (layout.Elevator), with the
    config's `elevator` section for what the layout leaves out."""
    e = layout_elevator
    size = tuple(e.size) if e.size else tuple(float(v) for v in cfg.size)
    return ElevatorSpec(e.pose, list(floors), size, float(e.door_width or cfg.door_width),
                        float(e.ride or cfg.ride), floors.index(e.start) if e.start else 0)


class Elevator:
    """The elevator in the running simulation. `step` runs before each physics
    step; `take_ride` (between steps, from the main loop) gives the floors of
    a finished ride once, for moving what was in the car."""

    def __init__(self, stage: Usd.Stage, spec: ElevatorSpec, door_cfg: SimpleNamespace, wall: float, robot_pose,
                 robot_radius: float):
        from .doors import Doors

        self.spec = spec
        self.wall = wall
        self.state = ElevatorState(spec)
        self.doors = Doors(stage, [spec.door(k) for k in range(len(spec.floors))], door_cfg, wall)
        self.robot_pose = robot_pose  # () -> world Pose2D
        self.robot_radius = robot_radius
        self.ride: tuple[int, int] | None = None
        self.changed = True
        self.robot_place = None  # (floor, in the car) as last published

    def go(self, name: str) -> tuple[bool, str]:
        ok, msg = self.state.go(name)
        self.changed = True
        return ok, msg

    def reset(self) -> None:
        self.state.reset()
        self.doors.reset()
        self.ride = None
        self.changed = True

    def step(self, dt: float) -> None:
        st = self.state
        before = (st.phase, st.floor)
        robot = self.robot_pose()
        place = (floor_of(robot.x), self.spec.inside(robot, st.floor, self.wall))
        if place != self.robot_place:
            self.robot_place = place
            self.changed = True
        blocked = self.spec.in_doorway(robot, st.floor, self.robot_radius, self.wall)
        for action in st.step(dt, self.doors.states[st.floor].state, blocked):
            if action[0] == "door":
                self.doors.command(self.spec.door(action[1]).id, action[2])
            else:
                self.ride = action[1:]
        self.doors.step(dt)
        if (st.phase, st.floor) != before or self.doors.take_changed():
            self.changed = True

    def take_ride(self) -> tuple[int, int] | None:
        ride, self.ride = self.ride, None
        return ride

    def take_changed(self) -> bool:
        changed, self.changed = self.changed, False
        return changed

    def status(self, robot: Pose2D) -> dict:
        st, names = self.state, self.spec.floors
        return {"floors": names, "floor": names[st.floor], "target": names[st.target], "state": st.phase,
                "door": self.doors.states[st.floor].state, "robot_floor": names[min(floor_of(robot.x), len(names) - 1)],
                "robot_in_car": self.spec.inside(robot, st.floor, self.wall)}
