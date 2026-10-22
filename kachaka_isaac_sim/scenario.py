"""Scenario files (scripts/run_scenarios.py): what to set up, what the robot
should do and how often, for batch runs that report numbers.

    name: room_movers           # optional; default: the file name
    world: room                 # optional
    layout: ../layouts/room_movers.yaml   # optional; relative to this file
    spawn: [0.0, 0.0, 0.0]      # optional; any layout key, over the layout file
    shelves: []
    seed: 3                     # layout seed and sensor noise seed
    repeat: 3                   # runs (default 1)
    restart: false              # true: a fresh simulator and Nav2 per run, seed + run
    clear_map: true             # clear the SLAM map at each reset (default true)
    timeout: 300                # s per step (default 300)
    sim_args: [--no-cameras]    # optional; more options for run_sim.py
    steps:
      - goal: [1.6, -0.3, 3.1416]           # Nav2 NavigateToPose, world frame
      - api: move_to_location               # a kachaka-api method
        args: {location_name_or_id: L01}
      - api: move_to_pose                   # x, y, yaw in the world frame
        args: {x: 0.0, y: 0.0, yaw: 0.0}
      - wait: 2.0                           # s

Poses are [x, y, yaw] in the world frame (m, rad), as in layout files. The
SLAM map frame starts at the spawn pose, so goal steps and move_to_pose are
moved into it. A run succeeds when every step does; a goal or move_to_pose
step also has to end within GOAL_TOLERANCE of its pose (ground truth).

Metrics per run (Run): sim time, path length (ground truth), collisions
(sim/collision), the smallest LiDAR clearance from the robot's body, and
Nav2 recoveries (NavigateToPose feedback).

Plain Python (no ROS): the runner reads scenarios on the host, and the ROS
side (scripts/scenario_episode.py) uses the metric helpers.
"""

from __future__ import annotations

import csv
import io
import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from .layout import KEYS as LAYOUT_KEYS
from .layout import parse_layout, read_layout
from .scenes import SPAWN

SCENARIO_KEYS = ("name", "layout", "repeat", "restart", "clear_map", "timeout", "sim_args", "steps")
STEP_KINDS = ("goal", "api", "wait")
GOAL_TOLERANCE = 0.35  # m, Nav2 goal checker 0.25 plus SLAM drift
DEFAULT_TIMEOUT = 300.0

# The body's collision box in base_footprint (= base_link in the plane):
# 0.387 x 0.240 m centred at x 0.0435 (kachaka_description _kachaka.urdf.xacro,
# base_link collision). The LiDAR (laser_frame) sits at x 0.156, turned by
# +90 deg about z (lidar_joint).
BODY_X = (0.0435 - 0.387 / 2, 0.0435 + 0.387 / 2)
BODY_Y = (-0.120, 0.120)
LIDAR_X = 0.156
LIDAR_YAW = math.pi / 2


class ScenarioError(ValueError):
    pass


@dataclass
class Step:
    kind: str  # goal, api, wait
    pose: tuple[float, float, float] | None = None  # goal (world frame)
    method: str = ""  # api
    args: dict = field(default_factory=dict)  # api
    seconds: float = 0.0  # wait

    def label(self) -> str:
        if self.kind == "goal":
            return "goal({:+.2f}, {:+.2f}, {:+.2f})".format(*self.pose)
        if self.kind == "api":
            return f"{self.method}({', '.join(f'{k}={v}' for k, v in self.args.items())})"
        return f"wait({self.seconds:g})"


@dataclass
class Scenario:
    name: str
    path: str
    layout: dict  # layout keys, the file's and the scenario's merged
    seed: int | None
    repeat: int
    restart: bool
    clear_map: bool
    timeout: float
    sim_args: list[str]
    steps: list[Step]

    @property
    def world(self) -> str:
        return self.layout.get("world") or "room"

    def run_seed(self, run: int) -> int | None:
        """Seed of a run: the same for every run unless the simulator restarts."""
        if self.seed is None:
            return None
        return self.seed + run if self.restart else self.seed

    def spawn(self, run: int = 0) -> tuple[float, float, float]:
        """The spawn pose of a run: the layout's (drawn with the run's seed
        if it is a range) or the world's default."""
        if "spawn" in self.layout:
            p = parse_layout(self.layout, seed=self.run_seed(run) or 0, world=self.world, path=self.path).spawn
            return p.x, p.y, p.yaw
        return SPAWN.get(self.world, (0.0, 0.0, 0.0))

    def episode(self, run: int, runs: list[int]) -> dict:
        """What scripts/scenario_episode.py needs, as JSON."""
        return {"name": self.name, "spawn": self.spawn(run), "runs": runs,
                "seeds": [self.run_seed(r) for r in runs], "clear_map": self.clear_map,
                "timeout": self.timeout, "steps": [asdict(s) for s in self.steps]}


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return parse_scenario(data, str(path), name=path.stem)


def parse_scenario(data: dict, path: str = "", name: str = "scenario") -> Scenario:
    if not isinstance(data, dict):
        raise ScenarioError(f"{path}: a scenario is a mapping")
    unknown = set(data) - set(SCENARIO_KEYS) - set(LAYOUT_KEYS)
    if unknown:
        raise ScenarioError(f"{path}: unknown key(s) {', '.join(sorted(unknown))}; "
                            f"known: {', '.join(SCENARIO_KEYS + LAYOUT_KEYS)}")
    layout = {}
    if data.get("layout"):
        lp = Path(data["layout"])
        if not lp.is_absolute():
            lp = Path(path).parent / lp
        if not lp.exists():
            raise ScenarioError(f"{path}: layout {lp} not found")
        layout = read_layout(lp)
    layout.update({k: v for k, v in data.items() if k in LAYOUT_KEYS})
    try:
        resolved = parse_layout(layout, seed=layout.get("seed", 0), world=layout.get("world"), path=path)
    except ValueError as e:
        raise ScenarioError(str(e)) from e
    if resolved.randomised and layout.get("seed") is None:
        raise ScenarioError(f"{path}: the layout draws poses from ranges: give a seed, so runs can be repeated")
    steps = [_step(s, i, path) for i, s in enumerate(data.get("steps") or [])]
    if not steps:
        raise ScenarioError(f"{path}: no steps")
    repeat = int(data.get("repeat", 1))
    if repeat < 1:
        raise ScenarioError(f"{path}: repeat must be at least 1")
    sim_args = data.get("sim_args") or []
    if not isinstance(sim_args, list):
        raise ScenarioError(f"{path}: sim_args is a list of options")
    seed = layout.get("seed")
    return Scenario(name=str(data.get("name", name)), path=path, layout=layout,
                    seed=int(seed) if seed is not None else None, repeat=repeat,
                    restart=bool(data.get("restart", False)), clear_map=bool(data.get("clear_map", True)),
                    timeout=float(data.get("timeout", DEFAULT_TIMEOUT)), sim_args=[str(a) for a in sim_args],
                    steps=steps)


def _step(s, i: int, path: str) -> Step:
    where = f"{path}: steps[{i}]"
    kinds = [k for k in STEP_KINDS if isinstance(s, dict) and k in s]
    if len(kinds) != 1:
        raise ScenarioError(f"{where}: give one of {', '.join(STEP_KINDS)}")
    kind = kinds[0]
    extra = set(s) - {kind} - ({"args"} if kind == "api" else set())
    if extra:
        raise ScenarioError(f"{where}: unknown key(s) {', '.join(sorted(extra))}")
    if kind == "goal":
        pose = s["goal"]
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            raise ScenarioError(f"{where}: goal is [x, y, yaw]")
        return Step("goal", pose=tuple(float(v) for v in pose))
    if kind == "api":
        args = s.get("args") or {}
        if not isinstance(s["api"], str) or not isinstance(args, dict):
            raise ScenarioError(f"{where}: api is a method name, args a mapping")
        if s["api"] == "move_to_pose" and not {"x", "y", "yaw"} <= set(args):
            raise ScenarioError(f"{where}: move_to_pose needs x, y and yaw")
        return Step("api", method=s["api"], args=dict(args))
    return Step("wait", seconds=float(s["wait"]))


def to_map(spawn, x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """World pose -> SLAM map frame (origin at the spawn pose)."""
    sx, sy, syaw = spawn
    dx, dy = x - sx, y - sy
    c, s = math.cos(syaw), math.sin(syaw)
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw))


def step_pose(step: dict) -> tuple[float, float, float] | None:
    """The world pose a step should end at, if it has one."""
    if step["kind"] == "goal":
        return tuple(step["pose"])
    if step["kind"] == "api" and step["method"] == "move_to_pose":
        a = step["args"]
        return float(a["x"]), float(a["y"]), float(a["yaw"])
    return None


# Metric helpers -------------------------------------------------------------

def path_length(points, min_step: float = 0.002) -> float:
    """Length of a sequence of (x, y), skipping moves below min_step (jitter)."""
    total = 0.0
    last = None
    for p in points:
        if last is None:
            last = p
            continue
        d = math.hypot(p[0] - last[0], p[1] - last[1])
        if d >= min_step:
            total += d
            last = p
    return total


def body_clearance(angle_min: float, angle_increment: float, ranges, range_min: float = 0.0,
                   range_max: float = math.inf) -> float:
    """Smallest distance from a scan's points to the body's outline (m);
    inf when the scan sees nothing."""
    best = math.inf
    for i, r in enumerate(ranges):
        if not (range_min <= r <= range_max) or not math.isfinite(r):
            continue
        a = angle_min + i * angle_increment + LIDAR_YAW
        x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
        dx = max(BODY_X[0] - x, 0.0, x - BODY_X[1])
        dy = max(BODY_Y[0] - y, 0.0, y - BODY_Y[1])
        best = min(best, math.hypot(dx, dy))
    return best


def count_recoveries(feedback) -> int:
    """Recoveries over all goals, from (goal id, number_of_recoveries)
    feedback pairs: the count goes up within a goal and restarts per goal."""
    most: dict = {}
    for goal, n in feedback:
        most[goal] = max(most.get(goal, 0), int(n))
    return sum(most.values())


# Results --------------------------------------------------------------------

@dataclass
class Run:
    scenario: str
    run: int
    seed: int | None
    success: bool
    time_s: float  # sim time from the first step to the end of the last
    wall_s: float
    path_m: float
    collisions: int
    collided_with: str  # ;-separated prim paths
    min_clearance_m: float  # LiDAR point to body outline
    recoveries: int
    failed_step: str  # the first step that failed, and why
    steps: list = field(default_factory=list)  # per step: label, ok, why, time_s, error_m

    @classmethod
    def from_dict(cls, d: dict) -> Run:
        return cls(**{k: d.get(k) for k in cls.__dataclass_fields__})


CSV_FIELDS = [f for f in Run.__dataclass_fields__ if f != "steps"]


def runs_csv(runs: list[Run]) -> str:
    out = io.StringIO()
    w = csv.DictWriter(out, CSV_FIELDS, lineterminator="\n")
    w.writeheader()
    for r in runs:
        row = {k: v for k, v in asdict(r).items() if k in CSV_FIELDS}
        for k, v in row.items():
            if isinstance(v, float):
                row[k] = "" if math.isnan(v) else ("inf" if math.isinf(v) else f"{v:.3f}")
        w.writerow(row)
    return out.getvalue()


def json_safe(v):
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: json_safe(x) for k, x in v.items()}
    if isinstance(v, list):
        return [json_safe(x) for x in v]
    return v


def runs_json(runs: list[Run], meta: dict | None = None) -> str:
    return json.dumps(json_safe({"meta": meta or {}, "runs": [asdict(r) for r in runs],
                                  "summary": summarize(runs)}), indent=2)


def _stats(values: list[float]) -> tuple[float, float]:
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return math.nan, math.nan
    return statistics.fmean(values), statistics.stdev(values) if len(values) > 1 else 0.0


def summarize(runs: list[Run]) -> list[dict]:
    """One row per scenario, in the order they first appear. Time and path
    length are over the successful runs only (a failed run stops early)."""
    rows = []
    for name in dict.fromkeys(r.scenario for r in runs):
        rs = [r for r in runs if r.scenario == name]
        ok = [r for r in rs if r.success]
        time_mean, time_sd = _stats([r.time_s for r in ok])
        path_mean, path_sd = _stats([r.path_m for r in ok])
        clear = [r.min_clearance_m for r in rs if r.min_clearance_m is not None and math.isfinite(r.min_clearance_m)]
        rows.append({
            "scenario": name, "runs": len(rs), "success": len(ok), "success_rate": len(ok) / len(rs),
            "time_mean_s": time_mean, "time_sd_s": time_sd, "path_mean_m": path_mean, "path_sd_m": path_sd,
            "collisions": sum(r.collisions for r in rs), "runs_with_collisions": sum(1 for r in rs if r.collisions),
            "min_clearance_m": min(clear) if clear else math.nan,
            "mean_min_clearance_m": statistics.fmean(clear) if clear else math.nan,
            "recoveries_mean": statistics.fmean(r.recoveries for r in rs),
        })
    return rows


def summary_table(rows: list[dict]) -> str:
    """The summary as a fixed-width text table."""

    def num(v, fmt):
        return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else format(v, fmt)

    head = ["scenario", "success", "time [s]", "path [m]", "collisions", "min clear [m]", "recoveries"]
    lines = [[r["scenario"], f"{r['success']}/{r['runs']}",
              f"{num(r['time_mean_s'], '.1f')} ± {num(r['time_sd_s'], '.1f')}",
              f"{num(r['path_mean_m'], '.2f')} ± {num(r['path_sd_m'], '.2f')}",
              f"{r['collisions']} ({r['runs_with_collisions']} runs)",
              num(r["min_clearance_m"], ".3f"), num(r["recoveries_mean"], ".1f")] for r in rows]
    widths = [max(len(str(x)) for x in col) for col in zip(head, *lines, strict=True)]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    out = [fmt.format(*head), fmt.format(*("-" * w for w in widths))]
    out += [fmt.format(*line) for line in lines]
    return "\n".join(out)
