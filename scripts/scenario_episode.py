"""ROS side of scripts/run_scenarios.py: run one scenario's steps against the
simulator with SLAM + Nav2 up, once per run, and print one line per run:

    RESULT {"scenario": ..., "success": true, "time_s": ..., ...}

Each run starts with sim/reset (and clears the SLAM map unless the scenario
says clear_map: false). Runs in the ROS image with the repository mounted:

    python3 /repo/scripts/scenario_episode.py '<json from Scenario.episode()>'

Metrics come from /kachaka/sim/ground_truth (time, path), /kachaka/sim/collision,
/kachaka/lidar/scan (clearance from the body) and the NavigateToPose
feedback (recoveries, also for goals that kachaka-api sends through Nav2).
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sim_reset import reset_sim
from std_msgs.msg import String

from kachaka_isaac_sim.scenario import (
    GOAL_TOLERANCE,
    Run,
    body_clearance,
    count_recoveries,
    json_safe,
    path_length,
    step_pose,
    to_map,
)

API_TARGET = "127.0.0.1:26400"
SETTLE = 0.5  # s after a step, for the ground truth to catch up


class Recorder(Node):
    """Collects what the metrics need, on a background executor."""

    def __init__(self):
        super().__init__("scenario_episode")
        self.lock = threading.Lock()
        self.gt: list[tuple[float, float, float]] = []  # t, x, y
        self.last_gt: tuple[float, float, float] | None = None
        self.clearance = math.inf
        self.step_clearance = math.inf  # since the current step started
        self.collisions: list[dict] = []
        self.feedback: list[tuple[bytes, int]] = []
        self.map_info = None
        self.recording = False
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(Odometry, "/kachaka/sim/ground_truth", self.on_gt, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/kachaka/lidar/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/kachaka/sim/collision", self.on_collision, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, "/navigate_to_pose/_action/feedback",
                                 self.on_feedback, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map_info", m.info), latched)

    def on_gt(self, m: Odometry) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        with self.lock:
            self.last_gt = (t, p.x, p.y)
            if self.recording:
                self.gt.append(self.last_gt)

    def on_scan(self, m: LaserScan) -> None:
        if not self.recording:
            return
        c = body_clearance(m.angle_min, m.angle_increment, m.ranges, m.range_min, m.range_max)
        with self.lock:
            self.clearance = min(self.clearance, c)
            self.step_clearance = min(self.step_clearance, c)

    def on_collision(self, m: String) -> None:
        if self.recording:
            with self.lock:
                self.collisions.append(json.loads(m.data))

    def on_feedback(self, m) -> None:
        if self.recording:
            with self.lock:
                self.feedback.append((bytes(m.goal_id.uuid), m.feedback.number_of_recoveries))

    def start(self) -> None:
        with self.lock:
            self.gt.clear()
            self.collisions.clear()
            self.feedback.clear()
            self.clearance = math.inf
            self.recording = True

    def stop(self) -> None:
        self.recording = False

    def now(self) -> float | None:
        """Sim time of the latest ground truth."""
        with self.lock:
            return self.last_gt[0] if self.last_gt else None

    def pose(self) -> tuple[float, float] | None:
        with self.lock:
            return self.last_gt[1:] if self.last_gt else None

    def map_covers(self, x: float, y: float) -> bool:
        i = self.map_info
        if i is None:
            return False
        x0, y0 = i.origin.position.x, i.origin.position.y
        return x0 < x < x0 + i.width * i.resolution and y0 < y < y0 + i.height * i.resolution


def wait_until(cond, timeout: float, poll: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(poll)
    return cond()


def sim_sleep(rec: Recorder, seconds: float) -> None:
    """Sleep in sim time (falls back to wall time without ground truth)."""
    start = rec.now()
    if start is None:
        time.sleep(seconds)
        return
    wait_until(lambda: (rec.now() or start) - start >= seconds, timeout=seconds * 20 + 5)


def run_goal(rec: Recorder, pose, timeout: float) -> tuple[bool, str]:
    x, y, yaw = pose
    # SLAM grows the map as the robot sees more; Nav2 rejects goals outside it.
    wait_until(lambda: rec.map_covers(x, y), 30.0)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    future = rec.nav.send_goal_async(goal)
    if not wait_until(future.done, 10.0) or not future.result().accepted:
        return False, "goal rejected"
    result = future.result().get_result_async()
    if not wait_until(result.done, timeout):
        future.result().cancel_goal_async()
        wait_until(result.done, 5.0)
        return False, "timeout"
    status = result.result().status
    if status != GoalStatus.STATUS_SUCCEEDED:
        return False, f"status {status}, error {result.result().result.error_code}"
    return True, "succeeded"


class Api:
    def __init__(self):
        self._client = None

    @property
    def client(self):
        if self._client is None:
            from kachaka_api import KachakaApiClient

            self._client = KachakaApiClient(API_TARGET)
        return self._client

    def call(self, method: str, args: dict, timeout: float) -> tuple[bool, str]:
        fn = getattr(self.client, method, None)
        if fn is None:
            return False, f"kachaka-api has no method {method}"
        out: dict = {}

        def target():
            try:
                out["result"] = fn(**args)
            except Exception as e:  # noqa: BLE001 (reported as the step's failure)
                out["error"] = f"{type(e).__name__}: {e}"

        th = threading.Thread(target=target, daemon=True)
        th.start()
        th.join(timeout)
        if th.is_alive():
            self.client.cancel_command()
            th.join(5.0)
            return False, "timeout"
        if "error" in out:
            return False, out["error"]
        res = out.get("result")
        if hasattr(res, "success") and not res.success:
            return False, f"error_code {res.error_code}"
        return True, "succeeded"


def run_once(rec: Recorder, api: Api, spec: dict, run: int, seed) -> Run:
    spawn = spec["spawn"]
    timeout = float(spec["timeout"])
    reset_node = rclpy.create_node("scenario_reset")
    try:
        reset_sim(reset_node, spec["clear_map"])
    finally:
        reset_node.destroy_node()
    wait_until(lambda: rec.map_info is not None and rec.now() is not None, 30.0)
    rec.start()
    wall0 = time.monotonic()
    t0 = rec.now()
    steps = []
    failed = ""
    for step in spec["steps"]:
        label = step_label(step)
        ts = rec.now()
        with rec.lock:
            rec.step_clearance = math.inf
            fb0 = len(rec.feedback)
        pose = step_pose(step)
        if step["kind"] == "goal":
            ok, why = run_goal(rec, to_map(spawn, *pose), timeout)
        elif step["kind"] == "api":
            args = dict(step["args"])
            if pose is not None:
                args["x"], args["y"], args["yaw"] = to_map(spawn, *pose)
            ok, why = api.call(step["method"], args, timeout)
        else:
            sim_sleep(rec, float(step["seconds"]))
            ok, why = True, "waited"
        error = math.nan
        if pose is not None:
            sim_sleep(rec, SETTLE)
            p = rec.pose()
            error = math.hypot(p[0] - pose[0], p[1] - pose[1]) if p else math.nan
            if ok and not error < GOAL_TOLERANCE:
                ok, why = False, f"ended {error:.2f} m from the pose"
        te = rec.now()
        dt = te - ts if te is not None and ts is not None else math.nan
        with rec.lock:
            clear, recov = rec.step_clearance, count_recoveries(rec.feedback[fb0:])
        steps.append({"step": label, "ok": ok, "why": why, "time_s": dt, "error_m": error,
                      "min_clearance_m": clear, "recoveries": recov})
        print(f"  [{spec['name']} #{run}] {label}: {'OK' if ok else 'FAIL'} ({why}, {dt:.1f} s"
              f"{f', error {error:.2f} m' if math.isfinite(error) else ''}, clearance {clear:.3f} m, "
              f"{recov} recoveries)", flush=True)
        if not ok:
            failed = f"{label}: {why}"
            break
    rec.stop()
    t1 = rec.now()
    with rec.lock:
        path = path_length([(x, y) for _, x, y in rec.gt])
        collisions = list(rec.collisions)
        clearance = rec.clearance
        recoveries = count_recoveries(rec.feedback)
    return Run(scenario=spec["name"], run=run, seed=seed, success=not failed,
               time_s=t1 - t0 if t0 is not None and t1 is not None else math.nan,
               wall_s=time.monotonic() - wall0, path_m=path, collisions=len(collisions),
               collided_with=";".join(sorted({str(c.get("with", "?")) for c in collisions})),
               min_clearance_m=clearance, recoveries=recoveries, failed_step=failed, steps=steps)


def step_label(step: dict) -> str:
    from kachaka_isaac_sim.scenario import Step

    return Step(**step).label()


def main() -> int:
    spec = json.loads(sys.argv[1])
    rclpy.init()
    rec = Recorder()
    executor = MultiThreadedExecutor()
    executor.add_node(rec)
    threading.Thread(target=executor.spin, daemon=True).start()
    api = Api()
    code = 0
    try:
        if any(s["kind"] == "goal" for s in spec["steps"]) and not rec.nav.wait_for_server(timeout_sec=60.0):
            print("navigate_to_pose action server not found", flush=True)
            return 1
        seeds = spec.get("seeds") or [None] * len(spec["runs"])
        for run, seed in zip(spec["runs"], seeds, strict=True):
            result = run_once(rec, api, spec, run, seed)
            d = result.__dict__
            print("RESULT " + json.dumps(json_safe(d)), flush=True)
            code |= 0 if result.success else 2
    finally:
        executor.shutdown()
        rec.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
