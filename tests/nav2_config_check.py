"""Check nav2_params.yaml, slam_toolbox.yaml and the behavior trees without the simulator.

Runs inside the ROS 2 image (CI does this), with fake_robot standing in for
Isaac Sim:

    docker run --rm -v "$PWD:/repo" kachaka-isaac-sim-ros:jazzy python3 /repo/tests/nav2_config_check.py

1. slam.launch.py nav:=true: slam_toolbox publishes a map, every lifecycle
   manager reports active, and a NavigateToPose goal succeeds (behavior tree).
2. navigation.launch.py on the map saved in step 1: AMCL + map_server come
   up and a goal succeeds again.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

STARTUP_TIMEOUT = 120.0
GOAL_TIMEOUT = 90.0
GOALS = [(0.8, -0.5, 0.0), (-0.5, 0.6, math.pi / 2)]
SLAM_MANAGERS = ["lifecycle_manager_navigation", "lifecycle_manager_shelf_docking"]
NAV_MANAGERS = ["lifecycle_manager_localization", *SLAM_MANAGERS]


class Stack:
    """Processes started in their own session, killed together."""

    def __init__(self, log_dir: Path):
        self.log_dir = log_dir
        self.procs: list[tuple[str, subprocess.Popen]] = []

    def start(self, name: str, *cmd: str) -> None:
        log = open(self.log_dir / f"{name}.log", "w")  # noqa: SIM115 (closed with the process)
        self.procs.append((name, subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)))

    def stop(self) -> None:
        for _, p in self.procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        end = time.monotonic() + 15.0
        for _, p in self.procs:
            try:
                p.wait(max(0.1, end - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        self.procs.clear()

    def dead(self) -> list[str]:
        return [name for name, p in self.procs if p.poll() is not None]

    def tail(self, lines: int = 60) -> None:
        for name, _ in self.procs or []:
            path = self.log_dir / f"{name}.log"
            print(f"----- {path} (last {lines} lines) -----")
            print("\n".join(path.read_text(errors="replace").splitlines()[-lines:]))


class Checker(Node):
    def __init__(self, name: str):
        super().__init__(name)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map: OccupancyGrid | None = None
        self.odom: Odometry | None = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.amcl_pose = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, what: str, cond, timeout: float, stack: Stack) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            if stack.dead():
                print(f"  exited early: {', '.join(stack.dead())}")
                return False
            self.spin_for(0.5)
        print(f"  timed out waiting for {what}")
        return False

    def is_active(self, manager: str) -> bool:
        client = self.create_client(Trigger, f"/{manager}/is_active")
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return False
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            return bool(future.result() and future.result().success)
        finally:
            self.destroy_client(client)

    def set_initial_pose(self, stack: Stack) -> bool:
        """Publish the initial pose until AMCL answers with a pose: AMCL drops
        it while it is not active yet, and then never publishes map -> odom."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.amcl_pose = None

        def taken() -> bool:
            if self.amcl_pose is None:
                self.initial_pose.publish(msg)
            return self.amcl_pose is not None

        return self.wait_for("/amcl_pose", taken, STARTUP_TIMEOUT, stack)

    def navigate(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        if not self.nav.wait_for_server(timeout_sec=10.0):
            return False, "no navigate_to_pose server"
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=GOAL_TIMEOUT)
        if not result.done():
            handle.cancel_goal_async()
            self.spin_for(1.0)
            return False, "timed out"
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"status {status}, error {result.result().result.error_code}"
        p = self.odom.pose.pose.position if self.odom else None
        where = f"odom ({p.x:.2f}, {p.y:.2f})" if p else "no odom"
        if p is None or math.hypot(p.x - x, p.y - y) > 0.3:
            return False, f"ended away from the goal: {where}"
        return True, where


def run_phase(node: Checker, stack: Stack, managers: list[str], localize: bool, check) -> None:
    ok = node.wait_for("/map", lambda: node.map is not None, STARTUP_TIMEOUT, stack)
    check("map published", ok, f"{node.map.info.width}x{node.map.info.height}" if node.map else "")
    if not ok:
        return
    if localize:
        # AMCL takes the initial pose only once it is active.
        check(f"{managers[0]} active", node.wait_for(managers[0], lambda: node.is_active(managers[0]),
                                                     STARTUP_TIMEOUT, stack))
        check("initial pose taken", node.set_initial_pose(stack))
        managers = managers[1:]
    for manager in managers:
        check(f"{manager} active", node.wait_for(manager, lambda m=manager: node.is_active(m), STARTUP_TIMEOUT, stack))
    for x, y, yaw in GOALS:
        ok, detail = node.navigate(x, y, yaw)
        check(f"navigate_to_pose ({x}, {y})", ok, detail)


def main() -> int:
    failures: list[str] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    log_dir = Path(os.environ.get("CHECK_LOG_DIR") or tempfile.mkdtemp(prefix="nav2_check_"))
    log_dir.mkdir(parents=True, exist_ok=True)
    map_base = log_dir / "ci_map"
    rclpy.init()
    stack = Stack(log_dir)
    try:
        print("== slam.launch.py nav:=true", flush=True)
        node = Checker("nav2_config_check_slam")
        stack.start("fake_robot", "ros2", "run", "kachaka_sim_bringup", "fake_robot")
        stack.start("slam", "ros2", "launch", "kachaka_sim_bringup", "slam.launch.py", "nav:=true", "rviz:=false")
        run_phase(node, stack, SLAM_MANAGERS, False, check)
        for _ in range(3):  # slow runners can miss the default 2 s map timeout
            saver = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(map_base),
                                    "--ros-args", "-p", "save_map_timeout:=10.0"],
                                   capture_output=True, text=True, timeout=60, check=False)
            saved = saver.returncode == 0 and map_base.with_suffix(".yaml").exists()
            if saved:
                break
        check("map_saver_cli", saved, "" if saved else (saver.stdout + saver.stderr)[-1500:])
        if failures:
            stack.tail()
        stack.stop()
        node.destroy_node()

        if map_base.with_suffix(".yaml").exists():
            print("== navigation.launch.py", flush=True)
            node = Checker("nav2_config_check_nav")
            stack.start("fake_robot_2", "ros2", "run", "kachaka_sim_bringup", "fake_robot")
            stack.start("navigation", "ros2", "launch", "kachaka_sim_bringup", "navigation.launch.py",
                        f"map:={map_base}.yaml", "rviz:=false")
            n = len(failures)
            run_phase(node, stack, NAV_MANAGERS, True, check)
            if len(failures) > n:
                stack.tail()
            stack.stop()
            node.destroy_node()
    finally:
        stack.stop()
        rclpy.try_shutdown()
    print(f"logs in {log_dir}")
    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
