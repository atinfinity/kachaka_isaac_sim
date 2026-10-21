"""Measure the simulator's top speeds with and without a docked shelf.

Manual cmd_vel (manual_control/cmd_vel) is capped at drive.max_linear_vel,
autonomous motion (the simulator-only sim/autonomous/cmd_vel, used by Nav2
through nav_bridge and by command_server) at drive.max_linear_vel_auto, and
both at drive.max_linear_vel_towing while a shelf is docked.

By default it starts scripts/fake_sim.py with a shelf over the spawn pose.
Runs inside the ROS 2 image (CI does this):

    docker run --rm -v "$PWD:/repo" kachaka-isaac-sim-ros:jazzy python3 /repo/tests/speed_limit_check.py

With --external it checks a simulator that is already running in the room
with a shelf over the robot and ~1.5 m free ahead, e.g.

    scripts/run_sim.py --headless --shelf 0 0 0 --no-people
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav2_config_check import Stack

ROOT = Path(__file__).resolve().parents[1]
NS = "/kachaka"
TOLERANCE = 0.03  # m/s
DRIVE = yaml.safe_load((ROOT / "config/robot.yaml").read_text())["drive"]
MANUAL = DRIVE["max_linear_vel"]
AUTO = DRIVE["max_linear_vel_auto"]
TOWING = DRIVE["max_linear_vel_towing"]

failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)
    if not ok:
        failures.append(name)


class SpeedCheck(Node):
    def __init__(self):
        super().__init__("speed_limit_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gt: Odometry | None = None
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", lambda m: setattr(self, "gt", m),
                                 qos_profile_sensor_data)
        self.pubs = {name: self.create_publisher(Twist, f"{NS}/{topic}", 10)
                     for name, topic in (("manual", "manual_control/cmd_vel"),
                                         ("autonomous", "sim/autonomous/cmd_vel"))}

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_ready(self, timeout=60.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and (self.gt is None or self.sim_now() <= 0.0):
            self.spin_for(0.1)
        return self.gt is not None

    def dock(self, on: bool) -> tuple[bool, str]:
        client = self.create_client(SetBool, f"{NS}/sim/shelf_dock")
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return False, "no sim/shelf_dock"
            future = client.call_async(SetBool.Request(data=on))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            res = future.result()
            return bool(res and res.success), res.message if res else "no answer"
        finally:
            self.destroy_client(client)

    def top_speed(self, source: str, v: float, sec: float) -> float:
        """Command `v` m/s for `sec` sim seconds; the fastest ground-truth
        speed in the second half (signed like `v`)."""
        start = self.sim_now()
        best = 0.0
        cmd = Twist()
        cmd.linear.x = float(v)
        while self.sim_now() < start + sec:
            self.pubs[source].publish(cmd)
            self.spin_for(0.05)
            if self.sim_now() > start + sec / 2 and self.gt is not None:
                speed = self.gt.twist.twist.linear.x
                if abs(speed) > abs(best):
                    best = speed
        stop_end = self.sim_now() + 1.0
        while self.sim_now() < stop_end:
            self.pubs[source].publish(Twist())
            self.spin_for(0.05)
        return best


def run_checks(n: SpeedCheck) -> None:
    if not n.wait_ready():
        check("simulator ready", False, "no sim/ground_truth")
        return
    ok, msg = n.dock(True)
    check("dock the shelf over the robot", ok, msg)
    if ok:
        v = n.top_speed("autonomous", 1.0, 1.2)
        check(f"towing, autonomous: {TOWING} m/s", abs(v - TOWING) < TOLERANCE, f"v = {v:.3f} m/s for 1.0 m/s")
        v = n.top_speed("manual", -1.0, 1.0)
        check(f"towing, manual: {min(MANUAL, TOWING)} m/s", abs(v + min(MANUAL, TOWING)) < TOLERANCE,
              f"v = {v:.3f} m/s for -1.0 m/s")
        ok, msg = n.dock(False)
        check("undock", ok, msg)
    v = n.top_speed("autonomous", 1.0, 1.2)
    check(f"alone, autonomous: {AUTO} m/s", abs(v - AUTO) < TOLERANCE, f"v = {v:.3f} m/s for 1.0 m/s")
    v = n.top_speed("manual", -1.0, 1.0)
    check(f"alone, manual: {MANUAL} m/s", abs(v + MANUAL) < TOLERANCE, f"v = {v:.3f} m/s for -1.0 m/s")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--external", action="store_true", help="check a running simulator instead of fake_sim")
    args = p.parse_args()
    rclpy.init()
    stack = None
    try:
        if not args.external:
            log_dir = Path(os.environ.get("CHECK_LOG_DIR") or tempfile.mkdtemp(prefix="speed_limit_check_"))
            log_dir.mkdir(parents=True, exist_ok=True)
            stack = Stack(log_dir)
            stack.start("fake_sim", sys.executable, str(ROOT / "scripts/fake_sim.py"), "--seed", "0",
                        "--api-port", "0", "--no-people", "--no-charger", "--shelf", "0", "0", "0")
        node = SpeedCheck()
        run_checks(node)
        node.destroy_node()
        if failures and stack:
            stack.tail()
    finally:
        if stack:
            stack.stop()
        rclpy.try_shutdown()
    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
