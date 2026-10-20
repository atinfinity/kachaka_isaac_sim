"""Run tests/api_check.py without a GPU: scripts/fake_sim.py stands in for
Isaac Sim, with SLAM, Nav2 and the command server as for the simulator.
Runs inside the ROS 2 image (CI does this):

    docker run --rm -v "$PWD:/repo" kachaka-isaac-sim-ros:jazzy python3 /repo/tests/api_fake_check.py

Needs build/kachaka.urdf (scripts/build_usd.py --urdf-only). Logs go to
CHECK_LOG_DIR (default: a temporary directory); FAKE_SIM_ARGS adds options
to scripts/fake_sim.py (e.g. --trace).
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav2_config_check import SLAM_MANAGERS, STARTUP_TIMEOUT, Checker, Stack

ROOT = Path(__file__).resolve().parents[1]
CHECK_TIMEOUT = 900.0


def main() -> int:
    log_dir = Path(os.environ.get("CHECK_LOG_DIR") or tempfile.mkdtemp(prefix="api_fake_check_"))
    log_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    stack = Stack(log_dir)
    code = 1
    try:
        stack.start("fake_sim", sys.executable, str(ROOT / "scripts/fake_sim.py"), "--seed", "0",
                    *os.environ.get("FAKE_SIM_ARGS", "").split())
        stack.start("slam", "ros2", "launch", "kachaka_sim_bringup", "slam.launch.py", "nav:=true", "rviz:=false")
        node = Checker("api_fake_check")
        ready = node.wait_for("/map", lambda: node.map is not None, STARTUP_TIMEOUT, stack)
        for manager in SLAM_MANAGERS:
            ready = ready and node.wait_for(manager, lambda m=manager: node.is_active(m), STARTUP_TIMEOUT, stack)
        node.destroy_node()
        if not ready:
            print("the stack did not come up", flush=True)
        else:
            print("== tests/api_check.py --no-cameras", flush=True)
            try:
                code = subprocess.run([sys.executable, str(ROOT / "tests/api_check.py"), "--no-cameras"],
                                      timeout=CHECK_TIMEOUT, check=False).returncode
            except subprocess.TimeoutExpired:
                print(f"api_check.py timed out after {CHECK_TIMEOUT:.0f} s", flush=True)
        if code != 0:
            stack.tail()
    finally:
        stack.stop()
        rclpy.try_shutdown()
    print(f"logs in {log_dir}")
    return code


if __name__ == "__main__":
    sys.exit(main())
