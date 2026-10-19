"""Put the simulator back to its start state before a check, through the
sim-only service /kachaka/sim/reset (std_srvs/SetBool, data = also clear the
SLAM map): robot at the spawn pose, shelves home, battery and odometry reset,
commands dropped.

    from sim_reset import reset_sim, add_reset_args
    ...
    if not args.no_reset:
        reset_sim(node)

As a script it resets once: python3 tests/sim_reset.py [--clear-map]
"""

import argparse
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SETTLE = 1.5  # s; the simulator ignores cmd_vel for 1 s after a reset


def add_reset_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--no-reset", action="store_true", help="do not reset the simulator first")
    ap.add_argument("--clear-map", action="store_true", help="also clear the SLAM map when resetting")


def reset_sim(node, clear_map: bool = False, ns: str = "/kachaka", timeout: float = 10.0) -> bool:
    """Reset and wait until command_server (if it runs) has dropped its
    commands. False if the simulator has no reset service."""
    client = node.create_client(SetBool, f"{ns}/sim/reset")
    states = []
    sub = node.create_subscription(String, f"{ns}/sim/command/state", lambda m: states.append(json.loads(m.data)),
                                   LATCHED)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            print("[reset] sim/reset not available; running from the current state", flush=True)
            return False
        future = client.call_async(SetBool.Request(data=clear_map))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        res = future.result()
        if res is None or not res.success:
            print(f"[reset] failed: {res.message if res else 'no answer'}", flush=True)
            return False
        reset_id = json.loads(res.message)["id"]
        # command_server answers with "reset": id on its state topic. Without
        # it (no Nav2 side) there is nothing to wait for.
        end = time.monotonic() + 5.0
        while time.monotonic() < end and not any(s.get("reset") == reset_id for s in states):
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + SETTLE
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"[reset] simulator reset (#{reset_id}{', map cleared' if clear_map else ''})", flush=True)
        return True
    finally:
        node.destroy_subscription(sub)
        node.destroy_client(client)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clear-map", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("sim_reset")
    ok = reset_sim(node, args.clear_map)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
