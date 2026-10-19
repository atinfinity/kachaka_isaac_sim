"""Check the step (cliff) sensors and collision reporting over ROS 2.

Needs the simulator in the default room with a drop east of the spawn pose:

    scripts/run_sim.sh --headless --drop 1.0 0.0 0.4 0.8

Run in the ROS 2 container (no Nav2 needed):

    python3 /tests/ros_hazard_check.py

1. Driving east, the step sensors see the drop: the robot stops before its
   casters reach the edge and pauses with errors 21051 and 21052
   (sim/errors, the codes GetError lists). Velocity commands do nothing.
2. sim/power_button resumes. Forward motion stays blocked over the edge;
   backing up works, and driving to the edge again pauses again.
3. With the front stop off, driving into the north wall is one collision on
   sim/collision; the robot keeps going (collision.stop is off).
4. After sim/reset, with the front stop on, the robot stops short of the
   wall: no collision.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sim_reset import LATCHED, add_reset_args, reset_sim
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

NS = "/kachaka"
EDGE_X = 0.8  # west edge of the drop (--drop 1.0 0.0 0.4 0.8)
CASTER_X = 0.19  # front casters, base_footprint frame
WALL_N = 2.0  # north wall, inner face
FRONT_X = 0.23  # front of the body


class Check(Node):
    def __init__(self):
        super().__init__("hazard_check")
        self.cliff = None
        self.errors = None
        self.collisions = []
        self.x = self.y = self.yaw = None
        self.create_subscription(String, f"{NS}/sim/cliff", lambda m: setattr(self, "cliff", json.loads(m.data)),
                                 LATCHED)
        self.create_subscription(String, f"{NS}/sim/errors", lambda m: setattr(self, "errors", json.loads(m.data)),
                                 LATCHED)
        self.create_subscription(String, f"{NS}/sim/collision", lambda m: self.collisions.append(json.loads(m.data)),
                                 10)
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", self._on_gt, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10)
        self.power = self.create_client(Trigger, f"{NS}/sim/power_button")
        self.front_stop = self.create_client(SetBool, f"{NS}/sim/front_stop/set_enabled")

    def _on_gt(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def spin_for(self, seconds: float, v: float | None = None, w: float = 0.0, until=None) -> bool:
        """Spin (sending v, w if v is given) for `seconds` or until `until()`."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if until is not None and until():
                return True
            if v is not None:
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        return until is not None and until()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()


def main() -> int:
    ap = argparse.ArgumentParser()
    add_reset_args(ap)
    args = ap.parse_args()
    rclpy.init()
    n = Check()
    if not args.no_reset:
        reset_sim(n, args.clear_map)
    failures = []

    def check(name, ok, detail=""):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)
        if not ok:
            failures.append(name)

    n.spin_for(1.0, until=lambda: n.x is not None and n.cliff is not None and n.errors is not None)
    check("start: on the floor, no errors", n.cliff is not None and not n.cliff["paused"] and n.errors == [],
          f"cliff {n.cliff}, errors {n.errors}")

    # 1. Drive to the drop.
    paused = n.spin_for(6.0, v=0.3, until=lambda: n.cliff and n.cliff["paused"])
    n.spin_for(0.5)
    check("drop: paused", paused, f"cliff {n.cliff}")
    check("drop: errors 21051, 21052", sorted(n.errors or []) == [21051, 21052], str(n.errors))
    x_stop = n.x
    check("drop: casters stopped before the edge", x_stop is not None and x_stop + CASTER_X < EDGE_X - 0.005,
          f"base x {x_stop:.3f}, casters at {x_stop + CASTER_X:.3f}, edge {EDGE_X}")
    n.spin_for(1.5, v=0.3)
    check("paused: velocity commands ignored", abs(n.x - x_stop) < 0.01, f"moved {n.x - x_stop:+.3f} m")

    # 2. Power button.
    res = n.call(n.power, Trigger.Request())
    n.spin_for(0.5)
    check("power button resumes", res is not None and res.success and n.errors == [] and not n.cliff["paused"],
          f"{res.message if res else 'no answer'}, errors {n.errors}")
    x0 = n.x
    n.spin_for(1.5, v=0.3)
    check("over the edge: forward blocked", n.x - x0 < 0.01 and not n.cliff["paused"], f"moved {n.x - x0:+.3f} m")
    n.spin_for(1.5, v=-0.2)
    check("backing up allowed", x0 - n.x > 0.15 and n.errors == [] and not any(n.cliff["over"]),
          f"moved {n.x - x0:+.3f} m, cliff {n.cliff}")
    again = n.spin_for(4.0, v=0.3, until=lambda: n.cliff["paused"])
    n.spin_for(0.3)
    check("back at the edge: paused again", again and sorted(n.errors) == [21051, 21052], str(n.errors))
    n.call(n.power, Trigger.Request())

    # 3. Collision with the front stop off.
    reset_sim(n)
    n.call(n.front_stop, SetBool.Request(data=False))
    n.collisions.clear()
    n.spin_for(5.0, v=0.0, w=0.5, until=lambda: n.yaw is not None and n.yaw > math.pi / 2 - 0.08)
    n.spin_for(1.0, v=0.0)
    turned = n.yaw
    hit = n.spin_for(12.0, v=0.3, until=lambda: n.collisions)
    n.spin_for(2.0, v=0.3)  # keep pushing: still one collision
    n.spin_for(0.5)
    c = n.collisions
    check("turned north", abs(turned - math.pi / 2) < 0.25, f"yaw {turned:.2f}")
    check("wall: one collision", hit and len(c) == 1 and c[0]["count"] == 1 and c[0]["with"].endswith("wall_n"),
          json.dumps(c))
    check("wall: no stop, no error (collision.stop off)", n.errors == [] and not n.cliff["paused"],
          f"errors {n.errors}")
    check("collision pose at the wall", bool(c) and abs(c[0]["pose"][1] + FRONT_X - WALL_N) < 0.05,
          f"pose {c[0]['pose'] if c else None}")

    # 4. Front stop on (sim/reset restores it): no collision.
    reset_sim(n)
    n.collisions.clear()
    n.spin_for(5.0, v=0.0, w=0.5, until=lambda: n.yaw is not None and n.yaw > math.pi / 2 - 0.08)
    n.spin_for(1.0, v=0.0)
    n.spin_for(10.0, v=0.3)
    check("front stop: stops short, no collision", not n.collisions and WALL_N - (n.y + FRONT_X) > 0.0,
          f"gap {WALL_N - (n.y + FRONT_X):.3f} m, {len(n.collisions)} collisions")

    reset_sim(n)
    n.destroy_node()
    rclpy.shutdown()
    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
