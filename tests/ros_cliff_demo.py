"""Step sensor and collision demo for scripts/record_demo.sh cliff (room world
with a drop east of the spawn pose: --drop 1.0 0.0 0.4 0.8).

Drives at the drop (the robot pauses with errors 21051/21052 and ignores
commands), resumes with the power button (forward stays blocked, backing up
works), then turns north and drives into the wall with the front stop off
(one event on sim/collision). Writes captions (wall_time,text) to the CSV
given as the first argument.

    python3 tests/ros_cliff_demo.py /build/cliff/captions.csv
"""

import csv
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

NS = "/kachaka"


class Demo(Node):
    def __init__(self, captions):
        super().__init__("cliff_demo", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.captions = captions
        with open(captions, "w", newline="") as f:
            csv.writer(f).writerow(["wall_time", "text"])
        self.gt = None
        self.cliff = None
        self.errors = None
        self.collisions = []
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", lambda m: setattr(self, "gt", m),
                                 qos_profile_sensor_data)
        self.create_subscription(String, f"{NS}/sim/cliff", lambda m: setattr(self, "cliff", json.loads(m.data)),
                                 latched)
        self.create_subscription(String, f"{NS}/sim/errors", lambda m: setattr(self, "errors", json.loads(m.data)),
                                 latched)
        self.create_subscription(String, f"{NS}/sim/collision",
                                 lambda m: self.collisions.append(json.loads(m.data)), 10)
        self.cmd = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10)
        self.power = self.create_client(Trigger, f"{NS}/sim/power_button")
        self.front_stop = self.create_client(SetBool, f"{NS}/sim/front_stop/set_enabled")

    def say(self, text):
        print(text, flush=True)
        with open(self.captions, "a", newline="") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", text])

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, sec, v=0.0, w=0.0, until=None):
        """Send cmd_vel for `sec` of sim time, or until `until()`."""
        end = self.sim_now() + sec
        while self.sim_now() < end:
            if until is not None and until():
                return True
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.cmd.publish(m)
            self.spin_for(0.05)
        return until is not None and until()

    def call(self, client, request):
        client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def x(self):
        return self.gt.pose.pose.position.x

    def yaw(self):
        q = self.gt.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Demo(sys.argv[1])
    while n.gt is None or n.cliff is None or n.errors is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.spin_for(1.0)

    n.say("Step sensors: cmd_vel 0.3 m/s toward a 15 cm drop")
    n.drive(10.0, 0.3, until=lambda: n.cliff["paused"])
    n.spin_for(0.5)
    n.say(f"Drop seen: wheels stop, paused with errors {sorted(n.errors)}")
    n.drive(1.5)
    x0 = n.x()
    n.say("Paused: cmd_vel 0.3 m/s is ignored")
    n.drive(2.5, 0.3)
    n.say(f"Paused: moved {abs(n.x() - x0) * 100:.0f} cm")
    n.drive(1.0)

    n.say("ros2 service call /kachaka/sim/power_button std_srvs/srv/Trigger")
    n.call(n.power, Trigger.Request())
    n.spin_for(0.5)
    n.say(f"Resumed (errors {n.errors}): forward over the edge stays blocked")
    n.drive(2.5, 0.3)
    n.say("Backing up away from the edge is allowed")
    n.drive(2.0, -0.2)
    n.drive(0.5)

    n.say("Turn north toward the wall")
    n.drive(6.0, 0.0, 0.5, until=lambda: n.yaw() > math.pi / 2 - 0.08)
    n.drive(0.5)
    n.say("ros2 service call /kachaka/sim/front_stop/set_enabled  {data: false}")
    n.call(n.front_stop, SetBool.Request(data=False))
    n.collisions.clear()
    n.spin_for(0.5)
    n.say("Front stop OFF: cmd_vel 0.3 m/s into the north wall")
    n.drive(12.0, 0.3, until=lambda: n.collisions)
    c = n.collisions[0] if n.collisions else {}
    n.say(f"sim/collision: with {c.get('with', '?').split('/')[-1]}, count {c.get('count', 0)}")
    n.drive(2.0, 0.3)
    n.drive(2.5, -0.2)
    n.call(n.front_stop, SetBool.Request(data=True))
    n.say("Front stop back ON")
    n.drive(1.5)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
