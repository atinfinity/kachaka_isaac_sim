"""Front obstacle stop demo for scripts/record_demo.sh front_stop (room world,
robot at the origin facing the east wall).

Drives at the wall with the stop on (halts short of it, backing up still
works), then with the stop off (drives into it), and switches it back on.
Writes captions (wall_time,text) to the CSV given as the first argument.

    python3 tests/ros_front_stop_demo.py /build/front_stop/captions.csv
"""

import csv
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool

NS = "/kachaka"
EAST_WALL = 2.5  # m, inner face of the room's east wall
FRONT_X = 0.23  # m, config/robot.yaml front_stop.front_x


class Demo(Node):
    def __init__(self, captions):
        super().__init__("front_stop_demo", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.captions = captions
        with open(captions, "w", newline="") as f:
            csv.writer(f).writerow(["wall_time", "text"])
        self.gt = None
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", lambda m: setattr(self, "gt", m),
                                 qos_profile_sensor_data)
        self.state = {}
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{NS}/sim/front_stop/state",
                                 lambda m: setattr(self, "state", json.loads(m.data)), latched)
        self.cmd = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10)
        self.client = self.create_client(SetBool, f"{NS}/sim/front_stop/set_enabled")

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

    def drive(self, sec, v=0.0, w=0.0, until_blocked=None):
        """Send cmd_vel for `sec` of sim time; call `until_blocked` once when the stop kicks in."""
        end = self.sim_now() + sec
        while self.sim_now() < end:
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.cmd.publish(m)
            self.spin_for(0.05)
            if until_blocked and self.state.get("blocked"):
                until_blocked()
                until_blocked = None

    def set_enabled(self, on):
        self.client.wait_for_service(timeout_sec=5.0)
        future = self.client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def gap(self):
        return EAST_WALL - self.gt.pose.pose.position.x - FRONT_X


def main():
    rclpy.init()
    n = Demo(sys.argv[1])
    while n.gt is None or not n.state:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.spin_for(1.0)

    n.say("Front obstacle stop ON (default): cmd_vel 0.3 m/s toward the wall")
    n.drive(10.0, 0.3, until_blocked=lambda: n.say("ON: blocked - forward motion stops, cmd_vel still 0.3 m/s"))
    n.say(f"ON: stopped {n.gap() * 100:.0f} cm short of the wall")
    n.drive(1.5, 0.3)
    n.say("ON: backing up and turning stay allowed")
    n.drive(2.0, -0.2)
    n.drive(2.0, 0.0, 0.8)
    n.drive(2.0, 0.0, -0.8)
    n.drive(0.5)

    n.say("ros2 service call /kachaka/sim/front_stop/set_enabled  {data: false}")
    n.set_enabled(False)
    n.spin_for(1.0)
    n.say("Front obstacle stop OFF: cmd_vel 0.2 m/s toward the wall")
    n.drive(4.5, 0.2)
    n.say("OFF: drives on until the body hits the wall")
    n.drive(1.5, 0.2)
    n.drive(2.0, -0.2)
    n.drive(0.5)

    n.say("ros2 service call /kachaka/sim/front_stop/set_enabled  {data: true}")
    n.set_enabled(True)
    n.spin_for(1.0)
    n.say("Front obstacle stop ON again: cmd_vel 0.3 m/s")
    n.drive(5.0, 0.3)
    n.say(f"ON: stopped {n.gap() * 100:.0f} cm short of the wall")
    n.drive(1.5, 0.3)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
