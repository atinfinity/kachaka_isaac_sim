"""Speed limit demo for scripts/record_demo.sh speed (room world, robot and a
shelf at the west end: --x -1.8 --shelf -1.8 0 0 --no-charger --no-people).

Docks the shelf, sends 1.0 m/s as a manual command (manual_control/cmd_vel)
and as autonomous motion (sim/autonomous/cmd_vel, what Nav2 and the command
server use), undocks and does the same alone. Captions show the speed each
reaches. Writes captions (wall_time,text) to the CSV given as the first
argument.

    python3 tests/ros_speed_demo.py /build/speed/captions.csv
"""

import csv
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool

NS = "/kachaka"


class Demo(Node):
    def __init__(self, captions):
        super().__init__("speed_demo", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.captions = captions
        with open(captions, "w", newline="") as f:
            csv.writer(f).writerow(["wall_time", "text"])
        self.gt = None
        self.create_subscription(Odometry, f"{NS}/sim/ground_truth", lambda m: setattr(self, "gt", m),
                                 qos_profile_sensor_data)
        self.pubs = {"manual": self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10),
                     "autonomous": self.create_publisher(Twist, f"{NS}/sim/autonomous/cmd_vel", 10)}
        self.dock_client = self.create_client(SetBool, f"{NS}/sim/shelf_dock")

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

    def drive(self, source, v, sec):
        """Command `v` m/s for `sec` sim seconds; return the top speed reached."""
        end = self.sim_now() + sec
        top = 0.0
        cmd = Twist()
        cmd.linear.x = float(v)
        while self.sim_now() < end:
            self.pubs[source].publish(cmd)
            self.spin_for(0.05)
            top = max(top, self.gt.twist.twist.linear.x)
        return top

    def stop(self, sec=1.0):
        end = self.sim_now() + sec
        while self.sim_now() < end:
            self.pubs["manual"].publish(Twist())
            self.spin_for(0.05)

    def dock(self, on):
        self.dock_client.wait_for_service(timeout_sec=5.0)
        future = self.dock_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        return bool(res and res.success)


def run(n, source, label, sec):
    topic = "manual_control/cmd_vel" if source == "manual" else "sim/autonomous/cmd_vel"
    n.say(f"{label}: 1.0 m/s on {topic}")
    top = n.drive(source, 1.0, sec)
    n.say(f"{label}: capped at {top:.2f} m/s")
    n.stop()


def main():
    rclpy.init()
    n = Demo(sys.argv[1])
    while n.gt is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.spin_for(1.0)
    n.say("Speed limits: manual 0.3 m/s; autonomous 0.8 m/s alone, 0.5 m/s towing")
    n.spin_for(2.0)
    n.say("Dock the shelf" + ("" if n.dock(True) else " (failed)"))
    n.spin_for(1.5)
    run(n, "manual", "Towing, manual", 1.5)
    run(n, "autonomous", "Towing, autonomous", 1.5)
    n.say("Undock" + ("" if n.dock(False) else " (failed)"))
    n.spin_for(1.5)
    run(n, "manual", "Alone, manual", 1.2)
    run(n, "autonomous", "Alone, autonomous", 1.5)
    n.spin_for(2.0)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
