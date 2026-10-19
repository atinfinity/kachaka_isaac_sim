"""Check simulated object detection on the real bridge's ROS 2 topic,
/kachaka/object_detection/result (kachaka_interfaces/ObjectDetectionListStamped).

Needs the simulator in the default room and kachaka_sim_bringup's
object_detection_relay (bridge.launch.py, included by slam.launch.py and
navigation.launch.py). Run in the ROS 2 container:

    python3 /tests/ros_detection_check.py

At the spawn pose the front camera sees the shelf and a person half behind a
box. The robot then turns once in place and must see people, the shelf and
the charger, with sane boxes, scores and distances.
"""

import argparse
import itertools
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from kachaka_interfaces.msg import ObjectDetection, ObjectDetectionListStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sim_reset import add_reset_args, reset_sim

NS = "/kachaka"
WIDTH, HEIGHT = 640, 480  # config/robot.yaml cameras.front
NAMES = {ObjectDetection.PERSON: "PERSON", ObjectDetection.SHELF: "SHELF", ObjectDetection.CHARGER: "CHARGER",
         ObjectDetection.DOOR: "DOOR"}
TURN_RATE = 1.0  # rad/s


class Check(Node):
    def __init__(self):
        super().__init__("detection_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.msgs: list[tuple[float, ObjectDetectionListStamped]] = []
        self.create_subscription(ObjectDetectionListStamped, f"{NS}/object_detection/result",
                                 lambda m: self.msgs.append((time.monotonic(), m)), qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, f"{NS}/manual_control/cmd_vel", 10)

    def spin_for(self, seconds: float, turn: float = 0.0) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if turn:
                msg = Twist()
                msg.angular.z = turn
                self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


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

    def labels(msgs):
        return {NAMES.get(d.label, d.label) for _, m in msgs for d in m.detection}

    n.spin_for(3.0)
    still = list(n.msgs)
    rate = (len(still) - 1) / (still[-1][0] - still[0][0]) if len(still) > 1 else 0.0
    check("result topic", len(still) > 5, f"{len(still)} messages, {rate:.1f} Hz (wall)")
    check("frame", bool(still) and still[-1][1].header.frame_id == "camera_front",
          still[-1][1].header.frame_id if still else "")
    check("at spawn: shelf and person", {"SHELF", "PERSON"} <= labels(still), str(sorted(labels(still))))

    n.msgs.clear()
    n.spin_for(2 * math.pi / TURN_RATE + 1.0, turn=TURN_RATE)
    n.spin_for(0.5)
    turned = list(n.msgs)
    check("turning once: people, shelf and charger", {"PERSON", "SHELF", "CHARGER"} <= labels(turned),
          str(sorted(labels(turned))))
    dets = [d for _, m in turned + still for d in m.detection]
    bad_roi = [d.roi for d in dets if d.roi.width < 1 or d.roi.height < 1
               or d.roi.x_offset + d.roi.width > WIDTH or d.roi.y_offset + d.roi.height > HEIGHT]
    check("boxes inside the image", not bad_roi, f"{len(dets)} detections, {len(bad_roi)} bad")
    check("scores in [0, 1]", all(0.0 <= d.score <= 1.0 for d in dets))
    dist = [d.distance_median for d in dets]
    check("distances", bool(dist) and all(0.0 < x <= 5.0 for x in dist),
          f"{min(dist):.2f}..{max(dist):.2f} m" if dist else "")
    stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for _, m in turned]
    check("stamps increase", all(b > a for a, b in itertools.pairwise(stamps)))

    n.destroy_node()
    rclpy.shutdown()
    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
