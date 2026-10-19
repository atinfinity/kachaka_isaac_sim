"""Publish the simulator's object detections like the real robot's bridge.

The simulator publishes its detections as JSON on /kachaka/sim/object_detection
(std_msgs/String), because Isaac Sim's bundled rclpy cannot load
kachaka_interfaces. This node republishes them as
/kachaka/object_detection/result (kachaka_interfaces/ObjectDetectionListStamped,
SensorDataQoS), the topic and type of the real kachaka-api ROS 2 bridge.

The JSON carries kachaka-api ObjectLabel values (PERSON = 1, ...); the ROS
message numbers its labels from 0 (PERSON = 0, ...), so they are converted the
way the real bridge does. A real Kachaka publishes the result topic itself and
never the sim topic, so running this node with one does nothing.
"""

import json

import rclpy
from kachaka_interfaces.msg import ObjectDetection, ObjectDetectionListStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# kachaka-api ObjectLabel -> kachaka_interfaces ObjectDetection label.
API_TO_ROS_LABEL = {1: ObjectDetection.PERSON, 2: ObjectDetection.SHELF, 3: ObjectDetection.CHARGER,
                    4: ObjectDetection.DOOR}


def to_msg(data: dict) -> ObjectDetectionListStamped:
    msg = ObjectDetectionListStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = (int(v) for v in data["stamp"])
    msg.header.frame_id = data["frame_id"]
    for o in data["objects"]:
        label = API_TO_ROS_LABEL.get(o["label"])
        if label is None:  # OBJECT_LABEL_UNSPECIFIED; the real bridge drops it too
            continue
        d = ObjectDetection()
        d.label = label
        d.roi.x_offset, d.roi.y_offset, d.roi.width, d.roi.height = (int(v) for v in o["roi"])
        d.score = float(o["score"])
        d.distance_median = float(o["distance"])
        msg.detection.append(d)
    return msg


class ObjectDetectionRelay(Node):
    def __init__(self):
        super().__init__("object_detection_relay")
        ns = self.declare_parameter("kachaka_namespace", "kachaka").value.strip("/")
        prefix = f"/{ns}" if ns else ""
        self.pub = self.create_publisher(ObjectDetectionListStamped, f"{prefix}/object_detection/result",
                                         qos_profile_sensor_data)
        self.create_subscription(String, f"{prefix}/sim/object_detection", self._on_json, qos_profile_sensor_data)

    def _on_json(self, msg: String) -> None:
        try:
            self.pub.publish(to_msg(json.loads(msg.data)))
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f"bad sim/object_detection message: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = ObjectDetectionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
