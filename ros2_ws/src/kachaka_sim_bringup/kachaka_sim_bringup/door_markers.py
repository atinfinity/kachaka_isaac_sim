"""Draw the doors in RViz as a MarkerArray on /door_markers.

Reads the simulator's door state on sim/doors (simulator only; latched, and
every 0.1 s while a panel moves) and draws each door's panel where it is in
the map frame, with its id and state. The doorway's walls are already in the
map, as the LiDAR sees them.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

# Kept apart from the LaserScan (red), the plan (blue) and the footprint
# (green) in config/*.rviz: a closed door is violet, a moving one yellow and
# an open one faint grey, out of the way like the panel itself.
OPEN = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.35)
CLOSED = ColorRGBA(r=0.6, g=0.35, b=0.95, a=0.85)
MOVING = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.85)
LABEL = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
COLORS = {"open": OPEN, "closed": CLOSED}
PARTS = 2  # panel, label


def _marker(frame: str, mid: int, kind: int, pose, color: ColorRGBA) -> Marker:
    x, y, yaw = pose
    k = Marker()
    k.header.frame_id = frame
    k.ns = "doors"
    k.id = mid
    k.type = kind
    k.action = Marker.ADD
    k.pose.position.x, k.pose.position.y = float(x), float(y)
    k.pose.orientation.z, k.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    k.color = color
    return k


def door_markers(msg: dict, frame: str) -> list[Marker]:
    """Markers for a sim/doors message: each door's panel and a label."""
    markers = []
    for i, d in enumerate(msg.get("doors", [])):
        panel = d.get("panel")
        if panel is None:
            continue
        length, thickness, height = panel["size"]
        body = _marker(frame, i * PARTS, Marker.CUBE, panel["pose"], COLORS.get(d["state"], MOVING))
        body.pose.position.z = height / 2
        body.scale.x, body.scale.y, body.scale.z = float(length), float(thickness), float(height)
        label = _marker(frame, i * PARTS + 1, Marker.TEXT_VIEW_FACING, d["pose"], LABEL)
        label.pose.position.z = height + 0.15
        label.scale.z = 0.12
        label.text = f"{d['id']} {d['state']}"
        markers += [body, label]
    return markers


class DoorMarkers(Node):
    def __init__(self):
        super().__init__("door_markers")
        prefix = self.declare_parameter("prefix", "/kachaka").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.pub = self.create_publisher(MarkerArray, "door_markers", LATCHED)
        self.create_subscription(String, f"{prefix}/sim/doors", self._on_doors, LATCHED)

    def _on_doors(self, msg: String) -> None:
        markers = door_markers(json.loads(msg.data), self.map_frame)
        if markers:
            self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = DoorMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
