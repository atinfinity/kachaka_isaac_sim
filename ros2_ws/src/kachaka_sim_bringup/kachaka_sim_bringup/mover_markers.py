"""Draw the moving people and boxes in RViz as a MarkerArray on /mover_markers.

Reads the simulator's ground truth on sim/movers (simulator only; the real
robot has no such topic) and draws each mover that is there in the map
frame: a person as an upright elliptic cylinder, a box as a box, with its
name and, while it moves, an arrow to where it will be in a second. A mover
that waits for the robot is drawn red.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

PERSON = ColorRGBA(r=0.95, g=0.6, b=0.2, a=0.8)
BOX = ColorRGBA(r=0.7, g=0.55, b=0.35, a=0.8)
BLOCKED = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.8)
ARROW = ColorRGBA(r=0.2, g=0.8, b=0.9, a=0.9)
LABEL = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
ARROW_SECONDS = 1.0  # the arrow shows the distance covered in this time
PARTS = 3  # body, label, arrow


def _marker(frame: str, mid: int, kind: int, x: float, y: float, yaw: float, color: ColorRGBA) -> Marker:
    k = Marker()
    k.header.frame_id = frame
    k.ns = "movers"
    k.id = mid
    k.type = kind
    k.action = Marker.ADD
    k.pose.position.x, k.pose.position.y = float(x), float(y)
    k.pose.orientation.z, k.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    k.color = color
    return k


def mover_markers(msg: dict, frame: str) -> list[Marker]:
    """Markers for a sim/movers message: DELETEALL first, so a mover that has
    gone away disappears, then the movers that are there."""
    clear = Marker()
    clear.header.frame_id = frame
    clear.action = Marker.DELETEALL
    markers = [clear]
    for i, m in enumerate(msg.get("movers", [])):
        if not m.get("present") or m.get("map_pose") is None:
            continue
        x, y, yaw = m["map_pose"]
        sx, sy, sz = m["size"]
        person = m["kind"] == "person"
        body = _marker(frame, i * PARTS, Marker.CYLINDER if person else Marker.CUBE, x, y, yaw,
                       BLOCKED if m.get("blocked") else PERSON if person else BOX)
        body.pose.position.z = sz / 2
        body.scale.x, body.scale.y, body.scale.z = float(sx), float(sy), float(sz)
        label = _marker(frame, i * PARTS + 1, Marker.TEXT_VIEW_FACING, x, y, yaw, LABEL)
        label.pose.position.z = sz + 0.15
        label.scale.z = 0.12
        label.text = m["id"]
        markers += [body, label]
        # Velocity in the world frame, turned into the map frame.
        vx, vy = m["velocity"]
        turn = yaw - m["pose"][2]
        c, s = math.cos(turn), math.sin(turn)
        mvx, mvy = c * vx - s * vy, s * vx + c * vy
        if math.hypot(mvx, mvy) < 1e-3:
            continue
        arrow = _marker(frame, i * PARTS + 2, Marker.ARROW, 0.0, 0.0, 0.0, ARROW)  # drawn from its points
        z = 0.05
        arrow.points = [Point(x=float(x), y=float(y), z=z),
                        Point(x=float(x + mvx * ARROW_SECONDS), y=float(y + mvy * ARROW_SECONDS), z=z)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.04, 0.08, 0.1
        markers.append(arrow)
    return markers


class MoverMarkers(Node):
    def __init__(self):
        super().__init__("mover_markers")
        prefix = self.declare_parameter("prefix", "/kachaka").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.pub = self.create_publisher(MarkerArray, "mover_markers", 10)
        self.create_subscription(String, f"{prefix}/sim/movers", self._on_movers, 10)

    def _on_movers(self, msg: String) -> None:
        self.pub.publish(MarkerArray(markers=mover_markers(json.loads(msg.data), self.map_frame)))


def main():
    rclpy.init()
    node = MoverMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
