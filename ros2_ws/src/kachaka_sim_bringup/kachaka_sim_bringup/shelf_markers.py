"""Draw the Kachaka Shelves in RViz as a MarkerArray on /shelf_markers.

robot_description stays the robot alone, as on the real Kachaka. Shelves
standing somewhere are drawn in the map frame from layout/shelves/list; the
shelf being towed is drawn in the robot's base_footprint frame, so it moves
with the robot between the 2 Hz shelf list updates.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from kachaka_interfaces.msg import ShelfList
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

# 3-layer Kachaka Shelf, as built in kachaka_isaac_sim/world.py. x is the
# shelf's depth (the robot drives in along it), y its width.
PLATES = ((0.104, 0.121), (0.435, 0.452), (0.649, 0.666))  # z ranges
LEG_INSET = (0.033, 0.049)  # leg centres from the plate edges (x, y)
LEG_SIZE = (0.012, 0.046)
CASTER_RADIUS = 0.03
PARTS = len(PLATES) + 4 + 4 + 1  # plates, legs, casters, label

BODY = ColorRGBA(r=0.25, g=0.26, b=0.28, a=0.9)
CASTER = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.9)
TOWED = ColorRGBA(r=0.2, g=0.45, b=0.75, a=0.9)
LABEL = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def _pose(x, y, yaw) -> Pose:
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def shelf_markers(index: int, name: str, frame: str, pose: Pose, depth: float, width: float,
                  towed: bool) -> list[Marker]:
    """The markers of one shelf standing at `pose` in `frame`."""
    markers = []

    def add(kind, x, y, z, sx, sy, sz, color):
        m = Marker()
        m.header.frame_id = frame  # stamp 0: RViz draws it with the latest TF
        m.ns = "shelves"
        m.id = index * PARTS + len(markers)
        m.type = kind
        m.action = Marker.ADD
        m.frame_locked = True
        # (x, y) is the part's offset in the shelf frame.
        yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
        m.pose.position.x = pose.position.x + x * math.cos(yaw) - y * math.sin(yaw)
        m.pose.position.y = pose.position.y + x * math.sin(yaw) + y * math.cos(yaw)
        m.pose.position.z = z
        m.pose.orientation = pose.orientation
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = color
        markers.append(m)
        return m

    body = TOWED if towed else BODY
    for z0, z1 in PLATES:
        add(Marker.CUBE, 0, 0, (z0 + z1) / 2, depth, width, z1 - z0, body)
    lx, ly = depth / 2 - LEG_INSET[0], width / 2 - LEG_INSET[1]
    z0, z1 = 2 * CASTER_RADIUS, PLATES[-1][1]
    corners = ((1, 1), (1, -1), (-1, 1), (-1, -1))
    for sx, sy in corners:
        add(Marker.CUBE, sx * lx, sy * ly, (z0 + z1) / 2, *LEG_SIZE, z1 - z0, body)
    for sx, sy in corners:
        d = 2 * CASTER_RADIUS
        add(Marker.SPHERE, sx * lx, sy * ly, CASTER_RADIUS, d, d, d, CASTER)
    label = add(Marker.TEXT_VIEW_FACING, 0, 0, PLATES[-1][1] + 0.12, 0, 0, 0.08, LABEL)
    label.text = name
    return markers


class ShelfMarkers(Node):
    def __init__(self):
        super().__init__("shelf_markers")
        prefix = self.declare_parameter("prefix", "/kachaka").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.shelves = None  # ShelfList
        self.order = []  # shelf ids in sim/layout order (the order of sim/shelf_detection)
        self.docked = ""
        self.towed_pose = None  # the towed shelf in the robot frame
        self.pub = self.create_publisher(MarkerArray, "shelf_markers", LATCHED)
        self.create_subscription(ShelfList, f"{prefix}/layout/shelves/list", self._on_shelves, LATCHED)
        self.create_subscription(String, f"{prefix}/sim/layout", self._on_layout, LATCHED)
        self.create_subscription(String, f"{prefix}/sim/shelf_docked", self._on_docked, LATCHED)
        self.create_subscription(PoseArray, f"{prefix}/sim/shelf_detection", self._on_detection,
                                 qos_profile_sensor_data)

    def _on_layout(self, msg: String) -> None:
        self.order = [s["id"] for s in json.loads(msg.data).get("shelves", [])]

    def _on_shelves(self, msg: ShelfList) -> None:
        self.shelves = msg
        self._publish()

    def _on_docked(self, msg: String) -> None:
        self.docked = msg.data
        self.towed_pose = None
        self._publish()

    def _on_detection(self, msg: PoseArray) -> None:
        if self.docked and self.docked in self.order:
            i = self.order.index(self.docked)
            if i < len(msg.poses):
                self.towed_pose = msg.poses[i]
                self._publish()

    def _publish(self) -> None:
        """All shelves, every time, so the latched message is complete."""
        if self.shelves is None:
            return
        out = MarkerArray()
        for i, s in enumerate(self.shelves.shelves):
            if s.id == self.docked and self.towed_pose is not None:
                frame, pose = self.base_frame, self.towed_pose
            else:
                frame, pose = self.map_frame, _pose(s.pose.x, s.pose.y, s.pose.theta)
            out.markers += shelf_markers(i, s.name or s.id, frame, pose, s.size.depth, s.size.width,
                                         s.id == self.docked)
        if out.markers:
            self.pub.publish(out)


def main():
    rclpy.init()
    node = ShelfMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
