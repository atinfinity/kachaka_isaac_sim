"""Relay /map to the global costmap with the cells under the robot set free.

The SLAM map keeps a shelf's legs from when the shelf stood still. After the
robot docks under it, those cells lie inside the towing footprint, the
inflation around them covers the robot's own cell and NavFn cannot plan out
until slam_toolbox clears them (up to a minute). The robot cannot be inside
an obstacle, so this node frees the cells under its current footprint
(`global_costmap/footprint`, plus a margin) on every map and footprint update.
Without a footprint message the map passes through unchanged.
"""

import math

import rclpy
from geometry_msgs.msg import Polygon
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def _inside(px: float, py: float, poly: list[tuple[float, float]]) -> bool:
    """Even-odd point-in-polygon test."""
    inside = False
    j = len(poly) - 1
    for i, (xi, yi) in enumerate(poly):
        xj, yj = poly[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


class FootprintMapFilter(Node):
    def __init__(self):
        super().__init__("footprint_map_filter")
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.margin = self.declare_parameter("margin", 0.05).value
        self.map = None
        self.footprint = None  # [(x, y)] in base_frame, grown by the margin
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(OccupancyGrid, "nav_map", LATCHED)
        self.create_subscription(OccupancyGrid, "map", self._on_map, LATCHED)
        self.create_subscription(Polygon, "global_costmap/footprint", self._on_footprint, LATCHED)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg
        self._publish()

    def _on_footprint(self, msg: Polygon) -> None:
        # Grow each vertex away from the robot's centre (the footprints are
        # rectangles around base_footprint).
        m = self.margin
        self.footprint = [(p.x + math.copysign(m, p.x), p.y + math.copysign(m, p.y)) for p in msg.points]
        self._publish()

    def _publish(self) -> None:
        if self.map is None:
            return
        out = self.map
        if self.footprint and len(self.footprint) >= 3:
            try:
                t = self.tf.lookup_transform(self.map.header.frame_id, self.base_frame, Time(),
                                             timeout=Duration(seconds=0.0))
                out = self._cleared(t.transform)
            except TransformException:
                pass  # robot not localized yet: pass the map through
        self.pub.publish(out)

    def _cleared(self, tf) -> OccupancyGrid:
        info = self.map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        q = tf.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = tf.translation.x, tf.translation.y
        world = [(rx + c * x - s * y, ry + s * x + c * y) for x, y in self.footprint]
        i0 = max(0, int((min(p[0] for p in world) - ox) / res))
        i1 = min(info.width - 1, int((max(p[0] for p in world) - ox) / res))
        j0 = max(0, int((min(p[1] for p in world) - oy) / res))
        j1 = min(info.height - 1, int((max(p[1] for p in world) - oy) / res))
        data = None
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                k = j * info.width + i
                if self.map.data[k] != 0 and _inside(ox + (i + 0.5) * res, oy + (j + 0.5) * res, world):
                    if data is None:
                        data = list(self.map.data)
                    data[k] = 0
        if data is None:
            return self.map
        out = OccupancyGrid(header=self.map.header, info=info)
        out.data = data
        return out


def main():
    rclpy.init()
    node = FootprintMapFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
