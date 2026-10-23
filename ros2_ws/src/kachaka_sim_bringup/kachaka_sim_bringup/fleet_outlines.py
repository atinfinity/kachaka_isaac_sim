"""The other robots' footprints, for each robot's costmaps (#17).

    ros2 run kachaka_sim_bringup fleet_outlines --ros-args -p robots:="['', 'kachaka_1']"

The LiDAR sits above most of the body: of another Kachaka it sees only the
housing around the laser and a bump at the back (fleet.LIDAR_HEIGHT_BODY in
the simulator), 9 cm of its 24 cm width. Nose to nose, two robots touch
before either costmap knows where the other's corners are. So, like
/parked_shelves for the shelves, every robot gets the outlines of all the
others as a PointCloud2 on {namespace}/other_robots, in the map frame:
each robot's own Nav2 footprint (local_costmap/published_footprint, which
grows while towing a shelf) placed with its localization (TF map ->
{prefix}base_footprint, AMCL's estimate, not the simulator's ground truth),
as a fleet sharing its poses would. The footprint topic only gives the
shape: it comes with the costmap's updates (5 Hz), too late for a robot
crossing at 0.5 m/s. The outline is placed with the newest TF instead, and
again PREDICT s ahead at the robot's current speed, so the others react to
where it is going.

`robots` lists the Nav2 namespaces, robot 0's first ('' for none).
"""

import math

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

OUTLINE_STEP = 0.025  # m between points, half a costmap cell (as command_server's shelf outlines)
POINT_Z = 0.1  # m, within the costmaps' obstacle heights
MAX_AGE = 1.0  # s; a pose older than this (its localization stopped) is left out
PREDICT = 0.5  # s ahead for the second outline
VELOCITY_WINDOW = 0.2  # s of TF to estimate the velocity from


def outline(corners, step: float = OUTLINE_STEP) -> list[tuple[float, float]]:
    """Points along the closed polygon `corners` ((x, y), ...), at most
    `step` apart."""
    points = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / step))
        points += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    return points


def transform_2d(points, x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def predict(now, before, dt: float, ahead: float):
    """Pose (x, y, yaw) `ahead` s after `now`, at the velocity from `before`
    to `now` (`dt` s apart)."""
    if dt <= 0:
        return now
    k = ahead / dt
    dyaw = math.remainder(now[2] - before[2], math.tau)
    return now[0] + (now[0] - before[0]) * k, now[1] + (now[1] - before[1]) * k, now[2] + dyaw * k


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def topic(namespace: str, name: str) -> str:
    return f"/{namespace.strip('/')}/{name}" if namespace.strip("/") else f"/{name}"


class FleetOutlines(Node):
    def __init__(self):
        super().__init__("fleet_outlines")
        self.robots = [str(r).strip("/") for r in self.declare_parameter("robots", ["", "kachaka_1"]).value]
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.rate = float(self.declare_parameter("rate", 20.0).value)
        self.bases = [f"{ns}/base_footprint" if ns else "base_footprint" for ns in self.robots]
        self.shapes = [None] * len(self.robots)  # footprint corners in the robot's base frame
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pubs = []
        for i, ns in enumerate(self.robots):
            self.create_subscription(PolygonStamped, topic(ns, "local_costmap/published_footprint"),
                                     lambda m, i=i: self._on_footprint(i, m), 1)
            self.pubs.append(self.create_publisher(PointCloud2, topic(ns, "other_robots"), 1))
        self.create_timer(1.0 / self.rate, self._publish)
        self.get_logger().info(f"outlines for {len(self.robots)} robots: {self.robots}")

    def _pose(self, frame: str, when: Time):
        t = self.tf.lookup_transform(self.map_frame, frame, when)
        p, q = t.transform.translation, t.transform.rotation
        return (p.x, p.y, yaw_of(q)), Time.from_msg(t.header.stamp)

    def _on_footprint(self, i: int, msg: PolygonStamped) -> None:
        """Keep the shape: the polygon in the robot's base frame, at its stamp."""
        if len(msg.polygon.points) < 3:
            return
        try:
            t = self.tf.lookup_transform(self.bases[i], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        p, q = t.transform.translation, t.transform.rotation
        self.shapes[i] = transform_2d([(c.x, c.y) for c in msg.polygon.points], p.x, p.y, yaw_of(q))

    def _in_map(self, i: int) -> list[tuple[float, float]]:
        """Robot i's outline in the map frame, now and PREDICT s ahead, or []
        if its shape or pose is unknown or old."""
        if self.shapes[i] is None:
            return []
        try:
            now, stamp = self._pose(self.bases[i], Time())
            before, then = self._pose(self.bases[i], stamp - Duration(seconds=VELOCITY_WINDOW))
        except TransformException:
            return []
        if self.get_clock().now() - stamp > Duration(seconds=MAX_AGE):
            return []
        ahead = predict(now, before, (stamp - then).nanoseconds * 1e-9, PREDICT)
        ring = outline(self.shapes[i])
        return transform_2d(ring, *now) + transform_2d(ring, *ahead)

    def _publish(self) -> None:
        robots = [self._in_map(i) for i in range(len(self.robots))]
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        for i, pub in enumerate(self.pubs):
            points = [(x, y, POINT_Z) for j, r in enumerate(robots) if j != i for x, y in r]
            pub.publish(create_cloud_xyz32(header, points))


def main():
    rclpy.init()
    node = FleetOutlines()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
