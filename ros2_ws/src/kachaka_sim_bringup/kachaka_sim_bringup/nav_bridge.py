"""Connect Nav2's generic topics to the Kachaka interface.

- /kachaka/odometry/odometry (SensorDataQoS, as on the real bridge) is
  republished on /odom with reliable QoS, which Nav2's odometry users expect.
- /cmd_vel from Nav2 is forwarded to /kachaka/manual_control/cmd_vel, or with
  the parameter cmd_vel_topic to another topic under the Kachaka namespace:
  bridge.launch.py picks the simulator-only sim/autonomous/cmd_vel when
  use_sim_time is true, where the simulator caps autonomous motion at its
  higher top speed instead of the manual 0.3 m/s.

Works the same against the simulator and a real Kachaka.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class NavBridge(Node):
    def __init__(self):
        super().__init__("kachaka_nav_bridge")
        ns = self.declare_parameter("kachaka_namespace", "kachaka").value.strip("/")
        prefix = f"/{ns}" if ns else ""
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        topic = self.declare_parameter("cmd_vel_topic", "manual_control/cmd_vel").value.strip("/")
        self.cmd_pub = self.create_publisher(Twist, f"{prefix}/{topic}", 10)
        self.create_subscription(Odometry, f"{prefix}/odometry/odometry", self.odom_pub.publish,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.cmd_pub.publish, 10)


def main():
    rclpy.init()
    node = NavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
