"""Relay between Nav2's generic topics (/odom, /cmd_vel) and the Kachaka namespace,
/map to the global costmap (/nav_map) with the cells under the robot freed, and
the simulator's object detections to the real bridge's
/kachaka/object_detection/result."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("kachaka_namespace", default_value="kachaka"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint",
                              description="the robot's base frame (kachaka_1/base_footprint for robot 1)"),
        Node(
            package="kachaka_sim_bringup",
            executable="nav_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "kachaka_namespace": LaunchConfiguration("kachaka_namespace"),
                # The simulator caps autonomous motion apart from manual cmd_vel.
                "cmd_vel_topic": PythonExpression([
                    "'sim/autonomous/cmd_vel' if '", LaunchConfiguration("use_sim_time"),
                    "'.lower() == 'true' else 'manual_control/cmd_vel'"]),
            }],
        ),
        Node(
            package="kachaka_sim_bringup",
            executable="object_detection_relay",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "kachaka_namespace": LaunchConfiguration("kachaka_namespace"),
            }],
        ),
        Node(
            package="kachaka_sim_bringup",
            executable="footprint_map_filter",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time"),
                         "base_frame": LaunchConfiguration("base_frame")}],
        ),
    ])
