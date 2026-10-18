"""Relay between Nav2's generic topics (/odom, /cmd_vel) and the Kachaka namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("kachaka_namespace", default_value="kachaka"),
        Node(
            package="kachaka_sim_bringup",
            executable="nav_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "kachaka_namespace": LaunchConfiguration("kachaka_namespace"),
            }],
        ),
    ])
