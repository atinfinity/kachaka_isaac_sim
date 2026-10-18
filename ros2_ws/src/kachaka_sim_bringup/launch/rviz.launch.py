"""RViz with the Kachaka model, sensors, map and Nav2 tools."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("kachaka_sim_bringup"), "config", "kachaka.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=config),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
