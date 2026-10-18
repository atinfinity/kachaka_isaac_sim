"""Navigate on a saved map: map_server + AMCL + Nav2.

    ros2 launch kachaka_sim_bringup navigation.launch.py map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("kachaka_sim_bringup")
    nav2 = FindPackageShare("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="map YAML from map_saver_cli"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="false for a real Kachaka"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("command_server", default_value="true",
                              description="serve kachaka_command/execute (RETURN_HOME) and auto homing; "
                                          "false for a real Kachaka, which has its own"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "bridge.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2, "launch", "localization_launch.py"]),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": use_sim_time,
                "params_file": params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2, "launch", "navigation_launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": params}.items(),
        ),
        Node(
            package="kachaka_sim_bringup",
            executable="command_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("command_server")),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "rviz.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
