"""Build a map with slam_toolbox while driving (teleop or Nav2).

    ros2 launch kachaka_sim_bringup slam.launch.py              # SLAM + RViz
    ros2 launch kachaka_sim_bringup slam.launch.py nav:=true    # also Nav2 (explore by goal)
    ros2 run nav2_map_server map_saver_cli -f ~/map             # save the map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("kachaka_sim_bringup")
    nav2 = FindPackageShare("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    commands = IfCondition(PythonExpression(
        ["'", LaunchConfiguration("nav"), "' == 'true' and '", LaunchConfiguration("command_server"), "' == 'true'"]))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="false for a real Kachaka"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("command_server", default_value="true",
                              description="serve kachaka_command/execute (RETURN_HOME, shelves) and auto homing; "
                                          "false for a real Kachaka, which has its own"),
        DeclareLaunchArgument("nav", default_value="false", description="also start Nav2"),
        DeclareLaunchArgument("slam_params_file",
                              default_value=PathJoinSubstitution([pkg, "config", "slam_toolbox.yaml"])),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([pkg, "config", "kachaka.rviz"])),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "bridge.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2, "launch", "navigation_launch.py"]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("nav")),
        ),
        Node(
            package="kachaka_sim_bringup",
            executable="command_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=commands,
        ),
        Node(
            package="kachaka_sim_bringup",
            executable="shelf_markers",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=commands,
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "shelf_docking.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": LaunchConfiguration("params_file")}.items(),
            condition=commands,
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "rviz.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time, "rviz_config": LaunchConfiguration("rviz_config")}.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
