"""A second Nav2 docking server for Kachaka Shelves, used by command_server.

Jazzy's docking server takes the docking direction per server (dock_backwards),
and the robot backs onto its charger but drives forward under a shelf, so the
shelf gets its own server. Its topics and actions carry a `shelf_` prefix.
Included by slam.launch.py (nav:=true) and navigation.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", description="nav2_params.yaml with a shelf_docking_server section"),
        Node(
            package="opennav_docking",
            executable="opennav_docking",
            name="shelf_docking_server",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {"use_sim_time": use_sim_time}],
            remappings=[
                ("dock_robot", "shelf_dock_robot"),
                ("undock_robot", "shelf_undock_robot"),
                ("detected_dock_pose", "shelf_detected_dock_pose"),
                ("dock_pose", "shelf_dock_pose"),
                ("staging_pose", "shelf_staging_pose"),
                ("filtered_dock_pose", "shelf_filtered_dock_pose"),
                ("docking_trajectory", "shelf_docking_trajectory"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_shelf_docking",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                         "node_names": ["shelf_docking_server"]}],
        ),
    ])
