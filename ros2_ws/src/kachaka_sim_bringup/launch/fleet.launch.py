"""Nav2 on a saved map for every robot of run_sim.py --robot ... (repeated).

    ros2 launch kachaka_sim_bringup fleet.launch.py map:=/maps/room.yaml robots:=2

Robot 0 gets navigation.launch.py (its kachaka_command server, shelves and
RViz included); robots 1 .. robots-1 get robot_nav.launch.py under
/kachaka_i. Each robot's AMCL waits for its own /initialpose
(/kachaka_i/initialpose), given in the shared map frame. fleet_outlines
gives every robot the others' footprints on {namespace}/other_robots.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _fleet(context):
    pkg = FindPackageShare("kachaka_sim_bringup")
    n = int(LaunchConfiguration("robots").perform(context))
    if n < 1:
        raise RuntimeError("robots must be 1 or more")
    common = {"map": LaunchConfiguration("map"), "use_sim_time": LaunchConfiguration("use_sim_time")}
    actions = [IncludeLaunchDescription(
        PathJoinSubstitution([pkg, "launch", "navigation.launch.py"]),
        launch_arguments={**common, "rviz": LaunchConfiguration("rviz")}.items(),
    )]
    for i in range(1, n):
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "robot_nav.launch.py"]),
            launch_arguments={**common, "namespace": f"kachaka_{i}"}.items(),
        ))
    # The other robots' footprints into each robot's costmaps and collision
    # monitor: the LiDAR sees only their laser housings.
    actions.append(Node(
        package="kachaka_sim_bringup", executable="fleet_outlines", output="screen",
        parameters=[{"robots": [""] + [f"kachaka_{i}" for i in range(1, n)],
                     "use_sim_time": LaunchConfiguration("use_sim_time")}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="map YAML, shared by all robots"),
        DeclareLaunchArgument("robots", default_value="2", description="how many robots the simulator runs"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_fleet),
    ])
