"""map_server + AMCL + Nav2 for one robot of a fleet, in its ROS namespace.

    ros2 launch kachaka_sim_bringup robot_nav.launch.py namespace:=kachaka_1 map:=/maps/room.yaml

For robot i >= 1 of run_sim.py --robot ... (repeated): its topics are under
/kachaka_i and its TF frames are kachaka_i/odom, kachaka_i/base_footprint.
Nav2 runs under /kachaka_i too (/kachaka_i/navigate_to_pose,
/kachaka_i/initialpose, ...), with nav2_params.yaml rewritten for it
(kachaka_sim_bringup.namespaced_params). TF stays on the shared /tf, and all
robots share the map frame. Robot 0 uses navigation.launch.py.

initial_pose:=x,y,yaw (the map frame) starts AMCL there; without it AMCL
waits for /kachaka_i/initialpose.

The shelf, charger and kachaka_command servers are robot 0's only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def _robot(context):
    from kachaka_sim_bringup.namespaced_params import namespaced_params_file, parse_pose

    ns = LaunchConfiguration("namespace").perform(context).strip("/")
    if not ns:
        raise RuntimeError("robot_nav.launch.py needs a namespace; use navigation.launch.py for robot 0")
    pkg = FindPackageShare("kachaka_sim_bringup")
    nav2 = FindPackageShare("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = namespaced_params_file(LaunchConfiguration("params_file").perform(context), ns,
                                    initial_pose=parse_pose(LaunchConfiguration("initial_pose").perform(context)))
    return [GroupAction([
        PushRosNamespace(ns),
        # Nav2's launch files remap /tf to tf (the namespace's own); the
        # first matching rule wins, and these come first.
        SetRemap("/tf", "/tf"),
        SetRemap("/tf_static", "/tf_static"),
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "bridge.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time, "kachaka_namespace": ns,
                              "base_frame": f"{ns}/base_footprint"}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2, "launch", "localization_launch.py"]),
            launch_arguments={"namespace": ns, "map": LaunchConfiguration("map"),
                              "use_sim_time": use_sim_time, "params_file": params}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2, "launch", "navigation_launch.py"]),
            launch_arguments={"namespace": ns, "use_sim_time": use_sim_time, "params_file": params}.items(),
        ),
    ])]


def generate_launch_description():
    pkg = FindPackageShare("kachaka_sim_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", description="the robot's namespace, e.g. kachaka_1"),
        DeclareLaunchArgument("map", description="map YAML, the same for every robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]),
                              description="written for robot 0; rewritten for this robot"),
        DeclareLaunchArgument("initial_pose", default_value="",
                              description="x,y,yaw in the map frame for AMCL's start; empty: wait for initialpose"),
        OpaqueFunction(function=_robot),
    ])
