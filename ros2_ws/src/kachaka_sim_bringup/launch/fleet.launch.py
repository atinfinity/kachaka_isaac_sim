"""Nav2 on a saved map for every robot of run_sim.py --robot ... (repeated).

    ros2 launch kachaka_sim_bringup fleet.launch.py map:=/maps/room.yaml robots:=2

Robot 0 gets navigation.launch.py (its kachaka_command server, shelves and
RViz included); robots 1 .. robots-1 get robot_nav.launch.py under
/kachaka_i. fleet_outlines gives every robot the others' footprints on
{namespace}/other_robots. RViz draws every robot's model
(/kachaka_i/robot_description, whose links carry the kachaka_i/ prefix).

Robot 0's AMCL starts at the map origin (its spawn on the saved maps).
initial_poses:=x1,y1,yaw1,x2,y2,yaw2,... gives the others' starts in the same
map frame; a robot without one waits for its /kachaka_i/initialpose.
"""

import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def initial_poses(text: str, n: int) -> list[str]:
    """"x1,y1,yaw1,x2,..." -> ["x1,y1,yaw1", ...] for robots 1 .. n-1 ("" for
    a robot without one)."""
    values = [v.strip() for v in text.split(",") if v.strip()]
    if len(values) % 3 or len(values) // 3 > n - 1:
        raise RuntimeError(f"initial_poses: want x,y,yaw for up to {n - 1} robots, got {text!r}")
    poses = [",".join(values[k:k + 3]) for k in range(0, len(values), 3)]
    return poses + [""] * (n - 1 - len(poses))


def fleet_rviz(src: str, n: int) -> str:
    """`src` (an RViz config) with a RobotModel for each of robots 1 .. n-1,
    written to a temporary file."""
    config = yaml.safe_load(Path(src).read_text())
    displays = config["Visualization Manager"]["Displays"]
    model = next(i for i, d in enumerate(displays) if d.get("Class") == "rviz_default_plugins/RobotModel")
    for i in range(n - 1, 0, -1):
        other = {**displays[model], "Name": f"RobotModel kachaka_{i}"}
        other["Description Topic"] = {**displays[model]["Description Topic"], "Value": f"/kachaka_{i}/robot_description"}
        displays.insert(model + 1, other)
    with tempfile.NamedTemporaryFile("w", prefix="fleet_", suffix=".rviz", delete=False) as out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def _fleet(context):
    pkg = FindPackageShare("kachaka_sim_bringup")
    n = int(LaunchConfiguration("robots").perform(context))
    if n < 1:
        raise RuntimeError("robots must be 1 or more")
    poses = initial_poses(LaunchConfiguration("initial_poses").perform(context), n)
    common = {"map": LaunchConfiguration("map"), "use_sim_time": LaunchConfiguration("use_sim_time")}
    rviz = fleet_rviz(LaunchConfiguration("rviz_config").perform(context), n)
    actions = [IncludeLaunchDescription(
        PathJoinSubstitution([pkg, "launch", "navigation.launch.py"]),
        launch_arguments={**common, "rviz": LaunchConfiguration("rviz"), "rviz_config": rviz}.items(),
    )]
    for i in range(1, n):
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "robot_nav.launch.py"]),
            launch_arguments={**common, "namespace": f"kachaka_{i}", "initial_pose": poses[i - 1]}.items(),
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
        DeclareLaunchArgument("initial_poses", default_value="",
                              description="x,y,yaw (map frame) of robots 1, 2, ... in a row"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution(
            [FindPackageShare("kachaka_sim_bringup"), "config", "kachaka.rviz"]),
            description="robot 0's config; a RobotModel is added for each other robot"),
        OpaqueFunction(function=_fleet),
    ])
