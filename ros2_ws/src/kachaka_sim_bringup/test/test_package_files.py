"""Static checks of the launch files, parameter files and behavior trees.

    colcon test --packages-select kachaka_sim_bringup
"""

import importlib.util
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
LAUNCH_FILES = sorted((PKG / "launch").glob("*.launch.py"))
BEHAVIOR_TREES = sorted((PKG / "behavior_trees").glob("*.xml"))


def load_yaml(name: str) -> dict:
    return yaml.safe_load((PKG / "config" / name).read_text())


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_launch_file_builds(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.generate_launch_description().entities


@pytest.mark.parametrize("name", ["nav2_params.yaml", "slam_toolbox.yaml", "kachaka.rviz", "demo.rviz"])
def test_config_parses(name):
    assert load_yaml(name)


def test_nav2_params_point_at_our_behavior_trees():
    params = load_yaml("nav2_params.yaml")["bt_navigator"]["ros__parameters"]
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        m = re.fullmatch(r"\$\(find-pkg-share kachaka_sim_bringup\)/(.+)", params[key])
        assert m, params[key]
        assert (PKG / m.group(1)).exists()


def test_nav2_params_frames_and_topics():
    params = load_yaml("nav2_params.yaml")
    slam = load_yaml("slam_toolbox.yaml")["slam_toolbox"]["ros__parameters"]
    assert slam["base_frame"] == "base_footprint"
    assert slam["scan_topic"] == "/kachaka/lidar/scan"
    assert params["amcl"]["ros__parameters"]["scan_topic"] == slam["scan_topic"]
    for costmap in ("local_costmap", "global_costmap"):
        p = params[costmap][costmap]["ros__parameters"]
        assert p["robot_base_frame"] == "base_footprint"
    assert "shelf_docking_server" in params


@pytest.mark.parametrize("path", BEHAVIOR_TREES, ids=lambda p: p.name)
def test_behavior_tree(path):
    root = ET.parse(path).getroot()
    assert root.tag == "root"
    main = root.get("main_tree_to_execute")
    trees = {t.get("ID") for t in root.iter("BehaviorTree")}
    assert main in trees
    # The robot cannot see behind it, so these trees must not back up.
    assert not [el for el in root.iter() if el.tag == "BackUp"]


def test_fake_robot_scan():
    from kachaka_sim_bringup.fake_robot import FOV, HALF_ROOM, LASER_XYZ, SAMPLES, scan_ranges

    ranges = scan_ranges(0.0, 0.0, 0.0)
    assert len(ranges) == SAMPLES
    front = ranges[SAMPLES // 2]  # straight ahead at angle 0 (even count: just left of it)
    assert front == pytest.approx(HALF_ROOM - LASER_XYZ[0], rel=1e-3)
    left = ranges[round((math.pi / 2 + FOV / 2) / FOV * (SAMPLES - 1))]
    assert left == pytest.approx(HALF_ROOM, rel=1e-2)
    # Facing the pillar at (1, 1) from the origin.
    towards = scan_ranges(-LASER_XYZ[0] * math.sqrt(0.5), -LASER_XYZ[0] * math.sqrt(0.5), math.pi / 4)
    assert min(towards) < math.sqrt(2) - 0.15
