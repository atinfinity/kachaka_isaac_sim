from pathlib import Path

import yaml
from kachaka_sim_bringup.namespaced_params import namespaced_params_file, rewrite, rewrite_value

SRC = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"


def params_for(ns: str) -> dict:
    return rewrite(yaml.safe_load(SRC.read_text()), ns, f"{ns}/")


def values(d, key=""):
    if isinstance(d, dict):
        for k, v in d.items():
            yield from values(v, str(k))
    elif isinstance(d, list):
        for v in d:
            yield from values(v, key)
    else:
        yield key, d


def test_frames_get_the_prefix_and_map_stays():
    p = params_for("kachaka_1")
    amcl = p["amcl"]["ros__parameters"]
    assert (amcl["base_frame_id"], amcl["odom_frame_id"], amcl["global_frame_id"]) == (
        "kachaka_1/base_footprint", "kachaka_1/odom", "map")
    local = p["local_costmap"]["local_costmap"]["ros__parameters"]
    glob = p["global_costmap"]["global_costmap"]["ros__parameters"]
    assert (local["global_frame"], glob["global_frame"]) == ("kachaka_1/odom", "map")
    assert local["robot_base_frame"] == glob["robot_base_frame"] == "kachaka_1/base_footprint"
    cm = p["collision_monitor"]["ros__parameters"]
    assert (cm["base_frame_id"], cm["odom_frame_id"]) == ("kachaka_1/base_footprint", "kachaka_1/odom")


def test_topics_move_into_the_namespace():
    p = params_for("kachaka_1")
    assert p["amcl"]["ros__parameters"]["scan_topic"] == "/kachaka_1/lidar/scan"
    assert p["bt_navigator"]["ros__parameters"]["odom_topic"] == "/kachaka_1/odom"
    glob = p["global_costmap"]["global_costmap"]["ros__parameters"]
    # Absolute: the costmap would resolve "nav_map" as /kachaka_1/global_costmap/nav_map.
    assert glob["static_layer"]["map_topic"] == "/kachaka_1/nav_map"
    assert glob["obstacle_layer"]["scan"]["topic"] == "/kachaka_1/lidar/scan"
    # Parked shelves are published once for the whole world.
    assert glob["obstacle_layer"]["parked_shelves"]["topic"] == "/parked_shelves"
    assert glob["obstacle_layer"]["other_robots"]["topic"] == "/kachaka_1/other_robots"  # fleet_outlines
    cm = p["collision_monitor"]["ros__parameters"]
    assert cm["FootprintApproach"]["footprint_topic"] == "/kachaka_1/local_costmap/published_footprint"
    assert cm["scan"]["topic"] == "/kachaka_1/lidar/scan"
    assert cm["other_robots"]["topic"] == "/kachaka_1/other_robots"


def test_nothing_points_at_the_first_robot():
    for key, v in values(params_for("kachaka_1")):
        if isinstance(v, str):
            assert v != "base_footprint" and "/kachaka/" not in v, key
            assert not (v == "odom" and key.endswith(("frame", "frame_id"))), key


def test_rewrite_value():
    assert rewrite_value("odom_topic", "odom", "r", "r/") == "odom"  # a topic, not a frame
    assert rewrite_value("fixed_frame", "odom", "r", "r/") == "r/odom"
    assert rewrite_value("topic", "/odom", "r", "r/") == "/r/odom"
    assert rewrite_value("yaml_filename", "/maps/room.yaml", "r", "r/") == "/maps/room.yaml"
    assert rewrite_value("max_vel_x", 0.5, "r", "r/") == 0.5


def test_file(tmp_path):
    assert namespaced_params_file(str(SRC), "") == str(SRC)
    out = namespaced_params_file(str(SRC), "/kachaka_1/", str(tmp_path))
    assert Path(out).parent == tmp_path
    expected = params_for("kachaka_1")
    expected["amcl"]["ros__parameters"]["set_initial_pose"] = False  # its start is not the map origin
    assert yaml.safe_load(Path(out).read_text()) == expected
