import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.fleet import fleet_specs, robot_poses


def test_one_robot_keeps_the_single_robot_names():
    cfg = load_config()
    (r,) = fleet_specs(cfg, [Pose2D(1.0, 2.0, 0.5)], 26400, 7)
    assert (r.name, r.namespace, r.frame_prefix, r.prim_path) == ("kachaka", "kachaka", "", "/World/kachaka")
    assert (r.api_port, r.seed, r.node_name) == (26400, 7, "kachaka_isaac_sim")
    assert r.config(cfg) is cfg


def test_more_robots_get_their_own_names_ports_and_seeds():
    cfg = load_config()
    specs = fleet_specs(cfg, [Pose2D(), Pose2D(1.0, 0.0, 0.0), Pose2D(2.0, 0.0, 0.0)], 26400, 7)
    r = specs[2]
    assert (r.name, r.namespace, r.frame_prefix, r.prim_path) == ("kachaka_2", "kachaka_2", "kachaka_2/",
                                                                  "/World/kachaka_2")
    assert (r.api_port, r.seed, r.node_name) == (26402, 2007, "kachaka_isaac_sim_2")
    assert r.pose == Pose2D(2.0, 0.0, 0.0)
    # The LiDAR ignores its own robot by prim prefix: no robot's path starts another's.
    paths = [s.prim_path + "/" for s in specs]
    assert not any(a != b and b.startswith(a) for a in paths for b in paths)


def test_a_robots_config_changes_only_its_names():
    cfg = load_config()
    r = fleet_specs(cfg, [Pose2D(), Pose2D()], 26400, None)[1]
    own = r.config(cfg)
    assert (own.ros.namespace, own.ros.frame_prefix) == ("kachaka_1", "kachaka_1/")
    assert (cfg.ros.namespace, cfg.ros.frame_prefix) == ("kachaka", "")
    assert own.lidar.samples == cfg.lidar.samples


def test_no_api_and_no_seed_stay_off():
    r = fleet_specs(load_config(), [Pose2D(), Pose2D()], 0, None)[1]
    assert (r.api_port, r.seed) == (0, None)


def test_robot_poses():
    assert robot_poses(None, Pose2D(1.0, 0.0, 0.0)) == [Pose2D(1.0, 0.0, 0.0)]
    assert robot_poses([[0, 1, 0], [2, 3, 1.5]], Pose2D()) == [Pose2D(0.0, 1.0, 0.0), Pose2D(2.0, 3.0, 1.5)]


def test_no_robots():
    with pytest.raises(ValueError):
        fleet_specs(load_config(), [], 26400, None)


def test_body_ray_hits():
    import math

    import numpy as np

    from kachaka_isaac_sim.drive import Pose2D
    from kachaka_isaac_sim.fleet import LIDAR_HEIGHT_BODY, body_ray_hits

    dirs = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0]])
    # A robot 2 m ahead, facing us: its housing front (x = 0.208) is nearest.
    hits = body_ray_hits((0.0, 0.0), dirs, [Pose2D(2.0, 0.0, math.pi)])
    assert abs(hits[0] - (2.0 - 0.208)) < 1e-9
    assert math.isinf(hits[1]) and math.isinf(hits[2])
    # Facing away: the back bump (x = -0.024) is hit first.
    assert abs(body_ray_hits((0.0, 0.0), dirs[:1], [Pose2D(2.0, 0.0, 0.0)])[0] - 1.976) < 1e-9
    # Between the parts (y = 0.04 misses the bump, hits the housing).
    assert abs(body_ray_hits((0.0, 0.04), dirs[:1], [Pose2D(2.0, 0.0, 0.0)])[0] - 2.116) < 1e-9
    # Out of range, passing beside, and the nearest of two robots.
    assert math.isinf(body_ray_hits((0.0, 0.0), dirs[:1], [Pose2D(2.0, 0.0, 0.0)], 1.5)[0])
    assert math.isinf(body_ray_hits((0.0, 0.1), dirs[:1], [Pose2D(2.0, 0.0, 0.0)])[0])
    near = body_ray_hits((0.0, 0.0), dirs[:1], [Pose2D(3.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)])[0]
    assert abs(near - 0.976) < 1e-9
    # Axis-aligned rays still work after a rotation, and every part is in the URDF box.
    assert abs(body_ray_hits((0.0, 0.0), dirs[2:], [Pose2D(0.0, 1.0, -math.pi / 2)])[0] - (1 - 0.208)) < 1e-9
    for cx, cy, hx, hy in LIDAR_HEIGHT_BODY:
        assert abs(cx) + hx <= 0.387 / 2 + 0.03 and abs(cy) + hy <= 0.12
