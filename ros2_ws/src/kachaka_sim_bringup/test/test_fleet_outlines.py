"""fleet_outlines: the other robots' footprints as costmap points.

    colcon test --packages-select kachaka_sim_bringup
"""

import itertools
import math

import pytest
from kachaka_sim_bringup.fleet_outlines import outline, predict, topic, transform_2d

FOOTPRINT = [(0.25, 0.13), (0.25, -0.13), (-0.16, -0.13), (-0.16, 0.13)]  # nav2_params.yaml


def test_outline_is_closed_and_dense():
    points = outline(FOOTPRINT, step=0.025)
    assert points[0] == FOOTPRINT[0]
    assert all(c in points for c in FOOTPRINT)
    ring = points + points[:1]
    assert max(math.dist(a, b) for a, b in itertools.pairwise(ring)) <= 0.025 + 1e-9
    xs, ys = zip(*points)
    assert (min(xs), max(xs), min(ys), max(ys)) == pytest.approx((-0.16, 0.25, -0.13, 0.13))


def test_transform_2d():
    (x, y), = transform_2d([(0.25, 0.0)], 1.0, 2.0, math.pi / 2)
    assert (x, y) == pytest.approx((1.0, 2.25))


def test_topic():
    assert topic("", "other_robots") == "/other_robots"
    assert topic("kachaka_1", "local_costmap/published_footprint") == "/kachaka_1/local_costmap/published_footprint"
    assert topic("/kachaka_1/", "other_robots") == "/kachaka_1/other_robots"


def test_predict():
    # 0.1 m and 0.1 rad in 0.2 s, 0.5 s ahead.
    assert predict((1.1, 2.0, 0.1), (1.0, 2.0, 0.0), 0.2, 0.5) == pytest.approx((1.35, 2.0, 0.35))
    assert predict((1.0, 2.0, 0.0), (1.0, 2.0, 0.0), 0.0, 0.5) == (1.0, 2.0, 0.0)
    # Across +-pi the turn is the short way.
    assert predict((0.0, 0.0, -3.1), (0.0, 0.0, 3.1), 0.2, 0.2)[2] == pytest.approx(-3.1 + (math.tau - 6.2))
