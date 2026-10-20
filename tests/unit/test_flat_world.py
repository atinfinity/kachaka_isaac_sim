import math

import numpy as np
import pytest

from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.flat_world import (
    Box,
    FlatShelves,
    any_inside,
    cast_rays,
    outline,
    room_boxes,
    shelf_legs,
    transform,
)
from kachaka_isaac_sim.scenes import ROOM_SIZE, SHELF_LEG_XY
from kachaka_isaac_sim.shelves import ShelfSpec


def test_cast_rays_axis_aligned_box():
    box = Box(2.0, 0.0, 0.0, 1.0, 1.0)
    d = cast_rays(0.0, 0.0, np.array([0.0, math.pi, math.pi / 2]), [box])
    assert d[0] == pytest.approx(1.5)
    assert math.isinf(d[1]) and math.isinf(d[2])


def test_cast_rays_rotated_box_and_nearest():
    # A square turned 45 deg: its corner points at the origin.
    diamond = Box(2.0, 0.0, math.pi / 4, 1.0, 1.0)
    near = Box(1.0, 0.0, 0.0, 0.2, 2.0)
    d = cast_rays(0.0, 0.0, np.array([0.0]), [diamond])
    assert d[0] == pytest.approx(2.0 - math.sqrt(0.5))
    assert cast_rays(0.0, 0.0, np.array([0.0]), [diamond, near])[0] == pytest.approx(0.9)


def test_cast_rays_ignores_box_around_origin():
    assert math.isinf(cast_rays(0.0, 0.0, np.array([0.0]), [Box(0.0, 0.0, 0.0, 1.0, 1.0)])[0])


def test_room_walls_all_around():
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    d = cast_rays(0.0, 0.0, angles, room_boxes())
    assert np.isfinite(d).all()
    assert d.max() <= math.hypot(*ROOM_SIZE) / 2 + 1e-6
    assert d[36] == pytest.approx(min(d[36], ROOM_SIZE[0] / 2))  # +x: a wall or something nearer


def test_people_add_boxes():
    assert len(room_boxes([(1.0, 1.0, 0.0)])) == len(room_boxes()) + 1


def test_shelf_legs_turn_with_the_shelf():
    legs = shelf_legs(Pose2D(1.0, 2.0, math.pi / 2))
    xy = sorted((round(b.x, 6), round(b.y, 6)) for b in legs)
    lx, ly = SHELF_LEG_XY
    want = sorted((round(1.0 - py, 6), round(2.0 + px, 6)) for px in (-lx, lx) for py in (-ly, ly))
    assert xy == want


def test_body_outline_and_collision():
    body = outline(-0.157, 0.23, 0.12)
    assert body[:, 0].min() == pytest.approx(-0.157) and body[:, 1].max() == pytest.approx(0.12)
    wall = [Box(0.5, 0.0, 0.0, 0.1, 2.0)]
    assert not any_inside(transform(body, Pose2D(0.0, 0.0, 0.0)), wall)
    assert any_inside(transform(body, Pose2D(0.25, 0.0, 0.0)), wall)
    # Turned 90 deg, the front is along +y and clears the wall.
    assert not any_inside(transform(body, Pose2D(0.25, 0.0, math.pi / 2)), wall)


def test_the_body_fits_between_shelf_legs():
    body = outline(-0.157, 0.23, 0.12)
    shelf = Pose2D(1.0, 0.0, 0.0)
    assert not any_inside(transform(body, shelf), shelf_legs(shelf))


def make_shelves():
    specs = [ShelfSpec("S01", "Shelf 1", "/World/Shelves/S01", Pose2D(1.0, 0.0, 0.0)),
             ShelfSpec("S02", "Shelf 2", "/World/Shelves/S02", Pose2D(-1.0, 0.0, math.pi / 2))]
    return FlatShelves(specs, 0.05, 0.2)


def test_dock_tow_and_undock():
    sh = make_shelves()
    assert sh.dock(Pose2D(0.5, 0.0, 0.0)) == (False, "no shelf above the robot")
    ok, msg = sh.dock(Pose2D(1.02, 0.0, math.pi))  # from the other side
    assert ok and msg == "docked with S01" and sh.docked == 0
    assert len(sh.legs()) == 4  # the towed shelf's legs are not obstacles
    sh.follow(Pose2D(0.0, 1.0, math.pi / 2))
    p = sh.poses[0]
    assert (p.x, p.y) == pytest.approx((0.0, 1.0 + 0.02), abs=1e-9)
    assert sh.undock() == (True, "undocked S01")
    sh.follow(Pose2D(5.0, 5.0, 0.0))
    assert (sh.poses[0].x, sh.poses[0].y) == pytest.approx((0.0, 1.02))


def test_reset():
    sh = make_shelves()
    sh.dock(Pose2D(1.0, 0.0, 0.0))
    sh.follow(Pose2D(0.0, 0.5, 0.0))
    assert sh.reset() == (False, "the robot tows S01")
    assert sh.reset("S03") == (False, "unknown shelf 'S03'")
    sh.undock()
    assert sh.reset("S01") == (True, "")
    assert (sh.poses[0].x, sh.poses[0].y) == (1.0, 0.0)
