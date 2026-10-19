import pytest

from kachaka_isaac_sim.hazards import Cliff, CollisionCounter, floor_tiles, hole_rect


def area(r):
    return (r[2] - r[0]) * (r[3] - r[1])


def overlap(a, b):
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


def test_hole_rect():
    assert hole_rect(1.0, -1.0, 0.4, 2.0) == pytest.approx((0.8, -2.0, 1.2, 0.0))


def test_floor_tiles_without_holes():
    assert floor_tiles([], 10.0) == [(-10.0, -10.0, 10.0, 10.0)]


@pytest.mark.parametrize("holes", [
    [hole_rect(1.0, 0.0, 0.5, 1.0)],
    [hole_rect(1.0, 0.0, 0.5, 1.0), hole_rect(-2.0, 1.0, 1.0, 1.0)],
    [hole_rect(0.0, 0.0, 1.0, 1.0), hole_rect(0.4, 0.4, 1.0, 1.0)],  # overlapping
    [hole_rect(9.9, 0.0, 1.0, 1.0)],  # past the edge
])
def test_floor_tiles_cover_all_but_the_holes(holes):
    ext = 10.0
    tiles = floor_tiles(holes, ext)
    for i, a in enumerate(tiles):
        for b in tiles[i + 1:]:
            assert not overlap(a, b)
        for h in holes:
            assert not overlap(a, h)
    # Sample points: on a tile exactly when outside every hole.
    for x in [i * 0.37 - 9.9 for i in range(54)]:
        for y in [j * 0.41 - 9.9 for j in range(49)]:
            in_hole = any(h[0] < x < h[2] and h[1] < y < h[3] for h in holes)
            on_tile = any(t[0] <= x <= t[2] and t[1] <= y <= t[3] for t in tiles)
            assert on_tile != in_hole, (x, y)


def test_floor_tiles_single_hole_is_few_tiles():
    assert len(floor_tiles([hole_rect(1.0, 0.0, 0.5, 1.0)], 10.0)) == 4


def test_cliff_pauses_on_new_drop():
    c = Cliff(2)
    assert not c.update([False, False])
    assert not c.blocked and not c.paused
    assert c.update([True, False])
    assert c.paused and c.blocked
    assert not c.update([True, True])  # already paused
    assert c.resume() and not c.paused
    assert c.blocked  # still over the edge: no forward motion
    assert not c.update([True, True])  # no new drop: stays resumed
    assert not c.update([False, True]) and c.blocked
    assert not c.update([False, False]) and not c.blocked
    assert c.update([False, True])  # a new drop pauses again
    assert not Cliff(1).resume()


def test_cliff_disabled_and_reset():
    c = Cliff(2, enabled=False)
    assert not c.update([True, True])
    assert not c.paused and not c.blocked
    c = Cliff(2)
    c.update([True, False])
    c.reset()
    assert not c.paused and c.over == [False, False]


def test_collision_counter_debounce_and_ignore():
    c = CollisionCounter(ignore=("/World/GroundPlane",), debounce=0.5)
    assert not c.contact("/World/GroundPlane", 0.0, True)
    assert not c.contact("/World/GroundPlane/CollisionPlane", 0.0, True)
    assert c.contact("/World/Room/wall_n", 1.0, True)
    assert not c.contact("/World/Room/wall_n", 1.1, False)  # lost
    assert not c.contact("/World/Room/wall_n", 1.3, True)  # flicker
    assert c.contact("/World/Room/box_a", 1.3, True)  # another object
    assert not c.contact("/World/Room/wall_n", 1.5, False)
    assert c.contact("/World/Room/wall_n", 2.1, True)
    assert c.count == 3
    c.reset()
    assert c.count == 0 and not c.last_seen
    assert c.contact("/World/Room/wall_n", 2.2, True)
