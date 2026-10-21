import math
from pathlib import Path

import pytest

from kachaka_isaac_sim.layout import SHELF_RADIUS, LayoutError, parse_layout, read_layout

LAYOUTS = Path(__file__).resolve().parents[2] / "layouts"


def test_fixed_layout():
    lay = parse_layout({
        "world": "office",
        "spawn": [1.0, 2.0, 0.5],
        "charger": [0.0, 0.0, 0.0],
        "shelves": [{"id": "A", "name": "Books", "pose": [3.0, 0.0, 0.0]}, {"pose": [4.0, 0.0, 1.0]}],
        "locations": [{"name": "Desk", "pose": [5.0, 1.0, 0.0]}],
        "people": [[6.0, 0.0, 0.0]],
        "boxes": [{"pose": [7.0, 0.0, 0.3], "size": [0.4, 0.3, 0.5]}],
        "walls": [[0, -2, 0, -1]],
        "doors": [{"pose": [0, -0.5, 1.5708], "kind": "hinged", "open": False, "cycle": [5, 3]}],
        "drops": [[1, 0, 0.4, 0.8]],
    }, seed=1)
    assert lay.world == "office" and not lay.randomised
    assert (lay.spawn.x, lay.spawn.y, lay.spawn.yaw) == (1.0, 2.0, 0.5)
    assert lay.charger.x == 0.0 and not lay.no_charger
    assert [(i, n) for i, n, _ in lay.shelves] == [("A", "Books"), ("S02", "Shelf 2")]
    assert lay.locations[0][:2] == ("L01", "Desk")
    assert lay.people[0].x == 6.0
    assert lay.boxes[0].name == "box01" and lay.boxes[0].size == (0.4, 0.3, 0.5) and lay.boxes[0].pose.yaw == 0.3
    assert lay.walls == [(0.0, -2.0, 0.0, -1.0)]
    d = lay.doors[0]
    assert (d.kind, d.open, d.cycle, d.width) == ("hinged", False, (5.0, 3.0), None)
    assert lay.drops == [(1.0, 0.0, 0.4, 0.8)]


def test_left_out_keys_keep_the_world_defaults():
    lay = parse_layout({"shelves": []})
    assert lay.shelves == [] and lay.spawn is None and lay.charger is None and not lay.no_charger
    assert lay.locations is None and lay.people is None
    assert parse_layout({"charger": None}).no_charger


def test_ranges_are_drawn_with_the_seed_and_stay_in_range():
    data = {"world": "empty", "shelves": [{"pose": [[0.0, 3.0], [-1.0, 1.0], [0.0, 3.1416]]} for _ in range(4)]}
    a = parse_layout(data, seed=5)
    b = parse_layout(data, seed=5)
    c = parse_layout(data, seed=6)
    assert a.randomised and a.seed == 5
    assert [s[2] for s in a.shelves] == [s[2] for s in b.shelves] != [s[2] for s in c.shelves]
    for _, _, p in a.shelves:
        assert 0.0 <= p.x <= 3.0 and -1.0 <= p.y <= 1.0 and 0.0 <= p.yaw <= 3.1416


def test_seed_from_the_file_and_a_random_one_without():
    data = {"seed": 3, "people": [[[0.0, 1.0], 0.0, 0.0]]}
    assert parse_layout(data).seed == 3
    assert parse_layout(data, seed=4).seed == 4
    assert isinstance(parse_layout({"people": [[[0.0, 1.0], 0.0, 0.0]]}).seed, int)


def test_randomised_objects_keep_clear():
    data = {"world": "empty", "clearance": 0.1, "spawn": [0.0, 0.0, 0.0],
            "shelves": [{"pose": [[-1.5, 1.5], [-1.5, 1.5], 0.0]} for _ in range(5)]}
    for seed in range(20):
        poses = [(0.0, 0.0)] + [(p.x, p.y) for _, _, p in parse_layout(data, seed=seed).shelves]
        for i, p in enumerate(poses):
            for q in poses[i + 1:]:
                assert math.dist(p, q) >= 2 * SHELF_RADIUS - 0.1  # spawn is smaller than a shelf


def test_room_walls_and_defaults_are_kept_clear():
    data = {"shelves": [{"pose": [[-2.5, 2.5], [-2.0, 2.0], 0.0]} for _ in range(3)]}
    for seed in range(20):
        lay = parse_layout(data, seed=seed, world="room", defaults={"charger": [(-2.27, 0.0, 0.0)]})
        for _, _, p in lay.shelves:
            assert abs(p.x) < 2.5 - SHELF_RADIUS and abs(p.y) < 2.0 - SHELF_RADIUS
            assert math.dist((p.x, p.y), (-2.27, 0.0)) > SHELF_RADIUS


def test_no_room_raises():
    data = {"world": "empty", "people": [[[0.0, 0.1], 0.0, 0.0] for _ in range(3)]}
    with pytest.raises(LayoutError, match="no free pose"):
        parse_layout(data, seed=1)


@pytest.mark.parametrize("data, match", [
    ({"shelf": []}, "unknown key"),
    ({"shelves": [{"pose": [0, 0]}]}, r"\[x, y, yaw\]"),
    ({"shelves": [{"pose": [[1, 0], 0, 0]}]}, "lo <= hi"),
    ({"shelves": [{"id": "A", "pose": [0, 0, 0]}, {"id": "A", "pose": [1, 0, 0]}]}, "duplicate"),
    ({"boxes": [{"pose": [0, 0, 0]}]}, "size"),
    ({"doors": [{"pose": [0, 0, 0], "kind": "revolving"}]}, "kind"),
    ({"walls": [[0, 0, 1]]}, "4 numbers"),
    ({"people": {"a": 1}}, "must be a list"),
])
def test_errors(data, match):
    with pytest.raises(LayoutError, match=match):
        parse_layout(data, seed=0)


@pytest.mark.parametrize("path", sorted(LAYOUTS.glob("*.yaml")), ids=lambda p: p.name)
def test_shipped_layouts_load(path):
    data = read_layout(path)
    parse_layout(data, world=data.get("world"), path=str(path))
