import io
import math

import numpy as np
import pytest
from PIL import Image

from kachaka_isaac_sim.map_world import bounds, is_map_world, load_map, merge_cells, occupied_at, wall_boxes
from kachaka_isaac_sim.maps import FREE, OCCUPIED, MapError, MapFile


def cover(rects, shape):
    out = np.zeros(shape, int)
    for r0, r1, c0, c1 in rects:
        out[r0:r1, c0:c1] += 1
    return out


def test_is_map_world():
    assert is_map_world("map:maps/a.yaml")
    assert not is_map_world("room") and not is_map_world(None)


@pytest.mark.parametrize("seed", range(5))
def test_merge_cells_covers_exactly_once(seed):
    occ = np.random.default_rng(seed).random((40, 30)) < 0.3
    rects = merge_cells(occ)
    assert (cover(rects, occ.shape) == occ.astype(int)).all()


def test_merge_cells_joins_walls():
    occ = np.zeros((20, 30), bool)
    occ[0, :] = occ[-1, :] = True  # two horizontal walls
    occ[:, 0] = occ[:, -1] = True  # two vertical ones
    occ[5:8, 10:14] = True  # a block
    assert len(merge_cells(occ)) == 2 + 2 + 1


def test_wall_boxes_world_frame():
    grid = np.full((4, 6), FREE, np.int16)
    grid[0, 2:5] = OCCUPIED  # bottom row (min y), x cells 2..4
    m = MapFile(grid, 0.1, (-1.0, -2.0, 0.0))
    (b,) = wall_boxes(m)
    assert (b.x, b.y, b.sx, b.sy, b.yaw) == pytest.approx((-0.65, -1.95, 0.3, 0.1, 0.0))


def test_wall_boxes_rotated_origin():
    grid = np.full((2, 2), FREE, np.int16)
    grid[0, 1] = OCCUPIED  # local centre (0.15, 0.05)
    m = MapFile(grid, 0.1, (1.0, 0.0, math.pi / 2))
    (b,) = wall_boxes(m)
    assert (b.x, b.y, b.yaw) == pytest.approx((0.95, 0.15, math.pi / 2))


def test_occupied_at():
    grid = np.full((10, 10), FREE, np.int16)
    grid[5, 5] = OCCUPIED  # world (0.55, 0.55) at 0.1 m cells, origin 0
    m = MapFile(grid, 0.1, (0.0, 0.0, 0.0))
    assert occupied_at(m, 0.55, 0.55)
    assert not occupied_at(m, 0.2, 0.2)
    assert occupied_at(m, 0.4, 0.55, radius=0.1)
    assert not occupied_at(m, 5.0, 5.0, radius=0.3)  # off the map


def test_bounds():
    m = MapFile(np.zeros((4, 6), np.int16), 0.5, (-1.0, -2.0, 0.0))
    assert bounds(m) == pytest.approx((-1.0, -2.0, 2.0, 0.0))


def test_load_map_yaml_and_image(tmp_path):
    grid = np.full((4, 6), FREE, np.int16)
    grid[0, :] = OCCUPIED
    m = MapFile(grid, 0.05, (-1.0, -2.0, 0.0), charger_pose=(0.5, 0.0, 3.14))
    for name, content in m.files("site").items():
        (tmp_path / name).write_bytes(content)
    got = load_map(tmp_path / "site.yaml")
    assert got.origin == (-1.0, -2.0, 0.0) and got.charger_pose == (0.5, 0.0, 3.14)
    assert (got.grid == grid).all()
    with pytest.raises(MapError, match="drop --map-resolution"):
        load_map(tmp_path / "site.yaml", resolution=0.1)

    img = np.full((3, 4), 255, np.uint8)
    img[0, :] = 0  # top row = max y
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="PNG")
    (tmp_path / "plan.png").write_bytes(buf.getvalue())
    with pytest.raises(MapError, match="resolution"):
        load_map(tmp_path / "plan.png")
    got = load_map(tmp_path / "plan.png", resolution=0.1)
    assert got.grid[-1].tolist() == [OCCUPIED] * 4 and got.origin == (0.0, 0.0, 0.0)
    with pytest.raises(MapError, match="cannot read"):
        load_map(tmp_path / "missing.yaml")
