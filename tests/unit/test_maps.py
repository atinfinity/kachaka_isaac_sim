import io

import numpy as np
import pytest
from PIL import Image

from kachaka_isaac_sim.maps import (
    FREE,
    OCCUPIED,
    PNG_FREE,
    PNG_OCCUPIED,
    UNKNOWN,
    MapError,
    MapFile,
    MapStore,
)


def sample() -> MapFile:
    grid = np.full((4, 6), FREE, np.int16)
    grid[0, :] = OCCUPIED  # bottom row (min y)
    grid[3, 5] = UNKNOWN
    return MapFile(grid, 0.05, (-1.0, -2.0, 0.0), name="room", charger_pose=(0.5, 0.0, 3.14))


def test_from_occupancy_grid():
    m = MapFile.from_occupancy_grid([0, 100, -1, 30, 65, 90], 3, 2, 0.05, (0, 0, 0))
    assert m.grid.tolist() == [[FREE, OCCUPIED, UNKNOWN], [FREE, OCCUPIED, OCCUPIED]]


def test_png_top_row_is_max_y():
    img = np.asarray(Image.open(io.BytesIO(sample().png())))
    assert img.shape == (4, 6)
    assert (img[-1] == PNG_OCCUPIED).all()
    assert img[0, 0] == PNG_FREE
    m = MapFile.from_image(sample().png(), 0.05)
    assert (m.grid == sample().grid).all()
    assert m.origin == (0.0, 0.0, 0.0)


def test_from_plain_image():
    gray = np.array([[0, 128, 255]], np.uint8)
    out = io.BytesIO()
    Image.fromarray(gray).save(out, format="PNG")
    m = MapFile.from_image(out.getvalue(), 0.1)
    assert m.grid.tolist() == [[OCCUPIED, UNKNOWN, FREE]]


def test_zip_round_trip():
    m = MapFile.from_zip(sample().zip("room"))
    assert (m.grid == sample().grid).all()
    assert m.resolution == 0.05
    assert m.origin == (-1.0, -2.0, 0.0)
    assert m.name == "room"
    assert m.charger_pose == pytest.approx((0.5, 0.0, 3.14))
    with pytest.raises(MapError):
        MapFile.from_zip(b"not a zip")


def test_store(tmp_path):
    store = MapStore(tmp_path / "maps", reserved=("sim",))
    assert store.save(sample(), "sim") == "sim_2"
    assert store.save(sample(), "my map!") == "my_map"
    assert store.save(sample(), "my map!") == "my_map_2"
    assert store.ids() == ["my_map", "my_map_2", "sim_2"]
    assert store.meta("my_map")["image"] == "my_map.pgm"
    assert (store.load("my_map").grid == sample().grid).all()
    for bad in ("../x", ".hidden", "missing"):
        with pytest.raises(MapError):
            store.load(bad)
