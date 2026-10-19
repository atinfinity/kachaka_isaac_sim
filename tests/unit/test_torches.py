from types import SimpleNamespace

import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.torches import Torches, clamp_level, light_intensity


def test_levels_are_clamped_to_0_255():
    assert clamp_level(-5) == 0
    assert clamp_level(128) == 128
    assert clamp_level(1000) == 255


def test_intensity_scales_with_the_level():
    assert light_intensity(0, 1000.0) == 0.0
    assert light_intensity(255, 1000.0) == 1000.0
    assert light_intensity(51, 1000.0) == pytest.approx(200.0)
    assert light_intensity(300, 1000.0) == 1000.0


def test_torches_start_off_and_keep_their_levels():
    t = Torches(SimpleNamespace(max_intensity=1000.0))
    assert t.state() == {"front": 0, "back": 0}
    assert t.set("front", 300) == 255
    assert t.set("back", 10) == 10
    assert t.state() == {"front": 255, "back": 10}
    with pytest.raises(ValueError):
        t.set("left", 1)


def test_config_has_torch_settings():
    c = load_config().torch
    assert c.front.frame_id == "camera_front"
    assert c.back.frame_id == "camera_back"
    assert c.max_intensity > 0
    assert 0 < c.cone_angle_deg <= 90
    assert len(c.color) == 3
