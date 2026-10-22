"""detection_features.py: shapes of the robot's maps, deterministic, image
dependent. Needs torch, torchvision and the MobileNetV3 weights (skipped
without them)."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("torchvision")
pytest.importorskip("cv2")

from kachaka_isaac_sim.detection_features import SHAPES, FeatureExtractor


@pytest.fixture(scope="module")
def extractor():
    fe = FeatureExtractor(device="cpu")
    try:
        fe(np.zeros((480, 640, 3), np.uint8))
    except Exception as e:  # noqa: BLE001 - weights not cached and offline
        pytest.skip(f"MobileNetV3 weights unavailable: {e}")
    return fe


def _image(seed):
    return (np.random.default_rng(seed).random((480, 640, 3)) * 255).astype(np.uint8)


def test_shapes(extractor):
    out = extractor(_image(0))
    assert list(out) == ["p4_out", "p5_out", "p6_out"]
    for name, a in out.items():
        assert a.shape == SHAPES[name] and a.dtype == np.float32 and np.isfinite(a).all()


def test_deterministic_across_instances(extractor):
    a = extractor(_image(1))
    b = FeatureExtractor(device="cpu")(_image(1))
    for k in a:
        np.testing.assert_allclose(a[k], b[k], atol=1e-4)


def test_depends_on_image(extractor):
    a, b = extractor(_image(2)), extractor(_image(3))
    assert not np.allclose(a["p5_out"], b["p5_out"])


def test_any_size(extractor):
    out = extractor(_image(4)[:300, :500].copy())
    assert out["p4_out"].shape == SHAPES["p4_out"]
