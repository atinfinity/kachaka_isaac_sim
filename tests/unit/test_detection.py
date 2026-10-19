import random

import numpy as np

from kachaka_isaac_sim.detection import (
    LABEL_CHARGER,
    LABEL_PERSON,
    LABEL_SHELF,
    DetectionParams,
    Target,
    detect,
    score_for,
    target_of,
)

TARGETS = [Target("/World/People/P01", LABEL_PERSON), Target("/World/Shelves/S01", LABEL_SHELF),
           Target("/World/Charger", LABEL_CHARGER)]
PATHS = {1: "/World/Room/wall_e", 2: "/World/People/P01/torso", 3: "/World/People/P01/head",
         4: "/World/Shelves/S01/visual", 5: "/World/Charger/panel", 6: "/World/People/P010/torso"}
NO_NOISE = DetectionParams(min_pixels=10, full_pixels=1000)


def frame(h=120, w=160):
    return np.ones((h, w), dtype=np.uint32), np.full((h, w), 4.0, dtype=np.float32)


def test_target_of_matches_subtrees_only():
    assert target_of("/World/People/P01", TARGETS) == 0
    assert target_of("/World/People/P01/head", TARGETS) == 0
    assert target_of("/World/People/P010/head", TARGETS) == -1
    assert target_of("/World/Charger/led", TARGETS) == 2


def test_roi_and_distance_from_visible_pixels():
    ids, dist = frame()
    ids[20:80, 30:50] = 2  # torso
    ids[10:20, 35:45] = 3  # head: same person
    dist[10:80, 30:50] = 2.0
    ids[60:100, 100:140] = 4  # shelf
    dist[60:100, 100:140] = 3.0
    dets = detect(ids, PATHS, dist, TARGETS, NO_NOISE)
    assert [d.label for d in dets] == [LABEL_PERSON, LABEL_SHELF]  # nearest first
    p, s = dets
    assert (p.x, p.y, p.width, p.height) == (30, 10, 20, 70)
    assert p.distance == 2.0
    assert (s.x, s.y, s.width, s.height) == (100, 60, 40, 40)
    assert s.distance == 3.0


def test_occluded_part_is_not_in_the_roi():
    ids, dist = frame()
    ids[20:80, 30:50] = 2
    dist[20:80, 30:50] = 2.0
    ids[50:80, 30:50] = 1  # something in front hides the lower half
    dist[50:80, 30:50] = 1.0
    (p,) = detect(ids, PATHS, dist, TARGETS, NO_NOISE)
    assert (p.y, p.height) == (20, 30)
    assert p.distance == 2.0


def test_small_far_and_unlisted_objects_are_dropped():
    ids, dist = frame()
    ids[0:3, 0:3] = 2  # 9 px < min_pixels
    ids[50:60, 50:60] = 5  # charger, too far
    dist[50:60, 50:60] = 9.0
    ids[80:100, 80:100] = 6  # not a target
    ids[100:110, 0:10] = 77  # id missing from the map
    assert detect(ids, PATHS, dist, TARGETS, NO_NOISE) == []


def test_invalid_depth_is_ignored():
    ids, dist = frame()
    ids[20:40, 20:40] = 5
    dist[20:40, 20:40] = np.inf
    dist[20:25, 20:40] = 1.5
    (c,) = detect(ids, PATHS, dist, TARGETS, NO_NOISE)
    assert c.label == LABEL_CHARGER and c.distance == 1.5


def test_score_grows_with_area():
    p = DetectionParams(min_pixels=100, full_pixels=10000, score_min=0.5, score_max=0.9)
    assert score_for(100, p) == 0.5
    assert abs(score_for(1000, p) - 0.7) < 1e-9
    assert score_for(10000, p) == 0.9
    assert score_for(50000, p) == 0.9


def test_noise_stays_in_the_image_and_in_range():
    ids, dist = frame()
    ids[0:120, 0:40] = 2
    p = DetectionParams(min_pixels=10, full_pixels=1000, roi_noise_px=5.0, score_noise=0.5, miss_rate=0.0)
    rng = random.Random(1)
    for _ in range(200):
        (d,) = detect(ids, PATHS, dist, TARGETS, p, rng)
        assert 0 <= d.x and d.x + d.width <= 160 and 0 <= d.y and d.y + d.height <= 120
        assert d.width >= 1 and d.height >= 1
        assert 0.0 <= d.score <= 1.0


def test_miss_rate():
    ids, dist = frame()
    ids[0:50, 0:50] = 2
    p = DetectionParams(min_pixels=10, miss_rate=0.3)
    rng = random.Random(2)
    hits = sum(len(detect(ids, PATHS, dist, TARGETS, p, rng)) for _ in range(1000))
    assert 620 < hits < 780


def test_params_from_config():
    from types import SimpleNamespace

    p = DetectionParams.from_config(SimpleNamespace(max_distance=3, min_pixels=50.0, unknown=1))
    assert p.max_distance == 3.0 and isinstance(p.max_distance, float)
    assert p.min_pixels == 50 and isinstance(p.min_pixels, int)


def test_to_dict():
    ids, dist = frame()
    ids[0:50, 0:50] = 2
    (d,) = detect(ids, PATHS, dist, TARGETS, NO_NOISE)
    assert d.to_dict() == {"label": LABEL_PERSON, "roi": [0, 0, 50, 50], "score": 0.95, "distance": 4.0}


def test_default_config():
    from kachaka_isaac_sim.config import load_config

    cfg = load_config()
    p = DetectionParams.from_config(cfg.object_detection)
    assert 0 < p.score_min < p.score_max <= 1 and p.min_pixels < p.full_pixels
    assert all(len(pose) == 3 for pose in cfg.people.room)
