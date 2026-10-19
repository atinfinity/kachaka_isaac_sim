"""Simulated object detection from the front camera's ground truth. Plain
Python + numpy, no Isaac Sim imports.

The renderer gives, per frame, an instance id for every pixel (which prim it
shows) and its distance to the camera. Each object to detect is a prim
subtree (a person, a shelf, the charger). An object is detected from its
visible pixels only, so anything in front of it hides it as it would hide it
from a real detector:

- roi: the box around the visible pixels (x_offset, y_offset, width, height
  in image pixels), with a little noise.
- distance_median: the median distance of the visible pixels.
- score: higher for larger visible areas, with a little noise.

How the real detector scores, how much it misses and how far it sees are not
published; the numbers in config/robot.yaml `object_detection` are guesses.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

import numpy as np

# kachaka-api ObjectLabel (protos/kachaka-api.proto).
LABEL_PERSON = 1
LABEL_SHELF = 2
LABEL_CHARGER = 3
LABEL_DOOR = 4
LABELS = {"person": LABEL_PERSON, "shelf": LABEL_SHELF, "charger": LABEL_CHARGER, "door": LABEL_DOOR}


@dataclass
class Target:
    """A prim subtree to detect, e.g. ("/World/People/P01", LABEL_PERSON)."""

    path: str
    label: int


@dataclass
class Detection:
    label: int
    x: int  # roi x_offset [px]
    y: int  # roi y_offset [px]
    width: int
    height: int
    score: float
    distance: float  # median distance of the visible pixels [m]; 0 = unknown

    def to_dict(self) -> dict:
        return {"label": self.label, "roi": [self.x, self.y, self.width, self.height],
                "score": round(self.score, 4), "distance": round(self.distance, 4)}


@dataclass
class DetectionParams:
    max_distance: float = 5.0  # m, nothing farther is detected
    min_pixels: int = 150  # visible pixels needed for a detection
    full_pixels: int = 20000  # visible pixels for the top score
    score_min: float = 0.5  # score at min_pixels
    score_max: float = 0.95  # score at full_pixels and above
    score_noise: float = 0.03
    roi_noise_px: float = 2.0
    miss_rate: float = 0.02  # chance to miss an object in a frame

    @classmethod
    def from_config(cls, cfg) -> DetectionParams:
        p = cls()
        for k in vars(p):
            if hasattr(cfg, k):
                setattr(p, k, type(getattr(p, k))(getattr(cfg, k)))
        return p


def target_of(path: str, targets: list[Target]) -> int:
    """Index of the target whose subtree holds `path`, or -1."""
    for i, t in enumerate(targets):
        if path == t.path or path.startswith(t.path + "/"):
            return i
    return -1


def score_for(pixels: int, p: DetectionParams) -> float:
    """Score from the visible area, log-linear from score_min to score_max."""
    if pixels >= p.full_pixels:
        return p.score_max
    lo, hi = math.log(max(p.min_pixels, 1)), math.log(max(p.full_pixels, p.min_pixels + 1))
    f = (math.log(max(pixels, 1)) - lo) / (hi - lo)
    return p.score_min + (p.score_max - p.score_min) * min(max(f, 0.0), 1.0)


def detect(instance_ids: np.ndarray, id_to_path: dict[int, str], distance: np.ndarray, targets: list[Target],
           params: DetectionParams, rng: random.Random | None = None) -> list[Detection]:
    """Detections in one frame.

    instance_ids: HxW uint instance id per pixel; id_to_path maps an id to
    the prim it shows. distance: HxW metres (inf or nan where nothing is
    hit). With `rng` None there is no noise and no missed objects.
    """
    if not targets or instance_ids.size == 0 or not id_to_path:
        return []
    h, w = instance_ids.shape[:2]
    # Instance id -> target index (+1, 0 = none), then one pass over the image.
    lut = np.zeros(max(max(id_to_path), int(instance_ids.max())) + 1, dtype=np.int32)
    for iid, path in id_to_path.items():
        lut[iid] = target_of(path, targets) + 1
    which = lut[instance_ids]
    out: list[Detection] = []
    for k in np.unique(which):
        if k == 0:
            continue
        ys, xs = np.nonzero(which == k)
        d = distance[ys, xs]
        d = d[np.isfinite(d) & (d > 0)]
        if xs.size < params.min_pixels or d.size == 0:
            continue
        dist = float(np.median(d))
        if dist > params.max_distance:
            continue
        if rng is not None and rng.random() < params.miss_rate:
            continue
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()) + 1, int(ys.min()), int(ys.max()) + 1
        score = score_for(int(xs.size), params)
        if rng is not None:
            def jitter(v: int, lo: int, hi: int) -> int:
                return min(max(round(v + rng.gauss(0.0, params.roi_noise_px)), lo), hi)

            x0, x1 = jitter(x0, 0, w - 1), jitter(x1, 1, w)
            y0, y1 = jitter(y0, 0, h - 1), jitter(y1, 1, h)
            x1, y1 = max(x1, x0 + 1), max(y1, y0 + 1)
            score += rng.gauss(0.0, params.score_noise)
        out.append(Detection(targets[k - 1].label, x0, y0, x1 - x0, y1 - y0, min(max(score, 0.0), 1.0), dist))
    out.sort(key=lambda o: o.distance)
    return out
