"""Mesh file readers. Plain numpy, no Isaac Sim."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_stl(path: Path, scale: float = 1.0) -> np.ndarray:
    """Binary STL -> (N*3, 3) float32 vertices, one triangle per 3 rows."""
    data = path.read_bytes()
    count = int(np.frombuffer(data[80:84], np.uint32)[0])
    tri = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data[84:84 + count * 50], dtype=tri)["v"].reshape(-1, 3) * scale
