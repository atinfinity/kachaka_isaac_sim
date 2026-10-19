import struct

import numpy as np
import pytest

from kachaka_isaac_sim.meshes import read_stl


def write_stl(path, triangles):
    data = bytearray(b"solid but binary".ljust(80, b" "))
    data += struct.pack("<I", len(triangles))
    for tri in triangles:
        data += struct.pack("<3f", 0, 0, 1)
        for v in tri:
            data += struct.pack("<3f", *v)
        data += struct.pack("<H", 0)
    path.write_bytes(bytes(data))


def test_read_binary_stl(tmp_path):
    tris = [[(0, 0, 0), (1000, 0, 0), (0, 1000, 0)], [(1, 2, 3), (4, 5, 6), (7, 8, 9)]]
    path = tmp_path / "mesh.stl"
    write_stl(path, tris)
    v = read_stl(path, 0.001)
    assert v.shape == (6, 3)
    assert v == pytest.approx(np.array(tris, dtype=float).reshape(-1, 3) * 0.001)
    assert read_stl(path)[1] == pytest.approx([1000, 0, 0])
