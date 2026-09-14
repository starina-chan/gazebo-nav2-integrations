"""Minimal binary STL writer.

No trimesh: normals are computed vectorized for every face at once, and the
whole file body is written in one buffered call instead of a per-triangle
Python loop.
"""
import struct

import numpy as np

_STL_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attr", "<u2"),
])


def write_binary_stl(path, vertices, faces):
    tri = vertices[faces]  # (F,3,3): 3 vertices per face, xyz per vertex

    edge1 = tri[:, 1] - tri[:, 0]
    edge2 = tri[:, 2] - tri[:, 0]
    normals = np.cross(edge1, edge2)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    np.divide(normals, lengths, out=normals, where=lengths != 0)

    records = np.zeros(len(faces), dtype=_STL_DTYPE)
    records["normal"] = normals
    records["vertices"] = tri

    with open(path, "wb") as f:
        f.write(struct.pack("<80x"))
        f.write(struct.pack("<I", len(faces)))
        f.write(records.tobytes())
