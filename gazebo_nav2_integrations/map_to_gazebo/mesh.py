"""Pure-numpy conversion of an occupancy grid to a mesh.

No ROS, no trimesh - plain numpy/opencv arrays in, numpy arrays out. Builds
one box per occupied-region boundary pixel (tracing the contour of each
occupied blob), matching the original map2gazebo algorithm, but as a single
batched/vectorized operation instead of one Python-level mesh object per
point - that per-point object construction was the actual perf bottleneck,
not the opencv contour step below (which is already fast C++).
"""
import cv2
import numpy as np

# Fixed per-box vertex/face layout: 8 vertices (0-3 = the 4 bottom corners,
# 4-7 = the same 4 corners raised by `height`), 12 triangles covering all 6
# faces. Every box uses this exact template, so winding is consistent across
# every box by construction - no per-box normal-fixing needed (unlike the
# original, which built each box as its own independent trimesh.Trimesh and
# had to check/fix each one).
_CORNER_DX = np.array([0, 0, 1, 1])
_CORNER_DY = np.array([0, 1, 0, 1])
_FACE_TEMPLATE = np.array([
    [0, 2, 4], [4, 2, 6], [1, 2, 0], [3, 2, 1],
    [5, 0, 4], [1, 0, 5], [3, 7, 2], [7, 6, 2],
    [7, 4, 6], [5, 4, 7], [1, 5, 3], [7, 3, 5],
])


def get_occupied_regions(map_array, threshold):
    """Return the outer (top-level) contours of cells above `threshold`.

    cv2.RETR_CCOMP classifies external contours at the top level of the
    hierarchy and interior contours (e.g. a hole inside a solid blob) one
    level down. Keeping only top-level contours means an occupied region
    fully enclosed by walls doesn't also emit a separate mesh for its
    interior obstacles - matches the original node's behavior.
    """
    map_array = map_array.astype(np.uint8)
    _, thresh_map = cv2.threshold(map_array, threshold, 100, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(
        thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        return []
    hierarchy = hierarchy[0]
    return [c for c, h in zip(contours, hierarchy) if h[3] == -1]


def contours_to_mesh(contours, resolution, origin_x, origin_y, height):
    """Build one upright box per contour boundary point, batched across every
    contour at once.

    Returns (vertices float32 (V, 3), faces int64 (F, 3)).
    """
    points = np.concatenate([c.reshape(-1, 2) for c in contours], axis=0)
    xs = points[:, 0].astype(np.float64)
    ys = points[:, 1].astype(np.float64)

    corner_x = (xs[:, None] + _CORNER_DX[None, :]) * resolution + origin_x  # (N,4)
    corner_y = (ys[:, None] + _CORNER_DY[None, :]) * resolution + origin_y  # (N,4)
    zeros = np.zeros_like(corner_x)
    bottom = np.stack([corner_x, corner_y, zeros], axis=-1)  # (N,4,3)
    top = bottom + np.array([0.0, 0.0, height])

    n = len(points)
    vertices = np.concatenate([bottom, top], axis=1).reshape(n * 8, 3)
    faces = (_FACE_TEMPLATE[None, :, :] + (np.arange(n) * 8)[:, None, None]).reshape(-1, 3)

    return _dedupe(vertices, faces)


def _dedupe(vertices, faces):
    """Merge coincident vertices, then drop the duplicate faces that creates.

    Adjacent boundary boxes share a wall: each contributes a face at the same
    3D location, wound in opposite directions. Left as-is, the exported mesh
    would have doubled/internal faces at every shared wall - the original
    code got this for free from trimesh's default vertex-merge-on-concatenate
    plus a unique_faces() call; this replicates both steps with plain numpy.
    """
    rounded = np.round(vertices, decimals=6)
    uniq_vertices, inverse = np.unique(rounded, axis=0, return_inverse=True)
    faces = inverse.reshape(-1)[faces]

    sort_key = np.sort(faces, axis=1)
    _, keep_idx = np.unique(sort_key, axis=0, return_index=True)
    faces = faces[keep_idx]

    return uniq_vertices.astype(np.float32), faces
