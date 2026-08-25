"""Cell classification, affinity matrices, and DBSCAN room clustering.

Faithful native port of the upstream ROSE2 topology layer
(`util/layout.py` classification/matrix/merge functions and
`util/matrice.py`). Traversal order, coordinate handling, and weight
semantics match the pinned upstream pipeline; `np.matrix` is replaced by
plain `np.ndarray` without changing the arithmetic.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from sklearn.cluster import DBSCAN

from .geometry import Cell, Segment


# ---------------------------------------------------------------------------
# Cell polygons (upstream util/layout.py create_polygon / classification_surface)
# ---------------------------------------------------------------------------


def _uniq_sorted_desc(points: List[List[float]]) -> List[List[float]]:
    """sorted(reverse=True) + consecutive-exact-duplicate removal."""
    ordered = sorted(points, reverse=True)
    result = []
    last = object()
    for item in ordered:
        if item == last:
            continue
        result.append(item)
        last = item
    return result


def _clockwise_key_builder(centroid: Tuple[float, float]):
    def algo(p: Sequence[float]) -> float:
        return (math.atan2(p[0] - centroid[0], p[1] - centroid[1]) + 2 * math.pi) % (2 * math.pi)
    return algo


def cell_polygon(cell: Cell) -> Optional[Polygon]:
    """Shapely polygon for a cell; degenerate cells are reported as outside."""
    points: List[List[float]] = []
    for b in cell.borders:
        points.append([float(b.x1), float(b.y1)])
        points.append([float(b.x2), float(b.y2)])
    points = _uniq_sorted_desc(points)
    if len(points) < 3:
        # Coincident extended lines can produce a degenerate cell. Older
        # Shapely paths reached this implicitly; Shapely 2 rejects it.
        return None
    centroid = (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )
    points.sort(key=_clockwise_key_builder(centroid))
    return Polygon(points)


def classification_surface(
    vertices: Sequence[Sequence[float]],
    cells: Sequence[Cell],
    threshold: float,
) -> Tuple[List[Cell], List[Cell], List[Polygon]]:
    """Keep cells overlapping the layout contour by at least area/threshold."""
    contour = Polygon(vertices)
    contour = contour.buffer(0)

    cells_out: List[Cell] = []
    cells_in: List[Cell] = []
    polygons_in: List[Polygon] = []
    for f in cells:
        cell = cell_polygon(f)
        if cell is None:
            f.set_out(True)
            f.set_partial(False)
            cells_out.append(f)
            continue
        if not cell.intersects(contour):
            f.set_out(True)
            f.set_partial(False)
            cells_out.append(f)
        else:
            if cell.intersection(contour).area >= cell.area / threshold:
                f.set_out(False)
                cells_in.append(f)
                polygons_in.append(cell)
            else:
                f.set_out(True)
                f.set_partial(False)
                cells_out.append(f)
    return cells_in, cells_out, polygons_in


def remove_frame_fringes(
    cells: List[Cell],
    polygons: List[Polygon],
    frame_bounds: Tuple[float, float, float, float],
    minimum_wall_support: float = 0.9,
    minimum_wall_span: float = 0.6,
    maximum_area_ratio: float = 0.15,
) -> Tuple[List[Cell], List[Polygon]]:
    """Remove small frame-side cells behind a strong layout-spanning wall.

    ROSE can infer an exterior wall inside a noisy free map margin. The free
    contour still classifies both sides as inside, so the margin becomes a
    spurious room. Keep this compatibility rule deliberately strict to avoid
    discarding ordinary boundary rooms.
    """
    xmin, ymin, xmax, ymax = frame_bounds
    minimum_length = minimum_wall_span * min(xmax - xmin, ymax - ymin)
    frame_tolerance = 1e-6
    dropped = set()

    for first in range(len(cells)):
        for second in range(first + 1, len(cells)):
            shared = _common_edge(cells[first], cells[second])
            if not shared:
                continue
            for edge in shared:
                wall_length = math.hypot(edge.x2 - edge.x1, edge.y2 - edge.y1)
                if (edge.weight or 0.0) < minimum_wall_support or wall_length < minimum_length:
                    continue
                first_area = polygons[first].area
                second_area = polygons[second].area
                if min(first_area, second_area) / max(first_area, second_area) > maximum_area_ratio:
                    continue
                smaller = first if first_area < second_area else second
                minx, miny, maxx, maxy = polygons[smaller].bounds
                touches_frame = (
                    abs(minx - xmin) <= frame_tolerance
                    or abs(miny - ymin) <= frame_tolerance
                    or abs(maxx - xmax) <= frame_tolerance
                    or abs(maxy - ymax) <= frame_tolerance
                )
                if touches_frame:
                    dropped.add(smaller)

    keep = [index for index in range(len(cells)) if index not in dropped]
    return [cells[index] for index in keep], [polygons[index] for index in keep]


def _common_edge(face1: Cell, face2: Cell) -> List[Segment]:
    return list(set(face1.borders).intersection(face2.borders))


def _adjacent(face1: Cell, face2: Cell) -> bool:
    return len(_common_edge(face1, face2)) > 0


# ---------------------------------------------------------------------------
# Affinity matrices and DBSCAN (upstream util/matrice.py + util/layout.py)
# ---------------------------------------------------------------------------


def create_matrices(
    cells: Sequence[Cell],
    sigma: float = 0.125,
    hard_wall_threshold: Optional[float] = 0.40,
) -> np.ndarray:
    """Symmetric normalized adjacency distance matrix X = 1 - M."""
    n = len(cells)
    if n == 0:
        return np.zeros((0, 0))

    matrix_l = np.zeros((n, n), dtype=float)
    for i, face in enumerate(cells):
        for j, face2 in enumerate(cells):
            if face is face2:
                val = 1.0
            elif _adjacent(face, face2):
                e = _common_edge(face, face2)
                w = e[0].weight or 0.0
                if hard_wall_threshold is not None and w >= hard_wall_threshold:
                    val = 0.0
                else:
                    val = math.exp(-w / sigma)
            else:
                val = 0.0
            matrix_l[i, j] = val

    row_sums = matrix_l.sum(axis=1)
    with np.errstate(divide='ignore'):
        inv_d = 1.0 / row_sums
    matrix_m = inv_d[:, np.newaxis] * matrix_l
    matrix_m = 0.5 * (matrix_m + matrix_m.T)
    return 1.0 - matrix_m


def dbscan_cluster_cells(
    eps: float,
    min_samples: int,
    x_dist: np.ndarray,
) -> np.ndarray:
    """DBSCAN on the precomputed cell distance matrix."""
    if x_dist.shape[0] == 0:
        return np.array([], dtype=int)
    af = DBSCAN(eps=eps, min_samples=min_samples, metric='precomputed').fit(x_dist)
    return af.labels_


def merge_cells(
    clusters: Sequence[int],
    cells: Sequence[Cell],
    polygon_cells: Sequence[Polygon],
) -> List[Polygon]:
    """Merge clustered cell polygons into room polygons (upstream merge_cells)."""
    rooms = []
    for label in set(clusters):
        polygons = []
        for index, cluster in enumerate(clusters):
            if (label == cluster) and not cells[index].out:
                polygons.append(polygon_cells[index])
        if not polygons:
            continue
        room = unary_union(polygons)
        rooms.append(room)
    return rooms
