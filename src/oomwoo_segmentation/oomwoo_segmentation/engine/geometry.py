"""Geometric primitives and layout extraction.

Faithful native port of the upstream ROSE2 geometry layer
(`object/Segment.py`, `object/ExtendedSegment.py`, `object/Line.py`,
`object/Surface.py`, `util/layout.py`, `util/mean_shift.py`,
`util/rototranslation.py`, and `util/matrice.py`). The math, traversal
order, and coordinate comparisons are preserved exactly so results match
the pinned upstream pipeline; only ROS 1 dependencies, disk I/O, and
deprecated APIs are removed.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from sklearn.cluster import DBSCAN


# ---------------------------------------------------------------------------
# Segment data model (upstream object/Segment.py)
# ---------------------------------------------------------------------------


class Segment:
    """A 2D line segment with clustering metadata."""

    def __init__(self, x1: float, y1: float, x2: float, y2: float) -> None:
        self.x1 = float(x1)
        self.y1 = float(y1)
        self.x2 = float(x2)
        self.y2 = float(y2)
        self.num_faces: int = 0
        self.spatial_cluster: Optional[int] = None
        self.wall_cluster: Optional[int] = None
        self.angular_cluster: Optional[float] = None
        self.cluster_index: Optional[int] = None
        self.weight: Optional[float] = None
        self.direction: Optional[float] = None
        self.branch: Optional[object] = None

    def set_cluster_index(self, i: int) -> None:
        self.cluster_index = i

    def set_weight(self, w: float) -> None:
        self.weight = w

    def set_direction(self, d: float) -> None:
        self.direction = d

    def set_angular_cluster(self, c: Optional[float]) -> None:
        self.angular_cluster = c

    def set_spatial_cluster(self, c: Optional[int]) -> None:
        self.spatial_cluster = c

    def set_branch(self, b: object) -> None:
        self.branch = b

    def set_wall_cluster(self, cluster: Optional[int]) -> None:
        self.wall_cluster = cluster

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)


class ExtendedSegment:
    """A wall line clipped to the map bounding box (upstream ExtendedSegment)."""

    def __init__(
        self,
        point1: Sequence[float],
        point2: Sequence[float],
        angular_cluster: Optional[float],
        spatial_cluster: Optional[object],
    ) -> None:
        self.x1 = float(point1[0])
        self.y1 = float(point1[1])
        self.x2 = float(point2[0])
        self.y2 = float(point2[1])
        self.angular_cluster = angular_cluster
        self.spatial_cluster = spatial_cluster
        self.weight: Optional[float] = None
        self.rooms_coverage: Optional[int] = None

    def set_weight(self, weight: float) -> None:
        self.weight = float(weight)


class Line:
    """Parametric infinite wall line (upstream object/Line.py)."""

    def __init__(
        self,
        point: np.ndarray,
        angular_cluster: Optional[float],
        spatial_cluster: Optional[int],
    ) -> None:
        self.point = np.asarray(point, dtype=float)
        self.angular_cluster = angular_cluster
        self.spatial_cluster = spatial_cluster


class Cell:
    """An atomic planar polygonal face bounded by closed edges (upstream Surface.Cell)."""

    def __init__(self, edges: Sequence[Segment]) -> None:
        self.borders = list(edges)
        self.out: Optional[bool] = None
        self.partial: Optional[bool] = None

    def set_out(self, o: bool) -> None:
        self.out = o

    def set_partial(self, p: bool) -> None:
        self.partial = p


# ---------------------------------------------------------------------------
# Segment math (upstream object/Segment.py module functions)
# ---------------------------------------------------------------------------


def seg_length(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def point_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    if t < 0.0:
        return math.hypot(px - x1, py - y1)
    if t > 1.0:
        return math.hypot(px - x2, py - y2)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def intersection_lines(m1: float, q1: float, m2: float, q2: float) -> np.ndarray:
    """Intersection of two lines in normal form ``m*x + y = q``."""
    coefficient = np.array([[m1, 1], [m2, 1]])
    known_term = np.array([q1, q2])
    return np.linalg.solve(coefficient, known_term)


def segments_intersect(
    x11: float, y11: float, x12: float, y12: float,
    x21: float, y21: float, x22: float, y22: float,
) -> bool:
    dx1 = x12 - x11
    dy1 = y12 - y11
    dx2 = x22 - x21
    dy2 = y22 - y21
    delta = dx2 * dy1 - dy2 * dx1
    if delta == 0:
        return False
    s = (dx1 * (y21 - y11) + dy1 * (x11 - x21)) / delta
    t = (dx2 * (y11 - y21) + dy2 * (x21 - x11)) / (-delta)
    return (0 <= s <= 1) and (0 <= t <= 1)


def segments_distance(
    x11: float, y11: float, x12: float, y12: float,
    x21: float, y21: float, x22: float, y22: float,
) -> float:
    if segments_intersect(x11, y11, x12, y12, x21, y21, x22, y22):
        return 0.0
    return min(
        point_segment_distance(x11, y11, x21, y21, x22, y22),
        point_segment_distance(x12, y12, x21, y21, x22, y22),
        point_segment_distance(x21, y21, x11, y11, x12, y12),
        point_segment_distance(x22, y22, x11, y11, x12, y12),
    )


def radiant_inclination(x1: float, y1: float, x2: float, y2: float) -> float:
    """Segment inclination via math.atan; negative slopes stay negative."""
    if x1 != x2:
        m = (y2 - y1) / (x2 - x1)
        return math.atan(m)
    return math.radians(90.0)


def intersection(
    x1: float, y1: float, x2: float, y2: float,
    x3: float, y3: float, x4: float, y4: float,
) -> np.ndarray:
    """Intersection point of the lines through two segments."""
    if x1 != x2 and x3 != x4:
        m1 = (y2 - y1) / (x2 - x1)
        m2 = (y4 - y3) / (x4 - x3)
        q1 = y1 - (m1 * x1)
        q2 = y3 - (m2 * x3)
        return intersection_lines(-m1, q1, -m2, q2)
    if x1 == x2:
        m2 = (y4 - y3) / (x4 - x3)
        q2 = y3 - (m2 * x3)
        y = m2 * x1 + q2
        return np.array([x1, y])
    if x3 == x4:
        m1 = (y2 - y1) / (x2 - x1)
        q1 = y1 - (m1 * x1)
        y = m1 * x3 + q1
        return np.array([x3, y])
    return np.array([np.nan, np.nan])


# ---------------------------------------------------------------------------
# Wall extraction (upstream util/layout.py)
# ---------------------------------------------------------------------------


def start_canny_and_hough(
    image: np.ndarray,
    rho: float = 1.0,
    theta: float = np.pi / 180.0,
    threshold: int = 25,
    min_line_length: int = 10,
    max_line_gap: int = 5,
) -> Tuple[List[Segment], int]:
    """Probabilistic Hough wall extraction from the structural raster."""
    raw_lines = cv2.HoughLinesP(
        image,
        rho=rho,
        theta=theta,
        threshold=threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap,
    )
    if raw_lines is None:
        return [], 0
    walls = [Segment(float(w[0]), float(w[1]), float(w[2]), float(w[3])) for w in (i[0] for i in raw_lines)]
    return walls, 0


def find_extremes(walls: Sequence[Segment]) -> np.ndarray:
    if not walls:
        return np.array([0.0, 0.0, 0.0, 0.0])
    x_coordinates = [w.x1 for w in walls] + [w.x2 for w in walls]
    y_coordinates = [w.y1 for w in walls] + [w.y2 for w in walls]
    return np.array([min(x_coordinates), max(x_coordinates), min(y_coordinates), max(y_coordinates)])


def external_contour(img_rgb: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[List[List[float]]]]:
    """Largest connected free-space contour (OOMWOO-corrected selection)."""
    if img_rgb.ndim == 3:
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    else:
        img_gray = img_rgb.copy()

    _, thresh = cv2.threshold(img_gray.copy(), 253, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = list(contours)
    if contours:
        contours.pop(0)
    img_contour = cv2.drawContours(thresh.copy(), contours, -1, (0, 255, 0), 3, cv2.LINE_8)
    contours, _ = cv2.findContours(img_contour, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # The desired layout boundary is the largest connected free-space
    # contour. Upstream instead selected the second non-free contour, which
    # crashes on cropped maps and can select a furniture island on real maps.
    free_space = np.where(img_gray > 253, 255, 0).astype(np.uint8)
    free_contours, _ = cv2.findContours(
        free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not free_contours:
        return None, None
    contours_max = max(free_contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(contours_max, True)
    screen_cnt = cv2.approxPolyDP(contours_max, 0.0002 * perimeter, True)

    vertices = []
    for c in screen_cnt:
        vertices.append([float(c[0][0]), float(c[0][1])])

    return screen_cnt, vertices


# ---------------------------------------------------------------------------
# Angular clustering (upstream util/mean_shift.py + util/layout.py)
# ---------------------------------------------------------------------------


def mean_shift(h: float, min_offset: float, walls: Sequence[Segment]) -> List[float]:
    """Cosine-kernel mean-shift over wall directions."""
    for wall in walls:
        direction = radiant_inclination(wall.x1, wall.y1, wall.x2, wall.y2)
        if direction < -0.26:
            direction += math.pi
        elif 0 > direction >= -0.26:
            direction = -direction
        wall.set_direction(direction)

    directions = [wall.direction for wall in walls]
    cluster_centers = directions[:]
    new_cluster_centers = _compute_new_cluster_centers(h, cluster_centers, directions)
    max_diff = _maximum_difference(cluster_centers, new_cluster_centers)
    cluster_centers = new_cluster_centers[:]
    while max_diff > min_offset:
        new_cluster_centers = _compute_new_cluster_centers(h, cluster_centers, directions)
        max_diff = _maximum_difference(cluster_centers, new_cluster_centers)
        cluster_centers = new_cluster_centers[:]
    return cluster_centers


def _compute_new_cluster_centers(
    h: float,
    cluster_centers: Sequence[float],
    directions: Sequence[float],
) -> List[float]:
    new_cluster_centers = []
    for alfa in cluster_centers:
        numerator = 0.0
        denominator = 0.0
        for teta in directions:
            kernel = 0.0
            condition = (1 - math.cos(alfa - teta)) / h
            if condition <= 1:
                kernel = math.pow(1 - condition, 2)
            numerator += kernel * teta
            denominator += kernel
        new_cluster_centers.append(numerator / denominator if denominator else alfa)
    return new_cluster_centers


def _maximum_difference(
    cluster_centers: Sequence[float],
    new_cluster_centers: Sequence[float],
) -> float:
    max_diff = 0.0
    for old, new in zip(cluster_centers, new_cluster_centers):
        if abs(new - old) > max_diff:
            max_diff = abs(new - old)
    return max_diff


def _indexes_to_be_deleted(
    num_min: int,
    min_length: float,
    cluster_centers: List[float],
    walls_list: List[Segment],
    diagonals: bool = True,
) -> set:
    indexes = []
    for cluster in set(cluster_centers):
        if cluster_centers.count(cluster) <= num_min:
            lengths = []
            candidates = []
            for index, cluster1 in enumerate(cluster_centers):
                if cluster == cluster1:
                    candidates.append(index)
                    m = walls_list[index]
                    lengths.append(seg_length(m.x1, m.y1, m.x2, m.y2))
            if all(l <= min_length for l in lengths):
                indexes.extend(candidates)
    if diagonals:
        for cluster in set(cluster_centers):
            if not (-0.15 < cluster < 0.15) and not (1.45 < cluster < 1.7) and not (-1.7 < cluster < -1.45):
                for index, cluster1 in enumerate(cluster_centers):
                    if cluster == cluster1:
                        indexes.append(index)
    return set(indexes)


def _merge_similar_cluster(cluster_centers: List[float]) -> bool:
    for cluster1 in set(cluster_centers):
        for cluster2 in set(cluster_centers):
            if (cluster1 != cluster2) and (abs(cluster1 - cluster2) <= 0.01):
                new_cluster = (cluster1 + cluster2) / 2
                for index, cluster3 in enumerate(cluster_centers):
                    if cluster3 == cluster1 or cluster3 == cluster2:
                        cluster_centers[index] = new_cluster
                return True
    return False


def cluster_ang(
    h: float,
    min_offset: float,
    walls: List[Segment],
    num_min: int = 3,
    min_lenght: float = 3.0,
    diagonals: bool = True,
) -> Tuple[set, List[Segment], List[Optional[float]]]:
    """Mean-shift angular clustering of wall directions."""
    cluster_centers = mean_shift(h, min_offset, walls)
    indexes = _indexes_to_be_deleted(num_min, min_lenght, cluster_centers, walls, diagonals)
    for i in sorted(indexes, reverse=True):
        del walls[i]
        del cluster_centers[i]
    united = _merge_similar_cluster(cluster_centers)
    while united:
        united = _merge_similar_cluster(cluster_centers)
    # assign_angular_cluster: positional correspondence between walls and centers
    for index, wall in enumerate(walls):
        wall.set_angular_cluster(cluster_centers[index])
    angular_clusters = [wall.angular_cluster for wall in walls]
    return indexes, walls, angular_clusters


def assign_orebro_direction(comp: Sequence[float], walls: Sequence[Segment]) -> List[Optional[float]]:
    """Snap wall angular clusters to the nearest even FFT direction pair."""
    angular_clusters: List[Optional[float]] = []
    for wall in walls:
        absolute_min = 1000.0
        index = 1000
        for i, direction in enumerate(comp):
            minimum = abs(direction - wall.direction)
            if minimum < absolute_min:
                absolute_min = minimum
                index = i
        if index % 2 == 0:
            wall.set_angular_cluster(comp[index])
        else:
            wall.set_angular_cluster(comp[index - 1])
        angular_clusters.append(wall.angular_cluster)
    return angular_clusters


# ---------------------------------------------------------------------------
# Spatial clustering (upstream util/layout.py + object/Segment.py)
# ---------------------------------------------------------------------------


def _get_distance_matrix(walls: Sequence[Segment]) -> List[List[float]]:
    matrix = []
    for segmento1 in walls:
        row = []
        for segmento2 in walls:
            row.append(segments_distance(
                segmento1.x1, segmento1.y1, segmento1.x2, segmento1.y2,
                segmento2.x1, segmento2.y1, segmento2.x2, segmento2.y2))
        matrix.append(row)
    return matrix


def _set_segment_label2(
    angular_ordered: List[List[Segment]],
    label: Sequence[np.ndarray],
) -> List[Segment]:
    c = 0
    for ang, l in zip(angular_ordered, label):
        for index, segment in enumerate(ang):
            if l[index] != -1:
                segment.set_wall_cluster(int(l[index]) + c)
            else:
                segment.set_wall_cluster(int(l[index]))
        c = c + len(set(l))
    walls = []
    for ang in angular_ordered:
        for segment in ang:
            walls.append(segment)
    return walls


def get_wall_clusters(
    walls: List[Segment],
    angular_clusters: Sequence[Optional[float]],
) -> List[int]:
    """Per-angle DBSCAN over pairwise wall distances producing wall_cluster ids."""
    ang = set(angular_clusters)

    angular_ordered = []
    for c in ang:
        row = []
        for wall in walls:
            if wall.angular_cluster == c:
                row.append(wall)
        angular_ordered.append(row)

    m = []
    for ang_group in angular_ordered:
        matrix = _get_distance_matrix(ang_group)
        m.append(matrix)

    label = []
    for mat in m:
        af = DBSCAN(eps=7, min_samples=1, metric='precomputed').fit(mat)
        label.append(af.labels_)

    walls = _set_segment_label2(angular_ordered, label)

    wall_cluster = []
    for segment in walls:
        wall_cluster.append(segment.wall_cluster)
    return wall_cluster


def _get_projected_points(wall1: Segment, wall2: Segment) -> Tuple[float, float]:
    mid1_x = (wall1.x1 + wall1.x2) / 2
    mid1_y = (wall1.y1 + wall1.y2) / 2
    mid2_x = (wall2.x1 + wall2.x2) / 2
    mid2_y = (wall2.y1 + wall2.y2) / 2
    d = wall1.angular_cluster
    m = math.tan(d)
    if m != 0:
        m_perp = -1 / m
        q1 = mid1_y - (m_perp * mid1_x)
        q2 = mid2_y - (m * mid2_x)
        mid2_projected = intersection_lines(-m_perp, q1, -m, q2)
        return mid2_projected[0], mid2_projected[1]
    return mid2_x, mid1_y


def _rototranslate(x0: float, y0: float, angle: float):
    alpha = float(angle) * math.pi / 180.0

    def rt(x: float, y: float) -> Tuple[float, float]:
        return (
            x0 + x * math.cos(alpha) - y * math.sin(alpha),
            y0 + x * math.sin(alpha) + y * math.cos(alpha),
        )
    return rt


def _inverse_rt(x0: float, y0: float, angle: float):
    alpha = float(angle) * math.pi / 180.0

    def irt(x: float, y: float) -> Tuple[float, float]:
        return (
            -x0 * math.cos(alpha) - y0 * math.sin(alpha) + math.cos(alpha) * x + math.sin(alpha) * y,
            x0 * math.sin(alpha) - y0 * math.cos(alpha) - x * math.sin(alpha) + y * math.cos(alpha),
        )
    return irt


def _transform_points(x0: float, y0: float, angle: float):
    rotation = 90 - angle
    inverse = -rotation
    irt = _inverse_rt(x0, y0, inverse)
    xc, yc = irt(0, 0)
    return _rototranslate(xc, yc, rotation)


def get_representatives(walls: List[Segment], wall_cluster: Sequence[int]) -> List[Segment]:
    """Median representative wall per wall cluster (upstream util/layout.py)."""
    representatives = []
    for cluster in set(wall_cluster):
        lista = []
        distance_vector = []
        for wall in walls:
            if wall.wall_cluster == cluster:
                lista.append(wall)

        candidate = lista[0]
        x = (candidate.x1 + candidate.x2) / 2
        y = (candidate.y1 + candidate.y2) / 2
        angle = candidate.angular_cluster * (180 / math.pi)

        xs = []
        ys = []
        for i in lista:
            x_projected, y_projected = _get_projected_points(candidate, i)
            xs.append(x_projected)
            ys.append(y_projected)

        my_function = _transform_points(x, y, angle)
        for px, py in zip(xs, ys):
            distance_vector.append(my_function(px, py)[0])

        index_representative = np.argsort(distance_vector)[len(distance_vector) // 2 - 1]
        representatives.append(lista[index_representative])
    return representatives


def lateral_separation(wall1: Segment, wall2: Segment) -> float:
    """Perpendicular distance between wall midpoints along their shared angle."""
    mid1_x = (wall1.x1 + wall1.x2) / 2
    mid1_y = (wall1.y1 + wall1.y2) / 2
    mid2_x = (wall2.x1 + wall2.x2) / 2
    mid2_y = (wall2.y1 + wall2.y2) / 2
    d = wall1.angular_cluster
    m = math.tan(d)
    if m != 0:
        m_perp = -1 / m
        q1 = mid1_y - (m_perp * mid1_x)
        q2 = mid2_y - (m * mid2_x)
        mid2_projected = intersection_lines(-m_perp, q1, -m, q2)
        mid2_projected_x = mid2_projected[0]
        mid2_projected_y = mid2_projected[1]
        return seg_length(mid1_x, mid1_y, mid2_projected_x, mid2_projected_y)
    return seg_length(mid1_x, mid1_y, mid1_x, mid2_y)


def spatial_clustering(threshold: float, wall_list: List[Segment]) -> List[Segment]:
    """Merge collinear walls within lateral threshold (upstream Segment.spatial_clustering)."""
    for index, wall1 in enumerate(wall_list):
        if wall1.spatial_cluster is None:
            for wall2 in wall_list[index + 1:]:
                if (wall2.spatial_cluster is None) and (wall1.angular_cluster == wall2.angular_cluster):
                    if lateral_separation(wall1, wall2) < threshold:
                        wall2.set_spatial_cluster(index)
            wall1.set_spatial_cluster(index)
        else:
            for wall2 in wall_list:
                if (wall2.spatial_cluster is None) and (wall1.angular_cluster == wall2.angular_cluster):
                    if lateral_separation(wall1, wall2) < threshold:
                        wall2.set_spatial_cluster(wall1.spatial_cluster)
                else:
                    if (
                        (wall2.spatial_cluster is not None)
                        and (wall2.spatial_cluster != wall1.spatial_cluster)
                        and (wall1.angular_cluster == wall2.angular_cluster)
                        and (lateral_separation(wall1, wall2) < threshold)
                    ):
                        for m in wall_list:
                            if m.spatial_cluster == wall2.spatial_cluster:
                                m.set_spatial_cluster(wall1.spatial_cluster)
    return wall_list


def _set_cluster_spaziale_to_outliers(
    walls: List[Segment],
    outliers: List[Segment],
    representatives_segments: List[Segment],
    lateral_threshold: float,
) -> None:
    for outlier in outliers:
        min_distance_to_cluster = 999999.0
        representative = representatives_segments[0]
        for segment in representatives_segments:
            dist = lateral_separation(outlier, segment)
            if outlier.angular_cluster == segment.angular_cluster:
                if dist <= min_distance_to_cluster:
                    min_distance_to_cluster = dist
                    representative = segment
        if min_distance_to_cluster <= lateral_threshold:
            outlier.spatial_cluster = representative.spatial_cluster
            outlier.wall_cluster = representative.wall_cluster
        else:
            new_cluster = walls.index(outlier)
            outlier.spatial_cluster = new_cluster
            representatives_segments.append(outlier)


def new_spatial_cluster(
    walls: List[Segment],
    representatives_segments: List[Segment],
    spatial_threshold: float,
) -> List[Optional[int]]:
    """Assign final spatial clusters to every wall (upstream util/layout.py)."""
    spatial_clusters = []
    for wall in walls:
        if wall.spatial_cluster is not None:
            spatial_clusters.append(wall.spatial_cluster)

    for cluster in list(set(spatial_clusters)):
        same_wall_cluster = []
        for segment in representatives_segments:
            if segment.spatial_cluster == cluster:
                same_wall_cluster.append(segment.wall_cluster)

        same_wall_cluster = list(set(same_wall_cluster))

        for segment in walls:
            if segment.wall_cluster in same_wall_cluster:
                segment.set_spatial_cluster(cluster)

    outliers = []
    for wall in walls:
        if wall.wall_cluster == -1:
            outliers.append(wall)
    _set_cluster_spaziale_to_outliers(
        walls, outliers, representatives_segments, spatial_threshold)

    spatial_cluster = []
    for wall in walls:
        spatial_cluster.append(wall.spatial_cluster)
    return spatial_cluster


# ---------------------------------------------------------------------------
# Extended lines and segments (upstream object/Line.py + ExtendedSegment.py)
# ---------------------------------------------------------------------------


def create_extended_lines(
    spatial_clusters: Sequence[Optional[int]],
    wall_list: Sequence[Segment],
    x_min: float,
    y_min: float,
) -> List[Line]:
    """One infinite line per spatial cluster through the median wall midpoint."""
    extended_lines = []
    for cluster in set(spatial_clusters):
        mid_points = []
        angle = None
        for wall in wall_list:
            if wall.spatial_cluster == cluster:
                if angle is None:
                    angle = wall.angular_cluster
                mid_x = (wall.x1 + wall.x2) / 2
                mid_y = (wall.y1 + wall.y2) / 2
                mid_points.append(np.array([mid_x, mid_y]))
        if angle == 0:
            mid_points.sort(key=lambda x: (x[1], x[0]))
            index = len(mid_points) // 2
            point = np.array([x_min, mid_points[index][1]])
        elif angle == math.radians(90):
            mid_points.sort(key=lambda x: (x[0], x[1]))
            index = len(mid_points) // 2
            point = np.array([mid_points[index][0], y_min])
        else:
            mid_points.sort(key=lambda x: (x[0], x[1]))
            index = len(mid_points) // 2
            point = mid_points[index]
        extended_lines.append(Line(point, angle, cluster))
    return extended_lines


def create_extended_segments(
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    extended_lines: Sequence[Line],
) -> List[ExtendedSegment]:
    """Clip infinite lines to the padded bounding box and append frame borders."""
    extended_seg = []
    for line in extended_lines:
        if line.angular_cluster == 0:
            point2 = np.array([x_max, line.point[1]])
            seg = ExtendedSegment(line.point, point2, line.angular_cluster, line.spatial_cluster)
            extended_seg.append(seg)
        elif line.angular_cluster == math.radians(90):
            point2 = np.array([line.point[0], y_max])
            seg = ExtendedSegment(line.point, point2, line.angular_cluster, line.spatial_cluster)
            extended_seg.append(seg)
        else:
            m = math.tan(line.angular_cluster)
            q = line.point[1] - (m * line.point[0])
            y_for_x_min = m * x_min + q
            y_for_x_max = m * x_max + q
            x_for_y_min = (y_min - q) / m
            x_for_y_max = (y_max - q) / m
            if line.angular_cluster < 0 or line.angular_cluster * 180 / math.pi > 90:
                if y_max < y_for_x_min:
                    point1 = np.array([x_for_y_max, y_max])
                else:
                    point1 = np.array([x_min, y_for_x_min])
                if y_min > y_for_x_max:
                    point2 = np.array([x_for_y_min, y_min])
                else:
                    point2 = np.array([x_max, y_for_x_max])
            else:
                if y_for_x_min > y_min:
                    point1 = np.array([x_min, y_for_x_min])
                else:
                    point1 = np.array([x_for_y_min, y_min])
                if y_for_x_max < y_max:
                    point2 = np.array([x_max, y_for_x_max])
                else:
                    point2 = np.array([x_for_y_max, y_max])
            seg = ExtendedSegment(point1, point2, line.angular_cluster, line.spatial_cluster)
            extended_seg.append(seg)

    # Synthetic frame segments are appended below. If Hough already found an
    # exactly coincident frame line, keeping both gives create_cells() two
    # indistinguishable edges; its traversal can then repeat two borders and
    # omit the opposite pair, producing the lower-left triangular cell.
    def on_frame(seg: ExtendedSegment) -> bool:
        vertical = np.isclose(seg.x1, seg.x2, atol=1e-6)
        horizontal = np.isclose(seg.y1, seg.y2, atol=1e-6)
        return (
            vertical and (
                np.isclose(seg.x1, x_min, atol=1e-6)
                or np.isclose(seg.x1, x_max, atol=1e-6))
            or horizontal and (
                np.isclose(seg.y1, y_min, atol=1e-6)
                or np.isclose(seg.y1, y_max, atol=1e-6))
        )

    extended_seg = [seg for seg in extended_seg if not on_frame(seg)]
    point1 = np.array([x_min, y_min])
    point2 = np.array([x_min, y_max])
    point3 = np.array([x_max, y_max])
    point4 = np.array([x_max, y_min])
    seg1 = ExtendedSegment(point1, point2, None, 'bordo1')
    seg2 = ExtendedSegment(point2, point3, None, 'bordo2')
    seg3 = ExtendedSegment(point4, point3, None, 'bordo3')
    seg4 = ExtendedSegment(point1, point4, None, 'bordo4')
    extended_seg.extend((seg1, seg2, seg3, seg4))
    return extended_seg


def _divide_segments(extended_segments: Sequence[ExtendedSegment]) -> List[List[ExtendedSegment]]:
    clusters = []
    ang = []
    for seg in extended_segments:
        angular_cluster = seg.angular_cluster
        if angular_cluster not in ang:
            ang.append(angular_cluster)
    for a in ang:
        tmp = []
        for s in extended_segments:
            if s.angular_cluster == a:
                tmp.append(s)
        clusters.append(tmp)
    return clusters


def _point_line_distance(segment1: ExtendedSegment, x1: float, y1: float) -> float:
    if segment1.x2 - segment1.x1 == 0:
        return math.fabs(segment1.x1 - x1)
    m = (segment1.y2 - segment1.y1) / (segment1.x2 - segment1.x1)
    q = -m * segment1.x1 + segment1.y1
    return math.fabs(y1 - m * x1 - q) / math.sqrt(1 + m * m)


def _get_clusters(
    extended_segments: Sequence[ExtendedSegment],
    distance: float,
) -> List[List[ExtendedSegment]]:
    merged_segments = []
    for i1, segment1 in enumerate(extended_segments):
        for i2 in range(i1 + 1, len(extended_segments)):
            segment2 = extended_segments[i2]
            if segment1.angular_cluster == segment2.angular_cluster:
                real_distance = _point_line_distance(segment1, segment2.x1, segment2.y1)
                if real_distance < distance:
                    if len(merged_segments) == 0:
                        merged_segments.append([segment1, segment2])
                    else:
                        index_1 = -1
                        index_2 = -1
                        for i, elem in enumerate(merged_segments):
                            if segment1 in elem:
                                index_1 = i
                            if segment2 in elem:
                                index_2 = i
                        if index_1 == index_2 == -1:
                            merged_segments.append([segment1, segment2])
                        if index_1 == -1 and index_2 != -1:
                            merged_segments[index_2].append(segment1)
                        if index_1 != -1 and index_2 == -1:
                            merged_segments[index_1].append(segment2)
                        if index_1 != index_2 and index_1 != -1 and index_2 != -1:
                            cluster1 = merged_segments[index_1]
                            cluster2 = merged_segments[index_2]
                            cluster3 = cluster1 + cluster2
                            merged_segments.remove(cluster1)
                            merged_segments.remove(cluster2)
                            merged_segments.append(cluster3)
    return merged_segments


def _reallocate_walls(seg1: ExtendedSegment, seg2: ExtendedSegment, walls: Sequence[Segment]) -> None:
    for wall in walls:
        if wall.spatial_cluster == seg2.spatial_cluster:
            wall.set_spatial_cluster(seg1.spatial_cluster)


def merge_together(
    extended_segments: List[ExtendedSegment],
    distance: float,
    walls: Sequence[Segment],
) -> List[ExtendedSegment]:
    """Drop lower-weight duplicates among near-coincident parallel lines."""
    extended_segments_v2 = []
    for seg in extended_segments:
        if seg.angular_cluster is not None:
            extended_segments_v2.append(seg)
    segments = _divide_segments(extended_segments_v2)
    merged_segments = []
    for el in segments:
        cl = _get_clusters(el, distance)
        merged_segments = merged_segments + cl
    seg_to_remove = []
    for cluster in merged_segments:
        cluster.sort(key=lambda s: s.weight, reverse=True)
        for j1, seg1 in enumerate(cluster):
            for j2 in range(j1 + 1, len(cluster)):
                seg2 = cluster[j2]
                if not (seg1 is None or seg2 is None):
                    dist = _point_line_distance(seg1, seg2.x1, seg2.y1)
                    if dist < distance:
                        _reallocate_walls(seg1, seg2, walls)
                        seg_to_remove.append(seg2)
                        cluster[j2] = None
    for seg in seg_to_remove:
        if seg in extended_segments_v2:
            extended_segments_v2.remove(seg)
    for seg in extended_segments:
        if seg.angular_cluster is None:
            extended_segments_v2.append(seg)
    return extended_segments_v2


def remove_less_representatives(
    extended_segments: Sequence[ExtendedSegment],
    threshold: float,
) -> Tuple[List[ExtendedSegment], List[ExtendedSegment]]:
    extended_segments_v2 = []
    li_removed = []
    for seg in extended_segments:
        if (seg.weight or 0.0) >= threshold:
            extended_segments_v2.append(seg)
        else:
            li_removed.append(seg)
    return extended_segments_v2, li_removed


def set_weight_offset(
    extended_segments: Sequence[ExtendedSegment],
    x_max: float, x_min: float, y_max: float, y_min: float,
) -> List[Tuple[Optional[float], Optional[object]]]:
    border_lines = []
    for seg in extended_segments:
        if (
            (seg.x1 == x_min and seg.x2 == x_min)
            or (seg.x1 == x_max and seg.x2 == x_max)
            or (seg.y1 == y_min and seg.y2 == y_min)
            or (seg.y1 == y_max and seg.y2 == y_max)
        ):
            seg.set_weight(1)
            border_lines.append((seg.angular_cluster, seg.spatial_cluster))
    return border_lines


# ---------------------------------------------------------------------------
# Edge weights (upstream object/Segment.py)
# ---------------------------------------------------------------------------


def structural_raster_support(edge: Segment, structural_image: np.ndarray) -> float:
    """Local structural-pixel coverage for an edge (OOMWOO recovery rule)."""
    image = np.asarray(structural_image, dtype=bool)
    sample_count = max(1, int(math.ceil(seg_length(
        edge.x1, edge.y1, edge.x2, edge.y2))))
    vertical = abs(edge.y2 - edge.y1) > abs(edge.x2 - edge.x1)
    hits = 0
    for t in np.linspace(0.0, 1.0, sample_count + 1):
        x = edge.x1 + t * (edge.x2 - edge.x1)
        y = edge.y1 + t * (edge.y2 - edge.y1)
        if vertical:
            coordinates = (
                (int(math.floor(x)), int(round(y))),
                (int(math.ceil(x)), int(round(y))),
            )
        else:
            coordinates = (
                (int(round(x)), int(math.floor(y))),
                (int(round(x)), int(math.ceil(y))),
            )
        if any(
            0 <= row < image.shape[0]
            and 0 <= column < image.shape[1]
            and image[row, column]
            for column, row in coordinates
        ):
            hits += 1
    return hits / (sample_count + 1)


def _project_point(x1: float, y1: float, x2: float, y2: float, x3: float, y3: float) -> np.ndarray:
    if x2 == x3:
        return np.array([x2, y1])
    if y2 == y3:
        return np.array([x1, y2])
    m = (y3 - y2) / (x3 - x2)
    q = y2 - m * x2
    m_perp = -(1 / m)
    q_perp = y1 - m_perp * x1
    return intersection_lines(-m, q, -m_perp, q_perp)


def _already_inside_segment(seg: Sequence[float], projections: Sequence[Sequence[float]]) -> bool:
    for seg2 in projections:
        if seg == seg2:
            return True
    return False


def _included(seg1: Sequence[float], segment_list: Sequence[Sequence[float]]) -> bool:
    x1, y1, x2, y2 = seg1
    for seg2 in segment_list:
        if seg1 != seg2:
            x3, y3, x4, y4 = seg2
            if (x1 == x2 == x3 == x4) and (y1 >= y3) and (y2 <= y4):
                return True
            if (x1 > x3) and (x2 <= x4):
                return True
            if (x1 >= x3) and (x2 < x4):
                return True
    return False


def _merge_overlapped(projections: List[list]) -> int:
    for seg1 in projections:
        x1, y1, x2, y2 = seg1
        for seg2 in projections:
            if seg1 != seg2:
                x3, y3, x4, y4 = seg2
                if (x1 == x2 == x3 == x4) and (y1 < y3) and (y3 <= y2 < y4):
                    union = [x1, y1, x2, y4]
                    projections.append(union)
                    projections.remove(seg1)
                    projections.remove(seg2)
                    return 1
                if (x1 < x3) and (x3 <= x2 < x4):
                    union = [x1, y1, x4, y4]
                    projections.append(union)
                    projections.remove(seg1)
                    projections.remove(seg2)
                    return 1
    return 0


def _compute_coverage(edge, projections: Sequence[Sequence[float]]) -> float:
    x1, y1, x2, y2 = edge.x1, edge.y1, edge.x2, edge.y2
    coverage = 0.0
    for segment in projections:
        x3, y3, x4, y4 = segment
        if x1 == x2 == x3 == x4:
            if (y3 <= y1) and (y4 >= y2):
                return y2 - y1
            if (y3 >= y1) and (y4 <= y2):
                coverage += (y4 - y3)
            if (y1 < y3 < y2) and (y4 > y2):
                coverage += (y2 - y3)
            if (y3 < y1) and (y1 < y4 < y2):
                coverage += (y4 - y1)
        else:
            if (x3 <= x1) and (x4 >= x2):
                return seg_length(x1, y1, x2, y2)
            if (x3 >= x1) and (x4 <= x2):
                coverage += seg_length(x3, y3, x4, y4)
            if (x1 < x3 < x2) and (x4 > x2):
                coverage += seg_length(x3, y3, x2, y2)
            if (x3 < x1) and (x1 < x4 < x2):
                coverage += seg_length(x1, y1, x4, y4)
    return coverage


def set_weights(
    edges: Sequence,
    wall_list: Sequence[Segment],
    structural_image: Optional[np.ndarray] = None,
) -> Sequence:
    """Coverage-based support weights; repairs edges via the structural raster."""
    for edge in edges:
        projections = []
        spatial_cluster = edge.spatial_cluster
        for wall in wall_list:
            if wall.spatial_cluster == spatial_cluster:
                point1 = _project_point(wall.x1, wall.y1, edge.x1, edge.y1, edge.x2, edge.y2)
                point2 = _project_point(wall.x2, wall.y2, edge.x1, edge.y1, edge.x2, edge.y2)
                tmp = [point1, point2]
                tmp.sort(key=lambda x: (x[0], x[1]))
                tmp2 = [tmp[0][0], tmp[0][1], tmp[1][0], tmp[1][1]]
                if not _already_inside_segment(tmp2, projections):
                    projections.append(tmp2)
        projections[:] = [tup for tup in projections if not _included(tup, projections)]
        merged = _merge_overlapped(projections)
        while merged == 1:
            merged = _merge_overlapped(projections)
        coverage = _compute_coverage(edge, projections)
        edge_len = seg_length(edge.x1, edge.y1, edge.x2, edge.y2)
        weight = coverage / edge_len if edge_len > 0 else 0.0
        # HoughLinesP can consume collinear pixels without returning every
        # local run. Only repair a zero-weight retained structural edge; never
        # perturb positive Hough-derived weights or synthetic frame edges.
        if (
            weight == 0
            and structural_image is not None
            and edge.angular_cluster is not None
        ):
            weight = max(weight, structural_raster_support(edge, structural_image))
        elif (
            structural_image is not None
            and edge.angular_cluster is not None
        ):
            raster_sup = structural_raster_support(edge, structural_image)
            weight = max(weight, raster_sup)
        edge.set_weight(weight)
    return edges


def create_short_ex_lines(
    line: ExtendedSegment,
    walls: Sequence[Segment],
    size: Sequence[int],
    extended_lines: Sequence[ExtendedSegment],
) -> Optional[ExtendedSegment]:
    """Shorten a dropped line to the wall-supported span (upstream recovery)."""
    x_min, x_max, y_min, y_max = size[0] + 20, 0, size[1] + 20, 0
    for wall in walls:
        if wall.spatial_cluster == line.spatial_cluster:
            x_min, x_max, y_min, y_max = _check_pos(x_min, x_max, y_min, y_max, wall.x1, wall.y1)
            x_min, x_max, y_min, y_max = _check_pos(x_min, x_max, y_min, y_max, wall.x2, wall.y2)
    x1, x2, y1, y2 = x_min, x_max, y_min, y_max
    x_min -= 100
    x_max += 100
    y_min -= 100
    y_max += 100
    if x_min < 0:
        x_min = 0
    if y_min < 0:
        y_min = 0
    if x_max > size[0]:
        x_max = size[0]
    if y_max > size[1]:
        y_max = size[1]
    if line.x2 - line.x1 == 0:
        point1 = np.array([line.x2, y_min])
        point2 = np.array([line.x2, y_max])
        point1_real = np.array([x1, y1])
        point2_real = np.array([x2, y2])
    else:
        m = (line.y2 - line.y1) / (line.x2 - line.x1)
        q = line.y1 - m * line.x1
        if x_max - x_min > y_max - y_min:
            y_for_x_min = m * x_min + q
            y_for_x_max = m * x_max + q
            y1 = m * x1 + q
            y2 = m * x2 + q
            point1 = np.array([x_min, y_for_x_min])
            point2 = np.array([x_max, y_for_x_max])
            point1_real = np.array([x1, y1])
            point2_real = np.array([x2, y2])
        else:
            x_for_y_min = (y_min - q) / m
            x_for_y_max = (y_max - q) / m
            x1 = (y1 - q) / m
            x2 = (y2 - q) / m
            point1 = np.array([x_for_y_min, y_min])
            point2 = np.array([x_for_y_max, y_max])
            point1_real = np.array([x1, y1])
            point2_real = np.array([x2, y2])
    vertices = []
    for l in extended_lines:
        if segments_intersect(l.x1, l.y1, l.x2, l.y2, point1[0], point1[1], point2[0], point2[1]):
            vertex = intersection(l.x1, l.y1, l.x2, l.y2, point1[0], point1[1], point2[0], point2[1])
            if not _inside_segment(vertex, x1, y1, x2, y2):
                vertices.append(vertex)
    if len(vertices) >= 2:
        d1, d2 = None, None
        n1, n2 = None, None
        for v in vertices:
            d = _p_to_p_dist(v, point1_real)
            if d1 is None or d < d1:
                d1 = d
                n1 = v
        for i, v in enumerate(vertices):
            if v[0] == n1[0] and v[1] == n1[1]:
                vertices.pop(i)
        for v in vertices:
            d = _p_to_p_dist(v, point2_real)
            if d2 is None or d < d2:
                d2 = d
                n2 = v
        if n1[0] - n2[0] == 0:
            if n1[1] > n2[1]:
                n1[1] += 10
                n2[1] -= 10
            else:
                n1[1] -= 10
                n2[1] += 10
        else:
            m = (line.y2 - line.y1) / (line.x2 - line.x1)
            q = line.y1 - m * line.x1
            if n1[0] > n2[0]:
                n1[0] += 5
                n1[1] = m * n1[0] + q
                n2[0] -= 5
                n2[1] = m * n2[0] + q
            else:
                n1[0] -= 5
                n1[1] = m * n1[0] + q
                n2[0] += 5
                n2[1] = m * n2[0] + q
        return ExtendedSegment(n1, n2, line.angular_cluster, line.spatial_cluster)
    return None


def _check_pos(
    x_min: float, x_max: float, y_min: float, y_max: float,
    x: float, y: float,
) -> Tuple[float, float, float, float]:
    if x < x_min:
        x_min = x
    if x > x_max:
        x_max = x
    if y < y_min:
        y_min = y
    if y > y_max:
        y_max = y
    return x_min, x_max, y_min, y_max


def _p_to_p_dist(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def _inside_segment(vertex: Sequence[float], x1: float, y1: float, x2: float, y2: float) -> bool:
    if x1 - 10 <= vertex[0] <= x2 + 10 and y1 - 10 <= vertex[1] <= y2 + 10:
        return True
    return False


# ---------------------------------------------------------------------------
# Edge creation and planar cells (upstream object/Segment.py + Surface.py)
# ---------------------------------------------------------------------------


def create_edges(extended_segments: Sequence[ExtendedSegment]) -> List[Segment]:
    """Split extended lines at their mutual intersections into atomic edges."""
    edges = []
    points = []
    for segment in extended_segments:
        x1, y1, x2, y2 = segment.x1, segment.y1, segment.x2, segment.y2
        for segment2 in extended_segments:
            x3, y3, x4, y4 = segment2.x1, segment2.y1, segment2.x2, segment2.y2
            if (segment != segment2) and (segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4)):
                point = intersection(x1, y1, x2, y2, x3, y3, x4, y4)
                points.append(point)
        points.sort(key=lambda x: (x[0], x[1]))
        for i, point in enumerate(points):
            if i < len(points) - 1:
                nxt = points[i + 1]
                if not ((point[0] == nxt[0]) and (point[1] == nxt[1])):
                    edge = Segment(point[0], point[1], nxt[0], nxt[1])
                    edge.set_angular_cluster(segment.angular_cluster)
                    edge.set_spatial_cluster(segment.spatial_cluster)
                    edges.append(edge)
        del points[:]
    return edges


def adjacent_edges(x: float, y: float, edge: Segment, edges: Sequence[Segment]) -> List[Segment]:
    """Edges other than `edge` touching the exact vertex (x, y)."""
    e = []
    for edge1 in edges:
        x1 = edge1.x1
        y1 = edge1.y1
        x2 = edge1.x2
        y2 = edge1.y2
        if (edge != edge1) and (((x1 == x) and (y1 == y)) or ((x2 == x) and (y2 == y))):
            e.append(edge1)
    return e


def _face_exist(edge: Segment, a: Segment, faces: Sequence[Cell]) -> bool:
    for face in faces:
        if (edge in face.borders) and (a in face.borders):
            return True
    return False


def _common_edge(face1: Cell, face2: Cell) -> List[Segment]:
    return list(set(face1.borders).intersection(face2.borders))


def _check_belonging(face: Cell, faces: Sequence[Cell]) -> bool:
    sides = face.borders
    for face2 in faces:
        sides2 = face2.borders
        if all(side in sides2 for side in sides) and all(side1 in sides for side1 in sides2):
            return True
        if len(_common_edge(face, face2)) >= 2:
            return True
    return False


def _create_n_faces(
    sides: int,
    faces: List[Cell],
    edges: List[Segment],
) -> bool:
    """Unified port of upstream create_3/4/5/6/7_faces.

    The chain-traversal order, exact float coordinate comparisons, and the
    `candidate is not first edge` closure guard for faces with >= 5 sides
    reproduce the upstream behavior (including the upstream 7-face branch
    that compares the wrong intermediate vertex, preserved for parity).
    """
    remaining_edges = []
    for edge in edges:
        if edge.num_faces < 2:
            remaining_edges.append(edge)

    for edge in remaining_edges:
        x2 = edge.x2
        y2 = edge.y2
        common_x_edge = edge.x1
        common_y_edge = edge.y1
        adjacent = adjacent_edges(common_x_edge, common_y_edge, edge, remaining_edges)
        for a in adjacent:
            if _face_exist(edge, a, faces):
                break
            if (a.x1 == common_x_edge) and (a.y1 == common_y_edge):
                common_a = (a.x2, a.y2)
            else:
                common_a = (a.x1, a.y1)

            if _extend_chain(
                [edge, a], common_a, (x2, y2), remaining_edges, faces, sides, edge,
            ):
                return True
    return False


def _extend_chain(
    chain: List[Segment],
    current: Tuple[float, float],
    target: Tuple[float, float],
    remaining_edges: Sequence[Segment],
    faces: List[Cell],
    sides: int,
    first_edge: Segment,
    prev_current: Optional[Tuple[float, float]] = None,
) -> bool:
    prev = chain[-1]
    position = len(chain) + 1
    final_level = position == sides
    # Upstream create_7_faces tests the 6th edge's endpoint against the
    # common vertex from two levels back (common_x_c instead of
    # common_x_d). Preserved verbatim for behavioral parity.
    test_point = prev_current if (sides == 7 and position == 6 and prev_current is not None) else current
    for cand in adjacent_edges(current[0], current[1], prev, remaining_edges):
        if (cand.x1 == test_point[0]) and (cand.y1 == test_point[1]):
            other = (cand.x2, cand.y2)
        else:
            other = (cand.x1, cand.y1)
        closes = (other == target) and (position <= 4 or cand is not first_edge)
        if final_level:
            if closes:
                face = Cell(chain + [cand])
                if not _check_belonging(face, faces):
                    for e in chain + [cand]:
                        e.num_faces += 1
                    faces.append(face)
                    return True
        else:
            if closes:
                continue
            if _extend_chain(
                chain + [cand], other, target, remaining_edges, faces, sides,
                first_edge, prev_current=current,
            ):
                return True
    return False


def create_cells(edges: List[Segment]) -> List[Cell]:
    """Atomic closed faces of 3 to 7 edges (upstream Surface.create_cells)."""
    faces: List[Cell] = []
    face = _create_n_faces(3, faces, edges)
    while face:
        face = _create_n_faces(3, faces, edges)
    face = _create_n_faces(4, faces, edges)
    while face:
        face = _create_n_faces(4, faces, edges)
    face = _create_n_faces(5, faces, edges)
    while face:
        face = _create_n_faces(5, faces, edges)
    face = _create_n_faces(6, faces, edges)
    while face:
        face = _create_n_faces(6, faces, edges)
    face = _create_n_faces(7, faces, edges)
    while face:
        face = _create_n_faces(7, faces, edges)
    return faces
