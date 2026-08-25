"""Post-processing: rasterization, geodesic 100% coverage, wall extraction, and diagnostics."""

from __future__ import annotations

from collections import deque
import math
from typing import List, Sequence, Tuple

import cv2
import numpy as np
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon

from ..models import DiagnosticImage, SegmentationResult, WallSegment
from ..render import render_segmentation, render_source_map
from ..source_map import SourceMap
from .geometry import ExtendedSegment


def extract_polygons(geometry) -> Tuple[Polygon, ...]:
    """Unpack Polygon or MultiPolygon geometries."""
    if geometry is None or geometry.is_empty:
        return ()
    if geometry.geom_type == 'Polygon':
        return (geometry,)
    if geometry.geom_type == 'MultiPolygon':
        return tuple(geometry.geoms)
    if geometry.geom_type == 'GeometryCollection':
        return tuple(
            p for part in geometry.geoms
            for p in extract_polygons(part)
        )
    return ()


def rasterize_rooms(
    rooms: Sequence[Polygon | MultiPolygon],
    shape: Tuple[int, int],
) -> np.ndarray:
    """Rasterize room polygons into an int32 label grid."""
    labels = np.zeros(shape, dtype=np.int32)
    valid_rooms = [r for r in rooms if r is not None and not r.is_empty]
    # Deterministic sort order
    ordered = sorted(
        valid_rooms,
        key=lambda r: (float(r.centroid.y), float(r.centroid.x), float(r.area)),
    )
    for label_idx, geom in enumerate(ordered, start=1):
        for poly in extract_polygons(geom):
            exterior = np.rint(np.asarray(poly.exterior.coords)).astype(np.int32)
            if exterior.size > 0:
                cv2.fillPoly(labels, [exterior], label_idx)
            for interior in poly.interiors:
                hole = np.rint(np.asarray(interior.coords)).astype(np.int32)
                if hole.size > 0:
                    cv2.fillPoly(labels, [hole], 0)
    return labels


def geodesic_coverage(
    labels: np.ndarray,
    source_map: SourceMap,
    cleanable: np.ndarray,
    min_component_size: int = 10,
) -> Tuple[np.ndarray, np.ndarray]:
    """Ensure all reachable cleanable free cells are assigned to a room without wall bleeding."""
    cleanable = cleanable.copy()
    labels = labels.copy()
    labels[~cleanable] = 0

    # Step 1: Filter tiny isolated noise components without seed labels
    num_labels, comp_labels, stats, _ = cv2.connectedComponentsWithStats(
        cleanable.astype(np.uint8), connectivity=8
    )
    for comp_idx in range(1, num_labels):
        area = stats[comp_idx, cv2.CC_STAT_AREA]
        comp_mask = (comp_labels == comp_idx)
        if area < min_component_size and not np.any(labels[comp_mask] > 0):
            cleanable[comp_mask] = False
            labels[comp_mask] = 0

    # Step 2: Multi-source BFS wavefront propagation
    unassigned = cleanable & (labels == 0)
    if not unassigned.any():
        return labels, cleanable

    rows, cols = labels.shape
    kernel = np.ones((3, 3), dtype=np.uint8)
    unassigned_dilated = cv2.dilate(
        unassigned.astype(np.uint8), kernel, iterations=1).astype(bool)
    seed_mask = (labels > 0) & cleanable & unassigned_dilated

    queue = deque()
    for r, c in zip(*np.nonzero(seed_mask)):
        queue.append((int(r), int(c), int(labels[r, c])))

    neighbor_offsets = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    )

    while queue:
        r, c, label = queue.popleft()
        for dr, dc in neighbor_offsets:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if cleanable[nr, nc] and labels[nr, nc] == 0:
                    labels[nr, nc] = label
                    queue.append((nr, nc, label))

    # Step 3: Handle any remaining unassigned components
    remaining_unassigned = cleanable & (labels == 0)
    if remaining_unassigned.any():
        num_rem, rem_labels, rem_stats, _ = cv2.connectedComponentsWithStats(
            remaining_unassigned.astype(np.uint8), connectivity=8
        )
        next_label = int(labels.max()) + 1 if labels.max() > 0 else 1
        for comp_idx in range(1, num_rem):
            comp_mask = (rem_labels == comp_idx)
            area = rem_stats[comp_idx, cv2.CC_STAT_AREA]
            if area >= min_component_size:
                labels[comp_mask] = next_label
                next_label += 1
            else:
                cleanable[comp_mask] = False
                labels[comp_mask] = 0

    return labels, cleanable


def extract_walls(
    source_map: SourceMap,
    extended_segments: Sequence[ExtendedSegment],
) -> Tuple[WallSegment, ...]:
    """Convert retained merged extended segments into map-frame detected walls."""
    walls: List[WallSegment] = []
    rect = (0, 0, source_map.width - 1, source_map.height - 1)
    _, _, yaw = source_map.origin

    for segment in extended_segments:
        p1 = (int(round(segment.x1)), int(round(segment.y1)))
        p2 = (int(round(segment.x2)), int(round(segment.y2)))
        inside, q1, q2 = cv2.clipLine(rect, p1, p2)
        if not inside or q1 == q2:
            continue
        x1, y1 = source_map.map_frame_from_pixel(float(q1[0]), float(q1[1]))
        x2, y2 = source_map.map_frame_from_pixel(float(q2[0]), float(q2[1]))
        local_direction = math.atan2(q2[1] - q1[1], q2[0] - q1[0]) % math.pi
        direction = (local_direction + yaw) % math.pi
        support = float(segment.weight) if segment.weight is not None else 0.0
        walls.append(WallSegment(
            x1=x1, y1=y1, x2=x2, y2=y2,
            support=min(max(support, 0.0), 1.0),
            direction_rad=direction,
        ))

    # Deterministic sort order
    walls.sort(key=lambda w: (-w.support, w.x1, w.y1, w.x2, w.y2))
    return tuple(walls)


def build_diagnostics(
    source_map: SourceMap,
    provisional: SegmentationResult,
    clean_image: np.ndarray,
    extended_segments: Sequence[ExtendedSegment],
) -> Tuple[DiagnosticImage, ...]:
    """Generate in-memory diagnostic images for inspection."""
    cleaned = np.where(clean_image > 0, 255, 0).astype(np.uint8)
    cleaned = cv2.cvtColor(cleaned[::-1, :], cv2.COLOR_GRAY2BGR)

    lines = render_source_map(source_map)
    lines_cell_order = lines[::-1, :].copy()
    for segment in extended_segments:
        p1 = (int(round(segment.x1)), int(round(segment.y1)))
        p2 = (int(round(segment.x2)), int(round(segment.y2)))
        cv2.line(lines_cell_order, p1, p2, (0, 0, 255), 1, cv2.LINE_AA)
    lines_rendered = lines_cell_order[::-1, :].copy()

    overlay = render_segmentation(source_map, provisional)

    return (
        DiagnosticImage('01_cleaned_map', cleaned),
        DiagnosticImage('02_extended_lines', lines_rendered),
        DiagnosticImage('03_labels_overlay', overlay),
    )
