"""Provider-neutral rendering for source maps and canonical room labels."""

from __future__ import annotations

import cv2
import numpy as np

from .models import SegmentationResult, WallSegment
from .source_map import SourceMap

COLOR_FREE = (255, 255, 255)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_UNKNOWN = (160, 160, 160)
COLOR_UNASSIGNED = (0, 165, 255)  # orange, BGR


def _to_image_orientation(image: np.ndarray) -> np.ndarray:
    return image[::-1, :]


def render_source_map(source_map: SourceMap, scale: int = 1) -> np.ndarray:
    """Render free, occupied, and unknown cells as a BGR image."""
    if scale < 1:
        raise ValueError('scale must be >= 1')
    image = np.empty((*source_map.cells.shape, 3), dtype=np.uint8)
    image[source_map.free_mask()] = COLOR_FREE
    image[source_map.occupied_mask()] = COLOR_OCCUPIED
    image[source_map.unknown_mask()] = COLOR_UNKNOWN
    image = _to_image_orientation(image)
    if scale != 1:
        image = cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return np.ascontiguousarray(image)


def region_color(label: int) -> tuple[int, int, int]:
    """Return a deterministic, visually distinct BGR color for a room label."""
    hue = (int(label) * 47) % 180
    hsv = np.uint8([[[hue, 200, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def _support_color(support: float) -> tuple[int, int, int]:
    """Stable BGR color for a wall support value: yellow (0) to red (1)."""
    support = min(max(float(support), 0.0), 1.0)
    return (0, int(round(255 * (1.0 - support))), 255)


def render_walls(
    source_map: SourceMap,
    walls: tuple[WallSegment, ...],
    scale: int = 1,
    *,
    base: np.ndarray | None = None,
) -> np.ndarray:
    """Overlay detected wall segments on the map, colored by support."""
    if scale < 1:
        raise ValueError('scale must be >= 1')
    image = render_source_map(source_map) if base is None else base.copy()
    # np.flip-style views have negative strides; OpenCV needs contiguous memory.
    cell_order = np.ascontiguousarray(image[::-1, :])
    for wall in walls:
        px1, py1 = source_map.pixel_from_map_frame(wall.x1, wall.y1)
        px2, py2 = source_map.pixel_from_map_frame(wall.x2, wall.y2)
        cv2.line(
            cell_order,
            (int(round(px1)), int(round(py1))),
            (int(round(px2)), int(round(py2))),
            _support_color(wall.support),
            1,
            cv2.LINE_AA,
        )
    image = np.ascontiguousarray(cell_order[::-1, :])
    if scale != 1:
        image = cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return image


def render_segmentation(
    source_map: SourceMap,
    result: SegmentationResult,
    scale: int = 1,
    alpha: float = 0.55,
    *,
    draw_labels: bool = True,
) -> np.ndarray:
    """Overlay canonical room labels and optional room numbers on the map."""
    if not 0.0 <= alpha <= 1.0:
        raise ValueError('alpha must be between 0 and 1')
    base = np.empty((*source_map.cells.shape, 3), dtype=np.uint8)
    base[source_map.free_mask()] = COLOR_FREE
    base[source_map.occupied_mask()] = COLOR_OCCUPIED
    base[source_map.unknown_mask()] = COLOR_UNKNOWN
    image = base.astype(np.float64)
    for region in result.regions:
        mask = result.mask_of(region.label)
        color = np.asarray(region_color(region.label), dtype=np.float64)
        image[mask] = (1.0 - alpha) * image[mask] + alpha * color
    image[result.unassigned_cleanable_mask] = COLOR_UNASSIGNED
    # np.flip returns a negative-stride view; OpenCV drawing requires a
    # contiguous buffer even when no resize is requested.
    image = np.ascontiguousarray(_to_image_orientation(image.astype(np.uint8)))

    if scale != 1:
        image = cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    if draw_labels:
        for region in result.regions:
            rows, cols = np.nonzero(result.mask_of(region.label))
            if not rows.size:
                continue
            x = int(round(float(cols.mean()) * scale))
            y = int(round((source_map.height - 1 - float(rows.mean())) * scale))
            text = str(region.label)
            font_scale = max(0.4, 0.45 * scale)
            thickness = max(1, scale)
            cv2.putText(
                image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (255, 255, 255), thickness + 2, cv2.LINE_AA)
            cv2.putText(
                image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return np.ascontiguousarray(image)
