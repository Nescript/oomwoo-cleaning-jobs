"""Visualization rendering for SourceMap / segmentation results (BGR images,
row 0 = top row)."""

from __future__ import annotations

import cv2
import numpy as np

from .segmentation import SegmentationResult
from .source_map import SourceMap

COLOR_FREE = (255, 255, 255)
COLOR_OCCUPIED = (0, 0, 0)
COLOR_UNKNOWN = (160, 160, 160)
#: cleanable but unclassified cells (watershed ridges, failed merges, etc.)
COLOR_UNCLASSIFIED = (0, 165, 255)  # orange (BGR)


def _to_image_orientation(img: np.ndarray) -> np.ndarray:
    """cells row order (row 0 = bottom row) -> image row order (row 0 = top row)."""
    return img[::-1, :]


def render_source_map(source_map: SourceMap, scale: int = 1) -> np.ndarray:
    """Render the base map: free=white, occupied=black, unknown=gray."""
    img = np.empty((*source_map.cells.shape, 3), dtype=np.uint8)
    img[source_map.free_mask()] = COLOR_FREE
    img[source_map.occupied_mask()] = COLOR_OCCUPIED
    img[source_map.unknown_mask()] = COLOR_UNKNOWN
    img = _to_image_orientation(img)
    if scale != 1:
        img = cv2.resize(
            img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return np.ascontiguousarray(img)


def region_color(label: int) -> tuple[int, int, int]:
    """Stable, mutually distinct color per label (BGR)."""
    hue = (label * 47) % 180
    hsv = np.uint8([[[hue, 200, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def render_segmentation(
    source_map: SourceMap,
    result: SegmentationResult,
    scale: int = 1,
    alpha: float = 0.55,
) -> np.ndarray:
    """Overlay candidate region colors on the base map; low-confidence regions
    are fainter; unclassified free cells are marked orange."""
    img = np.empty((*source_map.cells.shape, 3), dtype=np.uint8)
    img[source_map.free_mask()] = COLOR_FREE
    img[source_map.occupied_mask()] = COLOR_OCCUPIED
    img[source_map.unknown_mask()] = COLOR_UNKNOWN
    img = img.astype(np.float64)  # operate in cells row order; flip only at the end
    for region in result.regions:
        color = np.array(region_color(region.label), dtype=np.float64)
        mask = result.mask_of(region.label)
        a = alpha * (0.5 if region.low_confidence else 1.0)
        img[mask] = (1 - a) * img[mask] + a * color
    img[result.unclassified_free_mask] = COLOR_UNCLASSIFIED
    # doorway markers: solid magenta block for likely_door, gray otherwise
    # (cells row order)
    for doorway in result.doorways:
        r, c = doorway.center
        color = (255, 0, 255) if doorway.likely_door else (128, 128, 128)
        r0, r1 = max(0, r - 1), min(img.shape[0], r + 2)
        c0, c1 = max(0, c - 1), min(img.shape[1], c + 2)
        img[r0:r1, c0:c1] = color
    out = _to_image_orientation(img.astype(np.uint8))
    if scale != 1:
        out = cv2.resize(
            out, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return np.ascontiguousarray(out)
