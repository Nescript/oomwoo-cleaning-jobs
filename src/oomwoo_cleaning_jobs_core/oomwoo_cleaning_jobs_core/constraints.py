"""Pure-Python domain model and rasterization for Keepouts and Virtual Walls.

Constraints use metric coordinates in the map frame; rasterization strictly
follows the ``SourceMap`` convention that row 0 is the map's minimum y, and
honors the origin yaw. A Keepout is a polygon; a Virtual Wall is a linear
Keepout dilated by an explicit width.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from oomwoo_segmentation.models import WallSegment
from oomwoo_segmentation.source_map import SourceMap

Point = tuple[float, float]


def _points(points: tuple[Point, ...], description: str) -> tuple[Point, ...]:
    normalized = tuple((float(x), float(y)) for x, y in points)
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in normalized):
        raise ValueError(f'{description} coordinates must be finite')
    return normalized


@dataclass(frozen=True)
class Keepout:
    """Closed polygon Keepout in the map frame (no repeated closing point)."""

    identifier: str
    vertices: tuple[Point, ...]

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError('Keepout identifier must not be empty')
        vertices = _points(self.vertices, 'Keepout')
        if len(vertices) < 3:
            raise ValueError('Keepout needs at least three vertices')
        area2 = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(vertices, vertices[1:] + vertices[:1])
        )
        if math.isclose(area2, 0.0, abs_tol=1e-12):
            raise ValueError('Keepout vertices must not be collinear')
        object.__setattr__(self, 'vertices', vertices)


@dataclass(frozen=True)
class VirtualWall:
    """Linear Keepout in the map frame, represented with an explicit width."""

    identifier: str
    start: Point
    end: Point
    width_m: float

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError('VirtualWall identifier must not be empty')
        start, end = _points((self.start, self.end), 'VirtualWall')
        if start == end:
            raise ValueError('VirtualWall start and end must differ')
        width_m = float(self.width_m)
        if not math.isfinite(width_m) or width_m <= 0:
            raise ValueError('VirtualWall width_m must be a positive finite value')
        object.__setattr__(self, 'start', start)
        object.__setattr__(self, 'end', end)
        object.__setattr__(self, 'width_m', width_m)

    @classmethod
    def from_detected_wall(
        cls,
        identifier: str,
        wall: WallSegment,
        width_m: float,
    ) -> VirtualWall:
        """Convert a Detected Wall into a Virtual Wall candidate.

        Detected walls are algorithm suggestions; this conversion only
        happens on explicit user confirmation, so the resulting VirtualWall
        remains a user-owned, independently persisted constraint.
        """
        return cls(
            identifier=identifier,
            start=(wall.x1, wall.y1),
            end=(wall.x2, wall.y2),
            width_m=width_m,
        )

    @property
    def polygon(self) -> tuple[Point, Point, Point, Point]:
        """Dilate the center line by width_m into a rectangular Keepout."""
        x0, y0 = self.start
        x1, y1 = self.end
        length = math.hypot(x1 - x0, y1 - y0)
        half_width = self.width_m / 2.0
        nx = -(y1 - y0) / length * half_width
        ny = (x1 - x0) / length * half_width
        return (
            (x0 + nx, y0 + ny),
            (x1 + nx, y1 + ny),
            (x1 - nx, y1 - ny),
            (x0 - nx, y0 - ny),
        )


@dataclass(frozen=True)
class ConstraintSet:
    """Set of spatial constraints belonging to one Region Set."""

    keepouts: tuple[Keepout, ...] = ()
    virtual_walls: tuple[VirtualWall, ...] = ()

    def __post_init__(self) -> None:
        identifiers = [item.identifier for item in self.keepouts]
        identifiers.extend(item.identifier for item in self.virtual_walls)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError('Keepout and VirtualWall identifiers must be globally unique')

    def mask_for(self, source_map: SourceMap) -> np.ndarray:
        """Return the union mask of all Keepouts, same shape as the SourceMap."""
        mask = np.zeros(source_map.cells.shape, dtype=bool)
        for keepout in self.keepouts:
            mask |= _rasterize_polygon(keepout.vertices, source_map)
        for wall in self.virtual_walls:
            mask |= _rasterize_polygon(wall.polygon, source_map)
        return mask


def _rasterize_polygon(vertices: tuple[Point, ...], source_map: SourceMap) -> np.ndarray:
    """Rasterize a map-frame polygon, covering cells whose centers lie
    inside it (boundary included)."""
    ox, oy, yaw = source_map.origin
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    # map frame -> map-local frame; then convert cell-center coordinates to
    # OpenCV pixel centers.
    local = []
    for x, y in vertices:
        dx, dy = x - ox, y - oy
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        local.append((local_x / source_map.resolution - 0.5,
                      local_y / source_map.resolution - 0.5))

    # shift avoids whole-pixel rounding offsetting small polygons by one cell;
    # OpenCV (x, y) is (col, row).
    shift = 16
    polygon = np.rint(np.asarray(local, dtype=np.float64) * (1 << shift)).astype(np.int32)
    raster = np.zeros(source_map.cells.shape, dtype=np.uint8)
    cv2.fillPoly(raster, [polygon], color=1, lineType=cv2.LINE_8, shift=shift)
    return raster.astype(bool)
