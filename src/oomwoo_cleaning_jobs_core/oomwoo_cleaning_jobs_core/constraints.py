"""Pure-Python domain model and rasterization for Keepouts, Virtual Walls, and Spot Areas.

Constraints use metric coordinates in the map frame; rasterization strictly
follows the ``SourceMap`` convention that row 0 is the map's minimum y, and
honors the origin yaw. A Keepout is a polygon; a Virtual Wall is a linear
Keepout dilated by an explicit width. A Spot Area is a positive transient target
polygon retained at the constraint layer.

Virtual Walls are sealing barriers: they are rasterized as 8-connected bands
so that a closed wall chain provably blocks a 4-connected flood fill over the
remaining free space (grid Jordan curve property). Keepouts may optionally be
dilated by a configurable metric margin; Virtual Walls never take that margin.
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
class SpotArea:
    """Positive transient target polygon in the map frame (retained for spot cleaning)."""

    identifier: str
    vertices: tuple[Point, ...]
    name: str = 'Spot Area'

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError('SpotArea identifier must not be empty')
        vertices = _points(self.vertices, 'SpotArea')
        if len(vertices) < 3:
            raise ValueError('SpotArea needs at least three vertices')
        area2 = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(vertices, vertices[1:] + vertices[:1])
        )
        if math.isclose(area2, 0.0, abs_tol=1e-12):
            raise ValueError('SpotArea vertices must not be collinear')
        object.__setattr__(self, 'vertices', vertices)

    @classmethod
    def from_box(
        cls,
        center: Point,
        width_m: float,
        height_m: float,
        identifier: str = 'spot_area',
        name: str = 'Spot Area',
    ) -> SpotArea:
        """Create a rectangular SpotArea centered at `center` with metric width and height."""
        width_m = float(width_m)
        height_m = float(height_m)
        if not (math.isfinite(width_m) and width_m > 0 and math.isfinite(height_m) and height_m > 0):
            raise ValueError('SpotArea box dimensions must be positive finite values')
        cx, cy = float(center[0]), float(center[1])
        hw = width_m / 2.0
        hh = height_m / 2.0
        return cls(
            identifier=identifier,
            vertices=(
                (cx - hw, cy - hh),
                (cx + hw, cy - hh),
                (cx + hw, cy + hh),
                (cx - hw, cy + hh),
            ),
            name=name,
        )


@dataclass(frozen=True)
class ConstraintSet:
    """Set of spatial constraints belonging to one Region Set."""

    keepouts: tuple[Keepout, ...] = ()
    virtual_walls: tuple[VirtualWall, ...] = ()
    spot_area: SpotArea | None = None

    def __post_init__(self) -> None:
        identifiers = [item.identifier for item in self.keepouts]
        identifiers.extend(item.identifier for item in self.virtual_walls)
        if self.spot_area is not None:
            identifiers.append(self.spot_area.identifier)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError('Keepout, VirtualWall, and SpotArea identifiers must be globally unique')

    def with_spot_area(self, spot_area: SpotArea | None) -> ConstraintSet:
        """Return a copy of this ConstraintSet with updated spot_area."""
        return ConstraintSet(
            keepouts=self.keepouts,
            virtual_walls=self.virtual_walls,
            spot_area=spot_area,
        )

    def mask_for(self, source_map: SourceMap, keepout_margin_m: float = 0.0) -> np.ndarray:
        """Return the union mask of all negative constraints, same shape as the SourceMap.

        Keepout polygons use cell-center rasterization, optionally dilated by
        a circular ``keepout_margin_m`` (meters, default 0 = exact). The
        margin exists for deployments that require the whole robot footprint
        -- not just its center -- to stay out of a Keepout. Virtual Wall
        bands never take the margin: a wall's purpose is to seal an opening,
        not to mark a precise boundary.
        """
        margin = float(keepout_margin_m)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError('keepout_margin_m must be a non-negative finite value')
        mask = np.zeros(source_map.cells.shape, dtype=bool)
        for keepout in self.keepouts:
            mask |= _rasterize_polygon(keepout.vertices, source_map)
        if margin > 0.0 and mask.any():
            mask = _dilate_mask(mask, margin, source_map.resolution)
        for wall in self.virtual_walls:
            mask |= _rasterize_wall_band(wall, source_map)
        return mask

    def spot_mask_for(self, source_map: SourceMap) -> np.ndarray | None:
        """Return the rasterized mask of the retained spot_area if present, else None."""
        if self.spot_area is None:
            return None
        return _rasterize_polygon(self.spot_area.vertices, source_map)


def _map_to_pixel(point: Point, source_map: SourceMap) -> tuple[float, float]:
    """Map-frame point -> fractional OpenCV pixel (col, row), cell-center convention."""
    ox, oy, yaw = source_map.origin
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx, dy = point[0] - ox, point[1] - oy
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return (local_x / source_map.resolution - 0.5,
            local_y / source_map.resolution - 0.5)


def _dilate_mask(mask: np.ndarray, margin_m: float, resolution: float) -> np.ndarray:
    """Dilate a boolean mask by a circular margin in meters."""
    radius_px = int(math.ceil(margin_m / resolution))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1))
    return cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)


def _rasterize_wall_band(wall: VirtualWall, source_map: SourceMap) -> np.ndarray:
    """Rasterize a Virtual Wall as an 8-connected band.

    The band is the union of the wall's ``width_m`` rectangle (same
    cell-center semantics as Keepouts) and a 1-cell 8-connected spine along
    its center line. The spine guarantees that a closed chain of walls has
    no diagonal gaps, so a 4-connected flood fill over the remaining free
    space cannot leak through (grid Jordan curve property). For walls
    narrower than one cell this may widen the band slightly beyond the
    strict ``width_m`` rectangle; walls are sealing barriers, not precise
    boundaries, so the widening is intentional.
    """
    band = _rasterize_polygon(wall.polygon, source_map)
    shift = 16
    start = _map_to_pixel(wall.start, source_map)
    end = _map_to_pixel(wall.end, source_map)
    pt1 = (int(round(start[0] * (1 << shift))), int(round(start[1] * (1 << shift))))
    pt2 = (int(round(end[0] * (1 << shift))), int(round(end[1] * (1 << shift))))
    spine = np.zeros(source_map.cells.shape, dtype=np.uint8)
    cv2.line(spine, pt1, pt2, color=1, thickness=1, lineType=cv2.LINE_8, shift=shift)
    return band | spine.astype(bool)


def _rasterize_polygon(vertices: tuple[Point, ...], source_map: SourceMap) -> np.ndarray:
    """Rasterize a map-frame polygon, covering cells whose centers lie
    inside it (boundary included)."""
    # shift avoids whole-pixel rounding offsetting small polygons by one cell;
    # OpenCV (x, y) is (col, row).
    shift = 16
    local = [_map_to_pixel(vertex, source_map) for vertex in vertices]
    polygon = np.rint(np.asarray(local, dtype=np.float64) * (1 << shift)).astype(np.int32)
    raster = np.zeros(source_map.cells.shape, dtype=np.uint8)
    cv2.fillPoly(raster, [polygon], color=1, lineType=cv2.LINE_8, shift=shift)
    return raster.astype(bool)
