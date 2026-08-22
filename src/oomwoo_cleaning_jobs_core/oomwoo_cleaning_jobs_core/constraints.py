"""Keepout 与 Virtual Wall 的纯 Python 领域模型及栅格化。

约束使用 map frame 的米制坐标；栅格化时严格遵循 ``SourceMap`` 的
row 0 为地图最小 y 的约定，并处理 origin 的 yaw。Keepout 是多边形，
Virtual Wall 是按显式宽度膨胀后的线状 Keepout。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .source_map import SourceMap

Point = tuple[float, float]


def _points(points: tuple[Point, ...], description: str) -> tuple[Point, ...]:
    normalized = tuple((float(x), float(y)) for x, y in points)
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in normalized):
        raise ValueError(f'{description} 坐标必须为有限数值')
    return normalized


@dataclass(frozen=True)
class Keepout:
    """map frame 中的闭合多边形 Keepout（首尾点无需重复）。"""

    identifier: str
    vertices: tuple[Point, ...]

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError('Keepout identifier 不能为空')
        vertices = _points(self.vertices, 'Keepout')
        if len(vertices) < 3:
            raise ValueError('Keepout 至少需要三个顶点')
        area2 = sum(
            x1 * y2 - x2 * y1
            for (x1, y1), (x2, y2) in zip(vertices, vertices[1:] + vertices[:1])
        )
        if math.isclose(area2, 0.0, abs_tol=1e-12):
            raise ValueError('Keepout 顶点不能共线')
        object.__setattr__(self, 'vertices', vertices)


@dataclass(frozen=True)
class VirtualWall:
    """map frame 中以显式宽度表示的线状 Keepout。"""

    identifier: str
    start: Point
    end: Point
    width_m: float

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError('VirtualWall identifier 不能为空')
        start, end = _points((self.start, self.end), 'VirtualWall')
        if start == end:
            raise ValueError('VirtualWall 起点与终点不能相同')
        width_m = float(self.width_m)
        if not math.isfinite(width_m) or width_m <= 0:
            raise ValueError('VirtualWall width_m 必须为正的有限数值')
        object.__setattr__(self, 'start', start)
        object.__setattr__(self, 'end', end)
        object.__setattr__(self, 'width_m', width_m)

    @property
    def polygon(self) -> tuple[Point, Point, Point, Point]:
        """将中心线按 width_m 膨胀为矩形 Keepout。"""
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
    """属于一张 Region Set 的空间约束集合。"""

    keepouts: tuple[Keepout, ...] = ()
    virtual_walls: tuple[VirtualWall, ...] = ()

    def __post_init__(self) -> None:
        identifiers = [item.identifier for item in self.keepouts]
        identifiers.extend(item.identifier for item in self.virtual_walls)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError('Keepout 与 VirtualWall identifier 必须全局唯一')

    def mask_for(self, source_map: SourceMap) -> np.ndarray:
        """返回与 SourceMap 同 shape 的 Keepout 并集掩码。"""
        mask = np.zeros(source_map.cells.shape, dtype=bool)
        for keepout in self.keepouts:
            mask |= _rasterize_polygon(keepout.vertices, source_map)
        for wall in self.virtual_walls:
            mask |= _rasterize_polygon(wall.polygon, source_map)
        return mask


def _rasterize_polygon(vertices: tuple[Point, ...], source_map: SourceMap) -> np.ndarray:
    """栅格化 map frame 多边形，覆盖中心点位于其中（含边界）的 cell。"""
    ox, oy, yaw = source_map.origin
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    # map frame → map-local frame；再把 cell center 坐标换成 OpenCV 像素中心。
    local = []
    for x, y in vertices:
        dx, dy = x - ox, y - oy
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        local.append((local_x / source_map.resolution - 0.5,
                      local_y / source_map.resolution - 0.5))

    # shift 避免整像素 round 造成小多边形偏一格；OpenCV 的 (x, y) 即 (col, row)。
    shift = 16
    polygon = np.rint(np.asarray(local, dtype=np.float64) * (1 << shift)).astype(np.int32)
    raster = np.zeros(source_map.cells.shape, dtype=np.uint8)
    cv2.fillPoly(raster, [polygon], color=1, lineType=cv2.LINE_8, shift=shift)
    return raster.astype(bool)
