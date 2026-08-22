"""Source Map 的核心库表示与 identity 计算。

对应 docs/DEVELOPMENT.md「第一阶段实施决定 · 地图 identity 与变更检测」。

核心库零 ROS 依赖：ROS 适配层负责把 ``nav_msgs/msg/OccupancyGrid``
转成 :class:`SourceMap`。identity 规则（与文档一致）：

SHA-256(resolution 的 **float32** 字节 + width + height
        + origin position(x, y, 0) 与 orientation 四元数(0, 0, sin(yaw/2), cos(yaw/2))
        + 原始 int8 cell 数据)

resolution 规范化为 float32 是因为 `OccupancyGrid.info.resolution` 为
float32 而 map.yaml 为 float64；不规范化时同一张地图经话题与经文件
会得到不同 identity。

排除 ``header.stamp`` / ``frame_id`` / ``map_load_time``，不做三值化；
因此 ``SourceMap`` 干脆不携带这些字段。短 id 取 identity 前 12 位。
"""

from __future__ import annotations

import hashlib
import math
import struct

import numpy as np

# trinary 加载后 cell 的三种取值（与 nav2 map_io 一致）
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

#: 与 DEVELOPMENT.md 一致的默认 free 判定阈值（0 <= v < free_thresh*100 为自由）
DEFAULT_FREE_THRESH = 0.25


class SourceMap:
    """不可变的已保存地图（对应领域模型 Source Map）。

    cells 为 int8 数组，shape (height, width)，row 0 是地图最底行
    （OccupancyGrid data 的行优先约定，与图像文件的顶行在下相反）。
    """

    __slots__ = ('resolution', 'width', 'height', 'origin', 'cells')

    def __init__(
        self,
        resolution: float,
        width: int,
        height: int,
        origin: tuple[float, float, float],
        cells: np.ndarray,
    ) -> None:
        cells = np.ascontiguousarray(cells, dtype=np.int8)
        if cells.shape != (height, width):
            raise ValueError(
                f'cells shape {cells.shape} != (height, width) = {(height, width)}'
            )
        if len(origin) != 3:
            raise ValueError(f'origin 必须为 (x, y, yaw)，得到 {origin!r}')
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]))
        self.cells = cells

    @property
    def identity(self) -> str:
        """内容 hash（完整 sha256 hex）。hash 变更即视为新地图。

        resolution 先规范化为 float32 字节：`nav_msgs/OccupancyGrid.info.
        resolution` 是 float32，而 map.yaml 里是 float64——同一张地图
        经 /map 话题与经文件加载必须得到相同 identity。origin 在两个
        来源都是 float64（geometry_msgs/Pose），直接打包。"""
        x, y, yaw = self.origin
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        h = hashlib.sha256()
        h.update(struct.pack('<f', self.resolution))  # float32 规范化
        h.update(struct.pack('<II', self.width, self.height))
        # origin position (x, y, z=0) + orientation quaternion (0, 0, qz, qw)
        h.update(struct.pack('<ddddddd', x, y, 0.0, 0.0, 0.0, qz, qw))
        h.update(self.cells.tobytes())
        return h.hexdigest()

    @property
    def short_id(self) -> str:
        return self.identity[:12]

    def free_mask(self, free_thresh: float = DEFAULT_FREE_THRESH) -> np.ndarray:
        """已知自由 cell（可清扫）布尔掩码：0 <= v < free_thresh*100。"""
        return (self.cells >= 0) & (self.cells < free_thresh * 100)

    def unknown_mask(self) -> np.ndarray:
        return self.cells < 0

    def occupied_mask(self, free_thresh: float = DEFAULT_FREE_THRESH) -> np.ndarray:
        """已知但不可清扫 cell（障碍或高于 free 阈值的占用值）。"""
        return self.cells >= free_thresh * 100

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SourceMap):
            return NotImplemented
        return (
            self.resolution == other.resolution
            and self.width == other.width
            and self.height == other.height
            and self.origin == other.origin
            and np.array_equal(self.cells, other.cells)
        )

    def __repr__(self) -> str:
        return (
            f'SourceMap({self.width}x{self.height} @ {self.resolution} m/cell, '
            f'origin={self.origin}, id={self.short_id})'
        )
