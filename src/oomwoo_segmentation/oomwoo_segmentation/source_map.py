"""Core-library representation of a Source Map and its identity computation.

Corresponds to docs/DEVELOPMENT.md, "Phase 1 implementation decisions ·
map identity and change detection".

The core library has zero ROS dependencies: the ROS adapter layer converts
``nav_msgs/msg/OccupancyGrid`` into :class:`SourceMap`. Identity rule
(matching the document):

SHA-256(resolution as **float32** bytes + width + height
        + origin position(x, y, 0) and orientation quaternion
          (0, 0, sin(yaw/2), cos(yaw/2))
        + raw int8 cell data)

resolution is canonicalized to float32 because
`OccupancyGrid.info.resolution` is float32 while map.yaml stores float64;
without canonicalization the same map would yield different identities via
topic versus file.

``header.stamp`` / ``frame_id`` / ``map_load_time`` are excluded and no
trinarization is applied; ``SourceMap`` therefore simply does not carry
those fields. The short id is the first 12 characters of the identity.
"""

from __future__ import annotations

import hashlib
import math
import struct

import numpy as np

# the three cell values after trinary loading (matching nav2 map_io)
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

#: default free threshold, matching DEVELOPMENT.md
#: (0 <= v < free_thresh*100 is free)
DEFAULT_FREE_THRESH = 0.25


class SourceMap:
    """Immutable saved map (the Source Map of the domain model).

    cells is an int8 array of shape (height, width); row 0 is the map's
    bottom row (the row-major convention of OccupancyGrid data, opposite to
    image files where the top row comes first).
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
            raise ValueError(f'origin must be (x, y, yaw), got {origin!r}')
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]))
        self.cells = cells

    @property
    def identity(self) -> str:
        """Content hash (full sha256 hex). A changed hash means a new map.

        resolution is canonicalized to float32 bytes first:
        `nav_msgs/OccupancyGrid.info.resolution` is float32 while map.yaml
        stores float64 — the same map loaded via the /map topic and via file
        must produce the same identity. origin is float64 in both sources
        (geometry_msgs/Pose) and is packed directly."""
        x, y, yaw = self.origin
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        h = hashlib.sha256()
        h.update(struct.pack('<f', self.resolution))  # float32 canonicalization
        h.update(struct.pack('<II', self.width, self.height))
        # origin position (x, y, z=0) + orientation quaternion (0, 0, qz, qw)
        h.update(struct.pack('<ddddddd', x, y, 0.0, 0.0, 0.0, qz, qw))
        h.update(self.cells.tobytes())
        return h.hexdigest()

    @property
    def short_id(self) -> str:
        return self.identity[:12]

    def free_mask(self, free_thresh: float = DEFAULT_FREE_THRESH) -> np.ndarray:
        """Boolean mask of known free (cleanable) cells: 0 <= v < free_thresh*100."""
        return (self.cells >= 0) & (self.cells < free_thresh * 100)

    def unknown_mask(self) -> np.ndarray:
        return self.cells < 0

    def occupied_mask(self, free_thresh: float = DEFAULT_FREE_THRESH) -> np.ndarray:
        """Known but not cleanable cells (obstacles or occupancy values above
        the free threshold)."""
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
