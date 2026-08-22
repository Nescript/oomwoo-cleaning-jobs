"""ROS OccupancyGrid adapter; keeps ROS types out of the core package."""
from __future__ import annotations
import math
import numpy as np
from oomwoo_cleaning_jobs_core.source_map import SourceMap

def source_map_from_occupancy_grid(message) -> SourceMap:
    info = message.info
    width, height = int(info.width), int(info.height)
    data = np.asarray(message.data, dtype=np.int8)
    if data.size != width * height:
        raise ValueError('OccupancyGrid data length does not match width*height')
    q = info.origin.orientation
    # planar yaw from normalized quaternion; SourceMap identity intentionally omits header.
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return SourceMap(float(info.resolution), width, height,
                     (float(info.origin.position.x), float(info.origin.position.y), yaw),
                     data.reshape((height, width)))
