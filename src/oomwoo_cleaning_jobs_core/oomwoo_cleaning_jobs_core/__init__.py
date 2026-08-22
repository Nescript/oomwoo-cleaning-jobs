"""oomwoo_cleaning_jobs_core：纯 Python 核心库（零 ROS 依赖）。

范围与术语见仓库 docs/DEVELOPMENT.md。
"""

from .constraints import ConstraintSet, Keepout, VirtualWall
from .map_io import load_map_file
from .persistence import RegionSetStore, StoredRegionSet
from .source_map import (
    DEFAULT_FREE_THRESH,
    FREE,
    OCCUPIED,
    UNKNOWN,
    SourceMap,
)

__all__ = [
    'ConstraintSet',
    'Keepout',
    'VirtualWall',
    'RegionSetStore',
    'StoredRegionSet',
    'DEFAULT_FREE_THRESH',
    'FREE',
    'OCCUPIED',
    'UNKNOWN',
    'SourceMap',
    'load_map_file',
]
