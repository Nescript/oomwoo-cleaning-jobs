"""oomwoo_cleaning_jobs_core：纯 Python 核心库（零 ROS 依赖）。

范围与术语见仓库 docs/DEVELOPMENT.md。
"""

from .map_io import load_map_file
from .source_map import (
    DEFAULT_FREE_THRESH,
    FREE,
    OCCUPIED,
    UNKNOWN,
    SourceMap,
)

__all__ = [
    'DEFAULT_FREE_THRESH',
    'FREE',
    'OCCUPIED',
    'UNKNOWN',
    'SourceMap',
    'load_map_file',
]
