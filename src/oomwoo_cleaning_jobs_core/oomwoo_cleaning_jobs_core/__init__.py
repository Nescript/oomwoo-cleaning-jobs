"""oomwoo_cleaning_jobs_core: pure Python core library (zero ROS dependencies).

Scope and terminology: see docs/DEVELOPMENT.md in the repository.
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
