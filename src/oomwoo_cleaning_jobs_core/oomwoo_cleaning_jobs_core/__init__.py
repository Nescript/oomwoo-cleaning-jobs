"""Domain logic for editable and persisted cleaning Region Sets.

Room segmentation, map loading, and rendering live in ``oomwoo_segmentation``.
The map symbols are re-exported here only for existing callers.
"""

from oomwoo_segmentation.map_io import load_map_file
from oomwoo_segmentation.source_map import (
    DEFAULT_FREE_THRESH,
    FREE,
    OCCUPIED,
    UNKNOWN,
    SourceMap,
)

from .constraints import ConstraintSet, Keepout, VirtualWall
from .persistence import RegionSetStore, StoredRegionSet

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
