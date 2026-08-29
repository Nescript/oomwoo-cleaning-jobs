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

from .constraints import ConstraintSet, Keepout, SpotArea, VirtualWall
from .dock import DEFAULT_STAGING_OFFSET_M, load_dock_pose, staging_pose
from .persistence import PublishError, RegionSetStore, StoredRegionSet
from .targets import (
    CleaningTarget,
    configure_last_spot_area,
    configure_selected_regions,
    configure_spot_area,
    configure_whole_map,
    create_spot_region_set,
)

__all__ = [
    'ConstraintSet',
    'Keepout',
    'VirtualWall',
    'SpotArea',
    'RegionSetStore',
    'StoredRegionSet',
    'PublishError',
    'load_dock_pose',
    'staging_pose',
    'DEFAULT_STAGING_OFFSET_M',
    'CleaningTarget',
    'configure_whole_map',
    'configure_selected_regions',
    'configure_spot_area',
    'configure_last_spot_area',
    'create_spot_region_set',
    'DEFAULT_FREE_THRESH',
    'FREE',
    'OCCUPIED',
    'UNKNOWN',
    'SourceMap',
    'load_map_file',
]
