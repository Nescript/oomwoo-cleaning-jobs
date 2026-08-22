"""Keepout / Virtual Wall 约束与 Cleanable Space 集成测试。"""

import math

import numpy as np
import pytest

from fixtures import make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, VirtualWall
from oomwoo_cleaning_jobs_core.regions import UNASSIGNED, RegionSet
from oomwoo_cleaning_jobs_core.segmentation import segment
from oomwoo_cleaning_jobs_core.source_map import FREE, SourceMap
from oomwoo_cleaning_jobs_core.validation import validate_region_set


def _map_point(source: SourceMap, row: int, col: int) -> tuple[float, float]:
    """指定 cell 的 map frame 中心点，保留 SourceMap 的 yaw。"""
    x, y, yaw = source.origin
    local_x = (col + 0.5) * source.resolution
    local_y = (row + 0.5) * source.resolution
    return (
        x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def _cell_box(source: SourceMap, row: int, col: int, radius_cells: int = 1):
    """围住目标 cell 中心的 map-frame 小方框。"""
    x, y = _map_point(source, row, col)
    half = (radius_cells + 0.25) * source.resolution
    return ((x - half, y - half), (x + half, y - half),
            (x + half, y + half), (x - half, y + half))


def test_keepout_rasterizes_map_frame_polygon():
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        keepouts=(Keepout('table', _cell_box(source, 40, 15, radius_cells=2)),))

    mask = constraints.mask_for(source)

    assert mask[40, 15]
    assert mask[40, 13]
    assert not mask[40, 5]
    assert mask.shape == source.cells.shape


def test_virtual_wall_rasterizes_as_explicit_width_strip():
    source = make_two_rooms_map()
    start = _map_point(source, 40, 10)
    end = _map_point(source, 40, 24)
    constraints = ConstraintSet(
        virtual_walls=(VirtualWall('door-barrier', start, end, source.resolution),))

    mask = constraints.mask_for(source)

    assert mask[40, 17]
    assert not mask[30, 17]


def test_constraint_rasterization_honors_source_map_yaw():
    source = SourceMap(
        resolution=0.1,
        width=20,
        height=20,
        origin=(3.0, -1.0, math.pi / 2),
        cells=np.full((20, 20), FREE, dtype=np.int8),
    )
    constraints = ConstraintSet(
        keepouts=(Keepout('rotated', _cell_box(source, 4, 7)),))

    mask = constraints.mask_for(source)

    assert mask[4, 7]
    assert not mask[7, 4]


def test_constraints_exclude_candidates_and_clip_existing_regions():
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        keepouts=(Keepout('left-table', _cell_box(source, 40, 15, radius_cells=2)),))
    keepout_mask = constraints.mask_for(source)

    constrained = segment(
        source, cleanable_mask=source.free_mask() & ~keepout_mask)
    assert not (constrained.labels[keepout_mask] != UNASSIGNED).any()

    # 从已排除 Keepout 的候选初始化时，仍需保留原始 free mask，才能在
    # 日后移除约束时恢复 Cleanable Space（不会恢复被裁掉的 Region cell）。
    constrained_set = RegionSet.from_segmentation(
        constrained,
        resolution=source.resolution,
        origin=source.origin,
        base_cleanable=source.free_mask(),
        keepout_mask=keepout_mask,
    )
    constrained_set.apply_keepout_mask(np.zeros(source.cells.shape, dtype=bool))
    assert constrained_set.cleanable[40, 15]
    assert constrained_set.labels[40, 15] == UNASSIGNED

    original = segment(source)
    region_set = RegionSet.from_segmentation(
        original, resolution=source.resolution, origin=source.origin)
    assert (region_set.labels[keepout_mask] != UNASSIGNED).any()

    region_set.apply_keepout_mask(keepout_mask)
    assert not (region_set.labels[keepout_mask] != UNASSIGNED).any()
    assert not region_set.paint(region_set.regions()[0].label, keepout_mask)

    # 移除约束只恢复可清扫性，不复活此前被裁掉的 Region cell。
    region_set.apply_keepout_mask(np.zeros(source.cells.shape, dtype=bool))
    assert region_set.cleanable[40, 15]
    assert region_set.labels[40, 15] == UNASSIGNED


def test_constraint_inputs_and_validation_reject_wrong_shape():
    source = make_two_rooms_map()
    result = segment(source)
    region_set = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)

    with pytest.raises(ValueError, match='形状'):
        region_set.apply_keepout_mask(np.zeros((2, 2), dtype=bool))
    with pytest.raises(ValueError, match='形状'):
        segment(source, cleanable_mask=np.zeros((2, 2), dtype=bool))
    with pytest.raises(ValueError, match='形状'):
        validate_region_set(region_set, keepout_mask=np.zeros((2, 2), dtype=bool))
