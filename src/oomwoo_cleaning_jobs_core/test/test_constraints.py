"""Tests for Keepout / Virtual Wall constraints and Cleanable Space integration."""

import math

import numpy as np
import pytest

from fixtures import fake_segmentation, make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, SpotArea, VirtualWall
from oomwoo_cleaning_jobs_core.regions import UNASSIGNED, RegionSet
from oomwoo_segmentation.source_map import FREE, SourceMap
from oomwoo_cleaning_jobs_core.validation import validate_region_set


def _map_point(source: SourceMap, row: int, col: int) -> tuple[float, float]:
    """Map-frame center point of a given cell, honoring the SourceMap yaw."""
    x, y, yaw = source.origin
    local_x = (col + 0.5) * source.resolution
    local_y = (row + 0.5) * source.resolution
    return (
        x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def _cell_box(source: SourceMap, row: int, col: int, radius_cells: int = 1):
    """Small map-frame box enclosing the target cell center."""
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

    constrained = fake_segmentation(
        source, cleanable_mask=source.free_mask() & ~keepout_mask)
    assert not (constrained.labels[keepout_mask] != UNASSIGNED).any()

    # When initializing from candidates that already exclude the Keepout, the
    # original free mask must still be kept so the Cleanable Space can be
    # restored when the constraint is later removed (without reviving the
    # clipped Region cells).
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

    original = fake_segmentation(source)
    region_set = RegionSet.from_segmentation(
        original, resolution=source.resolution, origin=source.origin)
    assert (region_set.labels[keepout_mask] != UNASSIGNED).any()

    region_set.apply_keepout_mask(keepout_mask)
    assert not (region_set.labels[keepout_mask] != UNASSIGNED).any()
    assert not region_set.paint(region_set.regions()[0].label, keepout_mask)

    # Removing the constraint only restores cleanability; it does not revive
    # the previously clipped Region cells.
    region_set.apply_keepout_mask(np.zeros(source.cells.shape, dtype=bool))
    assert region_set.cleanable[40, 15]
    assert region_set.labels[40, 15] == UNASSIGNED


def test_constraint_inputs_and_validation_reject_wrong_shape():
    source = make_two_rooms_map()
    result = fake_segmentation(source)
    region_set = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)

    with pytest.raises(ValueError, match='shape'):
        region_set.apply_keepout_mask(np.zeros((2, 2), dtype=bool))
    with pytest.raises(ValueError, match='shape'):
        fake_segmentation(source, cleanable_mask=np.zeros((2, 2), dtype=bool))
    with pytest.raises(ValueError, match='shape'):
        validate_region_set(region_set, keepout_mask=np.zeros((2, 2), dtype=bool))


def test_virtual_wall_from_detected_wall():
    from oomwoo_segmentation.models import WallSegment

    wall = WallSegment(x1=1.0, y1=2.0, x2=3.0, y2=2.0,
                       support=0.9, direction_rad=0.0)
    converted = VirtualWall.from_detected_wall('wall-1', wall, width_m=0.1)

    assert converted.identifier == 'wall-1'
    assert converted.start == (1.0, 2.0)
    assert converted.end == (3.0, 2.0)
    assert converted.width_m == 0.1
    assert len(converted.polygon) == 4


def test_spot_area_creation_and_rasterization():
    source = make_two_rooms_map()
    spot = SpotArea.from_box(center=_map_point(source, 30, 20), width_m=0.5, height_m=0.5,
                             identifier='my_spot', name='Dinner Mess')

    assert spot.identifier == 'my_spot'
    assert spot.name == 'Dinner Mess'
    assert len(spot.vertices) == 4

    constraints = ConstraintSet(spot_area=spot)
    # mask_for is Keepout mask (negative constraints) - spot_area is positive and should not appear in mask_for
    assert not constraints.mask_for(source).any()

    # spot_mask_for returns the rasterized positive target
    spot_mask = constraints.spot_mask_for(source)
    assert spot_mask is not None
    assert spot_mask[30, 20]
    assert not spot_mask[10, 10]


def test_spot_area_validation_and_identifier_uniqueness():
    with pytest.raises(ValueError, match='identifier'):
        SpotArea(identifier='', vertices=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)))

    with pytest.raises(ValueError, match='three vertices'):
        SpotArea(identifier='too_few', vertices=((0.0, 0.0), (1.0, 0.0)))

    with pytest.raises(ValueError, match='collinear'):
        SpotArea(identifier='line', vertices=((0.0, 0.0), (1.0, 1.0), (2.0, 2.0)))

    with pytest.raises(ValueError, match='positive finite'):
        SpotArea.from_box(center=(0.0, 0.0), width_m=-1.0, height_m=0.5)

    # Identifier collision with keepout
    with pytest.raises(ValueError, match='globally unique'):
        ConstraintSet(
            keepouts=(Keepout('same_id', ((0.0, 0.0), (1.0, 0.0), (0.5, 1.0))),),
            spot_area=SpotArea('same_id', ((2.0, 2.0), (3.0, 2.0), (2.5, 3.0))),
        )

