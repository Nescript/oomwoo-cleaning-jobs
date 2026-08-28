"""Tests for cleaning target configuration (whole-map, selected regions, spot areas)."""

import math

import numpy as np
import pytest

from fixtures import fake_segmentation, make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import (
    ConstraintSet,
    Keepout,
    SpotArea,
    _rasterize_polygon,
)
from oomwoo_cleaning_jobs_core.regions import UNASSIGNED, RegionSet
from oomwoo_cleaning_jobs_core.targets import (
    CleaningTarget,
    configure_last_spot_area,
    configure_selected_regions,
    configure_spot_area,
    configure_whole_map,
    create_spot_region_set,
)


def _map_point(source, row: int, col: int) -> tuple[float, float]:
    x, y, yaw = source.origin
    local_x = (col + 0.5) * source.resolution
    local_y = (row + 0.5) * source.resolution
    return (
        x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def _make_sample_region_set():
    source = make_two_rooms_map()
    result = fake_segmentation(source)
    rs = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)
    return source, rs


def test_configure_whole_map():
    source, rs = _make_sample_region_set()
    target = configure_whole_map(rs)

    assert target.target_labels == (1, 2)
    assert target.labels == (1, 2)
    assert target.region_set is rs

    # Downstream querying
    assert target.mask_of(1).any()
    assert target.mask_of(2).any()
    assert len(target.regions()) == 2
    assert [r.label for r in target.regions()] == [1, 2]

    # Out-of-target label query fails
    with pytest.raises(ValueError, match='not in target_labels'):
        target.mask_of(999)


def test_configure_whole_map_empty_fails():
    source = make_two_rooms_map()
    empty_rs = RegionSet(
        labels=np.zeros(source.cells.shape, dtype=np.int32),
        cleanable=source.free_mask(),
        resolution=source.resolution,
        origin=source.origin,
    )
    with pytest.raises(ValueError, match='no regions'):
        configure_whole_map(empty_rs)


def test_configure_selected_regions_preserves_order():
    source, rs = _make_sample_region_set()
    # Request in specific order: [2, 1]
    target = configure_selected_regions(rs, [2, 1])

    assert target.target_labels == (2, 1)
    assert [r.label for r in target.regions()] == [2, 1]
    assert np.array_equal(target.mask_of(2), rs.mask_of(2))
    assert np.array_equal(target.mask_of(1), rs.mask_of(1))


def test_configure_selected_regions_validations():
    source, rs = _make_sample_region_set()

    with pytest.raises(ValueError, match='empty'):
        configure_selected_regions(rs, [])

    with pytest.raises(ValueError, match='does not exist'):
        configure_selected_regions(rs, [1, 999])


def test_configure_spot_area_builds_transient_region_set_and_updates_constraints():
    source, rs = _make_sample_region_set()
    initial_labels_copy = rs.labels.copy()

    center = _map_point(source, 30, 20)
    spot = SpotArea.from_box(center, width_m=1.0, height_m=1.0, identifier='mess', name='Coffee Spill')
    constraints = ConstraintSet()

    target, updated_constraints = configure_spot_area(
        source_map=source,
        constraints=constraints,
        spot=spot,
        robot_inscribed_radius=0.17,
    )

    # 1. Output target labels
    assert target.target_labels == (1,)
    assert target.regions()[0].name == 'Coffee Spill'
    assert target.mask_of(1)[30, 20]
    assert len(target.outline(1)) >= 1

    # 2. Original RegionSet must NOT be mutated
    assert np.array_equal(rs.labels, initial_labels_copy)

    # 3. Updated constraints retain the last used spot area
    assert updated_constraints.spot_area == spot


def test_spot_area_clips_with_free_space_and_keepout_and_spans_rooms():
    source, rs = _make_sample_region_set()
    # Place spot across the doorway connecting the two rooms (row 40, col 30)
    center = _map_point(source, 40, 30)
    spot = SpotArea.from_box(center, width_m=2.0, height_m=1.0, identifier='hallway_mess')

    # Add a keepout that covers a small portion in room 2 (row 40, col 40)
    keepout_box = SpotArea.from_box(_map_point(source, 40, 40), width_m=0.3, height_m=0.3, identifier='ko').vertices
    constraints = ConstraintSet(keepouts=(Keepout('ko', keepout_box),))

    target, _ = configure_spot_area(source, constraints, spot, robot_inscribed_radius=0.17)
    spot_mask = target.mask_of(1)

    # Spot area covers cleanable cells across room 1, doorway, and room 2
    assert spot_mask[40, 20]  # in room 1
    assert spot_mask[40, 30]  # in doorway
    assert spot_mask[40, 45]  # in room 2

    # Spot area does not cover keepout or outer wall
    assert not spot_mask[40, 40]  # keepout cell
    assert not spot_mask[0, 0]    # outer unknown/occupied


def test_spot_area_validation():
    source = make_two_rooms_map()
    # 1. Spot strictly inside unknown outside space (row 0, col 0, width 0.1m -> only cells 0, 1) fails
    center = _map_point(source, 0, 0)
    spot = SpotArea.from_box(center, width_m=0.1, height_m=0.1, identifier='in_unknown')
    with pytest.raises(ValueError, match='no cleanable space'):
        configure_spot_area(source, ConstraintSet(), spot)

    # 2. Spot smaller than robot footprint (5cm x 5cm) in open free space is ALLOWED
    center_free = _map_point(source, 30, 20)
    tiny_spot = SpotArea.from_box(center_free, width_m=0.05, height_m=0.05, identifier='tiny')
    target, _ = configure_spot_area(source, ConstraintSet(), tiny_spot, robot_inscribed_radius=0.17)
    assert target.target_labels == (1,)
    assert target.mask_of(1).any()

    # 3. Spot in an isolated narrow cavity where robot cannot navigate fails validation
    # Create keepouts that isolate row 40, cols 10:15 from all sides beyond 0.05m
    ko1 = SpotArea.from_box(_map_point(source, 38, 12), width_m=2.0, height_m=0.1, identifier='ko1').vertices
    ko2 = SpotArea.from_box(_map_point(source, 42, 12), width_m=2.0, height_m=0.1, identifier='ko2').vertices
    ko3 = SpotArea.from_box(_map_point(source, 40, 8), width_m=0.1, height_m=2.0, identifier='ko3').vertices
    ko4 = SpotArea.from_box(_map_point(source, 40, 16), width_m=0.1, height_m=2.0, identifier='ko4').vertices
    trap_constraints = ConstraintSet(keepouts=(
        Keepout('ko1', ko1), Keepout('ko2', ko2), Keepout('ko3', ko3), Keepout('ko4', ko4)
    ))
    trapped_spot = SpotArea.from_box(_map_point(source, 40, 12), width_m=0.1, height_m=0.1, identifier='trapped')
    with pytest.raises(ValueError, match='validation'):
        configure_spot_area(source, trap_constraints, trapped_spot, robot_inscribed_radius=0.17)


def test_configure_last_spot_area():
    source, rs = _make_sample_region_set()
    spot = SpotArea.from_box(_map_point(source, 30, 20), width_m=1.0, height_m=1.0,
                             identifier='last_sp', name='Last Spill')
    constraints = ConstraintSet(spot_area=spot)

    target = configure_last_spot_area(source, constraints, robot_inscribed_radius=0.17)
    assert target.target_labels == (1,)
    assert target.regions()[0].name == 'Last Spill'
    assert target.mask_of(1)[30, 20]

    # If no spot area saved, fails clearly
    empty_constraints = ConstraintSet()
    with pytest.raises(ValueError, match='No previously saved spot area'):
        configure_last_spot_area(source, empty_constraints)
