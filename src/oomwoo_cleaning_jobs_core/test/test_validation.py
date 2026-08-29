"""validation severity-grading tests."""

import math

import numpy as np

from fixtures import fake_segmentation, make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, VirtualWall
from oomwoo_cleaning_jobs_core.regions import RegionSet
from oomwoo_cleaning_jobs_core.validation import (
    LEVEL_ERROR,
    LEVEL_WARNING,
    check_masks_overlap,
    validate_region_set,
)


def _map_point(source, row, col):
    """Map-frame center point of a given cell, honoring the SourceMap yaw."""
    x, y, yaw = source.origin
    local_x = (col + 0.5) * source.resolution
    local_y = (row + 0.5) * source.resolution
    return (
        x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def _sealed_doorway_setup():
    """Two-room map with a Virtual Wall sealing the doorway (rows 35-44, col 30)."""
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        virtual_walls=(VirtualWall('door-seal', _map_point(source, 34, 30),
                                   _map_point(source, 45, 30), source.resolution),))
    band = constraints.mask_for(source)
    result = fake_segmentation(source, cleanable_mask=source.free_mask() & ~band)
    region_set = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin,
        base_cleanable=source.free_mask(), keepout_mask=band)
    return source, region_set, band


def _make_region_set():
    source = make_two_rooms_map()
    result = fake_segmentation(source)
    return source, RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)


def _codes(report, level):
    return {i.code for i in report.issues if i.level == level}


def test_clean_region_set_has_no_errors():
    """A Region Set produced by the normal editing path triggers no errors
    (invariants hold by construction)."""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert report.ok
    assert not report.errors


def test_unassigned_cleanable_is_warning_not_error():
    """Unassigned cleanable space is a warning; publishing is still allowed."""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_fully_assigned_set_has_no_unassigned_warning():
    source, rs = _make_region_set()
    # Paint every unassigned cleanable cell into the first Region
    label = rs.regions()[0].label
    rs.paint(label, rs.unassigned_cleanable_mask)
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' not in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_region_outside_cleanable_is_error():
    """Invariant check: hand-edited data pokes a Region cell onto a wall ->
    error."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    rs.labels[4, 10] = label  # bottom wall (occupied)
    rs.labels[0, 0] = label   # unknown
    report = validate_region_set(rs)
    assert not report.ok
    assert 'region_outside_cleanable' in _codes(report, LEVEL_ERROR)


def test_small_or_narrow_region_in_navigable_space_is_allowed():
    """A small or narrow strip region located in open navigable space is allowed
    because the robot can reach and sweep it from adjacent navigable positions."""
    source, rs = _make_region_set()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[40, 5:29] = True  # single row, 0.05 m wide, in open navigable room
    label = rs.create(stroke, name='Slit')
    assert label is not None
    report = validate_region_set(rs)
    assert report.ok
    assert 'region_unreachable' not in _codes(report, LEVEL_ERROR)


def test_isolated_unreachable_region_is_error():
    """A region in an isolated cavity where no navigable position can reach
    triggers region_unreachable error."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label

    # Surround region with keepouts so that no position within 0.17m of the region
    # can fit the robot footprint
    # Keep only a tiny 1-cell strip free and assign it to a Region
    cleanable = np.zeros(source.cells.shape, dtype=bool)
    cleanable[40, 10:15] = True  # 5 cells (0.25m x 0.05m), surrounded by walls/non-cleanable
    isolated_rs = RegionSet(
        labels=cleanable.astype(np.int32),
        cleanable=cleanable,
        resolution=source.resolution,
        origin=source.origin,
    )
    report = validate_region_set(isolated_rs, robot_inscribed_radius=0.17)
    assert not report.ok
    unreachable = [i for i in report.errors if i.code == 'region_unreachable']
    assert len(unreachable) == 1



def test_disconnected_core_is_warning():
    """Two roomy pieces joined by a 1-cell narrow throat: reachable core is
    split into two pieces -> warning, not error.

    The "unreachable ratio" metric is deliberately not used: the perimeter
    ring of any room is unreachable to the robot center (~30%), so that
    metric would always be a false positive for normal rooms."""
    source, rs = _make_region_set()
    # Paint on the left room: two 10x10 blocks (0.5 m, each with a reachable
    # core) + a 1-cell narrow throat
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[10:20, 8:18] = True    # block 1
    stroke[10:20, 20:29] = True   # block 2
    stroke[15, 18:20] = True      # narrow throat (0.05 m < robot diameter)
    label = rs.create(stroke, name='Dumbbell')
    assert label is not None
    report = validate_region_set(rs)
    assert report.ok  # warnings do not block publishing
    assert 'region_disconnected_core' in _codes(report, LEVEL_WARNING)
    assert 'region_unreachable' not in _codes(report, LEVEL_ERROR)
    # A normal room's core is one connected piece and must not be flagged
    normal = [i for i in report.warnings if i.code == 'region_disconnected_core']
    assert all(i.region == label for i in normal)


def test_keepout_intersection_is_error():
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    keepout = np.zeros(source.cells.shape, dtype=bool)
    keepout[40, 10:20] = True
    # Keepout intersecting a Region -> error
    assert rs.mask_of(label)[40, 10:20].any()
    report = validate_region_set(rs, keepout_mask=keepout)
    assert not report.ok
    assert 'region_in_keepout' in _codes(report, LEVEL_ERROR)
    # Non-intersecting Keepout -> no error
    keepout2 = np.zeros(source.cells.shape, dtype=bool)
    keepout2[4, 10:20] = True  # on a wall, inside no Region
    report2 = validate_region_set(rs, keepout_mask=keepout2)
    assert 'region_in_keepout' not in _codes(report2, LEVEL_ERROR)


def test_empty_region_set_is_error():
    source = make_two_rooms_map()
    rs = RegionSet(
        labels=np.zeros(source.cells.shape, dtype=np.int32),
        cleanable=source.free_mask(),
        resolution=source.resolution,
    )
    report = validate_region_set(rs)
    assert not report.ok
    assert 'empty_region_set' in _codes(report, LEVEL_ERROR)


def test_check_masks_overlap():
    source = make_two_rooms_map()
    mask_a = np.zeros(source.cells.shape, dtype=bool)
    mask_b = np.zeros(source.cells.shape, dtype=bool)
    mask_a[10:20, 10:20] = True
    mask_b[15:25, 15:25] = True
    issues = check_masks_overlap({1: mask_a, 2: mask_b})
    assert len(issues) == 1
    assert issues[0].code == 'region_overlap'
    mask_b[15:25, 15:25] = False
    mask_b[30:40, 30:40] = True
    assert check_masks_overlap({1: mask_a, 2: mask_b}) == []


def test_seeded_validation_flags_enclosed_region_and_preserves_cells():
    """Dock-seeded mode: the room behind a sealed doorway is enclosed; its
    Region keeps its cells but is rejected with region_enclosed."""
    source, rs, band = _sealed_doorway_setup()
    right_label = [i.label for i in rs.regions() if rs.mask_of(i.label)[40, 50]][0]
    left_label = [i.label for i in rs.regions() if rs.mask_of(i.label)[40, 15]][0]
    right_cells_before = int(rs.mask_of(right_label).sum())

    report = validate_region_set(
        rs, keepout_mask=band, wall_band_mask=band,
        seed_pose=_map_point(source, 40, 15))

    assert not report.ok
    enclosed = [i for i in report.errors if i.code == 'region_enclosed']
    assert {i.region for i in enclosed} == {right_label}
    # Only the wall band cells were clipped; enclosed cells are preserved.
    assert int(rs.mask_of(right_label).sum()) == right_cells_before
    assert int(rs.mask_of(right_label).sum()) == int(
        (rs.cleanable & (np.indices(rs.labels.shape)[1] > 30)).sum())
    # The enclosure error replaces the generic reachability error.
    assert 'region_unreachable' not in _codes(report, LEVEL_ERROR)
    # A sealing wall must not raise the seals-nothing warning.
    assert 'virtual_wall_seals_nothing' not in _codes(report, LEVEL_WARNING)
    assert left_label not in {i.region for i in enclosed}


def test_seeded_validation_without_walls_matches_global_semantics():
    """With a seed but no constraints, both rooms stay reachable and clean."""
    source, rs = _make_region_set()
    report = validate_region_set(rs, seed_pose=_map_point(source, 40, 15))
    assert report.ok
    assert 'region_enclosed' not in _codes(report, LEVEL_ERROR)
    assert 'dock_unreachable' not in _codes(report, LEVEL_ERROR)


def test_dock_unreachable_when_seed_constrained_or_off_grid():
    source, rs, band = _sealed_doorway_setup()
    # Seed on the wall band itself (occupied by the constraint).
    report = validate_region_set(rs, seed_pose=_map_point(source, 40, 30))
    assert not report.ok
    assert 'dock_unreachable' in _codes(report, LEVEL_ERROR)

    # Seed outside the map grid.
    far_away = (source.origin[0] + 1000.0, source.origin[1] + 1000.0)
    report = validate_region_set(rs, seed_pose=far_away)
    assert not report.ok
    assert 'dock_unreachable' in _codes(report, LEVEL_ERROR)


def test_virtual_wall_seals_nothing_warning():
    """A wall segment in the middle of an open room seals nothing -> warning."""
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        virtual_walls=(VirtualWall('free-standing', _map_point(source, 40, 10),
                                   _map_point(source, 40, 24), source.resolution),))
    band = constraints.mask_for(source)
    result = fake_segmentation(source, cleanable_mask=source.free_mask() & ~band)
    rs = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin,
        base_cleanable=source.free_mask(), keepout_mask=band)

    report = validate_region_set(
        rs, keepout_mask=band, wall_band_mask=band,
        seed_pose=_map_point(source, 45, 15))

    assert report.ok  # warning only, publishing still allowed
    assert 'virtual_wall_seals_nothing' in _codes(report, LEVEL_WARNING)
    assert 'region_enclosed' not in _codes(report, LEVEL_ERROR)


def test_wall_band_mask_shape_must_match_grid():
    source, rs = _make_region_set()
    import pytest
    with pytest.raises(ValueError, match='shape'):
        validate_region_set(rs, seed_pose=_map_point(source, 40, 15),
                            wall_band_mask=np.zeros((2, 2), dtype=bool))
